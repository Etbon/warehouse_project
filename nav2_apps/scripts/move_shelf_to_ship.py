#!/usr/bin/env python3

import rclpy
import math
import time
import tf2_geometry_msgs
import threading
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException
from tf_transformations import quaternion_from_euler
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# -------------------------
# Fixed mission coordinates
# -------------------------
INIT_X = -0.071
INIT_Y = 0.177
INIT_YAW = 0.025

LOAD_X = 6.00
LOAD_Y = -0.23
LOAD_YAW = -1.57

PRE_SHIP_X_ = 2.31
PRE_SHIP_Y = -0.055
PRE_SHIP_YAW = 1.57

SHIP_X = 2.50
SHIP_Y = 1.66
SHIP_YAW = 1.57

# -------------
# Make position
# -------------
def make_pose(navigator, x, y, yaw):
  # Build pose message
  pose = PoseStamped()
  pose.header.frame_id = "map"
  pose.header.stamp = navigator.get_clock().now().to_msg()
  
  # Fill position 
  pose.pose.position.x = x
  pose.pose.position.y = y
  pose.pose.position.z = 0.0
  
  # Fill oratation
  pose.pose.orientation.x = 0.0
  pose.pose.orientation.y = 0.0
  pose.pose.orientation.z = math.sin(yaw / 2.0)
  pose.pose.orientation.w = math.cos(yaw / 2.0)

  return pose

# ----------
# Wait timer
# ----------
def wait_until_arrived(navigator):
  while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    if feedback is not None:
      print("Waiting for arrival...")
    time.sleep(0.8)

# --------------------
# Shelf Detection Node  
# --------------------
class MoveShelfToShipNode(Node):
  def __init__(self):
    super().__init__("move_shelf_to_ship_node")

    # Declere laser scan
    self.laser_scan = None

    # Robot is carring the shelf
    self.loaded_mode = False

    # TF tools 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.static_tf_broadcaster = StaticTransformBroadcaster(self)

    # Initializa velocity comand 
    self.cmd_vel_pub = self.create_publisher(
        Twist,
        "/diffbot_base_controller/cmd_vel_unstamped",
        10
    )

    # Initilaize laser subscriber  
    self.laser_subscription = self.create_subscription(
        LaserScan, 
        "/scan",
        self.laser_callback,
        qos_profile_sensor_data
    )

    # Initialize filter laser scan for Nav2 publisher
    self.filter_scan_pub = self.create_publisher(
      LaserScan,
      "/scan_nav2",
      qos_profile_sensor_data
    )  

    # Initialize elevator-up publisher
    self.elevator_up_pub = self.create_publisher(
      String,
      "/elevator_up",
      10
    )

    # Initialize elevator-down publisher
    self.elevator_down_pub = self.create_publisher(
      String,
      "/elevator_down",
      10
    )

  # Laser Callback
  def laser_callback(self, msg):
    # Store the raw scan for shelf detection 
    self.laser_scan = msg
    
    # If robot is carrying the shelf publish a scan prepared for Nav2
    if self.loaded_mode:
      nav2_scan = self.filter_carried_shelf_from_scan(msg)
      # For now publish the same scan as /scan_filtered
      self.filter_scan_pub.publish(nav2_scan)
      
    # If robot is not carring the shelf publish the raw scan unchanged
    else:
      self.filter_scan_pub.publish(msg)    

  # Filtered carried shelf from scan 
  def filter_carried_shelf_from_scan(self, scan):
	  # 1. Create a new laser scan msg 
    filtered = LaserScan()
    
    # 2. Copy the matadate form the oiginal scan 
    filtered.header = scan.header
    filtered.angle_max = scan.angle_max
    filtered.angle_min = scan.angle_min
    filtered.angle_increment = scan.angle_increment
    filtered.time_increment = scan.time_increment
    filtered.scan_time = scan.scan_time
    filtered.range_max = scan.range_max
    filtered.range_min = scan.range_min

    # 3. Copy the original scan ranges and intensities 
    filtered.ranges = list(scan.ranges)
    filtered.intensities = list(scan.intensities)    

    # 4. Define the ignore region wher the carred shelf exists
    x_min = -0.4
    x_max = 0.4
    y_min = -0.45
    y_max = 0.45

		# 5. Loop throug each laser beam
    for beam_index, range_value in enumerate(filtered.ranges):
      if not math.isfinite(range_value):
        continue
      
      if range_value < filtered.range_min or range_value > filtered.range_max:
        continue
		  
      theta = filtered.angle_min + beam_index * filtered.angle_increment

      # 6. Convert each valid beam from ranges/angle x/y 
      x = range_value * math.cos(theta)
      y = range_value * math.sin(theta) 
      
		  # 7. If the x/y point is iside the point the region, hide it from Nav2
      inside_shelf_zone = (
        x_min <= x <= x_max and y_min <= y <= y_max          
      )

      if inside_shelf_zone:
        filtered.ranges[beam_index] = float("inf")

		# 8. Return the fitered scan
    return filtered

  # Findd shelf
  def find_reflective_indices(self, scan):
    # Creat a vector
    indices = []

    # Safty guard - laser data 
    if scan is None:
      self.get_logger().warn("No laser data found. Skipping detection.")
      return indices

    # Scan size 
    n = len(scan.ranges)

    # Safty guard - match laser size intesities 
    if len(scan.intensities) == 0 or len(scan.intensities) != n:
      self.get_logger().warn(f"Mismatched intensities: ranges={n}, intensities={len(scan.intensities)}")
      return indices

    # Pass through laser data 
    for i in range(n):
      r = scan.ranges[i]
      intensity = scan.intensities[i]
      
      # Filter NaN & Inf
      if not math.isfinite(r):
        continue
      
      # Keep high intensity values
      if intensity >= 8000.0:
        indices.append(i)

    return indices

  # Finde shelf leg 
  def find_shelf_leg_clusters(self, laser_indices):
    # Create cluester vector all 
    cluster = []
    current_cluster = []

    # Safety gaurd if empty
    if not laser_indices:
      return cluster

    # Create cluester vector one group
    current_cluster.append(laser_indices[0])

    # Loop throught the indecies 
    for i in range(1, len(laser_indices)):
      if laser_indices[i] - laser_indices[i - 1] < 5:
        current_cluster.append(laser_indices[i])
      else:
        cluster.append(current_cluster)
        current_cluster = [laser_indices[i]]
 
    # Save the lase currnet clusete
    if current_cluster:
      cluster.append(current_cluster)

    return cluster 
  
  # Set the avreage of one cluster
  def average_cluster_indices(self, clusters):
    # Declare avgrage cluster 
    average_indices = []

    for cluster in clusters:
      if not cluster:
        continue

      total_sum = 0

      for idx in cluster:
        total_sum += idx

      avg_index = total_sum // len(cluster)  
      average_indices.append(avg_index)

    return average_indices      

  # Get the [X,Y] of the shelf legs
  def average_indices_to_xy(self, scan, average_indices):
    # Make list 
    leg_points = []
    
    # Loop through each average index
    for i in average_indices:
      # Check index bounds
      if i < 0 or i >= len(scan.ranges):
        continue

      # Read range values and validate it 
      range_value = scan.ranges[i]
      if not math.isfinite(range_value) or range_value <= 0.0:
        continue

      # Compute the angle of the beam 
      theta = scan.angle_min + i * scan.angle_increment

      # Convert polar (range, theta) --> Cartesian (x, y)
      x = range_value * math.cos(theta)
      y = range_value * math.sin(theta)

      # Save the leg point
      leg_points.append((x,y))

    return leg_points

  # Tranform laser [X,Y] to odom  
  def laser_xy_to_odom(self, lx, ly, laser_frame, stamp):
    # Build point in laser frame
    p_laser = PointStamped() 
    p_laser.header.stamp = stamp
    p_laser.header.frame_id = laser_frame 
    p_laser.point.x = lx
    p_laser.point.y = ly 
    p_laser.point.z = 0.0

    try:
      # Set source frame to target frame -> odom
      p_odom = self.tf_buffer.transform(
        p_laser, 
        "odom", 
        timeout=Duration(seconds=0.2)
      )

      # Extract the TF odom values 
      ox = p_odom.point.x
      oy = p_odom.point.y 

      return True, ox, oy

    except TransformException as ex:
      self.get_logger().warn(f"Fail laser -> odom transform: {ex}")
      return False, None, None 
      
  # Set static TF
  def publish_cart_frame_static_once(self, x, y, parent_frame, child_frame, yaw_rad):
    # Build the stamped tranform
    t = TransformStamped()
    
    # Label the tranfrom
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    # Place the child frame translation in the parent frame 
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    # Convert yaw angle into quaternion rotation
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_rad) 
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw

    # Publish the new TF frame
    self.static_tf_broadcaster.sendTransform(t)
  
  # Drive to TF frame
  def drive_to_cart_frame(self, child_frame, xy_tol=0.10, max_time_sec=20.0):
    loop_hz = 20.0
    dt = 1.0 / loop_hz
    start_time = time.time()

    # Loop and finde the TF frame
    while rclpy.ok() and (time.time() - start_time) < max_time_sec:
      try: 
        T = self.tf_buffer.lookup_transform(
          "robot_base_link",
          child_frame,
          rclpy.time.Time()
        )

      except TransformException as ex:
        self.get_logger().warn(f"TF lookup failed: {ex}")
        time.sleep(dt)
        continue      

      # Read target position relative to the robot 
      dx = T.transform.translation.x
      dy = T.transform.translation.y

      # Compute straight-line distance to target
      distance = math.hypot(dx, dy)

      self.get_logger().info(
        f"Approche debug -> dx: {dx:.3f}, dy: {dy:.3f}, distance: {distance:.3f}"
      )
      
      # Stop if close enugh to target 
      if distance < xy_tol:
        self.cmd_vel_pub.publish(Twist())
        return True

      # Angle error: where is the target realative to the robot heading?
      yaw_err = math.atan2(dy, dx)

      # Linear speed: move foward based on foward distance
      v_prop = 0.6 * dx    

      # Safe speed: between 0.08 and 0.25 
      v_cap_near = min(max(distance, 0.08), 0.25)

      # Foward speed: clamp raw speed 
      v_cmd = min(max(v_prop, -v_cap_near), v_cap_near)

      # Angular speed:
      w_prop = 1.5 * yaw_err
      w_cmd = min(max(w_prop, -1.0), 1.0)

      # Build Twist  
      cmd = Twist()
      cmd.linear.x = v_cmd
      cmd.angular.z = w_cmd
      
      # Publish Twist  
      self.cmd_vel_pub.publish(cmd)
      time.sleep(dt)

    # Time out robot fail to reach the robot
    self.cmd_vel_pub.publish(Twist())
    return False

  # Get robot position in odom fram 
  def get_robot_pose_in_odom(self):
    try:
      T = self.tf_buffer.lookup_transform(
        "odom",
        "robot_base_link",
        rclpy.time.Time()  
      )

      # Position
      x = T.transform.translation.x
      y = T.transform.translation.y
      
      # Orentation quaternians 
      q = T.transform.rotation

      # Convert quatenion to yaw
      siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
      cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
      yaw = math.atan2(siny_cosp, cosy_cosp)

      return True, x, y, yaw

    except TransformException as ex:
      self.get_logger().warn(f"Failed to get robot odom pose: {ex}")
      return False, None, None, None

  # Clamp 
  def clamp(self, value, min_value, max_value):
    return max(min_value, min(value, max_value))
    
  # Normaliza angle to the range [-pi, pi]
  def normaliza_angle(self, angle):
    while angle > math.pi:
      angle -= 2.0 * math.pi
    
    while angle < -math.pi:
      angle += 2.0 * math.pi

    return angle 

  # Move foward 30cm  
  def creep_forward(self, distance_m, speed_mps=0.10):
    # Check if distanc is positive 
    if distance_m <= 0.0:
      return False
    
    # Read robot start position in odom
    ok, start_x, start_y, start_yaw = self.get_robot_pose_in_odom()

    if not ok:
      self.get_logger().warn("Cannot start creep: faild to read start odom pose.")
      return False

    # Loop timing
    loop_hz = 20.0
    dt = 1.0 / loop_hz
    start_time = time.time()

    # Build velocity command
    cmd = Twist()

    # Safty timeout 
    max_time_sec = distance_m / max(1e-6, speed_mps) + 5.0
    
    # Yaw corection turnig 
    angular_gain = 1.5
    max_w = 0.4
   
    while rclpy.ok() and (time.time() - start_time) < max_time_sec:
      # Read the robot current position
      ok, current_x, current_y, current_yaw = self.get_robot_pose_in_odom()

      # Safty gaurd
      if not ok:
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().warn("Creep failed: lost odom tranform.")
        return False
      
      # Compute the distanc traveled distance 
      traveled = math.hypot(current_x - start_x, current_y - start_y)
      
      # Compute yaw error
      yaw_error = start_yaw - current_yaw 
      raw_angular_cmd = angular_gain * yaw_error
      angular_cmd = self.clamp(raw_angular_cmd, -max_w, max_w)
      
      # Log pregress
      self.get_logger().info(
        f"Creep debug -> traveled: {traveled:.3f} / {distance_m:.3f} m"
      )
      self.get_logger().info(
        f"Yaw debug -> error: {yaw_error:.3f}, angular_cmd: {angular_cmd:.3f}"
      )  

      # Stop if reched distance
      if traveled >= distance_m:
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Mesured creep reached target distance.")
        return True

      # Move foward while correcting angle
      cmd.linear.x = speed_mps
      cmd.angular.z = angular_cmd
      self.cmd_vel_pub.publish(cmd)
      
      time.sleep(dt)

    # Time out & stop 
    self.cmd_vel_pub.publish(Twist())
    self.get_logger().warn("Mesured creep time out before reching target distance.")
    return False
    
  # Lift Shelf 
  def lift_shelf(self):
    msg = String()

    self.get_logger().info("Lifting shelf...")
    
    msg.data = "up"
    self.elevator_up_pub.publish(msg)

    return True 

  # Change robot footprint
  def set_nav2_footprint(self, footprint):

    # 1. Store the costmap node names 
    costmap_nodes = [
      "/local_costmap/local_costmap",
      "/global_costmap/global_costmap"
    ]

    # 2. Loop through each costmap node
    for node_name in costmap_nodes:
      # 3. Build the set_parameters service name 
      service_name = f"{node_name}/set_parameters"
      
      # 4. Creat a Service client 
      client = self.create_client(SetParameters, service_name)

      # 5. Wait for service to exist 
      if not client.wait_for_service(timeout_sec=2.0):
        self.get_logger().warn(f"Service not availabel: {service_name}")
        return False

      # 6. Build the footprint parameter 
      parameter = Parameter()
      parameter.name = "footprint"
      parameter.value = ParameterValue(
        type=ParameterType.PARAMETER_STRING,
        string_value=footprint
      )      

      # 7. Build the service request 
      request = SetParameters.Request()
      request.parameters = [parameter]

      # 8. Send the request 
      future = client.call_async(request)

      # 9. Check if the request faild
      rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

      if future.result() is None:
        self.get_logger().warn(f"Failed to call {service_name}")
        return False

      # 10. Check if Nav2 rejected the parameter
      for result in future.result().results:
        if not result.successful:
          self.get_logger().warn(
            f"Nav2 rejected footprint on {node_name}: {result.reason}"
          )
          return False

      self.get_logger().info(f"Update footprint on {node_name}")

    # 11. If both costmap were updated, retrun True
    return True 
  
  # Lower Shelf 
  def lower_shelf(self):
    msg = String()

    self.get_logger().info("Lowering shelf...")
    
    msg.data = "down"

    for _ in range(5):
      self.elevator_down_pub.publish(msg)
      time.sleep(0.1)

    return True

  # Change robot footprint back to original form 
  def set_nav2_circle_footprint(self, robot_radius):
    # Store both Nav2  costmap node names
    costmap_nodes = [
      "/local_costmap/local_costmap",
      "/global_costmap/global_costmap"
    ]

    # Loop through each costmap node
    for node_name in costmap_nodes:
      # Build the set parameters for each costmap
      service_name = f"{node_name}/set_parameters"

      # Create client to talk to this set_parameters service 
      client = self.create_client(SetParameters, service_name)

      # Wait until service is available 
      if not client.wait_for_service(timeout_sec=2.0):
        self.get_logger().warn(f"Service not available: {service_name}")
        return False

      # Create footprint parameters and set it to "[]" to clear polygon footprint
      footprint_param = Parameter()
      footprint_param.name = "footprint"
      footprint_param.value = ParameterValue(
        type=ParameterType.PARAMETER_STRING,
        string_value="[]"
      )

      # Create a robot_radius parameter and set it to radius we passed into the function
      radius_param = Parameter()
      radius_param.name = "robot_radius"
      radius_param.value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE,
        double_value=robot_radius
      )

      # Put both parameters into one SetParameters request 
      request = SetParameters.Request()
      request.parameters = [footprint_param, radius_param]

      # Send request to Nav2
      future = client.call_async(request) 

      # Wait to Nav2 answer
      start_time = time.time()

      rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

      # If Nav2 response empty, retrun False
      response = future.result()
      
      if response is None:
        self.get_logger().warn(f"Failed to call {service_name}")
        return False

      # If Nav2 rejected any parameter, retrun False
      for result in response.results:
        if not result.successful:
          self.get_logger().warn(
            f"Nav2 rejected circular footprint on {node_name}: {result.reason}"
          )
          return False

      self.get_logger().info(f"Restored circular footpint on {node_name} with robot_radius={robot_radius}"
      )

    # If both costmap accepted the change, retrun True 
    return True 

  # -------------
  # Control Logic 
  # -------------

  # --- Loding shelf behavior ---
  def loading_shelf_behavior(self):
    # ROS porcess one laser data callback cycle  
    start_time = time.time()
    while rclpy.ok() and self.laser_scan is None and (time.time() - start_time) < 5.0:
      time.sleep(0.05)

    # Safety guard is laser data is empty  
    if self.laser_scan is None:
      print("Laser scan is not recived.")
      return False

    # Laser date 
    scan = self.laser_scan 
  
    # Finde refelction indices
    reflective_indices = self.find_reflective_indices(scan)

    # Finde shelf legs clusters
    shelf_leg = self.find_shelf_leg_clusters(reflective_indices) 
    
    # Finde the avrage cluster of the legs
    avg_index = self.average_cluster_indices(shelf_leg)

    # Finde the [X,Y] of the index
    leg_points = self.average_indices_to_xy(scan, avg_index)
    
    # ----------------------
    # Set X,Y to odom frame   
    # ----------------------

    # --- Safty gaurd only 2 points ---
    if len(leg_points) < 2:
      self.get_logger().warn("Not enugh leg point found.")
      return False 
    
    x1_laser, y1_laser = leg_points[0] # Split first leg point into x1, y1
    x2_laser, y2_laser = leg_points[1] # Split second leg point into x2, y2
    
    # --- Tranfrom both leg to odom --- 
    ok1, x1_odom, y1_odom = self.laser_xy_to_odom(x1_laser, y1_laser, scan.header.frame_id, scan.header.stamp)
    
    ok2, x2_odom, y2_odom = self.laser_xy_to_odom(x2_laser, y2_laser, scan.header.frame_id, scan.header.stamp)

    # --- Safty guard --- 
    if not ok1 or not ok2:
      self.get_logger().warn("Failed to tranform shelf legs to odom")
      return False

    # --- Compute midpoint in odom frame ---
    mx_odom = 0.5 * (x1_odom + x2_odom) 
    my_odom = 0.5 * (y1_odom + y2_odom)

    # --- Compute shelf leg line vector ---
    dx_line = x2_odom - x1_odom 
    dy_line = y2_odom - y1_odom

    # --- Compute shelf leg line yaw ---
    shelf_line_yaw = math.atan2(dy_line, dx_line)

    # --- Compute approche yaw ---
    approch_yaw = self.normaliza_angle(shelf_line_yaw + math.pi / 2.0)
   
    self.get_logger().info(f"Shelf geometry ->"
      f"leg1: ({x1_odom:.3f},{y1_odom:.3f}), "
      f"leg2: ({x2_odom:.3f},{y2_odom:.3f}), "
      f"center: ({mx_odom:.3f},{my_odom:.3f}), "
      f"shelf_line_yaw: {shelf_line_yaw:.3f}"
    )

    # Set static TF
    self.publish_cart_frame_static_once(mx_odom, my_odom, "odom", "cart_frame", approch_yaw)
   
    self.get_logger().info(f"Publish cart_frame at odom -> x: {mx_odom:.3f}, y: {my_odom:.3f}")

    # Drive to TF 
    reach_cart_frame = self.drive_to_cart_frame("cart_frame")
    
    # Move foward 30cm 
    if not reach_cart_frame:
      self.get_logger().warn("Faild to reach cart frame.")
      return False
    
    ok_forward = self.creep_forward(0.30)

    if not ok_forward:
       self.get_logger().warn("Faild to creep forward.")
       return False

    # Lift shelf 
    ok_lift = self.lift_shelf()

    if not ok_lift:
      self.get_logger().warn("Fail to lift shelf.")
      return False

    # Change footpirnt      
    loaded_footprint = "[[0.35, 0.30], [0.35, -0.30], [-0.35, -0.30], [-0.35, 0.30]]"
    
    ok_footprint = self.set_nav2_footprint(loaded_footprint)  

    if not ok_footprint:
      self.get_logger().warn("Fail to update loaded footprint.")
      return False

    # Change the scan filter for Nav2 navigation
    self.loaded_mode = True
    self.get_logger().info("Loded mode ON: publishing filtered scan to /scan_nav2")

    return True  

  # --- Move backward using time only ---
  def creep_backward(self, distance_m=0.75, speed_mps=0.2):
    # Safety guards
    if distance_m <= 0.0:
      return False

    if speed_mps <= 0.0:
      return False

    # Compute how long to move
    duration_sec = (distance_m / speed_mps) + 2.0

    # Loop timing
    loop_hz = 20.0
    dt = 1.0 / loop_hz
    start_time = time.time()

    # Build backward command
    cmd = Twist()
    cmd.linear.x = -speed_mps
    cmd.angular.z = 0.0

    self.get_logger().info(
      f"Time backup started -> distance: {distance_m:.3f} m, "
      f"speed: {speed_mps:.3f} m/s, duration: {duration_sec:.2f} s"
    )

    # Publish backward command for fixed duration
    while rclpy.ok() and (time.time() - start_time) < duration_sec:
      self.cmd_vel_pub.publish(cmd)
      time.sleep(dt)

    # Stop robot
    self.cmd_vel_pub.publish(Twist())
    self.get_logger().info("Time backup finished.")

    return True

  # --- Unloding shelf behavior ---
  def unloading_shelf_behavior(self):  
    # Stop the robot before unloding
    self.cmd_vel_pub.publish(Twist())
    time.sleep(0.2)

    # Unlode the shelf
    ok_lower = self.lower_shelf()

    if not ok_lower:
      self.get_logger().warn("Failed to lower shelf.")
      return False

    # Wait for the shelf to settle on the floor
    time.sleep(1.0)

    # Robot in no longer carine shelf back to normal /scan_nav2 topic with no filter
    self.loaded_mode = False
    self.get_logger().info("Loding mode OFF: robot is not longer carrying shelf.")

    # Restore normal footpint
    normal_robot_radius = 0.25

    ok_footprint = self.set_nav2_circle_footprint(normal_robot_radius)

    if not ok_footprint:
      self.get_logger().warn("Failed to restore circular footprint")
      return False

    self.get_logger().info("Shelf unloded successfully.")
    return True


def main(args=None):  
  # Start ROS 
  rclpy.init(args=args)

  navigator = BasicNavigator()       # Create navigator objet to talk to Nav2
  shelf_to_ship_node = MoveShelfToShipNode()  # Create shelf loding behavior node object  

  # Create executor and add both nodes
  executor = MultiThreadedExecutor()
  executor.add_node(shelf_to_ship_node)

  # Spin executor in the background thread
  spin_thread = threading.Thread(target=executor.spin, daemon=True)
  spin_thread.start()
  
  try:
    # Build initial pose 
    init_pose = make_pose(navigator, INIT_X, INIT_Y, INIT_YAW)   # Set initial position
    navigator.setInitialPose(init_pose)  # Send initial position
    navigator.waitUntilNav2Active()

    # Go to loding position 
    loading_pose = make_pose(navigator, LOAD_X, LOAD_Y, LOAD_YAW)   # Creat a target goal position
    navigator.goToPose(loading_pose)

    # Wait so robot reaches goal
    wait_until_arrived(navigator)
    result = navigator.getResult() 
    print(f"Navigation result: {result}")

    # Abort loding behavior if naviagtion fail 
    if str(result) != "TaskResult.SUCCEEDED":
      print("Fail to reach loading positon.")
      return
    
    # Perform shelf loading behavior
    loading_ok = shelf_to_ship_node.loading_shelf_behavior()

    # Abort continuing if loading fail
    if not loading_ok:
      print("Loding shelf behavior failed.")
      return

    # Move backward a short distance
    moving_backward_ok = shelf_to_ship_node.creep_backward()

    if not moving_backward_ok:
      print("Failed to back up after loading shelf.")
      return 
    
    navigator.clearAllCostmaps()
    time.sleep(0.5)

    # Go to pre shipping position
    pre_shipping_pose = make_pose(navigator, PRE_SHIP_X_, PRE_SHIP_Y, PRE_SHIP_YAW)
    navigator.goToPose(pre_shipping_pose)
    
    wait_until_arrived(navigator)
    pre_shipping_result = navigator.getResult()
    print(f"Pre-shipping navigation result: {pre_shipping_result}")

    if str(pre_shipping_result) != "TaskResult.SUCCEEDED":
      print("Failed to reach pre-shipping positoin.")
      return

    # Go to shipping position
    shipping_pose = make_pose(navigator, SHIP_X, SHIP_Y, SHIP_YAW)
    navigator.goToPose(shipping_pose)
    
    # Wait until robot reaches shipping position 
    wait_until_arrived(navigator)
    shipping_result = navigator.getResult()
    print(f"Shipping navigation result: {shipping_result}")

    # Abort if shipping position fail
    if str(shipping_result) != "TaskResult.SUCCEEDED":
      print("Fail to reach shipping position")
      return
      
    # Perfoerm shelf Unloding behavior & restore originla footprint
    unloding_ok = shelf_to_ship_node.unloading_shelf_behavior()

    if not unloding_ok:
      print("Fail to unlode shelf")
      return

    # Move backward a short distance
    moving_backward_ok = shelf_to_ship_node.creep_backward()

    if not moving_backward_ok:
      print("Failed to back up after unloading shelf.")
      return 

    # Clear costmap
    navigator.clearAllCostmaps()
    time.sleep(0.5)

    # Go to inital position
    init_return_pose = make_pose(navigator, INIT_X, INIT_Y, INIT_YAW) 
    navigator.goToPose(init_return_pose)

    # Wait util arrived
    wait_until_arrived(navigator)
    init_result = navigator.getResult()
    print(f"Initial position navigation result: {init_result}")
  
    if str(init_result) != "TaskResult.SUCCEEDED":
      print("Fail to reach initial position")
      return 

  finally:
    executor.shutdown()
    shelf_to_ship_node.destroy_node()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()