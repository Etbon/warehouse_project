import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  # Pkg path
  pkg_share = FindPackageShare("path_planner_server")    
  
  # Reed use_sim_time
  use_sim_time_ = LaunchConfiguration("use_sim_time")

  # Sim yaml / Real yaml on the CLI
  planner_file = PythonExpression([
    "'planner_sim.yaml' if '", use_sim_time_,"' == 'True' else 'planner_real.yaml'"
  ])
  controller_file = PythonExpression([
    "'controller_sim.yaml' if '", use_sim_time_,"' == 'True' else 'controller_real.yaml'"
  ])
  recoveries_file = PythonExpression([
    "'recoveries_sim.yaml' if '", use_sim_time_,"' == 'True' else 'recoveries_real.yaml'"
  ])
  bt_navigator_file = PythonExpression([
    "'bt_navigator_sim.yaml' if '", use_sim_time_,"' == 'True' else 'bt_navigator_real.yaml'"
  ])  

  planner_yaml = PathJoinSubstitution([pkg_share, "config", planner_file])              # Planner path
  controller_yaml = PathJoinSubstitution([pkg_share, "config", controller_file])        # Controller path
  recoveries_yaml = PathJoinSubstitution([pkg_share, "config", recoveries_file])        # Recoveries path
  bt_navigator_yaml = PathJoinSubstitution([pkg_share, "config", bt_navigator_file])  	# Bt_navigator path
  
  config_rviz = PathJoinSubstitution([pkg_share, "rviz", "pathplanning.rviz"])   # Rviz config
  
  # Velocity comand remap
  cmd_vel_target = PythonExpression([
    "'/diffbot_base_controller/cmd_vel_unstamped' if '", use_sim_time_,"' == 'True' else '/cmd_vel'" 
  ])

  cmd_vel_remap = [("cmd_vel", cmd_vel_target)]

  return LaunchDescription([
    # Set use_sim_time in CLI
    DeclareLaunchArgument(
      name="use_sim_time",
      default_value= "True",
      description="Set use_sim_time for path planer launch",
    ),

    # Planner node
    Node(
      package="nav2_planner",
      executable="planner_server",
      name="planner_server",
      output="screen",
      parameters=[planner_yaml, {"use_sim_time": use_sim_time_}],
    ),

    # Controller node
    Node(
      package="nav2_controller",
      executable="controller_server",
      name="controller_server",
      output="screen",     
      parameters=[controller_yaml, {"use_sim_time": use_sim_time_}],
      remappings=cmd_vel_remap,
    ),

    # Recoveries node
    Node(
      package="nav2_behaviors",
      executable="behavior_server",
      name="recoveries_server",
      output="screen",
      parameters=[recoveries_yaml, {"use_sim_time": use_sim_time_}],
      remappings=cmd_vel_remap,
    ),
    
    # Bt_navigator
    Node(
      package="nav2_bt_navigator",
      executable="bt_navigator",
      name="bt_navigator",
      output="screen", 
      parameters=[bt_navigator_yaml, {"use_sim_time": use_sim_time_}] 
    ),
    
    # Lifecycle manager node
    Node(
      package="nav2_lifecycle_manager",
      executable="lifecycle_manager",
      name="lifecycel_manager_pathplanner",
      output="screen",
      parameters=[{"autostart": True,
                  "use_sim_time": use_sim_time_,
                  "node_names": ["planner_server",
                                 "controller_server",
                                 "recoveries_server",
                                 "bt_navigator"]}] 
    ),
    
    # Rviz
    Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="screen",
      arguments=["-d", config_rviz],
      parameters=[{"use_sim_time": use_sim_time_,}]
    ),  
  ])