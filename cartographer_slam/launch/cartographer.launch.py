import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  
  # Paths
  cartographer_config_dir = os.path.join(get_package_share_directory("cartographer_slam"), "config")
  config_rviz = os.path.join(get_package_share_directory("cartographer_slam"), "rviz", "mapping.rviz")
  
  # Set simulatiom time 
  use_sim_time = LaunchConfiguration("use_sim_time")

  # Set up lua file 
  config_basename =  PythonExpression([
    '"cartographer_sim.lua" if ', use_sim_time, ' == "True" else "cartographer_real.lua"'
  ])

  return LaunchDescription([
    
    # Set a default value
    DeclareLaunchArgument(
      "use_sim_time",
      default_value="True",
      description="Use simulation time if true",
    ),

    Node( 
      package="cartographer_ros",
      executable="cartographer_node",
      name="cartographer_node",
      output="screen",
      parameters=[{"use_sim_time": use_sim_time}],
      arguments=["-configuration_directory", cartographer_config_dir,
                 "-configuration_basename", config_basename],
    ),

    Node(
      package="cartographer_ros",
      executable="cartographer_occupancy_grid_node",
      name="occupancy_gird_node",
      output="screen",
      parameters=[{"use_sim_time": use_sim_time}],
      arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    ),

    Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", config_rviz],
    ),
  ])