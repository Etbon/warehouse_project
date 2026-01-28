import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  # Set file at runtime in the CLI
  map_file = LaunchConfiguration("map_file")
  use_sim_time = LaunchConfiguration("use_sim_time")
  
  # Get pkg path
  pkg_share = get_package_share_directory("map_server")

  # Build map path at *launch runtime* (map_file is a LaunchConfiguration, not a string yet)
  map_yaml = PathJoinSubstitution([pkg_share, "config", map_file])
  
  # Rviz config path
  rviz_config = os.path.join(pkg_share, "rviz", "map_display.rviz")
 
  return LaunchDescription([

    # Set arguments default
    DeclareLaunchArgument(
      name="map_file",
      default_value="warehouse_map_sim.yaml",
      description="Map YAML filename inside map_server/config",
    ),
    
    DeclareLaunchArgument(
      name="use_sim_time",
      default_value="False",
      description="Use real/sim time.",
    ),

    # Map Server
    Node(
      package="nav2_map_server",
      executable="map_server",
      name="map_server",
      output="screen",
      parameters=[{"use_sim_time": use_sim_time},
                  {"yaml_filename": map_yaml}],
    ),
    
    # Lifecycle Manager
    Node(
      package="nav2_lifecycle_manager",
      executable="lifecycle_manager",
      name="lifecycle_manager_mapper",
      output="screen",
      parameters=[{"use_sim_time": use_sim_time},
                  {"autostart": True},
                  {"node_names": ["map_server"]}],
    ),
    
    # RViz 
    Node(
      package="rviz2",
      executable="rviz2",
      name="rviz",
      output="screen",
      arguments=["-d", rviz_config],
    ),

  ]) 