import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  # Pkg path
  pkg_share = get_package_share_directory("localization_server")          

  # Read map_file from CLI 
  map_file = LaunchConfiguration("map_file")
  
  # Create a path only at runtime when the user pases the arguments 
  map_yaml = PathJoinSubstitution([pkg_share, "config", map_file])

  # Set AMCL config path for Sim/Real 
  amcl_yaml_file = PythonExpression(["'amcl_config_sim.yaml' if '_sim' in '", map_file, "' else 'amcl_config_real.yaml'"])  

  # Build AMCL.yaml full path at runtime
  amcl_yaml = PathJoinSubstitution([pkg_share, "config", amcl_yaml_file]) 

  # Set use_sim_time True/False -> if map _sim or not
  use_sim_time = PythonExpression(["'true' if '_sim' in '", map_file, "' else 'false'"])
  
  # Rviz path
  rivz_config = os.path.join(pkg_share, "rviz", "config.rviz")            

  return LaunchDescription([
    # Set argument
    DeclareLaunchArgument(
      name="map_file",
      default_value="warehouse_map_sim.yaml",
      description="Map YAML filename inside localization/config",
    ),
    
    # map_server node
    Node(
      package="nav2_map_server",
      executable="map_server",
      name="map_server",
      output="screen",
      parameters=[{"use_sim_time": use_sim_time},
                  {"yaml_filename": map_yaml}],
    ),
    
    # amcl node
    Node(
      package="nav2_amcl",
      executable="amcl",
      name="amcl",
      output="screen",
      parameters=[amcl_yaml],
    ),

    # lifecycle manaeger node
    Node(
      package="nav2_lifecycle_manager",
      executable="lifecycle_manager",
      name="lifecycle_manager",
      output="screen",
      parameters=[{"use_sim_time": use_sim_time},
                  {"autostart": True},
                  {"node_names": ["map_server", "amcl"]}],
    ),

    # rviz node
    Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="screen",
      arguments=["-d", rivz_config],
      parameters=[{"use_sim_time": use_sim_time}]
    ),
  ]) 