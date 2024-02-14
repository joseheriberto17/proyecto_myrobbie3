import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    
   package_dir = FindPackageShare('proyecto_myrobbie3')
   urdf_path = PathJoinSubstitution([package_dir, 'robot.urdf.xacro'])
   sim_time = True

   robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

   rsp_node = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{
                  'robot_description': robot_description_content,
                  'use_sim_time': sim_time
               }])

   gazebo = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            )

   # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
   spawn_entity = Node(package='gazebo_ros', 
                       executable='spawn_entity.py',
                       arguments=['-topic', 'robot_description','-entity', 'my_bot'],
                       output='screen')

   # Launch them all!
   
   


   return LaunchDescription([
   rsp_node,
   gazebo,
   spawn_entity
   ])