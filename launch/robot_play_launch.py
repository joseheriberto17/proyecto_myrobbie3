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
import math

def generate_launch_description():

   package_dir = FindPackageShare('proyecto_myrobbie3')
   params_path = PathJoinSubstitution([package_dir, 'config','params.yaml'])

   
   # nodo que actua como un puente entre ros2 y el protocolo de comunicacion mqtt
   mqtt_client = Node(package='mqtt_client', 
                       executable='mqtt_client',
                       parameters=[params_path],
                       output='screen')

   # nodo 
   node_tranfer_data = Node(package='proyecto_myrobbie3', 
                       executable='node_transfer_data',
                       output='screen')

   # ejecucion de cada nodo.
   return LaunchDescription([
   mqtt_client,
   node_tranfer_data
   ])