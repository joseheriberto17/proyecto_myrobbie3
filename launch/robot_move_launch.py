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
    
   # argumentos para rsp_node
   package_dir = FindPackageShare('proyecto_myrobbie3')
   urdf_path = PathJoinSubstitution([package_dir, 'robot.urdf.xacro'])
   sim_time = True

   robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

   path_world = os.path.join(get_package_share_directory('proyecto_myrobbie3'), 
                                                         'world', 
                                                         'wall.world')

   # nodo para publicar el estado cinemático de un robot en el sistema de coordenadas del robot.
   rsp_node = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{
                  'robot_description': robot_description_content,
                  'use_sim_time': sim_time
               }])

   # lanzamiento para inicializar el programa de gazebo. 
   gazebo = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('gazebo_ros'), 
                                             'launch', 
                                             'gazebo.launch.py')]),
               launch_arguments={
                  'world': path_world
               }.items()
               )

   #nodo para agregar modelos, robots u otros objetos al entorno de simulación en Gazebo.
   spawn_entity = Node(package='gazebo_ros', 
                       executable='spawn_entity.py',
                       arguments=['-topic', 'robot_description','-entity', 'my_robbie3','-y','-0.2','-Y',str(math.radians(0))],
                       output='screen')

   # ejecucion de cada nodo.
   return LaunchDescription([
   rsp_node,
   gazebo,
   spawn_entity
   ])