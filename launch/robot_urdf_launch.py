from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
   data = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               # buscar las carpeta de mismo paquete.
               FindPackageShare('urdf_launch'),
               'launch',
               'display.launch.py'
            ])
      ]),
      # parametro como argumentos de launch
      launch_arguments={
            'urdf_package': 'proyecto_myrobbie3',
            'urdf_package_path': 'robot.urdf.xacro'
      }.items()
   )

   return LaunchDescription([
      data
   ])