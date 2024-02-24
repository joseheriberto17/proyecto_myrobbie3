from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution



# este launch ejecuta rviz y joint_state_plublisher de modelo de robot  para hacer inpeccion de modelos solo con rviz2
def generate_launch_description():
   info =PathJoinSubstitution([
               # buscar las carpeta de mismo paquete.
               FindPackageShare('proyecto_myrobbie3'),
               'config',
               'urdf.rviz'
            ])

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
            'urdf_package_path': 'robot.urdf.xacro',
            'rviz_config': info
      }.items()
   )

   return LaunchDescription([
      data
   ])