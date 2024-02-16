import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'proyecto_myrobbie3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.dae')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose heriberto',
    maintainer_email='jose.heriberto@hotmail.es',
    description='este paquete es un proyecto donde se ejecuta una aplicacion de ros2 de robotica movil autonoma',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_main_myrobbie3 = proyecto_myrobbie3.nodo_main_myrobbie3:main'
        ],
    },
)
