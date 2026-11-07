import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonomous_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Configuración y mapa, para que el nodo los encuentre en share/autonomous_nav/
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png') + glob('maps/*.npy') + glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marc Costa',
    maintainer_email='marc.costa.brusco@gmail.com',
    description='Proyecto ROB: navegación autónoma del TurtleBot3 Burger',
    license='TODO',
    entry_points={
        'console_scripts': [
            'fase1_node = autonomous_nav.fase1_node:main',
            'prueba_rumbo = autonomous_nav.prueba_rumbo:main',
            'comprobar_pose = autonomous_nav.comprobar_pose:main',
            'scan_slam = autonomous_nav.scan_slam:main',
        ],
    },
)
