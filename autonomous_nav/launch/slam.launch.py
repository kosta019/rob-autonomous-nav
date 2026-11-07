# Lanza scan_slam (reenvía /scan a /scan_slam con el sello corregido) y slam_toolbox en modo online async,
# con los parámetros de config/slam.yaml.
# Uso: ros2 launch autonomous_nav slam.launch.py   (con una grabación o en Gazebo, añadir use_sim_time:=true)
# Para guardar el mapa: ros2 run nav2_map_server map_saver_cli -f mapa_demo
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav = get_package_share_directory('autonomous_nav')
    slam = get_package_share_directory('slam_toolbox')
    tiempo = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('stamp_offset', default_value='-0.10'),
        Node(package='autonomous_nav', executable='scan_slam', output='screen',
             parameters=[{'use_sim_time': tiempo, 'stamp_offset': LaunchConfiguration('stamp_offset')}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(nav, 'config', 'slam.yaml'),
                              'use_sim_time': tiempo}.items()),
    ])
