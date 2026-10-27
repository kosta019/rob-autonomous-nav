# Gazebo con el mundo del laboratorio (worlds/laboratorio.sdf) y el TurtleBot3 Burger.
# El origen de Gazebo es el del enunciado, así que la pose real del robot sale en coordenadas del enunciado.
#
# Uso:  ros2 launch autonomous_nav laboratorio.launch.py [gui:=false] [x:=.. y:=.. yaw:=..]
# Sin x/y/yaw, el robot aparece en initial_pose de config/fase1.yaml (la misma que usa fase1_node).
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav = get_package_share_directory('autonomous_nav')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz = get_package_share_directory('ros_gz_sim')
    modelo = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    with open(os.path.join(nav, 'config', 'fase1.yaml')) as f:
        x0, y0, yaw0 = yaml.safe_load(f)['fase1_node']['ros__parameters']['initial_pose']
    mundo = os.path.join(nav, 'worlds', 'laboratorio.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value=str(x0)),
        DeclareLaunchArgument('y', default_value=str(y0)),
        DeclareLaunchArgument('yaw', default_value=str(yaw0)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),

        # Servidor de Gazebo y, si gui:=true, la ventana
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s -v2 ', mundo], 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 '}.items(),
            condition=IfCondition(LaunchConfiguration('gui'))),

        # El robot, en la pose inicial
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', modelo,
                        '-file', os.path.join(tb3, 'models', 'turtlebot3_' + modelo, 'model.sdf'),
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.01',
                        '-Y', LaunchConfiguration('yaw')]),
        # Puente Gazebo <-> ROS (/cmd_vel, /odom, /scan, /clock...): el mismo de turtlebot3_gazebo
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p',
                        'config_file:=' + os.path.join(tb3, 'params', 'turtlebot3_' + modelo + '_bridge.yaml')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ])
