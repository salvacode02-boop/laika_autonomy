import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('laika_slam')
    inc = lambda f: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', f)))
    cfg = os.path.join(share, 'config', 'amcl.yaml')
    return LaunchDescription([
        inc('tf.launch.py'),
        inc('scan.launch.py'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[cfg]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[cfg]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ])
