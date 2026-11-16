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
    return LaunchDescription([
        inc('tf.launch.py'),
        inc('scan.launch.py'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=[os.path.join(share, 'config', 'slam_async.yaml')]),
    ])
