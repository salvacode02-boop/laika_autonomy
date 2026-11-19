import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('laika_imu_relay'), 'config', 'lowstate_imu_relay.yaml')
    return LaunchDescription([Node(package='laika_imu_relay', executable='lowstate_imu_relay',
                                   parameters=[cfg], output='screen')])
