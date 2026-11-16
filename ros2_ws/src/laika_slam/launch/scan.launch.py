import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('laika_slam'), 'config', 'pc2scan.yaml')
    return LaunchDescription([
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', output='screen', parameters=[cfg],
             remappings=[('cloud_in', '/laika/cloud_deskewed'), ('scan', '/scan')]),
    ])
