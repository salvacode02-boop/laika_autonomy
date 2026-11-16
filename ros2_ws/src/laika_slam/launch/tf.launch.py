from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='laika_slam', executable='odom_to_tf', name='odom_to_tf', output='screen'),
        # Fuente: radar_joint de unitree_ros/robots/go2_description/urdf/go2_description.urdf
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.28945', '0', '-0.046825', '0', '2.8782', '0', 'base_link', 'utlidar_lidar']),
    ])
