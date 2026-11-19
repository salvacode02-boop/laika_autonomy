from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('laika_slam'), 'config')
    relay_yaml = os.path.join(get_package_share_directory('laika_imu_relay'), 'config', 'lowstate_imu_relay.yaml')
    l1_yaml = os.path.join(cfg, 'l1_to_body_lio.yaml')
    bag = LaunchConfiguration('bag')

    # Remap y no parámetros: en Foxy el yaml gana sobre un override desde el launch.
    def l1(cond, remaps=()):
        return Node(package='laika_slam', executable='l1_to_body', name='l1_to_body', output='screen',
                    parameters=[l1_yaml], remappings=list(remaps), condition=cond)

    def relay(cond, remaps=()):
        return Node(package='laika_imu_relay', executable='lowstate_imu_relay', output='screen',
                    parameters=[relay_yaml], remappings=list(remaps), condition=cond)

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='false'),
        l1(UnlessCondition(bag)),
        l1(IfCondition(bag), [('/utlidar/cloud', '/bag/utlidar/cloud'), ('/utlidar/imu', '/bag/utlidar/imu')]),
        relay(UnlessCondition(bag)),
        relay(IfCondition(bag), [('/lowstate', '/bag/lowstate')]),
        # Todo lo de FAST-LIO bajo /lio para no chocar con Nav2/SLAM (sin /tf, /Odometry, /path globales).
        Node(package='fast_lio', executable='fastlio_mapping', name='laser_mapping', output='screen',
             parameters=[os.path.join(cfg, 'fast_lio_l1.yaml')],
             remappings=[('/tf', '/lio/tf'), ('/Odometry', '/lio/odometry'),
                         ('/cloud_registered', '/lio/cloud_registered'),
                         ('/cloud_registered_body', '/lio/cloud_registered_body'),
                         ('/path', '/lio/path'), ('/map_save', '/lio/map_save')],
             sigterm_timeout='20', sigkill_timeout='20'),
    ])
