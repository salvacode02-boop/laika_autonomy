from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('laika_slam'), 'config')
    l1_yaml = os.path.join(cfg, 'l1_to_body.yaml')
    bag = LaunchConfiguration('bag')

    def l1(cond, cloud=None, imu=None):
        params = [l1_yaml]
        if cloud:
            params.append({'cloud_topic': cloud, 'imu_topic': imu})
        return Node(package='laika_slam', executable='l1_to_body', name='l1_to_body',
                    output='screen', parameters=params, condition=cond)

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='false'),
        l1(UnlessCondition(bag)),
        l1(IfCondition(bag), '/bag/utlidar/cloud', '/bag/utlidar/imu'),
        # El PCD se escribe al recibir SIGINT: darle tiempo para cerrar.
        Node(package='point_lio_unilidar', executable='pointlio_mapping', name='laserMapping',
             output='screen', parameters=[os.path.join(cfg, 'point_lio_utlidar.yaml')],
             sigterm_timeout='60', sigkill_timeout='60'),
    ])
