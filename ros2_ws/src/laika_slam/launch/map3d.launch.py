from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    yaml = os.path.join(get_package_share_directory('laika_slam'), 'config', 'map3d.yaml')
    bag = LaunchConfiguration('bag')

    # Remap y no parámetros: en Foxy el yaml gana sobre un override desde el launch.
    def node(cond, remaps=()):
        # El PCD se guarda al cerrar: darle tiempo.
        return Node(package='laika_slam', executable='map3d_accumulator', name='map3d_accumulator',
                    output='screen', parameters=[yaml], remappings=list(remaps), condition=cond,
                    sigterm_timeout='30', sigkill_timeout='30')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='false'),
        node(UnlessCondition(bag)),
        node(IfCondition(bag), [('/utlidar/cloud_deskewed', '/bag/utlidar/cloud_deskewed'),
                                ('/tf', '/bag/tf')]),
    ])
