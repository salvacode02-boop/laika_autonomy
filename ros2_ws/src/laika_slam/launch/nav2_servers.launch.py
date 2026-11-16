import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Solo los servidores de Nav2 (sin localización ni puente): scripts/laika_fase4.sh los arranca con R2+X."""
    share = get_package_share_directory('laika_slam')
    cfg = lambda f: os.path.join(share, 'config', f)
    ctrl = [cfg('nav2_common.yaml'), LaunchConfiguration('controller_yaml')]
    bt = {'default_bt_xml_filename': os.path.join(share, 'behavior_trees', 'laika_navigate.xml')}
    nav = lambda pkg, exe, name, params: Node(
        package=pkg, executable=exe, name=name, output='screen', parameters=params)
    return LaunchDescription([
        DeclareLaunchArgument('controller_yaml', default_value=cfg('controller_dwb.yaml')),
        nav('nav2_planner', 'planner_server', 'planner_server', [cfg('nav2_common.yaml')]),
        nav('nav2_controller', 'controller_server', 'controller_server', ctrl),
        nav('nav2_recoveries', 'recoveries_server', 'recoveries_server', [cfg('nav2_common.yaml')]),
        nav('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', [cfg('nav2_common.yaml'), bt]),
        nav('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation',
            [{'autostart': True, 'node_names': [
                'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator']}]),
    ])
