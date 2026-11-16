import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('laika_slam')
    cfg = lambda f: os.path.join(share, 'config', f)
    armed = LaunchConfiguration('armed')
    ctrl = [cfg('nav2_common.yaml'), LaunchConfiguration('controller_yaml')]
    bt = {'default_bt_xml_filename': os.path.join(share, 'behavior_trees', 'laika_navigate.xml')}
    nav = lambda pkg, exe, name, params, **kw: Node(
        package=pkg, executable=exe, name=name, output='screen', parameters=params, **kw)
    bridge = lambda remap, **kw: Node(
        package='laika_slam', executable='cmd_vel_to_sport', name='cmd_vel_to_sport',
        output='screen', remappings=[('sport_request_out', remap)], **kw)
    return LaunchDescription([
        # armed:=false (defecto) -> topico de prueba; armed:=true -> /api/sport/request (solo con aprobacion del usuario)
        DeclareLaunchArgument('armed', default_value='false'),
        DeclareLaunchArgument('force_active', default_value='false',
                              description='solo dry-run: simula R2+X (se ignora si armed)'),
        DeclareLaunchArgument('controller_yaml', default_value=cfg('controller_dwb.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'localization.launch.py'))),
        nav('nav2_planner', 'planner_server', 'planner_server', [cfg('nav2_common.yaml')]),
        nav('nav2_controller', 'controller_server', 'controller_server', ctrl),
        nav('nav2_recoveries', 'recoveries_server', 'recoveries_server', [cfg('nav2_common.yaml')]),
        nav('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', [cfg('nav2_common.yaml'), bt]),
        nav('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation',
            [{'autostart': True, 'node_names': [
                'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator']}]),
        bridge('/laika/sport_request_dry', condition=UnlessCondition(armed),
               parameters=[{'dry_run_force_active': LaunchConfiguration('force_active')}]),
        bridge('/api/sport/request', condition=IfCondition(armed)),
    ])
