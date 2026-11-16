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
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'nav2_servers.launch.py')),
            launch_arguments={'controller_yaml': LaunchConfiguration('controller_yaml')}.items()),
        bridge('/laika/sport_request_dry', condition=UnlessCondition(armed),
               parameters=[{'dry_run_force_active': LaunchConfiguration('force_active')}]),
        bridge('/api/sport/request', condition=IfCondition(armed)),
    ])
