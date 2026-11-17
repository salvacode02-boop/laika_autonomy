import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('laika_slam')
    inc = lambda f: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', f)))
    cfg = os.path.join(share, 'config', 'amcl.yaml')
    return LaunchDescription([
        # Ruta (en el robot) del .yaml del mapa. Va aparte de amcl.yaml: en Foxy el yaml gana sobre los overrides de launch.
        DeclareLaunchArgument('map', default_value='/home/unitree/laika_maps/fase2_cuarto1.yaml'),
        inc('tf.launch.py'),
        inc('scan.launch.py'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[cfg]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ])
