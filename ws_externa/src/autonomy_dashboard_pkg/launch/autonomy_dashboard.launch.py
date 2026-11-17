#!/usr/bin/env python3
"""
Launch del Autonomy Dashboard (C++ / Qt5 / OpenGL).

  ros2 launch autonomy_dashboard_pkg autonomy_dashboard.launch.py
  ros2 launch autonomy_dashboard_pkg autonomy_dashboard.launch.py team_name:=MiEquipo mission:=Prelim2

Si no se pasan team_name / mission se usan los de config/autonomy_params.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    params = [os.path.join(get_package_share_directory('autonomy_dashboard_pkg'),
                           'config', 'autonomy_params.yaml')]

    # Solo sobrescribe el YAML si se pasó el argumento
    overrides = {}
    for name in ('team_name', 'mission'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    if overrides:
        params.append(overrides)

    return [Node(
        package='autonomy_dashboard_pkg',
        executable='autonomy_dashboard',
        name='autonomy_dashboard_node',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team_name', default_value='', description='Nombre del equipo'),
        DeclareLaunchArgument('mission', default_value='', description='Nombre de la misión'),
        OpaqueFunction(function=launch_setup),
    ])
