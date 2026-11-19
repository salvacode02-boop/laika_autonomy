from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='laika_slam', executable='odom_to_tf', name='odom_to_tf', output='screen'),
        # Ángulos de las 12 articulaciones para el modelo del dashboard (la cinemática se calcula allá: no toca el árbol TF)
        Node(package='laika_slam', executable='joint_states', name='joint_states', output='screen'),
        # Fuente: config/l1_to_body_lio.yaml (cloud_R, cloud_t; calibración de 2026-11-19, ver decisiones.md).
        # Traslación = radar_joint del URDF go2_description. Rotación R_body_lidar como yaw/pitch/roll ZYX
        # (-1.018587, -0.126537, -2.916297), la que usa l1_to_body (el Ry 164.9 deg del URDF tenía ~120 deg de yaw de error).
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.28945', '0', '-0.046825', '-1.018587', '-0.126537', '-2.916297', 'base_link', 'utlidar_lidar']),
    ])
