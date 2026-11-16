# laika_slam

Fase 1. Por ahora: TF del robot (el robot no publica TF).

- `odom_to_tf`: `/utlidar/robot_odom` -> `/tf` `odom -> base_link`, con el stamp original del mensaje.
- `tf.launch.py`: `odom_to_tf` + estático `base_link -> utlidar_lidar` (`radar_joint` del URDF de go2_description).

Lanzar (Foxy, robot): `ros2 launch laika_slam tf.launch.py`
Tópicos: sub `/utlidar/robot_odom`; pub `/tf`, `/tf_static`.
