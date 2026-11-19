# laika_slam

Fase 1. Por ahora: TF del robot (el robot no publica TF).

- `odom_to_tf`: `/utlidar/robot_odom` -> `/tf` `odom -> base_link`, con el stamp original del mensaje.
- `tf.launch.py`: `odom_to_tf` + estático `base_link -> utlidar_lidar` (`radar_joint` del URDF de go2_description).

Lanzar (Foxy, robot): `ros2 launch laika_slam tf.launch.py`
Tópicos: sub `/utlidar/robot_odom`; pub `/tf`, `/tf_static`.

- `joint_states`: `/lf/lowstate` (unitree_go/LowState) -> `/joint_states` (sensor_msgs/JointState, 20 Hz) con los 12 nombres del URDF en el orden de `motor_state` (FR, FL, RR, RL; hip, thigh, calf), stamp `now()`. Lo lanza `tf.launch.py`. Lo usa el modelo del dashboard (la cinemática se calcula allá).
- `localization.launch.py` acepta `map:=<ruta .yaml en el robot>` (por defecto `/home/unitree/laika_maps/fase2_cuarto1.yaml`); el `yaml_filename` ya no está en `amcl.yaml`.
- `tf.launch.py`: el estático `base_link -> utlidar_lidar` usa `cloud_R`/`cloud_t` de `config/l1_to_body_lio.yaml` (2026-11-19; el del URDF tenía ~120 deg de yaw de error). Solo lo usa RViz.
- `lio_compare` (modo sombra, no publica): compara la pose 2D de `/lio/odometry` (base_link = p + R*T) con TF `map -> base_link`; log de dxy/dyaw cada 5 s y CSV `~/laika_bags/lio_compare_<fecha>.csv`. Se lanza con `fast_lio.launch.py compare:=true` (o `scripts/laika_fase4.sh --slam --lio`).
