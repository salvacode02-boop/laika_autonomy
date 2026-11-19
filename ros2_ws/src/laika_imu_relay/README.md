# laika_imu_relay
`lowstate_imu_relay`: suscribe `/lowstate` (unitree_go/LowState, best effort, depth 10) y publica
`sensor_msgs/Imu` en `/laika/body_imu` (frame `base_link`, stamp = now - `latency`, ~500 Hz).
Orientacion, giroscopio y acelerometro de `imu_state` (IMU del cuerpo). No toca `/lf/lowstate`.
Lanzar: `ros2 launch laika_imu_relay lowstate_imu_relay.launch.py` (en el robot, Foxy).
Parametros en `config/lowstate_imu_relay.yaml`. `latency` = 0.0 (+-10 ms, de movimiento lento; re-medir con bag rapido).
