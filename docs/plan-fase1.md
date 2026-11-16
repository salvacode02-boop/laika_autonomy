# Plan de la Fase 1: mapa 2D con `slam_toolbox` a bordo

Fecha: 2026-11-16. Estado: propuesta, pendiente de aprobación del usuario.
Autor: `laika-advisor`.

## 0. Decisiones clave

| # | Decisión | Justificación |
|---|---|---|
| D1 | Usar `/utlidar/cloud_deskewed` (en `odom`) como entrada de `pointcloud_to_laserscan`. | Ya viene corregida por movimiento. Solo necesita `odom → base_link`, así que el extrínseco del LiDAR (inclinado ~165°) queda fuera de la ruta crítica del SLAM. Además, estar en `odom` permite acumular nubes si resultan escasas (C2). |
| D2 | `target_frame: base_link`; el `LaserScan` sale en `base_link`. | Es lo que pide `CLAUDE.md`. Para `slam_toolbox` el láser queda con offset identidad respecto a `base_frame`. |
| D3 | Un solo nodo propio, `odom_to_tf`, publica `odom → base_link` con el **stamp original** del mensaje (nunca `now()`). | Nube, odometría y TF quedan en el mismo reloj (el del robot), sin depender de la sincronización entre Jetsons. |
| D4 | `base_link → utlidar_lidar` se publica como TF estático, solo para ver la nube cruda y validar. | No lo usa el SLAM (ver D1); un error ahí no rompe el mapa. |
| D5 | Todo el pipeline corre en la Jetson del robot; la externa solo corre RViz y `map_saver_cli`. | Principio de diseño de `CLAUDE.md`. Guardar desde la externa evita instalar `nav2_map_server` en Foxy. |
| D6 | La externa es servidor `chrony`; la Jetson del robot es cliente. | La externa tiene salida a internet por WiFi; el robot no. |
| D7 | No se usa `robot_state_publisher` ni el URDF completo. | Exigiría `joint_states` y no aporta al mapa 2D. |

Fuera del plan por no ser necesario: Nav2, localización sobre mapa guardado, EKF/`robot_localization`, filtros de nube, arranque automático por systemd.

## 1. Arquitectura

| Nodo | Máquina | Suscribe | Publica |
|---|---|---|---|
| (fábrica) | Robot | n/a | `/utlidar/cloud_deskewed`, `/utlidar/cloud`, `/utlidar/robot_odom` |
| `odom_to_tf` (`laika_slam`) | Jetson robot | `/utlidar/robot_odom` | `/tf`: `odom → base_link` |
| `static_transform_publisher` | Jetson robot | n/a | `/tf_static`: `base_link → utlidar_lidar` |
| `pointcloud_to_laserscan_node` | Jetson robot | `/utlidar/cloud_deskewed` (remap de `cloud_in`), TF | `/scan` |
| `async_slam_toolbox_node` | Jetson robot | `/scan`, TF | `/map`, `/tf`: `map → odom` |
| `rviz2` | Externa | `/map`, `/scan`, `/tf`, `/tf_static`, nube (opcional) | n/a |
| `map_saver_cli` | Externa | `/map` | `maps/*.pgm` y `*.yaml` |

## 2. Árbol TF

`map → odom → base_link → utlidar_lidar`

| Transformación | Quién la publica | Valor | Fuente / certeza |
|---|---|---|---|
| `map → odom` | `slam_toolbox` | dinámico | n/a |
| `odom → base_link` | `odom_to_tf` | pose de `/utlidar/robot_odom`, stamp del mensaje | Tópico verificado en `CLAUDE.md` |
| `base_link → utlidar_lidar` | TF estático | xyz = `0.28945 0 -0.046825` m; rpy = `0 2.8782 0` rad (pitch ≈ 164.9°) | `radar_joint` (padre `base`, hijo `radar`) en `robots/go2_description/urdf/go2_description.urdf` de `unitreerobotics/unitree_ros`, rama `master`, consultado el 2026-11-16 |

Incertidumbres del TF estático, que se confirman en el paso 6:
- El URDF llama a los marcos `base` y `radar`. Se **supone** que equivalen a `base_link` y `utlidar_lidar`.
- En Foxy el orden posicional de `static_transform_publisher` es `x y z yaw pitch roll`. Los argumentos quedan `0.28945 0 -0.046825 0 2.8782 0 base_link utlidar_lidar`. Confundir pitch con roll o yaw es el error típico.

Reglas para no duplicar TF:
- Solo `odom_to_tf` publica `odom → base_link`, y solo se lanza desde el launch único del robot.
- En la externa no se lanza ningún publicador de TF.
- `odom_to_tf` valida que `header.frame_id == odom` y `child_frame_id == base_link`. Si no coinciden, registra un error y no publica.
- La prueba del paso 5 verifica el número de publicadores.

## 3. Parámetros iniciales

### 3.1 `pointcloud_to_laserscan`

Geometría que condiciona todo: el L1 mira hacia abajo, con su eje a ~15° de la vertical hacia el frente. Hacia los lados solo ve por debajo de su propia altura (z ≈ -0.05 en `base_link`). Hacia el frente ve hasta ~15° sobre la horizontal. Hacia atrás lo tapa el cuerpo. Esto se deduce del URDF y del campo de visión nominal del L1 (360° × 90°), no está medido.

| Parámetro | Valor | Justificación |
|---|---|---|
| `target_frame` | `base_link` | D2 |
| `min_height` | `-0.17` (provisional) | Regla: `z_piso + 0.15`. Se supone `z_piso ≈ -0.32`; se mide en el paso 4. |
| `max_height` | `0.50` | Aprovecha lo que el L1 ve al frente; hacia los lados no hay puntos por encima de ~-0.05. |
| `range_min` | `0.5` | Excluye patas y cuerpo; se mide desde `base_link`, no desde el LiDAR. |
| `range_max` | `10.0` | El L1 da ~20 m a baja reflectividad, pero la nube lejana es escasa; un cuarto no necesita más. |
| `angle_min` / `angle_max` | `-3.14159` / `3.14159` | 360°; el sector trasero quedará vacío. |
| `angle_increment` | `0.0087` (0.5°) | 720 haces. Subir a `0.0175` si hay pocos haces válidos. |
| `scan_time` | `0.0667` | ~15 Hz |
| `use_inf` | `true` | Haces vacíos como `inf`; `slam_toolbox` los ignora. |
| `transform_tolerance` | `0.05` | Margen para el TF de odometría. |

### 3.2 `slam_toolbox` (`async_slam_toolbox_node`)

Partir del `mapper_params_online_async.yaml` de la rama `foxy-devel` y cambiar solo esto:

| Parámetro | Valor | Justificación |
|---|---|---|
| `odom_frame` / `map_frame` / `base_frame` | `odom` / `map` / `base_link` | Árbol TF del proyecto (el default es `base_footprint`). |
| `scan_topic` | `/scan` | n/a |
| `mode` | `mapping` | n/a |
| `resolution` | `0.05` | Estándar; el ruido del L1 (~2 cm) no justifica más fino. |
| `max_laser_range` | `10.0` | Igual a `range_max`. |
| `minimum_travel_distance` | `0.2` | Cuarto pequeño y odometría de patas: nodos frecuentes para que el scan matching corrija antes. |
| `minimum_travel_heading` | `0.2` | Igual; los giros en sitio son lo que más deforma el mapa. |
| `minimum_time_interval` | `0.2` | Coherente con 15 Hz. |
| `transform_timeout` | `0.2` | Default. Subir a `0.5` solo si hay errores de extrapolación en el robot. |
| `tf_buffer_duration` | `30.0` | Default. |
| `transform_publish_period` | `0.02` | Default; `map → odom` a 50 Hz. |
| `map_update_interval` | `2.0` | Respuesta más rápida en RViz; la Orin NX lo soporta. |
| `throttle_scans` | `1` | En modo async ya se descartan scans si no da tiempo. |
| `do_loop_closing` | `true` | Necesario para evitar paredes dobles al volver al inicio. |
| `loop_search_maximum_distance` | `3.0` | Default, adecuado a un cuarto. |
| `use_scan_matching` / `use_scan_barycenter` | `true` / `true` | Default. |
| `enable_interactive_mode` | `false` | No se usa el plugin de RViz. |
| `debug_logging` | `false` | n/a |

Solver (Ceres) y demás parámetros: sin cambios.

### 3.3 RViz (externa)

| Display | Tópico | QoS |
|---|---|---|
| Fixed Frame | `map` | n/a |
| Map | `/map` | Reliable + Transient Local |
| LaserScan | `/scan` | Best Effort |
| TF | n/a | n/a |
| PointCloud2 (apagado por defecto) | `/utlidar/cloud_deskewed` | Best Effort |

## 4. Pasos de implementación

Convenciones: "R" es la Jetson del robot (por SSH, siempre con `source ~/laika_ros_env.sh`), "E" es la externa. **[APROBACIÓN]** indica que hay que preguntar al usuario antes de ejecutar.

| # | Máq. | Qué se hace | Archivos en el repo | Prueba y resultado esperado |
|---|---|---|---|---|
| 1 | E | Diagnosticar el timeout de SSH. No seguir hasta resolverlo. | n/a | `nc -zv -w 3 192.168.123.18 22` conecta; `ssh -o ConnectTimeout=5 unitree@192.168.123.18 hostname` devuelve el nombre. Si falla: `ip neigh show 192.168.123.18` (descarta conflicto de IP) y `ssh -v`. Pedir al usuario que confirme que la Jetson del robot terminó de arrancar. |
| 2 | E | Script de entorno de la externa. | `scripts/env_external.sh` | `source scripts/env_external.sh && ros2 topic list` muestra `/utlidar/cloud_deskewed` y `/utlidar/robot_odom`. |
| 3 | R | Crear `~/laika_ros_env.sh` con el contenido exacto de `CLAUDE.md`. No se toca `.bashrc`. | `scripts/laika_ros_env.sh` (copia versionada) | `ssh unitree@192.168.123.18 'source ~/laika_ros_env.sh && ros2 topic list'` lista los tópicos `/utlidar/*`. |
| 4 | R | Mediciones previas, solo lectura. Registrar los resultados en `docs/decisiones.md`. | `docs/decisiones.md` | Ver la tabla 4.1. |
| 5 | E→R | Paquete `laika_slam` (ament_python) con el nodo `odom_to_tf` y `launch/tf.launch.py` (nodo más TF estático). Copiar con `rsync` a `~/laika_ws/src`; `colcon build --packages-select laika_slam`. | `ros2_ws/src/laika_slam/` (`package.xml`, `setup.py`, `laika_slam/odom_to_tf.py`, `launch/tf.launch.py`) | En tmux: `ros2 launch laika_slam tf.launch.py`. (a) `ros2 run tf2_ros tf2_echo odom base_link` da valores que cambian al mover el robot a mano. (b) `ros2 topic info /tf -v` muestra exactamente 1 publicador. (c) `ros2 run tf2_tools view_frames.py` muestra `odom → base_link → utlidar_lidar`, sin ramas sueltas. (d) `top`: `odom_to_tf` usa menos de 20 % de un núcleo. |
| 6 | E | Validar el TF estático. RViz con Fixed Frame `odom` y dos displays: `/utlidar/cloud` y `/utlidar/cloud_deskewed`. Robot quieto y de pie. | n/a | Las dos nubes coinciden (piso plano a la misma altura, paredes superpuestas, error menor a ~5 cm). Si no coinciden, parar y pedir diagnóstico: equivalencia de marcos u orden de ángulos. No bloquea el SLAM (D4). |
| 7 | R | **[APROBACIÓN: sudo/apt]** Instalar `ros-foxy-pointcloud-to-laserscan` y `ros-foxy-slam-toolbox`. Antes, sin sudo: `apt-cache policy` de ambos. Si no hay candidato, pasar al Plan B (sección 5). | n/a | `ros2 pkg prefix slam_toolbox` y `ros2 pkg prefix pointcloud_to_laserscan` devuelven una ruta. |
| 8 | E→R | Config y launch de `pointcloud_to_laserscan` (parámetros de 3.1, con `min_height` ajustado según el paso 4; remap `cloud_in` → `/utlidar/cloud_deskewed`). | `laika_slam/config/pc2scan.yaml`, `launch/scan.launch.py` | `ros2 topic hz /scan` da ~15 Hz. `ros2 topic echo /scan --once` muestra `frame_id: base_link`. Haces finitos: al menos 150 de 720 con el robot quieto en un cuarto. En RViz (E) el scan dibuja las paredes y no el piso. |
| 9 | E→R | Config y launch de `slam_toolbox` (3.2), más un launch único `mapping.launch.py` que incluye TF, scan y SLAM. | `laika_slam/config/slam_async.yaml`, `launch/mapping.launch.py` | En tmux: `ros2 launch laika_slam mapping.launch.py`. `ros2 topic echo /map --once --field info` da resolución 0.05. `tf2_echo map odom` responde. `view_frames.py` muestra el árbol completo. Log sin "Message Filter dropping message" continuo. |
| 10 | E | **[APROBACIÓN: sudo en la externa]** `apt install ros-humble-nav2-map-server`. | n/a | `ros2 pkg executables nav2_map_server` lista `map_saver_cli`. |
| 11 | E | Config de RViz (3.3) y script de arranque. | `rviz/fase1.rviz`, `scripts/rviz_fase1.sh` | Con el robot quieto, RViz muestra el mapa inicial, el scan y el TF sin errores en ningún display. |
| 12 | E, R | **[APROBACIÓN: sudo, apt y systemd en ambas]** Sincronizar relojes. En E: instalar `chrony` (reemplaza a `systemd-timesyncd`) y añadir `allow 192.168.123.0/24` y `local stratum 10`. En R: revisar primero `timedatectl` y luego instalar `chrony` con `server 192.168.123.50 iburst`. Si apt falla en R, usar `NTP=192.168.123.50` en `/etc/systemd/timesyncd.conf`. Hacerlo con el pipeline detenido. | `docs/chrony.md` (líneas añadidas en cada máquina) | En R: `chronyc tracking` muestra referencia `192.168.123.50` y "System time" menor a 5 ms. En E: `chronyc clients` lista `192.168.123.18`. |
| 13 | E, R | **[APROBACIÓN: el usuario camina el robot]** Prueba corta: 2 m en línea recta, giro de 360° en el sitio y regreso. | n/a | Paredes simples, sin arcos fantasma frente al robot, y el mapa no gira con el robot. Si falla, ver la sección 7. |
| 14 | E, R | **[APROBACIÓN: el usuario camina el robot]** Prueba final (sección 8). | `maps/cuarto1.pgm`, `maps/cuarto1.yaml` | Sección 8. |

El paso 12 puede adelantarse a cualquier momento después del 3; por D3 no bloquea el SLAM.

### 4.1 Mediciones del paso 4

| Dato | Comando (en R) | Para qué sirve |
|---|---|---|
| Frecuencia de la odometría | `ros2 topic hz /utlidar/robot_odom` | Si es menor a 30 Hz, subir `transform_tolerance`. Si es muy alta, vigilar la CPU de `odom_to_tf`. |
| QoS de los publicadores | `ros2 topic info -v /utlidar/robot_odom` y lo mismo para `/utlidar/cloud_deskewed` | `odom_to_tf` debe suscribirse con QoS de sensor (best effort). |
| Marcos de la odometría | `ros2 topic echo /utlidar/robot_odom --once` (ver `header.frame_id` y `child_frame_id`) | Confirmar `odom` y `base_link`. |
| Reloj de los stamps | Comparar `header.stamp` de `/utlidar/cloud_deskewed` y de `/utlidar/robot_odom` con `date +%s.%N` | Lo crítico es que **ambos tópicos estén en el mismo reloj** (diferencia menor a 0.1 s entre sí). Anotar el offset respecto al reloj de la Jetson. |
| Densidad de la nube | `ros2 topic echo /utlidar/cloud_deskewed --once --field width` y `ros2 topic hz` | Si hay menos de ~2000 puntos por mensaje, es probable que haga falta C2. |
| Altura del piso | `tf2_echo odom base_link` con el robot de pie y quieto; anotar z. Se confirma en el paso 6 en RViz (Fixed Frame `base_link`, color por eje Z). | Fijar `min_height = z_piso + 0.15`. |

## 5. Plan B: compilar desde código en el robot

Aplica si el paso 7 no encuentra candidatos en apt.

| Tema | Acción |
|---|---|
| Primero, recuperar apt | Si `apt update` falla por llave expirada de ROS (`EXPKEYSIG`), renovar la llave. Si Foxy ya no está en `packages.ros.org`, usar `snapshots.ros.org` (snapshot final de Foxy). Ambas cosas son **[APROBACIÓN: sudo]**. No verificado el estado actual de esos repositorios. |
| `pointcloud_to_laserscan` | `github.com/ros-perception/pointcloud_to_laserscan`, rama `foxy` (existencia verificada el 2026-11-16). Clonar en `~/laika_ws/src`. |
| `slam_toolbox` | `github.com/SteveMacenski/slam_toolbox`, rama `foxy-devel` (existencia verificada el 2026-11-16). |
| Dependencias | `rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy` **[APROBACIÓN: sudo]**. Las principales son `libceres-dev`, `libsuitesparse-dev`, `libtbb-dev`, `libboost-all-dev`, `qtbase5-dev` y los paquetes `ros-foxy-rviz-*` (los pide el plugin de RViz de `slam_toolbox`). |
| Compilación | `colcon build --packages-up-to slam_toolbox pointcloud_to_laserscan --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2`. Sin `Release`, el scan matching es demasiado lento. |
| Prueba | La misma del paso 7. |
| No tocar | `unitree_go`, `unitree_api` y `laika_basics` en `~/laika_ws`. |

## 6. Pasos que requieren aprobación

| Paso | Motivo |
|---|---|
| 7 (y Plan B) | `sudo apt` en el robot; posible cambio de llaves o repositorios de apt. |
| 10 | `sudo apt` en la externa. No lo exige `CLAUDE.md`, pero conviene avisar. |
| 12 | `sudo`, instalación y servicio systemd de hora en ambas máquinas; salto del reloj del robot. |
| 13, 14 | El usuario debe caminar el robot con el control. |
| Nunca en esta fase | Publicar en `/api/sport/request` o `/lowcmd`, tocar `laika-flip.service`, editar `~/.bashrc` del robot, modificar `/unitree`, reiniciar el robot. |

## 7. Riesgos y contingencias

| # | Riesgo | Cómo se detecta | Corrección |
|---|---|---|---|
| R1 | El piso entra en el scan cuando el cuerpo cabecea: 2° de pitch elevan el piso 0.15 m a ~4 m. | Arcos o "paredes" fantasma frente al robot al caminar (paso 13). | **C1:** `odom_to_tf` publica además una hoja `base_link → base_stabilized` (roll y pitch inversos); `target_frame` y `base_frame` pasan a `base_stabilized`. Paliativo: subir `min_height` o bajar `range_max` a 6 m. |
| R2 | Nube demasiado escasa para el scan matching (el L1 da ~21 600 puntos/s). | Menos de 150 haces válidos (paso 8); el mapa "resbala" en los giros. | `angle_increment: 0.0175`. Si no alcanza, **C2:** nodo que acumula 3 a 5 nubes `cloud_deskewed` (ya están en `odom`) antes de convertir. |
| R3 | Los stamps de la nube y de la odometría vienen de relojes distintos. | Medición del paso 4; "Message Filter dropping message" o "extrapolation" en el robot. | Pedir diagnóstico. La opción probable es re-estampar ambos al recibirlos en R, con pérdida de precisión. |
| R4 | El `odom` de `cloud_deskewed` no es el mismo que el de `/utlidar/robot_odom`. | En RViz con Fixed Frame `base_link`, la nube se desplaza o gira respecto al robot al caminar. | Cambiar a `/utlidar/cloud` con el TF estático (requiere pasar el paso 6). |
| R5 | TF estático incorrecto. | Paso 6. | No bloquea; se corrige antes de la fase 2. |
| R6 | TF duplicado. | `ros2 topic info /tf -v` con más publicadores de los esperados; el robot "salta" en RViz. | Matar el proceso sobrante; confirmar que el launch no quedó corriendo dos veces en tmux. |
| R7 | QoS incompatible. | RViz sin datos aunque `ros2 topic hz` funciona; `ros2 topic info -v` muestra Reliable frente a Best Effort. | Ajustar el display de RViz (3.3). |
| R8 | DDS de la externa sale por WiFi. | `ros2 topic list` vacío en E. | Usar siempre `scripts/env_external.sh` y reiniciar `ros2 daemon`. |
| R9 | Foxy ↔ Humble con tipos no estándar. | Advertencias de tipo desconocido en E. | Ignorar; no hacer `echo` de tópicos `unitree_*` desde E. |
| R10 | Deriva de la odometría de patas en giros. | Paredes dobles al cerrar el recorrido. | Caminar más lento, cerrar el lazo y bajar `minimum_travel_heading` a `0.1`. |
| R11 | Sector trasero ciego (el cuerpo tapa el LiDAR). | Mapa incompleto detrás del robot. | Es esperado: el recorrido debe incluir giros para cubrir todo. |
| R12 | Carga de CPU en la Jetson del robot. | `top`; mensajes de cola llena en `slam_toolbox`. | `throttle_scans: 2`; `map_update_interval: 5.0`. |

## 8. Prueba final

1. En R (tmux): `ros2 launch laika_slam mapping.launch.py`. En E: `scripts/rviz_fase1.sh`.
2. **[APROBACIÓN]** El usuario camina el robot despacio por el perímetro del cuarto, con un giro completo en cada esquina, y vuelve al punto inicial repasando ~2 m del tramo de salida para cerrar el lazo.
3. Criterio visual en RViz: cada pared aparece una sola vez (grosor de 2 celdas o menos, es decir ≤ 10 cm), las esquinas quedan a ~90° y el tramo repetido se superpone al original.
4. En E: `ros2 run nav2_map_server map_saver_cli -f maps/cuarto1`.
5. Resultado esperado: existen `maps/cuarto1.pgm` y `maps/cuarto1.yaml`; el `.yaml` dice `resolution: 0.05`; la imagen abierta coincide con lo visto en RViz.
6. Opcional, en R: `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/unitree/laika_maps/cuarto1'}"` guarda el grafo para reutilizarlo.
7. Commit de los mapas y entrada en `docs/decisiones.md` con los valores finales de `min_height`, `angle_increment` y cualquier contingencia aplicada.

---

**No verificado** (sin acceso al robot al redactar; se confirma en los pasos 4, 6 y 7):
- La disponibilidad de los paquetes Foxy en apt.
- La altura real del piso en `base_link`.
- La densidad de `cloud_deskewed`.
- El reloj de origen de los stamps.
- La equivalencia de `radar` con `utlidar_lidar`.

**Riesgo principal.** El L1 mira hacia abajo, así que el margen entre piso y pared es estrecho. Es probable que haga falta la contingencia C1 (marco estabilizado) tras la primera caminata; quedó fuera del diseño base para respetar el `target_frame: base_link` de `CLAUDE.md`.

## Fuentes

- [go2_description.urdf (unitreerobotics/unitree_ros)](https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/go2_description/urdf/go2_description.urdf)
- [slam_toolbox, ramas](https://github.com/SteveMacenski/slam_toolbox/branches/all)
- [pointcloud_to_laserscan, ramas](https://github.com/ros-perception/pointcloud_to_laserscan/branches/all)
