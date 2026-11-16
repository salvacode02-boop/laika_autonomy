# Laika: autonomía del Unitree Go2 EDU

Este archivo es el contexto del proyecto para Claude Code. Léelo completo antes de hacer cualquier cosa.
Todo lo que dice "verificado" fue comprobado en el robot real el 2026-11-15.

## Objetivo

Construir la autonomía del robot por fases. **La fase actual es la 4** (autonomía por cable, plan en `docs/plan-fase4.md`). Las fases 1 y 2 están terminadas; la 3 está pospuesta (el WiFi del Go2 no reenvía y no hay router con alimentación).

| Fase | Meta | Criterio de "terminado" |
|---|---|---|
| **1** | Mapa **2D** con `slam_toolbox`, visible en RViz en la Jetson externa, por **Ethernet** | El usuario camina el robot con el control por un cuarto; el mapa en RViz queda consistente (sin paredes dobles) y se guarda en archivo (`.pgm` + `.yaml`) |
| 2 | Mapa **3D** con Point-LIO adaptado por Unitree para el L1 (`point_lio_unilidar`, buscar port a ROS 2) | Nube 3D consistente en RViz y guardada en `.pcd` |
| 3 | Pasar la visualización a **WiFi** (router montado en el perro) | Lo mismo que fases 1 y 2, desde la red WiFi (pospuesta) |
| **4** | **Autonomía por cable**: Nav2 + AMCL sobre `maps/fase2_cuarto1`, ir a una pose dada en RViz | 3 metas seguidas a ≤ 0.25 m y ≤ 15°, una con obstáculo, sin contacto, y una parada probada en plena marcha |

Fuera de alcance por ahora: operación sin supervisión, exploración, waypoints, obstáculos dinámicos, marcha atrás.

## Hardware y red

- **Robot:** Unitree Go2 EDU. LiDAR **L1** integrado en la cabeza, con IMU interna. La computadora principal (`192.168.123.161`) es cerrada: no se toca.
- **Jetson del robot** (Orin NX 16 GB): `192.168.123.18`, usuario `unitree`, Ubuntu 20.04, JetPack 5.1.1, **ROS 2 Foxy**, interfaz `eth0`.
- **Jetson externa** (aquí corre Claude Code): Ubuntu 22.04, **ROS 2 Humble**. Interfaz hacia el robot: `enP8p1s0` = `192.168.123.50`. También tiene WiFi (`wlP1p1s0`) en otra red: DDS **debe** fijarse a `enP8p1s0`.
- **Acceso al robot:** `ssh unitree@192.168.123.18` con **llave SSH**. Nunca escribas contraseñas en archivos ni en el repo.

## Entorno ROS 2 / DDS (crítico, aquí falla casi todo)

Reglas comprobadas:
- **Dominio 0** en todas las máquinas (`ROS_DOMAIN_ID=0`).
- **CycloneDDS** en todas (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`). El robot usa CycloneDDS 0.10.2 compilado en `~/cyclonedds_ws`. El `rmw_cyclonedds_cpp` de Humble es compatible.
- **Foxy (robot) ↔ Humble (externa):** los mensajes estándar (`PointCloud2`, `Odometry`, `OccupancyGrid`, `TF`, `LaserScan`) se comunican bien. Los mensajes propios de Unitree (`unitree_go`, `unitree_api`) solo existen compilados en el robot.

### En la Jetson del robot
Al entrar por SSH interactivo, el `.bashrc` pregunta `ros:foxy(1) noetic(2)?`. **En comandos SSH no interactivos esa pregunta no aparece y ROS no se carga.** Crea (una vez) `~/laika_ros_env.sh` en el robot y úsalo siempre:

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/laika_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
export ROS_DOMAIN_ID=0
```

Ejemplo: `ssh unitree@192.168.123.18 'source ~/laika_ros_env.sh && ros2 topic list'`.
Para procesos largos (SLAM) usa `tmux` en el robot.

### En la Jetson externa
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP8p1s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```
Guarda esto como script del repo (por ejemplo `scripts/env_external.sh`).

### SDK de Python de Unitree
No se usa en esta fase. Si algún día se usa: **nunca** en el mismo proceso ni terminal que ROS 2 (choque de DDS → segfault comprobado).

## Datos que publica el robot (verificado)

| Tópico | Tipo | Marco (`frame_id`) | Notas |
|---|---|---|---|
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | `utlidar_lidar` | Nube cruda del L1, ~15 Hz |
| `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | `odom` | Ya corregida por movimiento y expresada en `odom` |
| `/utlidar/imu` | `sensor_msgs/Imu` | (revisar) | IMU interna del L1 |
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | `odom` → `base_link` | Odometría del robot |
| `/sportmodestate` | `unitree_go/SportModeState` | n/a | Estado de alto nivel |
| `/tf`, `/tf_static` | n/a | n/a | **El robot NO publica TF.** Hay que construir el árbol |

RViz en la Jetson externa ya muestra `/utlidar/cloud_deskewed` con Fixed Frame `odom`. El video de la cámara se recibe por GStreamer (multicast `230.1.1.1:1720`), no es parte de esta fase.

### Software de fábrica: no depender de él
- `/unitree/module/graph_pid_ws` (`QT_Server` + `lio_sam_ros2`) está configurado para **LiDARs externos** (Hesai / Livox MID-360: tópicos `rslidar_points`, `livox/lidar`, `dog_imu_raw`). **No sirve para el L1**, y sus extrínsecos tampoco aplican.
- Los tópicos `/uslam/*` existen pero están inactivos.
- **`/unitree` es de solo lectura.** Nunca modifiques nada ahí.

## Lo que hay que construir en la fase 1

1. **Árbol TF:** `map → odom → base_link → utlidar_lidar`
   - `odom → base_link`: nodo que republica `/utlidar/robot_odom` como TF.
   - `base_link → utlidar_lidar`: transformación fija. Sacar los valores del modelo oficial del Go2 (URDF, por ejemplo `go2_description`) y **documentar la fuente**.
   - `map → odom`: lo publica `slam_toolbox`.
2. **`pointcloud_to_laserscan`:** convertir la nube a `LaserScan`. Decidir y justificar si usar `cloud_deskewed` (ya en `odom`) o `cloud` (en `utlidar_lidar`), con `target_frame: base_link` y una banda de altura adecuada para un cuadrúpedo.
3. **`slam_toolbox`** (modo online async), corriendo **en la Jetson del robot**. Verificar si `ros-foxy-slam-toolbox` y `ros-foxy-pointcloud-to-laserscan` se pueden instalar por `apt` (Foxy ya no tiene soporte oficial); si no, compilar desde código.
4. **Configuración de RViz** guardada en el repo, para la Jetson externa.
5. **Guardar el mapa** (`nav2_map_server` / `map_saver_cli`).
6. **Hora sincronizada** entre las dos Jetson (por ejemplo con `chrony`). TF falla con errores de "extrapolation" si los relojes difieren.

Principio de diseño: **el SLAM corre a bordo** (Jetson del robot). La Jetson externa solo visualiza. Así, al pasar a WiFi solo cambia la visualización.

## Repositorio y despliegue

- Trabaja en un repositorio **git** en la Jetson externa. Commits pequeños y descriptivos.
- Los paquetes que corren en el robot se copian a `~/laika_ws/src` del robot (por ejemplo con `rsync`) y se compilan **ahí** con `colcon build` (Foxy).
- El `~/laika_ws` del robot ya contiene `unitree_go`, `unitree_api` y `laika_basics`: **no los borres**.
- El robot tiene `/etc/apt/sources.list` corregido (mirror chino reemplazado por `ports.ubuntu.com`).

## Reglas de seguridad (obligatorias)

- **Nunca** publiques en `/lowcmd` ni uses `SportClient`. En `/api/sport/request` solo publica el nodo `cmd_vel_to_sport` (IDs 1008 Move y 1003 StopMove), y solo arrancado con `armed:=true` en una prueba que el usuario aprobó explícitamente. Cada prueba con publicación real o en la que el robot camine solo necesita su propia aprobación del usuario.
- `laika-flip.service` (systemd) y `~/laika_ws/flip*_trigger.py` disparan saltos con combinaciones del control. El 2026-11-16 el usuario autorizó: dejar en `flip3_trigger.py` solo L1+Y (BackFlip) y deshabilitar el proceso por ahora. Fuera de eso, **no los modifiques** sin preguntar.

### Consideraciones del usuario para la fase 4 (2026-11-16)
- El usuario prueba **solo**: el control remoto es su parada en mano.
- **R2+X** activa el modo autónomo desde el control (combinación sin otro uso).
- **L2+Y** detiene el movimiento autónomo: manda StopMove y desactiva la autonomía, **sin apagar ni desplomar al robot** (nunca Damp). Siempre detiene; nunca arranca.
- Por ahora, cada ruta nueva se muestra en RViz (ruta planeada y velocidad de corrección) y la terminal pide **confirmar con Enter** antes de ejecutarla. Es temporal, para medir reacción y precisión del planeador; después se quita.
- Primeras pruebas con trayectorias cortas (1 m, 2 m), con el botón de paro activo.
- Pregunta antes de: instalar paquetes con `sudo` en el robot, editar `~/.bashrc` del robot, reiniciar el robot, o tocar servicios systemd.
- Si una prueba requiere que el robot camine, **pídeselo al usuario** y espera su confirmación.

## Flujo de trabajo multiagente

Hay dos subagentes en `.claude/agents/`:

- **`laika-advisor`** (modelo Fable): piensa. Diseña, decide valores y diagnostica fallas. **No escribe código.**
- **`laika-implementer`** (modelo Sonnet): construye. Escribe paquetes, launch files, configs y scripts siguiendo el plan aprobado.

La sesión principal coordina:

1. Pide a `laika-advisor` el plan de la fase 1 → lo guarda en `docs/plan-fase1.md`.
2. **Muestra el plan al usuario y espera su aprobación.**
3. Pasa cada paso del plan a `laika-implementer`.
4. Ejecuta las pruebas (en el robot por SSH, RViz en la externa) y reporta.
5. Si algo falla: resume el síntoma (comando, error, lo que se esperaba) y pásaselo a `laika-advisor` para diagnóstico. **No le pases logs completos**: Fable es el modelo más caro, úsalo para decisiones, no para leer texto largo.

Registra decisiones importantes en `docs/decisiones.md` (qué se decidió, por qué, fecha).
