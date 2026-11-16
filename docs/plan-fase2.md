# Plan de la Fase 2: mapa 3D con Point-LIO a bordo (L1 del Go2)

Fecha: 2026-11-16. Estado: propuesta, pendiente de aprobación del usuario. Autor: `laika-advisor`.

Convenciones: "R" es la Jetson del robot (SSH, siempre con `source ~/laika_ros_env.sh`), "E" es la externa. **[APROBACIÓN]** indica que hay que preguntar al usuario antes de ejecutar.

## 0. Decisiones clave

| # | Decisión | Justificación |
|---|---|---|
| F1 | Usar el Point-LIO de `jizhang-cmu/autonomy_stack_go2`, rama `foxy-humble`, y tomar **solo** el paquete `src/slam/point_lio_unilidar`. | Es el único port hallado hecho para este caso exacto: Go2 EDU, L1 integrado, `/utlidar/cloud` + `/utlidar/imu`, Jetson a bordo con Foxy en Ubuntu 20.04. |
| F2 | No usar `dfloreaa/point_lio_ros2` (rama `main`) como base. | Solo está probado en Humble; el issue #4 (¿Foxy?) no tiene respuesta. Los issues #4 y #15 reportan deriva sin resolver con el L1 en Go2 EDU usando la configuración estándar. Queda como plan B de diagnóstico en E. |
| F3 | Configuración "solo giroscopio" del stack de CMU: aceleración en cero, `gravity = [0,0,0]`, `gravity_align: false`. | Es lo que usa el único stack que reporta funcionar con el L1 en el Go2. Evita la vibración de la marcha y hace irrelevantes las unidades y el signo del acelerómetro. |
| F4 | Preprocesado en un nodo propio, `l1_to_body`, dentro de `laika_slam` (Python + numpy, patrón de `odom_to_tf`). No se copia `transform_sensors` de CMU. | Hace lo mismo que `transform_everything.py` (rotar al marco del cuerpo, quitar retornos del propio robot, corregir la IMU, re-estampar). El de CMU depende de `sensor_msgs_py`, `tf_transformations` y `transforms3d` (sin internet ni sudo en R) y lee `~/Desktop/imu_calib_data.yaml` con ruta fija. |
| F5 | **Nunca** copiar a R ni ejecutar `calibrate_imu` ni `base_autonomy` de ese stack. | `calibrate_imu` publica en `/api/sport/request` (hace girar al robot) y `base_autonomy` lo conduce. Lo prohíbe `CLAUDE.md`. La calibración se hace offline desde un bag, con el usuario girando el robot con el control. |
| F6 | Point-LIO corre **a bordo** (R). | Ver sección 1.1. |
| F7 | Árbol TF de la fase 2 separado: `camera_init → aft_mapped`. No se conecta con `map → odom → base_link`. | No duplica `odom → base_link` ni `map → odom`. Los nombres vienen fijos en el código de CMU y no chocan con la fase 1. |
| F8 | Reloj: mismo esquema de D3 revisada (stamp original + offset único), aplicado por `l1_to_body` a nube e IMU. | Conserva la sincronía nube↔IMU, que vienen del mismo reloj `.161`. Point-LIO solo compara stamps entre sí, no con `now()`. |
| F9 | Iterar con un bag reproducido en R en **`ROS_DOMAIN_ID=42`**. | El robot publica `/utlidar/*` en vivo en el dominio 0 y se mezclaría con el bag. Es una excepción acotada a la regla "dominio 0" de `CLAUDE.md`, por eso requiere aprobación. |
| F10 | `chrony` sigue pospuesto. | RViz con Fixed Frame `camera_init` muestra nubes ya expresadas en `camera_init`; no necesita TF entre relojes. |

Fuera del plan: cierre de lazo 3D, alineación con gravedad, fusión con `/utlidar/robot_odom`, unir los árboles TF, octomap, Nav2, tópico reducido para WiFi (fase 3).

### 0.1 Qué se verificó y qué no (2026-11-16, solo lectura web)

| Verificado | No verificado |
|---|---|
| `unitreerobotics/point_lio_unilidar` es solo ROS 1 (catkin). Su `config/unilidar_l1.yaml` trae `lidar_type: 5`, `timestamp_unit: 0`, `acc_norm: 9.81`, `extrinsic_T: [0.007698, 0.014655, -0.00667]`, `extrinsic_R` identidad. | Que el paquete de CMU **compile** en R (GCC 9, PCL 1.10, Eigen 3.3.7). Su README declara Foxy/20.04 a bordo; no se compiló. |
| CMU: `utlidar.yaml`, `mapping_utlidar.launch` y `laserMapping.cpp` (tópicos, marcos, guardado de PCD; no usa `now()`). | Que los demás fuentes (`preprocess.h`, `Estimator.*`, `IMU_Processing.hpp`) no incluyan cabeceras de `pcl_ros`. `laserMapping.cpp` no las incluye. |
| CMU: `CMakeLists.txt` y `package.xml` piden `pcl_ros`, que no está en R. En ARM compila con `MP_PROC_NUM=1` (un hilo). | Licencia del paquete de CMU (Point-LIO original es GPL-2.0; revisar `LICENSE` al copiar). |
| CMU: `transform_everything.py` (rotación de la nube, caja de filtrado, fórmulas del giroscopio, aceleración a cero, offset de reloj). | La matriz IMU→cuerpo de CMU **no cuadra** con el acelerómetro medido hoy (ver 3.2). Se resuelve con datos en el paso 3. |
| `calibrate_imu.cpp` publica en `/api/sport/request`; fórmulas: bias = media en reposo, `z2x = -mx/mz`, `z2y = -my/mz`. | Si el stamp de `/utlidar/cloud` es el inicio o el fin del barrido. |
| `dfloreaa/point_lio_ros2`: Humble, pide `pcl_ros`, issues #4 y #15 abiertos. | Tamaño real de punto de `/cloud_registered` (el ancho de banda de 1.1 es estimado). |

Las fuentes se leyeron a través de resúmenes web: cotejar los valores numéricos de CMU con los archivos reales al copiarlos en el paso 5.

Discrepancia a aclarar: `docs/decisiones.md` dice "Orin NX (8 núcleos)" y hoy `nproc` da 4. Probablemente es el modo de energía (`nvpmodel`). El plan asume 4 y no lo cambia (requeriría sudo).

## 1. Arquitectura

| Nodo | Máquina | Suscribe | Publica |
|---|---|---|---|
| (fábrica) | Robot | n/a | `/utlidar/cloud` (15.4 Hz), `/utlidar/imu` (250 Hz), `/utlidar/robot_odom` |
| `l1_to_body` (`laika_slam`) | R | `/utlidar/cloud`, `/utlidar/imu` | `/utlidar/transformed_cloud` (marco `body`), `/utlidar/transformed_imu` |
| `pointlio_mapping` (`point_lio_unilidar`) | R | los dos `transformed_*` | `/cloud_registered` (marco `camera_init`), `/aft_mapped_to_init` (Odometry), `/path`, `/Laser_map` (una sola vez), TF `camera_init → aft_mapped`; al cerrar, `PCD/scans.pcd` |
| `rviz2` | E | `/cloud_registered`, `/path`, `/tf` | n/a |

El pipeline de la fase 1 (`mapping.launch.py`) queda intacto y **no se lanza junto** con la fase 2 por defecto (CPU de 4 núcleos, ver R8).

### 1.1 Por qué a bordo

| Criterio | A bordo (R) | En la externa (E) |
|---|---|---|
| Principio de `CLAUDE.md` | Cumple | No cumple |
| Carga | L1 da ~62 000 pts/s; Point-LIO en un hilo; se espera menos de 1 núcleo, más ~0.5 de `l1_to_body`. RAM de sobra (13 GB libres). | Sobra CPU |
| Fase 3 (WiFi) | Solo cruza la visualización, y se puede reducir sin afectar al SLAM. | Cruzarían nube cruda (~2 MB/s) e IMU a 250 Hz; cualquier pérdida o jitter rompe el LIO. |
| Compilación | Foxy, sin `pcl_ros`: parche de 3 o 4 líneas. | Humble, port probado. Solo como plan B de diagnóstico. |

Ancho de banda de `/cloud_registered` (estimado): ~4000 pts × 48 B × 15.4 Hz ≈ 3 MB/s (24 Mbit/s). Bien por Ethernet; en la fase 3 habrá que publicar una versión reducida.

## 2. Árbol TF

Fase 2: `camera_init → aft_mapped` (dinámico, de Point-LIO). Fase 1, sin cambios: `map → odom → base_link → utlidar_lidar`.

| Transformación | Quién | Valor | Fuente / certeza |
|---|---|---|---|
| `camera_init → aft_mapped` | `pointlio_mapping` | dinámico; `camera_init` es la pose del marco `body` al arrancar | Código de CMU (nombres fijos). |
| `utlidar_lidar → body` (no se publica; lo aplica `l1_to_body`) | `l1_to_body` | R = Ry(2.8782 rad); luego z −= 0.046825 m; x no se traslada | `radar_joint` del URDF oficial (`unitree_ros`, `go2_description.urdf`), mismos valores que usa CMU. Verificación: paso 4. |
| `utlidar_imu → body` (solo rotación; no se publica) | `l1_to_body` | candidata A o B (3.2) | **Incierto**; se decide con datos en el paso 3. |
| IMU ↔ LiDAR dentro de Point-LIO | `config` | `extrinsic_T: [0.007698, 0.014655, -0.00667]`, `extrinsic_R` identidad, `extrinsic_est_en: false` | `unilidar_l1.yaml` de Unitree; CMU lo deja igual. Está definido en el marco del sensor y aquí se aplica en `body` sin rotar: error menor a 2 cm, irrelevante con aceleración en cero. |

Reglas:
- No renombrar `camera_init`/`aft_mapped` a `odom`/`base_link`.
- No publicar TF estático entre los dos árboles (`aft_mapped` y `base_link` tendrían dos padres).
- En E no se publica TF.

## 3. Parámetros iniciales

### 3.1 `l1_to_body` (`config/l1_to_body.yaml`)

| Parámetro | Valor | Justificación |
|---|---|---|
| `cloud_R` (3×3 por filas) | `[-0.9655, 0, 0.2605, 0, 1, 0, -0.2605, 0, -0.9655]` | Ry(164.9°), URDF. |
| `cloud_z_offset` | `-0.046825` | URDF (z de `radar_joint`). |
| `self_box` (se **eliminan** los puntos dentro, ya en `body`) | x ∈ (−0.7, −0.1), y ∈ (−0.3, 0.3), z ∈ (−0.6468, −0.0468) | Cuerpo y patas del robot; valores de CMU. |
| `imu_R` (3×3) | A o B, según el paso 3 | Ver 3.2. |
| `gyro_bias` | del paso 3 | Media del giroscopio en reposo, ya en `body`. |
| `ang_z2x_proj`, `ang_z2y_proj` | del paso 3 (inicial 0, 0) | Corrige la fuga del giro en z hacia x/y. Los defaults de CMU (0.15, −0.28) son de su robot; no usarlos. |
| `zero_acc` | `true` | F3. |
| `out_frame` | `body` | Coincide con Point-LIO. |

Reglas del nodo:
- La nube de salida conserva `fields`, `point_step` y los bytes de `intensity`, `ring` y `time`. Solo se reescriben x/y/z y se quitan filas.
- Stamp de salida = stamp original + offset único (`now() - stamp` del primer mensaje; recalcular si se desvía más de 0.5 s, como `odom_to_tf`). El mismo offset para nube e IMU.
- Orden del giroscopio: `w = imu_R · w_raw`; `w -= gyro_bias`; `w.x += z2x · w.z`; `w.y += z2y · w.z`.
- Orientación de salida identidad; aceleración (0, 0, 0).

### 3.2 Rotación IMU → cuerpo: dos candidatas

Con c = cos 15.1° = 0.9655 y s = sin 15.1° = 0.2605:

| Candidata | `imu_R` | Origen | Problema |
|---|---|---|---|
| A | `[c, 0, s, 0, -1, 0, s, 0, -c]` | `transform_everything.py` y `calibrate_imu.cpp` de CMU (invierte y, z y rota 15.1°) | Aplicada al acelerómetro medido hoy (2.855, −0.124, 9.708) da (5.29, 0.12, −8.63): no queda vertical. |
| B | `[c, 0, -s, 0, 1, 0, s, 0, c]` | Deducida del acelerómetro medido hoy | Da (0.23, −0.12, 10.12): vertical. Pero el acelerómetro y el giroscopio podrían no compartir convención. |

No suponer: el paso 3 elige comparando el giroscopio transformado con la velocidad angular de `/utlidar/robot_odom`.

Otros datos: la norma medida del acelerómetro es 10.12 m/s² (3 % sobre g). Confirma m/s² (`acc_norm: 9.81` si algún día se usa) y sugiere un bias o escala que refuerza F3.

### 3.3 Point-LIO (`config/point_lio_utlidar.yaml`)

Partir de `utlidar.yaml` más los parámetros inline de `mapping_utlidar.launch` de CMU. Cambios marcados con (*).

| Parámetro | Valor | Justificación |
|---|---|---|
| `use_sim_time` (*) | `false` | CMU trae `true`; no hay `/clock` (Foxy `bag play` no lo publica). |
| `common.lid_topic` / `imu_topic` | `/utlidar/transformed_cloud` / `/utlidar/transformed_imu` | Salidas de `l1_to_body`. |
| `common.time_lag_imu_to_lidar` | `0.0` | Ajustable si las paredes se emborronan solo al girar (R4). |
| `preprocess.lidar_type` | `5` | Handler del L1: usa x, y, z, intensity y `time`; no usa `ring` (medido siempre 1, no afecta). |
| `preprocess.timestamp_unit` | `0` | `time` en segundos, relativo, 0 a 0.0627 s (medido). |
| `preprocess.blind` | `0.5` | Unitree y CMU. |
| `mapping.imu_time_inte` (*) | `0.004` | 250 Hz medido (CMU trae 0.01). |
| `mapping.acc_norm` | `9.81` | IMU en m/s² (medido). Sin efecto con `zero_acc`. |
| `mapping.gravity_align`, `gravity`, `gravity_init` | `false`, `[0,0,0]`, `[0,0,0]` | F3. El eje z del mapa es el del `body` al arrancar: arrancar con el robot de pie y quieto. |
| `mapping.extrinsic_est_en`, `extrinsic_T`, `extrinsic_R` | `false`, sección 2 | n/a |
| `use_imu_as_input` | `false` | Inline de CMU (modelo de salida). |
| `prop_at_freq_of_imu`, `check_satu` | `true`, `true` | Inline de CMU. |
| `point_filter_num`, `space_down_sample` | `1`, `true` | La nube es escasa: no descartar puntos. |
| `filter_size_surf`, `filter_size_map` | `0.1`, `0.1` | CMU. Bajar a 0.05 si las patas delgadas no aparecen. |
| `init_map_size`, `cube_side_length` | `10`, `1000.0` | CMU. |
| `publish.path_en` (*) | `true` | Ver la trayectoria en RViz. |
| `publish.scan_publish_en` / `scan_bodyframe_pub_en` | `true` / `false` | n/a |
| `pcd_save.pcd_save_en` (*) | `true` | Criterio de terminado. |
| `pcd_save.interval` | `-1` | Un solo archivo al cerrar. Acumula en RAM (~3 MB/s estimado): caminatas de 15 min como máximo. |
| Resto (covarianzas, `plane_thr`, `match_s`, `satu_*`) | sin cambios | n/a |

Launch propio `point_lio.launch.py`: `l1_to_body` más `pointlio_mapping`, este con `sigterm_timeout: '60'` y `sigkill_timeout: '60'` (el PCD se escribe al recibir SIGINT; el default de 5 s puede truncarlo). Sin RViz. Sin los remaps de CMU (`/registered_scan`, `/state_estimation`).

### 3.4 RViz (`rviz/fase2.rviz`, E)

| Display | Tópico | Ajuste |
|---|---|---|
| Fixed Frame | `camera_init` | n/a |
| PointCloud2 | `/cloud_registered` | Best Effort; Decay Time 60 s (subir si E aguanta); Points de 2 px; color por eje Z |
| Path | `/path` | n/a |
| TF | n/a | Solo `camera_init` y `aft_mapped` |

## 4. Pasos de implementación

| # | Máq. | Qué se hace | Archivos en el repo | Prueba y resultado esperado |
|---|---|---|---|---|
| 0 | R | Comprobaciones previas, solo lectura. | `docs/decisiones.md` | `cat /sys/devices/system/cpu/online` (anotar). `ls /usr/include/python3.8/Python.h` existe. `python3 -c "import numpy"` sin error. `ros2 pkg prefix pcl_conversions tf2_geometry_msgs` devuelve rutas. Si falta algo, parar y avisar (implicaría sudo). |
| 1 | E→R | Script de grabación cruda: `/utlidar/cloud`, `/utlidar/imu`, `/utlidar/robot_odom`. Copiarlo a R como en la fase 1. | `scripts/grabar_bag_crudo.sh` | `bash -n` sin errores. |
| 2 | R | **[APROBACIÓN: el usuario camina el robot]** Grabar `~/laika_bags/fase2_cuarto1` con el protocolo 4.1. | n/a | `ros2 bag info`: mensajes de cloud ≈ 15.4 × duración e imu ≈ 250 × duración (±3 %); tamaño ≈ 2.2 MB/s. |
| 3 | E→R | Script offline que lee el `.db3` con `sqlite3` + `rclpy.serialization` (sin dependencias nuevas) y calcula lo de 4.2. Debe abortar con error si ninguna candidata supera el umbral. | `scripts/analizar_imu.py`, resultados en `docs/decisiones.md` | Tabla 4.2 completa; una candidata con correlación ≥ 0.9 en z y signo correcto. Si ninguna: parar y pedir diagnóstico con los números. |
| 4 | E→R | Nodo `l1_to_body` (3.1) con su config, entrada en `setup.py`, `colcon build --packages-select laika_slam`. Probar con el bag en el dominio 42 (4.3). | `laika_slam/laika_slam/l1_to_body.py`, `config/l1_to_body.yaml`, `setup.py` | (a) `ros2 topic hz`: `transformed_cloud` ≈ 15.4, `transformed_imu` ≈ 250. (b) `echo`: `frame_id: body`, `point_step: 32`, mismos `fields`, stamp a menos de 1 s de `date +%s`; IMU con aceleración 0. (c) En el tramo quieto, el pico del histograma de z está en [−0.36, −0.30] (piso medido en −0.33) y el piso se ve horizontal en RViz (Fixed Frame `body`). (d) Sin puntos dentro de `self_box`. (e) `top`: menos de 70 % de un núcleo. |
| 5 | E | Con internet: clonar `autonomy_stack_go2` (`foxy-humble`) fuera del repo y copiar **solo** `src/slam/point_lio_unilidar` a `ros2_ws/src/third_party/`. Anotar URL, rama, hash y licencia. Parche mínimo: quitar `pcl_ros` de `CMakeLists.txt` (`find_package` y lista de dependencias) y de `package.xml`, y quitar `exec_depend transform_sensors`. Crear `PCD/.gitkeep`. | `ros2_ws/src/third_party/point_lio_unilidar/`, `UPSTREAM.md` ahí mismo, `.gitignore` (`*.pcd`) | `git status` muestra solo ese paquete; `grep -r "api/sport" ros2_ws/src/third_party` no devuelve nada. |
| 6 | E→R | `rsync` a `~/laika_ws/src/` y compilar: `MAKEFLAGS=-j2 colcon build --packages-select point_lio_unilidar --cmake-args -DCMAKE_BUILD_TYPE=Release`. No tocar `unitree_go`, `unitree_api`, `laika_basics`. | n/a | `ros2 pkg executables point_lio_unilidar` lista `pointlio_mapping`. Si falla por cabeceras de `pcl_ros`: sección 5. Otro error: resumirlo y pedir diagnóstico. |
| 7 | E→R | Config (3.3) y launch. | `laika_slam/config/point_lio_utlidar.yaml`, `launch/point_lio.launch.py` | Con el bag en el dominio 42: `/cloud_registered` ≈ 15 Hz con `frame_id: camera_init`; `/aft_mapped_to_init` sin NaN; en los primeros 15 s quietos la posición se mueve menos de 2 cm; `top`: `pointlio_mapping` bajo 100 % de un núcleo, sin retraso creciente (el log no acumula "lidar loop back" ni avisos de buffer). |
| 8 | E | Config de RViz y script de arranque con el dominio como argumento (default 0). | `rviz/fase2.rviz`, `scripts/rviz_fase2.sh` | `scripts/rviz_fase2.sh 42` durante la reproducción: la nube se acumula y el Path avanza, sin errores en los displays. |
| 9 | R, E | Aceptación offline con el bag completo (sección 8, criterios 1 a 5). Cerrar con Ctrl-C en el tmux del launch y esperar "exit". Script para traer el PCD a `maps/`. | `scripts/traer_pcd.sh` | Existe `~/laika_ws/src/point_lio_unilidar/PCD/scans.pcd`; su cabecera (`head -n 11`) muestra `POINTS` mayor a 1 000 000 y el tamaño coincide con `POINTS × tamaño de punto`. Si no pasa, tabla 4.4 y luego diagnóstico. |
| 10 | E | **[APROBACIÓN: sudo en E]** `sudo apt install pcl-tools` (`pcl_viewer`, `pcl_voxel_grid`). | n/a | `pcl_viewer maps/fase2_cuarto1.pcd` abre la nube. |
| 11 | R, E | **[APROBACIÓN: el usuario camina el robot]** Prueba final en vivo, dominio 0 (sección 8). | `maps/fase2_cuarto1_5cm.pcd` (reducida, sí en git), `docs/decisiones.md` | Sección 8. |

### 4.1 Protocolo del bag (paso 2; lo camina el usuario)

1. Robot de pie y quieto sobre una marca en el piso, 15 s.
2. Dos giros completos en el sitio hacia la izquierda, lentos (~10 s por vuelta). Quieto 10 s.
3. Perímetro del cuarto a paso lento. Acercarse de frente (desde 2 m o más) a cada mesa alta y pasar junto a las patas delgadas.
4. Volver a la marca, con la misma orientación. Quieto 10 s.

Duración objetivo: 4 a 6 min (~0.6 a 0.8 GB). Nadie debe caminar cerca del robot.

Limitación física, no del software: el L1 está a ~0.3 m y hacia el frente ve hasta ~15° sobre la horizontal. De una cubierta de mesa verá el **canto y la cara inferior**, no la cara superior.

### 4.2 Qué calcula `analizar_imu.py` (paso 3)

| Dato | Cómo | Criterio / uso |
|---|---|---|
| Elegir A o B | Correlación por eje entre `imu_R · w_raw` y la velocidad angular de `/utlidar/robot_odom` (`twist.angular`; si viene en cero, derivar el yaw de la orientación) | Gana la que dé correlación ≥ 0.9 en z con signo positivo en el giro a la izquierda. |
| Unidades del giroscopio | Integral de w.z en los dos giros | ≈ 4π rad (±5 %) y coincide con el yaw de `robot_odom`. |
| `gyro_bias` | Media en los 15 s iniciales, ya en `body` | Esperado: menor a 0.02 rad/s por eje. |
| `ang_z2x_proj`, `ang_z2y_proj` | En los giros: `-mx/mz`, `-my/mz` (medias sin bias) | Si alguno supera 0.3 en valor absoluto, `imu_R` es sospechosa: avisar. |
| Periodo de la IMU | Media y máximo de Δstamp | ≈ 0.004 s; huecos mayores a 0.02 s: anotar. |
| Sincronía nube↔IMU | Diferencia entre el primer stamp de nube y el de IMU más cercano | Menor a 0.1 s (mismo reloj). |

### 4.3 Reproducción aislada (pasos 4, 7, 8, 9)

- En R, tres paneles de tmux, todos con `source ~/laika_ros_env.sh && export ROS_DOMAIN_ID=42`: (1) el launch o nodo bajo prueba, (2) `ros2 bag play ~/laika_bags/fase2_cuarto1`, (3) comandos de prueba.
- En E: `scripts/rviz_fase2.sh 42`.
- Reiniciar el launch antes de **cada** reproducción (los stamps retroceden y Point-LIO descarta todo).
- Nunca lanzar la fase 2 en el dominio 0 mientras se reproduce un bag en el dominio 0.

### 4.4 Ajustes si la aceptación offline falla (uno a la vez, anotando el resultado)

| Síntoma | Ajuste |
|---|---|
| El mapa gira o se abre al girar el robot | Revisar `imu_R` y las proyecciones del paso 3; probar la otra candidata. |
| Paredes emborronadas solo en giros | `time_lag_imu_to_lidar`: probar +0.065 y −0.065 (stamp en inicio o fin del barrido). |
| Deriva lenta en línea recta o en z | `filter_size_surf`/`filter_size_map` a 0.2; `lidar_meas_cov` a 0.001. |
| Faltan patas delgadas | `filter_size_*` a 0.05; caminar más lento. |
| Diverge al arrancar (NaN) | Confirmar que los primeros segundos son quietos; pedir diagnóstico con el primer mensaje de `/aft_mapped_to_init`. |

Si tras tres ajustes no pasa: resumen a `laika-advisor`. Opción de reserva a evaluar ahí: usar `/utlidar/robot_odom` como pose para registrar la nube cruda, que según el issue #4 de dfloreaa da mejor resultado que Point-LIO con el L1.

## 5. Plan B de compilación

| Problema | Acción |
|---|---|
| Faltan cabeceras de `pcl_ros` | Preferida, sin sudo: clonar en E `ros-perception/perception_pcl` rama `foxy`, copiar `pcl_ros` a `~/laika_ws/src` y compilarlo en R. Alternativa **[APROBACIÓN: internet + sudo en R]**: `apt install ros-foxy-pcl-ros`. |
| Falta `Python.h` o matplotlib-cpp | Quitar `PythonLibs`/`matplotlibcpp` del `CMakeLists.txt` y el `#include <Python.h>` (no se usan). Documentar en `UPSTREAM.md`. |
| No compila por otra razón | Diagnóstico con `laika-advisor` (primer error del compilador, no el log entero). |
| Diagnóstico cruzado | Compilar `dfloreaa/point_lio_ros2` en E (Humble) y reproducir ahí una copia del bag en el dominio 42, solo para separar "problema del port" de "problema de datos". No es la solución final (F6). |

## 6. Pasos que requieren aprobación

| Paso | Motivo |
|---|---|
| Plan completo | Uso de `ROS_DOMAIN_ID=42` para reproducir bags (excepción a `CLAUDE.md`). |
| 2, 11 | El usuario camina el robot con el control. |
| 10 | `sudo apt` en E. |
| 5 (plan B), solo si hace falta | Internet (tethering) y `sudo apt` en R. |
| Nunca en esta fase | Publicar en `/api/sport/request` o `/lowcmd`; ejecutar `calibrate_imu` o `base_autonomy`; tocar `laika-flip.service`, `~/.bashrc` de R, `/unitree`, `nvpmodel`; reiniciar el robot. |

## 7. Riesgos

| # | Riesgo | Cómo se detecta pronto | Corrección |
|---|---|---|---|
| R1 | Deriva de Point-LIO con el L1 en el Go2 (reportada por varios usuarios). | Paso 9, criterio 1. | F3, calibración del paso 3, tabla 4.4, opción de reserva. Es el riesgo principal. |
| R2 | `imu_R` equivocada. | Paso 3 (correlación); el mapa rota al girar. | Cambiar de candidata; es solo config. |
| R3 | El paquete de CMU no compila en R. | Paso 6. | Sección 5. |
| R4 | Stamp de la nube en el fin del barrido (65 ms de desfase con la IMU). | Emborronado solo en giros. | `time_lag_imu_to_lidar` (4.4). |
| R5 | Mezcla de bag y datos en vivo. | Frecuencias al doble (`hz` ≈ 30 y 500). | Dominio 42 (4.3). |
| R6 | PCD truncado o perdido al cerrar. | Tamaño distinto de `POINTS × tamaño de punto`; falta el directorio `PCD/`. | `sigterm_timeout` 60 s; `PCD/.gitkeep`; esperar a que el proceso termine. |
| R7 | RAM o disco por acumular el PCD. | `free -h` durante la caminata. | Máximo 15 min; si hace falta más, `interval: 1000` (varios archivos). |
| R8 | CPU con 4 núcleos (`l1_to_body` en Python a 250 Hz más Point-LIO). | `top`; frecuencia de `/cloud_registered` bajo 14 Hz. | No lanzar la fase 1 a la vez; si no alcanza, pasar `l1_to_body` a C++. |
| R9 | Mapa inclinado (sin alineación con gravedad). | El piso sube o baja a lo largo del cuarto. | Arrancar de pie y quieto en piso plano; hasta ~2° se acepta en esta fase. |
| R10 | Offsets de reloj distintos entre `odom_to_tf` y `l1_to_body` (ms). | Solo importa si algún día se fusionan los dos árboles. | Fuera de alcance; anotarlo en `decisiones.md`. |
| R11 | RViz en E lento con millones de puntos. | FPS bajos. | Bajar el Decay Time; el juicio final se hace sobre el `.pcd`. |
| R12 | Mensajes Foxy↔Humble / QoS. | `/cloud_registered` visible en R y no en E. | Display en Best Effort; usar `env_external.sh`. Los tipos son estándar. |
| R13 | PCD grande en git. | Archivo mayor a 50 MB. | `*.pcd` ignorado; solo se versiona la versión reducida a 5 cm. |

## 8. Prueba final y criterio de "consistente"

Procedimiento en vivo (paso 11), dominio 0:
1. R (tmux): `ros2 launch laika_slam point_lio.launch.py`, con el robot de pie y quieto sobre la marca. E: `scripts/rviz_fase2.sh`.
2. **[APROBACIÓN]** El usuario repite el protocolo 4.1 (sin los giros de calibración).
3. Antes de cerrar, en R: leer un mensaje de `/aft_mapped_to_init` y anotar la posición.
4. Ctrl-C en el launch; esperar a que termine.
5. E: `scripts/traer_pcd.sh fase2_cuarto1` deja `maps/fase2_cuarto1.pcd`; `pcl_voxel_grid maps/fase2_cuarto1.pcd maps/fase2_cuarto1_5cm.pcd -leaf 0.05,0.05,0.05`.

Criterios (los mismos en la aceptación offline del paso 9):

| # | Criterio | Umbral |
|---|---|---|
| 1 | Error de retorno: posición final de `/aft_mapped_to_init` con el robot de vuelta en la marca | Menor a 0.15 m en horizontal y 0.10 m en z |
| 2 | Paredes simples: en `pcl_viewer`, vista superior, cada pared es una sola lámina | Grosor ≤ 10 cm; sin paredes dobles ni "fantasmas" girados |
| 3 | Piso único | Una sola lámina; desnivel entre extremos del cuarto ≤ 0.3 m (R9) |
| 4 | Coincide con la fase 1 | La planta se superpone a ojo con `maps/cuarto1.pgm`; una distancia pared a pared difiere menos de 10 cm de la medida con cinta por el usuario (opcional) |
| 5 | Lo que faltaba en 2D | Se distinguen el canto o la cara inferior de la mesa alta y sus patas delgadas |
| 6 | Archivo | `maps/fase2_cuarto1.pcd` abre en `pcl_viewer` y `POINTS` coincide con el tamaño |

Cierre: commit de configs, `maps/fase2_cuarto1_5cm.pcd` y entrada en `docs/decisiones.md` (candidata de `imu_R`, bias, proyecciones, ajustes de 4.4 aplicados, error de retorno).

## Fuentes

- [unitreerobotics/point_lio_unilidar (ROS 1)](https://github.com/unitreerobotics/point_lio_unilidar) y su [`unilidar_l1.yaml`](https://raw.githubusercontent.com/unitreerobotics/point_lio_unilidar/main/config/unilidar_l1.yaml)
- [jizhang-cmu/autonomy_stack_go2, rama `foxy-humble`](https://github.com/jizhang-cmu/autonomy_stack_go2): [`utlidar.yaml`](https://raw.githubusercontent.com/jizhang-cmu/autonomy_stack_go2/foxy-humble/src/slam/point_lio_unilidar/config/utlidar.yaml), [`mapping_utlidar.launch`](https://raw.githubusercontent.com/jizhang-cmu/autonomy_stack_go2/foxy-humble/src/slam/point_lio_unilidar/launch/mapping_utlidar.launch), [`transform_everything.py`](https://raw.githubusercontent.com/jizhang-cmu/autonomy_stack_go2/foxy-humble/src/utilities/transform_sensors/transform_sensors/transform_everything.py), [`calibrate_imu.cpp`](https://raw.githubusercontent.com/jizhang-cmu/autonomy_stack_go2/foxy-humble/src/utilities/calibrate_imu/src/calibrate_imu.cpp)
- [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2), issues [#4 (Foxy)](https://github.com/dfloreaa/point_lio_ros2/issues/4) y [#15 (deriva con L1 en Go2 EDU)](https://github.com/dfloreaa/point_lio_ros2/issues/15)
- [go2_description.urdf (unitree_ros)](https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/go2_description/urdf/go2_description.urdf)

## 9. Revisión (2026-11-16): acumulación de cloud_deskewed en map

Sustituye a Point-LIO (ver `docs/decisiones.md`). La fase 1 no se modifica. Autor: `laika-advisor`; aprobada por el usuario.

### 9.1 Decisiones

| # | Decisión | Justificación |
|---|---|---|
| V1 | Nodo nuevo `map3d_accumulator` en `laika_slam` (Python + numpy), separado de `odom_to_tf`. | `odom_to_tf` es de un hilo y publica TF a 150 Hz; cargarlo retrasaría el TF de la fase 1. |
| V2 | Entradas: `/utlidar/cloud_deskewed` cruda y `/tf` por suscripción directa (sin `tf2_ros`), filtrando `map → odom`. | La nube filtrada de `odom_to_tf` no sirve para 3D. |
| V3 | A cada nube se le aplica el **último `map → odom` recibido**. No se usa lookup por stamp ni offset de reloj. | Es lo validado offline. La nube ya está en `odom` y la corrección cambia lento. Con bags, un lookup por stamp mezclaría tres relojes. |
| V4 | Sin `map → odom`, o con el último recibido hace más de 1.0 s (tiempo de recepción): la nube se **descarta** y se avisa (throttle 5 s). | Mejor un hueco que una pared doble. |
| V5 | Corrección 2D: yaw + (x, y); z sin cambios. | Es lo que entrega `slam_toolbox`. |
| V6 | Topics de entrada por **remapping**, no por parámetro. | En Foxy el yaml ganó al override del launch. |

### 9.2 Mapa y salidas

| Elemento | Valor | Justificación |
|---|---|---|
| Vóxel | 0.05 m; clave int64 empaquetada | Prueba offline; igual a `cuarto1.pgm`. |
| Conteo mínimo | visto en ≥ 2 nubes distintas (deduplicar dentro de cada nube) | Quita ruido suelto. |
| Fusión | pendientes → mapa global a 1 Hz (`np.unique`) | Acota la CPU. |
| Recorte | z ∈ [−0.5, 2.5] m; puntos finitos | Medido p1/p99: −0.20/1.65. |
| Piso y zona cercana | **Se conservan** | El criterio 3 necesita piso; el vóxel ya limita la redundancia. |
| `/laika/map3d` | `PointCloud2`, marco `map`, x y z float32, vóxeles válidos, 0.5 Hz, stamp `now()`; Reliable + Transient Local, depth 1 | ~140 000 pts ≈ 1.7 MB por mensaje (~0.85 MB/s). RViz abierto tarde recibe el último. |
| Versión reducida (fase 3) | **No se implementa ahora** | Bastará un segundo publicador a 10 cm y 0.2 Hz. |
| Guardado | Servicio `/laika/map3d/save` (`std_srvs/Trigger`) y también al cerrar; archivo temporal + renombrar | No depender solo del cierre (R6). |
| Archivo | `~/laika_maps/map3d.pcd` (parámetro `pcd_path`); PCD v0.7 **binario**, campos x y z | ~1.7 MB: cabe en git sin reducir. Se omite intensity. |

RViz (`rviz/fase2_map3d.rviz`, E): Fixed Frame `map`; PointCloud2 `/laika/map3d` (Reliable, Transient Local, color por Z); Map `/map`.

### 9.3 Pasos

| # | Máq. | Qué | Archivos | Prueba |
|---|---|---|---|---|
| 1 | E→R | Nodo, config, launch con argumento `bag` (remapea entradas a `/bag/...`), `setup.py`. Funciones puras (empaquetar, transformar) con prueba de asserts. `colcon build --packages-select laika_slam`. | `laika_slam/map3d_accumulator.py`, `config/map3d.yaml`, `launch/map3d.launch.py`, `test/test_map3d.py` | `python3 test/test_map3d.py` sin error; `ros2 pkg executables laika_slam` lista el nodo. |
| 2 | R | Offline. **Sin `mapping.launch.py` corriendo.** Reproducir `cuarto1` remapeando **todos** sus tópicos (`/map`, `/scan`, `/tf`, `/tf_static`, `/utlidar/cloud_deskewed`, `/utlidar/robot_odom`) a `/bag/...`. Otro panel: `ros2 launch laika_slam map3d.launch.py bag:=true`. Reiniciar el nodo en cada reproducción. | n/a | `ros2 topic hz /laika/map3d` ≈ 0.5; log con 110 000–170 000 vóxeles válidos al final; `top`: menos de 50 % de un núcleo; `ros2 topic list` sin duplicados fuera de `/bag/`. |
| 3 | R, E | Guardar y traer. Adaptar `traer_pcd.sh` a la ruta nueva. | `scripts/traer_pcd.sh`, `rviz/fase2_map3d.rviz`, `maps/fase2_cuarto1_bag.pcd` | `ros2 service call /laika/map3d/save std_srvs/srv/Trigger` da `success: true`; `head -n 11` muestra `POINTS` y tamaño = cabecera + POINTS × 12. Criterios 2 a 5 en RViz. |
| 4 | R, E | **[APROBACIÓN: el usuario camina]** En vivo: `mapping.launch.py` en un panel y `map3d.launch.py` en otro; protocolo 4.1 sin giros de calibración; `save` antes de cerrar. | `maps/fase2_cuarto1.pcd`, `docs/decisiones.md` | 9.4 completo; `/scan` sigue a 5 Hz; `top` total menor a 2 núcleos. |

### 9.4 Criterio de terminado

| # | Criterio | Umbral |
|---|---|---|
| 1 | Retorno a la marca (`tf2_echo map base_link`, inicio y fin) | < 0.15 m horizontal; z de `odom` < 0.10 m |
| 2 | Paredes simples (RViz, vista superior) | Grosor ≤ 10 cm, sin dobles |
| 3 | Piso único | ≥ 90 % de los vóxeles con z < 0.15 m dentro de una banda de 15 cm |
| 4 | Coincide con la fase 1 | Se superpone con `/map` / `maps/cuarto1.pgm` |
| 5 | Lo que faltaba en 2D | Canto o cara inferior de la mesa alta y sus patas; paredes sobre 0.9 m |
| 6 | Archivo | `.pcd` en `maps/`, `POINTS` coincide con el tamaño |

### 9.5 Riesgos

| # | Riesgo | Detección | Corrección |
|---|---|---|---|
| N1 | CPU (4 núcleos; `/tf` a ~200 Hz en Python) | `top`; `/scan` bajo 5 Hz | Procesar 1 de cada 2 nubes. |
| N2 | RAM o mensaje creciente en áreas grandes | Conteo de vóxeles en el log | Bajar frecuencia; vóxel de publicación mayor. |
| N3 | Deriva en z, roll, pitch sin corregir | Criterios 1 y 3 | Aceptada; caminatas cortas, piso plano. |
| N4 | Cierre de lazo: lo ya acumulado no se recorrige | Aviso si `map → odom` salta más de 0.10 m o 2° | Aceptado; si hay paredes dobles, diagnóstico. |
| N5 | TF faltante (slam caído) | Aviso V4; el conteo deja de crecer | Reiniciar la fase 1. |
| N6 | Personas en movimiento dejan fantasmas | A la vista | Nadie cerca. |

Sin verificar (se resuelve en las pruebas): que RViz dibuje la nube con Fixed Frame `map` cuando el bag no publica `map` en `/tf` (está remapeado); los umbrales de CPU, de la banda del piso y del aviso de salto son iniciales.

**Obsoleto del plan original:** decisiones F1–F3 y F7–F10, pasos 1–9 y 11, secciones 3.1–3.3, 4.2–4.4 y 5, y riesgos R1–R4, R6, R8–R10; el paso 10 (`pcl-tools`) pasa a opcional.
