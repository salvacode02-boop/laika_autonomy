# Decisiones

## 2026-11-16: plan de la fase 1 aprobado

Ver `docs/plan-fase1.md` (decisiones D1 a D7).

## 2026-11-16: mediciones del paso 4 (robot de pie y quieto)

| Dato | Medido | Consecuencia |
|---|---|---|
| `/utlidar/robot_odom` | 150 Hz; `odom` → `base_link` | Coincide con el plan; `odom_to_tf` valida esos marcos. |
| `/utlidar/cloud_deskewed` | 15.4 Hz; ~10 600 pts/msg; campos `x y z intensity` (float32, `point_step` 32) | Hay suficientes puntos en bruto. |
| QoS de los publicadores | Reliable / Volatile (odom, cloud, cloud_deskewed) | Los suscriptores pueden ser Best Effort o Reliable. |
| Reloj de los stamps | Odom y nube usan el mismo reloj, ~2060 s **detrás** del reloj de la Jetson del robot (probablemente es el reloj de la `.161`) | Confirma D3: TF con el stamp del mensaje, nunca `now()`. `chrony` entre Jetsons no corrige este desfase. Riesgo de "extrapolation" en nodos que usan `now()`. |
| Altura del piso en `base_link` | z ≈ −0.33 m (pico del histograma entre −0.35 y −0.30); `base_link` a 0.304 m en `odom`; roll/pitch ≈ −1.9°/1.4° en reposo | `min_height = −0.17` (z_piso + 0.15) **confirmado**. |
| Puntos sobre piso + 0.15 m | ~190 por nube; 95 % de los puntos de la nube caen a menos de 1 m (piso cercano) | Densidad útil baja: R2 es probable. Confirmar en el paso 8 con el criterio de ≥150 haces válidos; si no se alcanza, aplicar C2 (acumular nubes). |

## 2026-11-16: `odom_to_tf` se queda a 150 Hz

Usa ~30 % de un núcleo (Python, 150 msg/s). Eso es ~4 % de la CPU total de la Orin NX (8 núcleos), así que la meta de 20 % del plan no aplica como límite. Si `slam_toolbox` necesita CPU, publicar 1 de cada 3 mensajes (50 Hz, stamp original). Con un scan de 15 Hz el error de interpolación quedaría en milímetros.

## 2026-11-16: apt de ROS en el robot

`ros-foxy-slam-toolbox` y `ros-foxy-pointcloud-to-laserscan` no aparecen en `apt-cache`. El repositorio de ROS en el robot es el mirror `mirrors.tuna.tsinghua.edu.cn`. Se decide en el paso 7 (requiere aprobación de sudo).

## 2026-11-16: D3 revisada: stamp original + offset común al reloj de la Jetson del robot

- `slam_toolbox` 2.4.1 (Foxy) estampa `map → odom` con `now()`; con stamps ~2060 s atrasados, `map → base_link` no tiene tiempo común y /scan no se dibuja.
- Un offset único (`now() - stamp` del primer mensaje de odometría) aplicado a TF y nube conserva la alineación odom/nube; `odom_to_tf` lo recalcula si se desvía >0.5 s y republica la nube en `/laika/cloud_deskewed`.
- Se descartó cambiar el reloj del sistema y compilar `slam_toolbox` desde código.

## 2026-11-16: C2 aplicada

- Motivo: en una caminata corta sin paredes el scan tenía solo 77/720 haces con rango; además el scan solo ve ~180° al frente.
- Ventana deslizante de 10 nubes a 5 Hz, publicada en `/laika/cloud_deskewed` por `odom_to_tf`.
- Filtro de altura en `odom` (z - z_base en [-0.12, 0.50], distancia horizontal ≥ 0.6 m); sustituye a C1.
- Scan frontal ±1.75 rad a 1° (201 haces), `min_height -0.3`, `max_height 0.6`, `range_min 0.6`.
- Python vectorizado (numpy) en lugar de C++.
- `slam_toolbox` sin cambios.
- El objeto a 0.8–1.1 m del frente izquierdo era el usuario: los rangos del scan quedan validados.

## 2026-11-16: `angle_max: 1.75003` en `pc2scan.yaml`

Con `angle_max: 1.75`, `pointcloud_to_laserscan` publica 200 haces y `slam_toolbox` calcula 201 por redondeo, así que rechaza todos los scans (`expected 201`). El valor 1.75003 hace que ambos cuenten 201. Si se cambia `angle_increment` o el rango, revisar este error en el log.

Resultado de C2 con el robot quieto: 145–156 de 201 haces finitos (criterio ≥120), `/scan` a 5 Hz sin huecos (máx 0.204 s). `odom_to_tf` usa 31–45 % de un núcleo; se acepta (≈5 % de la CPU total).

## 2026-11-16: prueba final de la fase 1

- Recorrido del perímetro con cierre de lazo. Mapa en `maps/cuarto1.pgm` y `maps/cuarto1.yaml` (resolución 0.05, 330 × 200 celdas, ~16.5 × 10 m).
- Paredes continuas y simples (1–2 celdas), esquinas a ~90°, sin paredes dobles visibles. Log del robot sin errores.
- Valores finales: `min_height` efectivo por el filtro del acumulador (`z_min_rel` −0.12 respecto a la base, ≈ piso + 0.21); scan ±1.75003 rad a 0.0175 rad; ventana de 10 nubes a 5 Hz; `slam_toolbox` con los parámetros del plan, sin cambios.
- Ruido conocido: el usuario se atravesó cerca del punto de inicio para acomodar el cable.
- Bags en el robot: `~/laika_bags/caminata2` (886 MB) y `~/laika_bags/cuarto1` (1.6 GB).

## 2026-11-16: revisión del mapa final por el usuario y cierre de la fase 1

El usuario validó `maps/cuarto1` contra el cuarto real: "bastante bien".
- **Puerta de cristal:** bien detectada como obstáculo, con el exterior como espacio libre.
- **Hueco detrás del usuario y las sillas (zona de inicio):** bien detectado como libre.
- **Mesa alta del centro (~1.5–2 m, con ~40 cm libres debajo):** solo aparece la estructura inferior central. La cubierta que sobresale no aparece. Es una limitación del diseño: el filtro conserva puntos hasta 0.50 m sobre la base (≈0.82 m sobre el piso), y hacia los lados el L1 solo ve por debajo de su altura. Para el robot ese espacio sí es transitable, salvo que la altura libre sea menor a la del Go2.
- **Patas delgadas de la mesa alta junto al inicio:** detección parcial. Es esperable con un scan de 1°, 5 Hz y objetos de pocos cm.
- **Pendiente para la fase 2:** la nube 3D (Point-LIO) resuelve mesas y voladizos. Si se usa Nav2 con el mapa 2D, considerar un `max_height` igual a la altura del robot más un margen.

`chrony` (paso 12 del plan) **se omite** en la fase 1 por decisión del usuario. Con el ajuste de hora (D3 revisada) no bloquea nada. Se revisa en la fase 2.

**Fase 1 terminada.**

## 2026-11-16: fase 2, paso 0 (comprobaciones en el robot)

| Comprobación | Resultado |
|---|---|
| Núcleos | `cpu/online` = 0-3, `nproc` = 4. Solo hay 4 núcleos activos (probablemente por el modo `nvpmodel`, que no se toca). Se corrige el "8 núcleos" de entradas anteriores: `odom_to_tf` a ~35 % es ~9 % del total, no ~5 %. |
| `Python.h`, numpy | Presentes (numpy 1.17.4). |
| `pcl_conversions`, `tf2_geometry_msgs`, cabeceras PCL 1.10 | Presentes. |
| `pcl_ros` | No está: se quita del paquete de CMU (paso 5) o se aplica el plan B. |
| `ros2 bag play --remap` | **Existe en Foxy.** |
| Internet | Disponible por el teléfono del usuario (`usb0`). |

**Cambio al plan (F9):** las reproducciones de bags se hacen en el **dominio 0** con `--remap` (`/utlidar/cloud` → `/bag/utlidar/cloud`, `/utlidar/imu` → `/bag/utlidar/imu`, `/utlidar/robot_odom` → `/bag/utlidar/robot_odom`), y `l1_to_body` recibe los tópicos de entrada como parámetro. El usuario prefiere mantener todo en el dominio 0. Ya no hace falta `ROS_DOMAIN_ID=42`. Regla: mientras se reproduce un bag no debe correr el `point_lio.launch.py` en vivo, porque ambos publicarían `/utlidar/transformed_*` y `/cloud_registered`.

## 2026-11-16: Fase 2 paso 3: calibración de la IMU

Análisis offline de `~/laika_bags/fase2_cuarto1` con `scripts/analizar_imu.py` (copia del bag en la externa, fuera del repo). Tramos detectados con odom: quieto 1–36.6 s; giros 37.6–90.3 s (con pausas; desplazamiento máx 0.14 m; yaw de odom 12.75 rad ≈ 4π). `twist.angular` de odom viene con datos (se usó).

| | A | B |
|---|---|---|
| corr (x, y, z) con odom | 0.73, 0.28, **0.987** | 0.46, −0.28, −0.96 |
| integral w.z en giros | 10.17 rad (0.80 × yaw odom) | −9.17 rad |
| `gyro_bias` (body) | (0.0075, −0.0005, 0.0016) | (0.0073, 0.0005, 0.0024) |
| `ang_z2x_proj` / `ang_z2y_proj` | 0.075 / −0.227 | 0.486 / −0.252 |
| acel. quieta en body | (5.01, 0.00, −8.80) no vertical | (−0.10, 0.00, 10.13) vertical |

- **Candidata elegida: A** (única con corr_z ≥ 0.9 y signo positivo en giros a la izquierda; B sale con signo negativo). Para el giroscopio manda este criterio; la aceleración con A no es vertical, pero `zero_acc: true` la anula. Confirma que acelerómetro y giroscopio no comparten convención (el temor del plan).
- B descartada además por |z2x| = 0.49 > 0.3.
- **Valores para `l1_to_body`:** `imu_R` = A; `gyro_bias` = (0.0075, −0.0005, 0.0016); `ang_z2x_proj` = 0.075; `ang_z2y_proj` = −0.227.
- **Unidades:** m/s² para el acelerómetro (|a| = 10.13). **Giroscopio: la integral de A es 20 % menor que el yaw de odom (criterio ±5 % NO cumplido).** Integral de `twist.z` de odom = 12.28 (yaw de orientación 12.75), así que odom es consistente consigo mismo. Pendiente de aclarar: factor de escala del giroscopio (~1.23–1.25) o subestimación; no es deg/s (el factor sería 57). Se reporta al orquestador; no se aplica escala sin decidir.
- `ang_z2y_proj` = −0.227 queda bajo el umbral 0.3 pero es grande; vigilar.
- **Periodo de la IMU:** media 4.002 ms, máx 18.6 ms, 0 huecos > 0.02 s. 53094 mensajes.
- **Nube:** 15 Hz, Δ entre nubes 0.0649 s; campo `time` por punto float32 de 0 a ≈0.0624 s, así que el **stamp es el INICIO del barrido**. Primer stamp de nube − IMU más cercana = −54.6 ms (mismo reloj).

## 2026-11-16: fase 2, escala del giroscopio: no se aplica (`gyro_scale = 1.0`)

En los giros, el giroscopio integra ~0.86 del yaw de odom. Para distinguir escala de bias se hizo una regresión de w (proyectado sobre el eje de giro medido, [0.138, −0.216, −0.967] en el marco IMU) contra `twist.z` de odom, en ventanas de 0.5 s:

| Tramo | Pendiente | Intercepto |
|---|---|---|
| Giro 1 (42.6–51.6 s, ~0.62 rad/s) | 0.956 | +0.002 rad/s |
| Giro 2 (62.1–83.6 s, ~0.28 rad/s) | 0.990 | −0.065 rad/s |
| Todos | 1.020 | −0.056 rad/s |

La pendiente es ≈ 1: **no es error de escala**. El déficit es un **bias que aparece al caminar** (~−0.06 rad/s), que pesa más en el giro lento. Point-LIO estima el bias del giroscopio en línea, así que debería absorberlo. IMU confirmada a 249.9 Hz.

Configuración inicial (criterio del advisor): `imu_R` = A + proyecciones, `gyro_scale` = 1.0. Variante de reserva si las paredes se emborronan al girar: rotación que lleva z_body al eje medido, con z2x = z2y = 0.

## 2026-11-16: fase 2, se cambia Point-LIO por acumular `cloud_deskewed` corregida con `slam_toolbox`

Point-LIO (port de CMU) falló con el bag `fase2_cuarto1` en tres intentos (plan 4.4):

| Corrida | Resultado |
|---|---|
| Base (`filter_size` 0.1) | Yaw a la deriva desde el inicio (~0.25 rad/s); error de retorno 6.4 m |
| `filter_size` 0.5 | Arranque estable; error de retorno 7.75 m; nube difusa |
| 0.5 + `imu_en: false` | Diverge por completo (272 m) |

Antes de eso se encontró y corrigió un bug: en Foxy, el yaml de `l1_to_body` ganaba sobre el override de parámetros del launch, así que con `bag:=true` se procesaba la nube en vivo. Ahora el launch usa remappings.

Prueba offline de la alternativa (bag `cuarto1` de la fase 1): se acumuló `/utlidar/cloud_deskewed` (en `odom`) aplicándole el `map → odom` de `slam_toolbox` grabado en `/tf`, en vóxeles de 5 cm. Paredes simples y rectas, esquinas a 90°, la mesa del centro definida y las partes altas de las paredes por encima de 0.9 m. Con la odometría sola, las paredes salen dobles (hasta 10° de deriva de yaw).

**Decisión (aprobada por el usuario):** la fase 2 usa un nodo a bordo que acumula `cloud_deskewed` en el marco `map` con la corrección de `slam_toolbox`, publica para RViz y guarda `.pcd`. Point-LIO y `l1_to_body` se quedan en el repo sin usarse.

Limitaciones aceptadas:
- La corrección es 2D (x, y, yaw).
- Se depende del deskew de Unitree.
- La cubierta de la mesa alta casi no se ve, por la geometría del L1 (a ~0.3 m del piso, mirando hacia abajo).

## 2026-11-16: fase 2 revisada, prueba offline con el bag `cuarto1` (pasos 9.3.1–9.3.3)

- `map3d_accumulator`: 4416 nubes procesadas, 0 descartadas, 141 819 vóxeles válidos (la prueba offline previa dio ~142 000). `/laika/map3d` a 0.5 Hz; 14–31 % de un núcleo. 9 avisos de salto de `map → odom` (cierres de lazo de `slam_toolbox`).
- La entrada se suscribe con QoS **Reliable**: con best effort se perdían casi todas las nubes fragmentadas.
- PCD binario de 1.7 MB con tamaño consistente: `maps/fase2_cuarto1_bag.pcd`.
- **Criterio 3 (piso único):** la métrica del plan ("≥ 90 % de los vóxeles con z < 0.15 m en una banda de 15 cm") da 80–86 %, pero mide mal: cuenta también patas de sillas y la base de paredes y muebles. Con una medida directa, el piso es una sola lámina:
  - mediana de z por celda de 1 m entre −0.075 y +0.025 m en 71 celdas;
  - plano ajustado con 0.07° de inclinación;
  - pico del histograma en −0.10..0.00 m.
- **Criterio 3 revisado:** mediana de z del piso por celda de 1 m con un rango ≤ 0.10 m e inclinación ≤ 1°. Se cumple.

## 2026-11-16: fase 2 en vivo (paso 9.3.4), fase 2 cerrada

Caminata en el cuarto1 con `mapping.launch.py` + `map3d.launch.py` a bordo y RViz en la externa.
- Retorno a la marca (`map → base_link`): inicio (0.034, 0.027) m, −13.6°; final (−0.046, 0.037) m, −12.1°. Error 8 cm y 1.5° (límite 15 cm). Cumple.
- 8525 nubes, 0 descartadas; 160 985 puntos en `maps/fase2_cuarto1.pcd` (1.9 MB). Mapa 2D de la misma caminata: `maps/fase2_cuarto1.pgm/.yaml`.
- Piso: inclinación 0.22°, mediana por celda en un rango de 0.15 m (criterio revisado: 0.10 m). **Aceptado por el usuario**; se atribuye al balanceo del cuerpo al caminar.
- Pared derecha: banda de ~37 cm de grosor por encima de 1.3 m, delgada abajo; también apareció en el bag. Se interpreta como estructura real de la pared, no deriva (esquinas y parte baja salen simples).
- **El usuario confirmó que los mapas quedaron bien: fase 2 terminada.** Pendiente para después: chrony y fase 3 (WiFi).

## 2026-11-16: relojes sincronizados con chrony (plan en `docs/plan-chrony.md`)

**Decisión:** la Jetson externa es el servidor chrony (pools de internet + `local stratum 10`, `allow 192.168.123.0/24`) y reemplaza a `systemd-timesyncd`. El robot es cliente con una sola fuente (`server 192.168.123.50 iburst minpoll 2 maxpoll 4`, `makestep 1 -1`, pools desactivados). No se tocó código.

Resultados:
- Robot sincronizado con la externa: |System time| de 1–5 µs.
- Tras reiniciar el robot (RTC en 1970): chrony saltó a la hora correcta 3 s después de arrancar.
- Sin internet en la externa: Stratum 10 local; el robot siguió con `^*` y menos de 3 µs. Al volver el WiFi regresó a los pools sin desfasar al robot.
- Regresión: `/scan` a 5.0 Hz, `tf2_echo map base_link` responde, el log no tiene errores ni "extrapolation". `map3d bag:=true` con `cuarto1`: 4480 nubes, 0 descartadas, 142 609 vóxeles (antes 141 819).
- **Hallazgo:** la `.161` no se quedó en 1970: sus stamps pasaron a 2026, pero siguen ~2066 s detrás del robot. Parece que toma la hora de la Jetson del robot con un desfase fijo. El offset por software de `odom_to_tf` lo sigue cubriendo.

Reglas de operación:
- Antes de lanzar el SLAM: `ssh unitree@192.168.123.18 chronyc tracking` debe mostrar Reference ID `192.168.123.50`. Si el reloj salta con los launches corriendo, reiniciarlos.
- Si el robot arranca sin la externa queda en 1970: no compilar con `colcon` en ese estado.
- `apt-daily` del robot solo actualiza listas (sin `unattended-upgrades`): se deja como está.

## 2026-11-16: fase 3, prueba del WiFi propio del robot (`Go2_41771`)

El usuario quiso probar, antes de montar el router, recibir los datos por el WiFi del robot.
- La Jetson del robot no tiene WiFi. `Go2_41771` lo da la `.161` (`192.168.12.1/24`); la externa recibe `192.168.12.125`.
- Prueba con rutas temporales: en la externa `192.168.123.18/32 via 192.168.12.1`; en el robot `192.168.12.0/24 via 192.168.123.161`.
  - Externa → `.18` por WiFi: sin respuesta (ping y SSH).
  - Robot → `192.168.12.1`: responde (es una dirección de la propia `.161`). Robot → `192.168.12.125`: sin respuesta.
- **Conclusión:** la `.161` no reenvía tráfico entre su WiFi y la red cableada. La `.161` es cerrada, así que no se puede activar. **El WiFi del robot no sirve para ROS: la fase 3 sigue con el router montado en el perro.**

## 2026-11-16: fase 4, paso 0 y control remoto

- Nav2 Foxy 0.4.7 instalado por `apt` en el robot (AMCL, map_server, NavFn, DWB, Regulated Pure Pursuit, recoveries, bt_navigator). No se clona `navigation2`: la rama Foxy es la misma versión. Los planeadores nuevos (Smac, MPPI) requieren Humble: si se necesitan, irían en la externa.
- `laika-flip.service` deshabilitado por el usuario (`systemctl disable --now`). `~/laika_ws/flip3_trigger.py` deja solo L1+Y (BackFlip); respaldo en `flip3_trigger.py.bak`.
- **Control remoto:** `/wirelesscontroller` no entrega pulsaciones (ni por ROS ni por el SDK). Se leen de `/lowstate` (`unitree_go/LowState`, ~500 Hz), campo `wireless_remote`: `keys = b[2] | b[3] << 8`. Medidos con el usuario: **R2+X = 1040** (activar autonomía), **L2+Y = 2080** (detener), R2 = 16, L2 = 32, Y = 2048.
- `/api/sport/request` tiene 11 publicadores internos de Unitree (bare DDS); ninguno nuestro.

## 2026-11-16: fase 4, pasos 2-4 en dry-run (robot quieto)

Correcciones al puente `cmd_vel_to_sport` antes de cualquier prueba armada:
- Lee el control de **`/lf/lowstate`** (20 Hz) en vez de `/lowstate` (500 Hz): CPU del puente de ~70 % a ~8 %.
- **Paro por bits:** L2+Y detiene si están los bits de L2 (32) y de Y (2048), aunque haya otros botones presionados. Activar exige exactamente R2+X (1040). El paro gana siempre.
- **Watchdog del control:** en ACTIVE, más de 0.5 s sin `/lf/lowstate` → StopMove 1 s e IDLE (sin control no hay botón de paro).

Pruebas en dry-run (`navigation.launch.py force_active:=true`, salida `/laika/sport_request_dry`):
- Meta a 1 m: `/plan` de 46 puntos (1.31 m). Move (1008) empieza 0.1 s después del Enter.
- Ctrl-C de `aprobar_rutas.py` (deadman): StopMove (1003) durante 0.9 s a partir de 0.6 s del Ctrl-C, luego silencio. La latencia la fija el timeout de 0.5 s + el tick de 0.1 s.
- Meta nueva: ningún Move durante 7 s hasta el nuevo Enter.
- Ctrl-C del puente: 3 StopMove al cerrar.
- `/api/sport/request`: solo los 11 publicadores internos de Unitree.
- AMCL acepta `robot_model_type: omnidirectional`.
- Sin probar en vivo: R2+X y L2+Y reales (solo test unitario) y el watchdog del control.

## 2026-11-16: fase 4, paso 2 en vivo (localización AMCL, el usuario camina con el control)

- Pose inicial publicada en la marca (la de inicio de la fase 2: 0.03, 0.02, −14°).
- Retorno a la marca según AMCL: (−0.065, −0.086) m y −13.3°, contra (0.032, 0.022) m y −13.6° al inicio. Error 0.145 m y 0.3°: dentro del límite de 0.15 m, por poco. Incluye el error de colocar el robot a mano.
- Ajuste scan–mapa (haces a ≤ 0.10 m de una celda ocupada), 109 muestras: media 70 %, mínimo 50 %, sin desplomarse. No llega a 100 % porque hay muebles bajos y personas.
- **Incidente:** el Ethernet de la externa (`enP8p1s0`) se cayó 3 veces entre 13:25:41 y 13:27:08 (link down/up). AMCL corre a bordo y no se afectó. En la navegación armada, el deadman detendría al robot ante una caída, pero hay que asegurar el conector o el cable.

## 2026-11-16: fase 4, paso 5 (primera publicación real, aprobada por el usuario)

- `cmd_vel_to_sport` solo, armado (`-r sport_request_out:=/api/sport/request`), sin Nav2 ni deadman: únicamente podía mandar StopMove.
- El control real funcionó: R2+X → `ACTIVE`; 31 s después, L2+Y → StopMove 1 s → `IDLE`.
- `/sportmodestate.mode` = 0 antes y después: el robot no cambió de modo. Puente detenido al terminar.
- Sin probar: el watchdog de `/lf/lowstate`. No hay forma de cortar ese tópico sin tocar la `.161`; queda cubierto solo por la lógica.

## 2026-11-16: fase 4, paso 6 (movimiento mínimo, aprobado por el usuario)

Puente armado, sin Nav2. Las órdenes las genera `scripts/prueba_movimiento.py`. El diagnóstico (`/api/sport/response` y `/sportmodestate`) se registró en el robot.
- **Zona muerta de la marcha del Go2:**
  - vx = 0.1 y wz = 0.2 solo inclinan el cuerpo;
  - vx = 0.2 es inestable: a veces camina, a veces arranca y se frena, a veces solo se estira, aunque los 83 Move vuelven con `code=0`;
  - wz = 0.4 gira (~24° en 2 s);
  - **vx = 0.3 camina de forma continua 8 s** (real 0.08–0.31, media 0.22 m/s, ~1.8 m).
- Para medir el umbral, el usuario aprobó subir `VX_MAX` a 0.4 (antes 0.25).
- `error_code=100` y `mode=0`/`gait_type=0` son normales en este firmware (en reposo y caminando). `obstaclesAvoidSwitch=false` según `/multiplestate`. Nadie más publica 1008/1003.
- Signos correctos: wz > 0 gira a la izquierda; vx > 0 avanza.
- **Paradas:**
  - corte de `/cmd_vel`: quieto ≲ 1 s;
  - corte del deadman: quieto ~0.7 s;
  - **L2+Y real a 0.3 m/s:** primer StopMove 70 ms después y robot quieto en 0.5–1 s; las órdenes siguientes quedan ignoradas (IDLE).
- **El joystick NO detiene la orden automática: se suma.** Joystick a la derecha durante el Move vx=0.3 → vy −0.3..−0.4 mientras vx sigue ~0.25. Las únicas paradas son L2+Y, el deadman (Ctrl-C o cable), el Ctrl-C del puente y, como último recurso, L2+B (Damp).

## 2026-11-16: fase 4, ajuste de Nav2 a la marcha del Go2 (dry-run)

- **Puente:**
  - `VX_MAX` 0.3 (bajado del 0.4 de las pruebas).
  - **Zona muerta:** vx no nula < 0.25 se sube a 0.25; giro en el lugar (vx = 0) con |wz| < 0.4 se sube a 0.4. Por debajo de 0.02 m/s / 0.05 rad/s se toma como cero.
  - Test ampliado.
- **Controladores:** DWB `max_vel_x`/`max_speed_xy` 0.3; RPP `desired_linear_vel` 0.3.
- **Huella:** `robot_radius` 0.40 → huella rectangular `[[±0.38, ±0.18]]` (R8). Con el radio, la marca de inicio quedaba "inscrita": hay una celda ocupada del mapa a 0.33 m (izquierda-adelante), NavFn fallaba con "failed to create plan" y el costo en la marca era 99.
- **Dry-run:** desde la marca, meta a 1.5 m → `/plan` de 71 puntos (1.85 m, rodea ese obstáculo). Enter → Move `x=0.300, z=-0.237` a 10 Hz. Ctrl-C del deadman → 10 StopMove.
