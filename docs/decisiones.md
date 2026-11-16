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
