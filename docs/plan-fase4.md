# Plan de la fase 4: autonomía por cable (ir a una pose en `cuarto1`)

Plan de `laika-advisor` del 2026-11-16. Los IDs y el JSON de la API salen del código de `unitree_ros2` (`ros2_sport_client.h/.cpp`). Lo que depende del firmware queda "sin verificar" y tiene un paso que lo resuelve.

## 0. Alcance
- **Meta:** con el mapa `maps/fase2_cuarto1`, el usuario da la pose inicial y la meta en RViz. El robot llega a ≤ 0.25 m y ≤ 15° evitando obstáculos estáticos, a ≤ 0.25 m/s, con el usuario detrás sosteniendo el cable.
- **Para después:** waypoints, exploración, SLAM y navegación a la vez, obstáculos dinámicos, marcha atrás, WiFi, uso del mapa 3D, operación sin supervisión.

## 1. Decisiones y arquitectura
| # | Decisión | Justificación |
|---|---|---|
| A1 | **AMCL + `map_server`** con el `.pgm`; `slam_toolbox` apagado. | La localización de `slam_toolbox` necesita un `.posegraph` que no se guardó. Dos publicadores de `map → odom` romperían el TF. |
| A2 | Nav2 Foxy **a bordo** si está en `apt`. **Plan B:** Nav2 Humble en la externa (no compilar Nav2 en Foxy). | Mantiene el principio "a bordo". En el plan B, el puente y el watchdog siguen en el robot. |
| A3 | Nodo `cmd_vel_to_sport` en `laika_slam` (robot). Es el **único** que publica `unitree_api/Request` y solo usa los IDs 1008 y 1003. | Un solo punto de control. |
| A4 | Salida con nombre relativo `sport_request_out`. El launch la remapea: `armed:=false` (por defecto) → `/laika/sport_request_dry`; `armed:=true` → `/api/sport/request`. | Dry-run real. Se hace por remapping porque en Foxy el yaml gana sobre los parámetros del launch. |
| A5 | **Hombre muerto:** el puente exige `/laika/deadman` (`std_msgs/Empty`, ≥ 2 Hz) desde la externa. | Cubre en un solo mecanismo el Ctrl-C y la desconexión del cable. |
| A6 | Metas por el tópico `/goal_pose` desde RViz, no por la acción. | `nav2_msgs` cambia entre Foxy y Humble. |
| A7 | Árbol de comportamiento **sin `Spin` ni `BackUp`**: solo `Wait` y limpiar costmaps. | El scan no ve atrás (±100°) y girar enrolla el cable. |

**Nodos:**
- **Robot:** `tf.launch.py`, `scan.launch.py`, `map_server`, `amcl`, `planner_server` (NavFn), `controller_server` (DWB), `recoveries_server`, `bt_navigator`, `lifecycle_manager`, `cmd_vel_to_sport`.
- **Externa:** RViz y el publicador de `/laika/deadman`.

**API de sport:**
- **Move:** `api_id = 1008`, `parameter = '{"x": vx, "y": vy, "z": vyaw}'`.
- **StopMove:** `api_id = 1003`.
- **Sin verificar:** cuánto dura un Move sin renovarse y si el joystick gana. Se mide en el paso 6.

## 2. Parámetros iniciales
- **Puente:**
  - Límites: `vx` ∈ [0, 0.25], `vy` = 0, `wz` ∈ ±0.5; Move a 10 Hz.
  - Manda StopMove a 10 Hz durante 1 s y luego guarda silencio si pasa cualquiera de esto: `/cmd_vel` lleva más de 0.3 s sin llegar, el deadman lleva más de 0.5 s sin llegar, llega un NaN o llega un comando cero.
  - Manda StopMove al cerrarse.
- **AMCL:**
  - Modelo `OmniMotionModel` con `likelihood_field`, `max_beams` 60 y 500–2000 partículas.
  - Rango del láser de 0.6 a 10 m; `update_min_d/a` 0.1; `transform_tolerance` 0.5.
- **Costmaps:**
  - `robot_radius` 0.40, `inflation_radius` 0.60, resolución 0.05 (confirmar contra el URDF).
  - Local rodante de 4 × 4 m a 5 Hz con `/scan`.
  - Global: estática + obstáculos + inflación.
- **DWB:**
  - Velocidades máximas: `max_vel_x` 0.25, `min_vel_x` 0, `max_vel_y` 0, `max_vel_theta` 0.5.
  - Aceleraciones 0.5, a 10 Hz.
  - Tolerancias: 0.20 m y 0.25 rad.
  - Odometría: `/utlidar/robot_odom`.

## 3. Seguridad (obligatoria)
- **Paradas, de la más suave a la más dura:**
  1. Ctrl-C del deadman.
  2. Desconectar el cable.
  3. Ctrl-C del puente.
  4. Joystick (si el paso 6d confirma que gana).
  5. L2+B (Damp): el robot se desploma; último recurso.
- **Personas:** se recomiendan dos, una con el cable y el control y otra en la terminal del deadman. Con una sola persona, el paso 6d es requisito para el paso 7.
- **Cable:** el usuario va detrás del robot. Las metas quedan dentro del radio del cable, sin rodear muebles y con menos de 180° de giro neto por meta.
- **`laika-flip.service`:** no se toca. En el paso 0 se leen sus combinaciones (solo lectura). Durante la autonomía, el control solo se usa para detener.
- **Postura:** el usuario pone al robot de pie con el control. El puente nunca manda StandUp, Damp ni otros IDs.
- **Aprobaciones:** cada una es un mensaje del usuario para esa prueba concreta.

## 4. Pasos
| # | Qué | Prueba y criterio |
|---|---|---|
| 0 | Solo lectura en el robot. | Hay candidato de `ros-foxy-navigation2`/`nav2-bringup`. Quién publica en `/api/sport/request`. Combinaciones de `laika-flip.service`. `unitree_api/msg/Request` coincide con la sección 1. `/utlidar/robot_odom` da un `twist` distinto de cero al caminar. |
| 1 | **[APROBACIÓN: sudo]** `apt install` de Nav2 en el robot, o el plan B. | `ros2 pkg list` muestra los paquetes de Nav2. |
| 2 | Solo localización con AMCL. **[APROBACIÓN: el usuario camina con el control]** | El scan cae sobre las paredes. Retorno a la marca < 0.15 m. `slam_toolbox` no aparece en `/tf`. |
| 3 | Puente con funciones puras y test de asserts. Dry-run con `ros2 topic pub /cmd_vel`. | El test pasa. Limita `vx` y bloquea la reversa. Al cortar `/cmd_vel` o el deadman sale 1003 en < 0.5 s. No publica en `/api/sport/request`. |
| 4 | Nav2 en dry-run, robot quieto, meta en RViz y una caja delante. | Ruta libre. La caja aparece en el costmap. `/cmd_vel` dentro de los límites. Sin `Spin` ni `BackUp`. |
| 5 | **[APROBACIÓN: primera publicación real]** `armed:=true` sin Nav2; solo StopMove. | El robot no cambia de postura. El control sigue respondiendo. |
| 6 | **[APROBACIÓN: movimiento mínimo]** `wz` 0.2 durante 2 s, luego `vx` 0.1 durante 2 s. | (a) Ctrl-C del pub detiene. (b) Ctrl-C del deadman detiene. (c) Desconectar el cable detiene. Las tres en < 0.5 s. (d) Quién gana: joystick o Move. (e) Los signos son correctos. |
| 7 | **[APROBACIÓN por escalón]** Nav2 real: metas a 1 m, luego a 3 m, luego con una caja en la ruta. | **Terminado:** 3 metas seguidas a ≤ 0.25 m y ≤ 15°, una con obstáculo, sin contacto, y una parada por deadman en plena marcha. |
| 8 | Registrar en `docs/decisiones.md`; actualizar `CLAUDE.md`. | Commit. |

## 5. Riesgos
| # | Riesgo | Mitigación |
|---|---|---|
| R1 | Zona ciega: atrás y a menos de 0.6 m | Sin reversa ni spin; nadie junto al robot en marcha. |
| R2 | Estela de 2 s en el scan crea obstáculos fantasma | Ventana menor solo para navegar. |
| R3 | Stamps de `/utlidar/robot_odom` ~2066 s atrasados | Republicar la odometría con el stamp corregido. |
| R4 | AMCL se pierde: scan de 200° a 5 Hz, cuarto simétrico | Pose inicial cuidadosa; más haces y partículas. |
| R5 | El firmware ignora Move o la evasión del Go2 interfiere | Diagnóstico con `/sportmodestate`. |
| R6 | Otro nodo o `laika-flip` publica en `/api/sport/request` | Revisar en el paso 0. |
| R7 | Un Move persiste si muere el puente | Medirlo en el paso 6; persona con el control. |
| R8 | Pasos estrechos sin ruta | Huella poligonal del URDF. |
| R9 | La mesa alta queda como espacio libre en el mapa | No poner metas que crucen bajo la mesa. |
