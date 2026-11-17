# Plan: dashboard de Laika (copia en `ws_externa/src/autonomy_dashboard_pkg`)

Objetivo: el dashboard reemplaza a RViz, `aprobar_rutas.py` y `ver_pose.py` para operar la fase 4. Todo lo que mueve el robot sigue pasando por `cmd_vel_to_sport` y sus reglas: R2+X, L2+Y, aprobación por meta y modo sin cable.

## Bloques (en este orden; cada uno se prueba en dry-run antes del siguiente)

**A. Navegación visible en el mapa 2D**
- Capas activables: costmap global y local (`/global_costmap/costmap`, `/local_costmap/costmap`, colores por costo), ruta global `/plan`, trayectoria local `/local_plan`, huella del robot y flecha de `/cmd_vel`.
- Panel de estado: estado del puente (`/laika/bridge_state`: IDLE, ACTIVE, MOVING, SC:…), velocidad comandada y velocidad real.

**B. Metas y pose desde el mapa**
- Herramientas "Meta" y "Pose inicial": clic y arrastre en el mapa 2D (posición + orientación), que publican `/goal_pose` y `/initialpose`.
- Botón **Aprobar ruta**: reemplaza el Enter de `aprobar_rutas.py`, con la misma regla (una aprobación por meta y la clave de `goal_key`).
- **Hombre muerto** en modo con cable: el dashboard publica `/laika/deadman` desde el hilo de la UI. Si la UI se congela o se cierra, el robot se detiene. En modo sin cable se muestra el aviso "Ctrl-C/cerrar NO detiene: usa L2+Y".

**C. Modos y mapas**
- Arranque o parada del robot desde el dashboard. Llama a `scripts/laika_fase4.sh` y a `laika_fase4_parar.sh` con las opciones: **mapa en vivo** (`--slam`) o **mapa cargado**, con o sin cable, y prueba o armado.
- **Cargar mapa** desde archivo:
  - `.pgm` + `.yaml`: el formato nativo.
  - **GeoTIFF** exportado por el dashboard: se decodifica por colores (libre blanco, ocupado azul, desconocido gris) y se ignoran la trayectoria, los POIs y la rejilla.
  - **PLY**: corte en una banda de altura (la del scan, 0.10–0.50 m sobre el piso) → rejilla 2D. La nube también se muestra en 3D.
  - Al cargar, se convierte a `.pgm` + `.yaml`, se copia al robot y AMCL arranca con ese mapa (el `map_server` corre a bordo, como hoy).
- **Guardar mapa**: `.pgm` + `.yaml` (`map_saver`), además de los PLY, GeoTIFF y CSV que ya existen.

**D. Modelo del robot**
- Se descarga `go2_description` de `unitree_ros` (Unitree, BSD-3; de ahí ya salió el `radar_joint`) a `ws_externa/src`.
- En el robot, un nodo nuevo `joint_states` toma los 12 ángulos de los motores de `/lf/lowstate` y publica `/joint_states`. `robot_state_publisher` (con el URDF) publica la TF de cada eslabón.
- **3D:** las mallas del Go2 (assimp, ya instalado), colocadas por TF, muestran cómo se mueven las patas.
- **2D:** la silueta del modelo vista desde arriba, en lugar de la flecha.

## Seguridad
- El dashboard nunca publica en `/api/sport/request`.
- El hombre muerto sale del hilo de la UI y solo existe en modo con cable.
- Cerrar el dashboard en modo con cable detiene el robot. En modo sin cable no lo detiene, y la interfaz lo dice.
- La aprobación es de un solo uso, por meta.

## Pruebas
- Cada bloque se prueba en dry-run (`--prueba`).
- B y C, además, con el robot quieto.
- Las pruebas armadas necesitan aprobación del usuario, como siempre.
