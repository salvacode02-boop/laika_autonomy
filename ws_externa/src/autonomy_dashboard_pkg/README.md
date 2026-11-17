# 🤖 RoboCup Rescue 2026 - Autonomy Dashboard

**Dashboard profesional para visualización y control de navegación autónoma en competencias de RoboCup Rescue.**

## 📋 Características

### ✨ Visualización en Tiempo Real
- **Mapa 2D SLAM** - Muestra el layout del laberinto con posición del robot en tiempo real
- **Nube 3D** - Visualización de puntos Fast-LIO con escalado de altura (colores por Z)
- **Pose del Robot** - Coordenadas X, Y y ángulo theta actualizado en vivo
- **Exploración** - Barra de progreso mostrando % de área explorada

### 🎮 Controles
- ▶️ **START MAPPING** - Inicia captura de mapas
- ⏸ **PAUSE** - Pausa el mapeo
- ▶️ **RESUME** - Continúa mapeo
- 💾 **QUICK MAP** - Guarda mapa rápido (<5 min)
- 💾 **COMPREHENSIVE MAP** - Guarda mapa completo (>15 min)
- 📦 **SAVE 3D CLOUD** - Exporta como PLY
- 📋 **SAVE OBJECTS** - Exporta POIs como CSV
- 🗺️ **EXPORT GeoTIFF** - Genera GeoTIFF con información espacial
- ⚙️ **AUTO-EXPORT** - Exporta automáticamente a los 20 minutos
- 🔄 **RESET MISSION** - Reinicia la misión

### 📊 Estado del Sistema
- Modo de autonomía (IDLE, MAPPING, EXPLORING, NAVIGATING, FAULT)
- Conexión ROS (Conectado/Desconectado)
- Posición actual del robot (X, Y, Θ)
- Área explorada (%)
- Número de puntos nube (cloud points)
- Tiempo transcurrido / Tiempo máximo
- Objetos detectados
- Información del equipo y misión

### 🔄 Arquitectura

```
┌─────────────────────────────────────────────┐
│   autonomy_dashboard_node.py                │
│   (ROS Node - Agrega datos)                 │
└────────┬────────────────────────────────────┘
         │
         ├── Subscriptions:
         │   ├── /cloud_registered (PointCloud2)
         │   ├── /map (OccupancyGrid)
         │   ├── /odom (Odometry)
         │   ├── /amcl_pose (PoseStamped)
         │   └── /autonomy/status (String)
         │
         └── Publications:
             ├── /autonomy/status
             ├── /autonomy/robot_pose
             ├── /autonomy/mapped_area_percent
             └── /autonomy/detected_objects

┌─────────────────────────────────────────────┐
│   autonomy_ui.py (PyQt5)                    │
│   ┌───────────────┬────────────────────┐   │
│   │   Map 2D      │  Map 3D Cloud      │   │
│   ├───────────────┴────────────────────┤   │
│   │   Status Panel (Pose, Area, etc)   │   │
│   ├────────────────────────────────────┤   │
│   │   Control Buttons (Start/Save/etc) │   │
│   └────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

## 📦 Instalación

### Requisitos
```bash
sudo apt install python3-pyqt5 python3-matplotlib
pip3 install numpy
```

### Compilar Paquete
```bash
cd ~/RRL-2026-Robot
colcon build --packages-select autonomy_dashboard_pkg
source install/setup.bash
```

## 🚀 Uso

### Opción 1: Launch file completo
```bash
ros2 launch autonomy_dashboard_pkg autonomy_dashboard.launch.py \
  team_name:=YourTeam \
  mission:=Prelim1
```

### Opción 2: Ejecutar nodo ROS
```bash
ros2 run autonomy_dashboard_pkg autonomy_dashboard_node.py
```

### Opción 3: Ejecutar solo interfaz PyQt5
```bash
ros2 run autonomy_dashboard_pkg autonomy_ui.py
```

## 📁 Estructura del Proyecto

```
autonomy_dashboard_pkg/
├── autonomy_dashboard/
│   ├── __init__.py
│   ├── autonomy_dashboard_node.py    # Nodo ROS principal
│   ├── autonomy_ui.py                # Interfaz PyQt5
│   └── widgets/
│       ├── __init__.py
│       ├── map_viewer_2d.py          # Widget mapa 2D
│       ├── cloud_viewer_3d.py        # Widget nube 3D
│       └── status_panel.py           # Panel de estado
├── config/
│   └── autonomy_params.yaml          # Parámetros
├── launch/
│   └── autonomy_dashboard.launch.py  # Launch file
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🎯 Requisitos RoboCup 2026

El dashboard cumple con los requisitos de Mapping & Object Detection:

### ✅ Formatos Soportados
- **2D Map**: GeoTIFF (formato especificado)
- **3D Map**: PLY (point cloud)
- **Objects**: CSV con formato RoboCup 2026

### ✅ Funcionalidades Autonomía
- Mapeo autónomo del laberinto
- Detección automática de objetos
- Auto-exportación a los 20 minutos
- Información de pose en tiempo real
- Visualización del progreso de exploración

### ✅ Multipliers
- **Teleop (1x)**: Operador controla manualmente
- **Comms Degradation (2x)**: Con comunicación degradada
- **Autonomy (5x)**: Navegación totalmente autónoma

## 🔧 Configuración

Editar `config/autonomy_params.yaml`:

```yaml
autonomy_dashboard_node:
  ros__parameters:
    team_name: "TuEquipo"
    mission: "Prelim1"
    output_dir: "/ruta/salida"
    max_mapping_time: 1200  # 20 min
```

## 🎬 Flujo de Uso en Competencia

1. **Conecta Hardware**
   - LIDAR (Livox/Ouster)
   - Robot base ROS2
   - Sensor de objetos

2. **Inicia ROS Stack**
   ```bash
   # Terminal 1: Fast-LIO + Nav2
   ros2 launch fastlio_pkg mapping.launch.py
   
   # Terminal 2: Autonomy Dashboard
   ros2 launch autonomy_dashboard_pkg autonomy_dashboard.launch.py
   ```

3. **En el Dashboard**
   - Presiona "START MAPPING"
   - Navega el robot por el laberinto (automático o teleop)
   - Presiona "ENABLE AUTO-EXPORT" (se guardará en 20 min)
   - O manualmente: presiona "SAVE" en cualquier momento

4. **Exporta Resultados**
   - Los archivos se guardan en formato RoboCup 2026
   - PLY para nube 3D
   - GeoTIFF para mapa 2D
   - CSV con objetos detectados

## 📊 Datos Exportados

### Archivos PLY
```
RoboCup2026-TeamName-Mission-HH-MM-SS-map.ply
```
- Formato ASCII PLY
- X, Y, Z de cada punto
- Compatible con CloudCompare, Meshlab

### GeoTIFF
```
TeamName-Mission-HH-MM-SS.tiff
```
- Mapa rasterizado con grid
- Escala y orientación incluidas
- Compatible con QGIS, ArcGIS

### CSV Objects
```
RoboCup2026-TeamName-Mission-HH-MM-SS-pois.csv
```
- Formato específico RoboCup 2026
- detection,time,type,name,x,y,z,robot,mode
- Objetos detectados (QR, Hazmat, Víctimas)

## 🐛 Troubleshooting

### "ImportError: No module named 'PyQt5'"
```bash
sudo apt install python3-pyqt5
```

### "No se reciben datos del cloud"
- Verificar que Fast-LIO está publicando en `/cloud_registered`
- Revisar: `ros2 topic list` y `ros2 topic info /cloud_registered`

### "No se ve el mapa SLAM"
- Verificar que Nav2/SLAM está publicando en `/map`
- Revisar: `ros2 topic echo /map` (primeras líneas)

### Interfaz lenta
- Reducir `max_points` en config (downsampling)
- Aumentar update_rate: `update_rate: 5` (Hz)

## 📝 Licencia

BSD License - RoboCup Rescue 2026

## 👥 Autor

RoboCup Rescue Team 2026

## 🔗 Referencias

- RoboCup Rescue Rules 2026D
- Fast-LIO2 SLAM
- Nav2 Framework
- PyQt5 Documentation

## Laika: navegación, metas y hombre muerto (bloques A y B)

Copia adaptada en `ws_externa/` (la original de RRL no se toca). Se lanza con `scripts/dashboard.sh` (config `config/dashboard_laika.yaml`).

- **Capas 2D** (casillas en la barra lateral): costmap global y local (`/global_costmap/costmap`, `/local_costmap/costmap`; el local se lleva a `map` con la TF), ruta global `/plan` (verde), ruta local `/local_plan` (amarilla), huella (`/local_costmap/published_footprint` o la huella fija del parámetro `footprint`) y flecha de `/cmd_vel`.
- **Panel Navegación:** estado del puente (`/laika/bridge_state`, con color), velocidad comandada (`/cmd_vel`) y real (twist de `/utlidar/robot_odom`).
- **Herramientas Meta / Pose inicial** (botones de la vista 2D): clic = posición, arrastrar = orientación, soltar = publica `/goal_pose` o `/initialpose` (covarianza como RViz). Sin herramienta activa, pan/zoom.
- **Aprobar ruta:** reemplaza `scripts/aprobar_rutas.py` (misma lógica y clave `goal_key`); publica `/laika/plan_approved` a 5 Hz. Funciona también con metas de RViz.
- **Hombre muerto:** `/laika/deadman` a 5 Hz desde un QTimer del hilo de la UI (no se publica si el puente está en modo `SC:`). En modo sin cable se muestra un aviso rojo.
- Nunca publica en `/api/sport/request` ni `/lowcmd`.

### Prueba sin robot
`scripts/dashboard_fake_nav.py [ESTADO]` simula mapa, TF, costmaps, `/plan` (tras una meta) y estado del puente. Usar SOLO en un dominio aislado (`ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1`, sin `env_external.sh`). Con `DASHBOARD_SELFTEST=<carpeta>` el dashboard simula clics reales (Meta, Aprobar, Pose inicial) y guarda capturas PNG; `DASHBOARD_SELFTEST_GOAL=x,y,x2,y2` fija la meta y su arrastre (map).

## Laika: panel Robot, mapas cargados y modelo del Go2 (bloques C y D)

**Panel ROBOT** (barra lateral): Mapa (En vivo / Cargado), Cable (Con cable / Sin cable), Modo (Prueba / Armado).
- **Arrancar** ejecuta `scripts/laika_fase4.sh` con `--slam` o `--mapa <yaml>`, `--sin-cable`, `--prueba`, y siempre `--sin-attach --sin-rviz`, en un `QProcess` (la UI no se bloquea: el hombre muerto sale de ella). **Parar** ejecuta `scripts/laika_fase4_parar.sh`. La salida de los scripts sale en el cuadro de mensajes del panel. "Armado" pide confirmación en un cuadro explícito (por defecto No). No se puede cerrar el dashboard mientras corre Arrancar/Parar/Guardar/Cargar.
- **Cargar mapa…** acepta `.yaml` (+`.pgm`), GeoTIFF y PLY exportados por este dashboard. Se convierte (hilo aparte) a `.pgm`+`.yaml` en `maps/cargados/`, se muestra en el 2D (como `/map` local, no se publica a ROS) y queda elegido para Arrancar con Mapa = Cargado (el script copia el par al robot, `~/laika_maps/`, y AMCL arranca con él; con un mapa distinto de `fase2_cuarto1` no se publica la pose de la marca: usa "Pose inicial"). Al elegir "En vivo" otra vez, el 2D vuelve a `/map`.
- **GeoTIFF:** `writeGeoTiff` guarda en el TIFF (tag ImageDescription) la geometría exacta (`res`, px por celda, márgenes, `xmax/ymax`). Al cargar, cada celda (f×f px) es ocupada si algún pixel es el azul de ocupado, desconocida si algún pixel es del tablero gris, y libre si no (blanco, rejilla, trayectoria o POI encima). Ida y vuelta verificada: el `.pgm` es idéntico al del mapa original. Limite: una trayectoria/POI que tape una celda desconocida entera la haría libre. Los GeoTIFF exportados antes de este cambio no traen la geometría: hay que exportarlos de nuevo.
- **PLY:** se lee el ascii (x y z), piso = percentil 2 de z, rejilla de 0.05 m; ocupado = ≥2 puntos con z entre piso+0.10 y piso+0.93 (banda del scan: `pc2scan.yaml` `max_height` 0.6 sobre `base_link` + piso ~0.33 m bajo `base_link`; el inicio sube de 0.03 a 0.10 para no contar el propio piso); libre = celdas con puntos de piso (±0.10 m) y no ocupadas; el resto desconocido (sin raytracing). Parámetros en `config/dashboard_laika.yaml` (`ply_to_grid`). La nube también se muestra en la vista 3D.
- **Guardar mapa (.pgm/.yaml)** pide un nombre y ejecuta `scripts/guardar_mapa.sh <nombre>` (`map_saver_cli` sobre `/map` -> `maps/<nombre>`).

**Modelo del Go2:** `robot_model.cpp` lee el URDF (`urdf::Model`) y las mallas `.dae` (assimp; ~400k triángulos, carga ~0.9 s). La cinemática se calcula en el dashboard con `/joint_states` (nodo `joint_states` de `laika_slam` en el robot, 20 Hz, a partir de `/lf/lowstate`); no se usa `robot_state_publisher` para no tocar el árbol TF del robot (su raíz del URDF es `base`, no `base_link`). Sin `/joint_states` reciente (>2 s) se dibuja de pie. 3D: mallas con luz fija (GLSL 120); 2D: unión de los cascos convexos XY de cada eslabón (puntos extremos de cada malla) en lugar de la flecha; la huella de Nav2 es una capa aparte. assimp aplica una rotación Z_UP→Y_UP que anula la matriz del nodo de los `.dae`; se deshace en `loadMesh`.

Autoprueba: `DASHBOARD_SELFTEST_CD=<carpeta>` con `DASHBOARD_CD_LOAD`, `DASHBOARD_CD_RUN=live|loaded` (siempre Prueba, nunca Armado), `DASHBOARD_CD_WAIT`, `DASHBOARD_CD_EXPORT`, `DASHBOARD_CD_CAM=yaw,pitch,dist`; `DASHBOARD_FPS=1` imprime los FPS del 3D.
