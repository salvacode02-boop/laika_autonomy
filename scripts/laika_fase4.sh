#!/bin/bash
# Fase 4: arranca todo en tmux (sesión "laika4"). Corre en la Jetson externa.
#
#   scripts/laika_fase4.sh           ARMADO: el puente publica en /api/sport/request (solo con aprobación del usuario)
#   scripts/laika_fase4.sh --prueba  dry-run: el puente publica en /laika/sport_request_dry y se activa solo
#   scripts/laika_fase4.sh --slam    en vez del mapa guardado, hace el mapa en vivo (slam_toolbox) y navega sobre él
#   scripts/laika_fase4.sh --sin-cable  MODO SIN CABLE (combinable con --slam y --prueba), ver abajo
#   --mapa <ruta .yaml>  localiza sobre ese mapa (copia .pgm+.yaml al robot, ~/laika_maps/); si no es fase2_cuarto1 no hay pose de la marca
#   --sin-attach  no hace tmux attach al final (lo usa el dashboard);  --sin-rviz  sin ventana de RViz
#   scripts/laika_fase4_parar.sh     detiene todo en orden
#
# Al inicio solo corren: localización (mapa + AMCL), el puente en IDLE (no manda nada), RViz y la ventana de pose.
# Con R2+X el puente pasa a ACTIVE y entonces arrancan Nav2 (en el robot) y la terminal de aprobación (aquí).
# Una ventana con 6 paneles: base, puente, nav, aprobar, pose, rviz (clic para elegir panel).
#
# MODO SIN CABLE: base, puente (-p sin_cable:=true) y Nav2 corren en una sesión tmux EN EL ROBOT (laika4r, logs en
# ~/laika4r_<panel>.log) y Nav2 arranca desde el inicio. Los paneles base/puente/nav de aquí solo muestran los logs
# (tail -F); aprobar, pose y rviz siguen locales. Flujo: meta en RViz -> Enter en "aprobar" -> desconectar el cable,
# esperar 10 s -> R2+X. Para: L2+Y, llegar, 90 s, meta nueva o sin /lf/lowstate.
# CERRAR LA TMUX LOCAL NO DETIENE NADA en modo sin cable: para parar del todo, scripts/laika_fase4_parar.sh o L2+Y.
#
# Cerrar todo (con cable): tmux kill-session -t laika4 (o kill-server); el puente manda StopMove al recibir SIGHUP.
set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT=unitree@192.168.123.18
S=laika4
ENV_R='source ~/laika_ros_env.sh && export PYTHONUNBUFFERED=1'
ENV_E="source $D/env_external.sh && export PYTHONUNBUFFERED=1"
# Pose de la marca de inicio (la de la caminata de la fase 2); si el robot está en otro lugar, corrígela en RViz.
MARCA='{header: {frame_id: map}, pose: {pose: {position: {x: 0.034, y: 0.027}, orientation: {z: -0.118, w: 0.993}}, covariance: [0.06,0,0,0,0,0, 0,0.06,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}'

ATTACH=1; RVIZ=1; MAPA_LOCAL=''; MAPA_ARG=''; MARCA_OK=1
SALIDA=/api/sport/request; EXTRA=''; MODO='ARMADO'
BASE=localization.launch.py; MAPA='mapa guardado (AMCL)'; NAVARGS=''; SINCABLE=0
while [ $# -gt 0 ]; do
  a=$1; shift
  case $a in
    --sin-attach) ATTACH=0 ;;
    --sin-rviz) RVIZ=0 ;;
    --mapa) MAPA_LOCAL=${1:?--mapa necesita la ruta de un .yaml}; shift ;;
    --prueba) SALIDA=/laika/sport_request_dry; EXTRA='-p dry_run_force_active:=true'; MODO='PRUEBA (dry-run)' ;;
    --sin-cable) SINCABLE=1 ;;
    --slam) BASE=mapping.launch.py; MAPA='mapa en vivo (slam_toolbox)'
            NAVARGS='planner_yaml:=/home/unitree/laika_ws/install/laika_slam/share/laika_slam/config/planner_slam.yaml' ;;
    *) echo "Opción desconocida: $a (usa --prueba, --slam, --sin-cable, --mapa <yaml>, --sin-attach, --sin-rviz)"; exit 1 ;;
  esac
done

[ -n "$MAPA_LOCAL" ] && [ $BASE = mapping.launch.py ] && { echo "--mapa y --slam son excluyentes."; exit 1; }
tmux has-session -t $S 2>/dev/null && { echo "Ya existe la sesión $S. Entra con: tmux attach -t $S (o párala con scripts/laika_fase4_parar.sh)"; exit 1; }
ssh -o ConnectTimeout=5 $ROBOT true || { echo "No hay SSH al robot ($ROBOT)."; exit 1; }
# Regla de chrony: no lanzar si el robot no está sincronizado con la externa.
ssh $ROBOT 'chronyc tracking' | grep -q 'Reference ID.*192.168.123.50' || { echo "El robot no está sincronizado con la externa (chronyc tracking)."; exit 1; }
ssh $ROBOT 'tmux has-session -t laika4r 2>/dev/null' && { echo "Ya existe la sesión laika4r en el robot (¿modo sin cable en marcha?). Párala con scripts/laika_fase4_parar.sh"; exit 1; }
ssh $ROBOT 'tmux ls 2>/dev/null' | grep -q . && { echo "Hay sesiones tmux en el robot; ciérralas antes (ssh $ROBOT tmux ls)."; exit 1; }

# Mapa elegido: copiar .pgm+.yaml al robot; AMCL arranca con ese yaml. Pose de la marca solo con fase2_cuarto1.
if [ -n "$MAPA_LOCAL" ]; then
  [ -f "$MAPA_LOCAL" ] || { echo "No existe $MAPA_LOCAL"; exit 1; }
  IMG=$(sed -n 's/^image: *//p' "$MAPA_LOCAL" | tr -d "\"' \r"); IMG=${IMG:?el yaml no tiene image:}
  [ -f "$(dirname "$MAPA_LOCAL")/$IMG" ] || { echo "Falta la imagen $IMG junto a $MAPA_LOCAL"; exit 1; }
  ssh $ROBOT 'mkdir -p ~/laika_maps'
  scp -q "$MAPA_LOCAL" "$(dirname "$MAPA_LOCAL")/$IMG" $ROBOT:laika_maps/ || { echo "Falló scp del mapa."; exit 1; }
  MAPA_REMOTO="/home/unitree/laika_maps/$(basename "$MAPA_LOCAL")"
  MAPA_ARG="map:=$MAPA_REMOTO"; MAPA="mapa cargado $(basename "$MAPA_LOCAL")"
  [ "$(basename "$MAPA_LOCAL" .yaml)" = fase2_cuarto1 ] || MARCA_OK=0
fi

# Espera a que el puente pase a ACTIVE (R2+X). En el robot (Foxy) y aquí (Humble).
ESPERA="until ros2 topic echo /laika/bridge_state 2>/dev/null | grep -m1 -q ACTIVE; do sleep 1; done"

ARGS_PUENTE="-r sport_request_out:=$SALIDA $EXTRA"
CMD_BASE="$ENV_R && ros2 launch laika_slam $BASE $MAPA_ARG"
CMD_NAV="$ENV_R && echo 'Esperando R2+X para arrancar Nav2...' && $ESPERA && ros2 launch laika_slam nav2_servers.launch.py $NAVARGS"
CMD_APR="$ENV_E && echo 'Esperando R2+X...' && $ESPERA && echo 'Activo: este panel es el hombre muerto (Ctrl-C = parar).' && python3 $D/aprobar_rutas.py"
if [ $SINCABLE = 1 ]; then
  [ "$SALIDA" = /laika/sport_request_dry ] && ARGS_PUENTE="$ARGS_PUENTE -p dry_run_keys_topic:=/laika/dry_keys"
  ARGS_PUENTE="$ARGS_PUENTE -p sin_cable:=true"
  CMD_PUENTE_R="$ENV_R && ros2 run laika_slam cmd_vel_to_sport --ros-args $ARGS_PUENTE"
  CMD_NAV_R="$ENV_R && ros2 launch laika_slam nav2_servers.launch.py $NAVARGS"
  # Sesión remota: shells sin rc (el .bashrc del robot pregunta ros:foxy?), pipe-pane antes de lanzar para no perder el inicio.
  ssh $ROBOT bash -s <<REMOTO
set -e
tmux new-session -d -s laika4r -n base 'bash --norc'
tmux new-window -t laika4r -n puente 'bash --norc'
tmux new-window -t laika4r -n nav 'bash --norc'
for w in base puente nav; do tmux pipe-pane -t laika4r:\$w "cat > ~/laika4r_\$w.log"; done
tmux send-keys -t laika4r:base "$CMD_BASE" Enter
tmux send-keys -t laika4r:puente "$CMD_PUENTE_R" Enter
tmux send-keys -t laika4r:nav "$CMD_NAV_R" Enter
REMOTO
  CMD_BASE="ssh $ROBOT 'tail -n 200 -F ~/laika4r_base.log'"
  CMD_PUENTE="ssh $ROBOT 'tail -n 200 -F ~/laika4r_puente.log'"
  CMD_NAV="ssh $ROBOT 'tail -n 200 -F ~/laika4r_nav.log'"
  CMD_APR="$ENV_E && echo 'SIN CABLE: Ctrl-C aquí NO detiene; usa L2+Y.' && python3 $D/aprobar_rutas.py"
else
  CMD_BASE="ssh -t $ROBOT \"$CMD_BASE\""
  CMD_PUENTE="ssh -t $ROBOT \"$ENV_R && ros2 run laika_slam cmd_vel_to_sport --ros-args $ARGS_PUENTE\""
  CMD_NAV="ssh -t $ROBOT \"$CMD_NAV\""
fi

# Todo en una sola ventana, en paneles con título. Cerrar tmux (kill-session/kill-server) corta los ssh:
# los procesos del robot reciben SIGHUP y el puente lo trata como Ctrl-C (manda StopMove).
P_BASE=$(tmux new-session -d -P -F '#{pane_id}' -s $S -n laika "$CMD_BASE; read -p 'base terminó. Enter para cerrar'")
panel() {  # panel <título> <comando>
  local id; id=$(tmux split-window -P -F '#{pane_id}' -t $S:laika "$2")
  tmux select-pane -t "$id" -T "$1"; tmux select-layout -t $S:laika tiled > /dev/null; echo "$id"
}
tmux select-pane -t "$P_BASE" -T base
panel puente "$CMD_PUENTE; read -p 'puente terminó. Enter para cerrar'" > /dev/null
panel nav "$CMD_NAV; read -p 'nav terminó. Enter para cerrar'" > /dev/null
# Desde el dashboard (--sin-attach) no hay panel aprobar: el dashboard aprueba y es el ÚNICO hombre muerto
# (si aprobar_rutas.py corriera detrás, cerrar el dashboard no detendría el robot).
if [ $ATTACH = 1 ]; then
  P_APR=$(panel aprobar "$CMD_APR; read -p 'aprobar terminó. Enter para cerrar'")
else
  P_APR=$P_BASE
fi
if [ "$BASE" = mapping.launch.py ]; then  # con SLAM el mapa nace donde está el robot: no hay pose inicial
  panel pose "$ENV_E && python3 $D/ver_pose.py" > /dev/null
elif [ $MARCA_OK = 0 ]; then  # otro mapa: la pose inicial la pone el usuario (herramienta "Pose inicial")
  panel pose "$ENV_E && python3 $D/ver_pose.py" > /dev/null
else
  panel pose "$ENV_E && sleep 20 && ros2 topic pub -t 3 -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"$MARCA\" > /dev/null && echo 'Pose inicial en la marca publicada.' ; python3 $D/ver_pose.py" > /dev/null
fi
[ $RVIZ = 1 ] && panel rviz "$D/rviz_fase4.sh" > /dev/null
tmux set-option -t $S pane-border-status top > /dev/null
tmux set-option -t $S pane-border-format ' #{pane_title} ' > /dev/null
tmux set-option -t $S mouse on > /dev/null  # clic para elegir panel
tmux select-pane -t "$P_APR"
echo "Sesión $S lista: $MODO, $MAPA. Presiona R2+X para arrancar la navegación."
[ "$BASE" = mapping.launch.py ] && echo "Para guardar el mapa: scripts/guardar_mapa.sh <nombre> (antes de cerrar)"
if [ $SINCABLE = 1 ]; then echo "SIN CABLE: cerrar esta tmux NO detiene el robot. Para parar: L2+Y o scripts/laika_fase4_parar.sh"
else echo "Para cerrar todo: tmux kill-session -t $S   (o scripts/laika_fase4_parar.sh)"; fi
[ $ATTACH = 0 ] && exit 0
[ -n "$TMUX" ] && tmux switch-client -t $S || tmux attach -t $S
