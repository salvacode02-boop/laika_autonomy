#!/bin/bash
# Fase 4: arranca todo en tmux (sesión "laika4"). Corre en la Jetson externa.
#
#   scripts/laika_fase4.sh           ARMADO: el puente publica en /api/sport/request (solo con aprobación del usuario)
#   scripts/laika_fase4.sh --prueba  dry-run: el puente publica en /laika/sport_request_dry y se activa solo
#   scripts/laika_fase4.sh --slam    en vez del mapa guardado, hace el mapa en vivo (slam_toolbox) y navega sobre él
#   scripts/laika_fase4_parar.sh     detiene todo en orden
#
# Al inicio solo corren: localización (mapa + AMCL), el puente en IDLE (no manda nada), RViz y la ventana de pose.
# Con R2+X el puente pasa a ACTIVE y entonces arrancan Nav2 (en el robot) y la terminal de aprobación (aquí).
# Una ventana con 6 paneles: base, puente, nav, aprobar, pose, rviz (clic para elegir panel).
# Cerrar todo: tmux kill-session -t laika4 (o kill-server); el puente manda StopMove al recibir SIGHUP.
set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT=unitree@192.168.123.18
S=laika4
ENV_R='source ~/laika_ros_env.sh && export PYTHONUNBUFFERED=1'
ENV_E="source $D/env_external.sh && export PYTHONUNBUFFERED=1"
# Pose de la marca de inicio (la de la caminata de la fase 2); si el robot está en otro lugar, corrígela en RViz.
MARCA='{header: {frame_id: map}, pose: {pose: {position: {x: 0.034, y: 0.027}, orientation: {z: -0.118, w: 0.993}}, covariance: [0.06,0,0,0,0,0, 0,0.06,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}'

SALIDA=/api/sport/request; EXTRA=''; MODO='ARMADO'
BASE=localization.launch.py; MAPA='mapa guardado (AMCL)'; NAVARGS=''
for a in "$@"; do
  case $a in
    --prueba) SALIDA=/laika/sport_request_dry; EXTRA='-p dry_run_force_active:=true'; MODO='PRUEBA (dry-run)' ;;
    --slam) BASE=mapping.launch.py; MAPA='mapa en vivo (slam_toolbox)'
            NAVARGS='planner_yaml:=/home/unitree/laika_ws/install/laika_slam/share/laika_slam/config/planner_slam.yaml' ;;
    *) echo "Opción desconocida: $a (usa --prueba y/o --slam)"; exit 1 ;;
  esac
done

tmux has-session -t $S 2>/dev/null && { echo "Ya existe la sesión $S. Entra con: tmux attach -t $S (o párala con scripts/laika_fase4_parar.sh)"; exit 1; }
ssh -o ConnectTimeout=5 $ROBOT true || { echo "No hay SSH al robot ($ROBOT)."; exit 1; }
# Regla de chrony: no lanzar si el robot no está sincronizado con la externa.
ssh $ROBOT 'chronyc tracking' | grep -q 'Reference ID.*192.168.123.50' || { echo "El robot no está sincronizado con la externa (chronyc tracking)."; exit 1; }
ssh $ROBOT 'tmux ls 2>/dev/null' | grep -q . && { echo "Hay sesiones tmux en el robot; ciérralas antes (ssh $ROBOT tmux ls)."; exit 1; }

# Espera a que el puente pase a ACTIVE (R2+X). En el robot (Foxy) y aquí (Humble).
ESPERA="until ros2 topic echo /laika/bridge_state 2>/dev/null | grep -m1 -q ACTIVE; do sleep 1; done"

# Todo en una sola ventana, en paneles con título. Cerrar tmux (kill-session/kill-server) corta los ssh:
# los procesos del robot reciben SIGHUP y el puente lo trata como Ctrl-C (manda StopMove).
P_BASE=$(tmux new-session -d -P -F '#{pane_id}' -s $S -n laika "ssh -t $ROBOT \"$ENV_R && ros2 launch laika_slam $BASE\"; read -p 'base terminó. Enter para cerrar'")
panel() {  # panel <título> <comando>
  local id; id=$(tmux split-window -P -F '#{pane_id}' -t $S:laika "$2")
  tmux select-pane -t "$id" -T "$1"; tmux select-layout -t $S:laika tiled > /dev/null; echo "$id"
}
tmux select-pane -t "$P_BASE" -T base
panel puente "ssh -t $ROBOT \"$ENV_R && ros2 run laika_slam cmd_vel_to_sport --ros-args -r sport_request_out:=$SALIDA $EXTRA\"; read -p 'puente terminó. Enter para cerrar'" > /dev/null
panel nav "ssh -t $ROBOT \"$ENV_R && echo 'Esperando R2+X para arrancar Nav2...' && $ESPERA && ros2 launch laika_slam nav2_servers.launch.py $NAVARGS\"; read -p 'nav terminó. Enter para cerrar'" > /dev/null
P_APR=$(panel aprobar "$ENV_E && echo 'Esperando R2+X...' && $ESPERA && echo 'Activo: este panel es el hombre muerto (Ctrl-C = parar).' && python3 $D/aprobar_rutas.py; read -p 'aprobar terminó. Enter para cerrar'")
if [ "$BASE" = mapping.launch.py ]; then  # con SLAM el mapa nace donde está el robot: no hay pose inicial
  panel pose "$ENV_E && python3 $D/ver_pose.py" > /dev/null
else
  panel pose "$ENV_E && sleep 20 && ros2 topic pub -t 3 -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"$MARCA\" > /dev/null && echo 'Pose inicial en la marca publicada.' ; python3 $D/ver_pose.py" > /dev/null
fi
panel rviz "$D/rviz_fase4.sh" > /dev/null
tmux set-option -t $S pane-border-status top > /dev/null
tmux set-option -t $S pane-border-format ' #{pane_title} ' > /dev/null
tmux set-option -t $S mouse on > /dev/null  # clic para elegir panel
tmux select-pane -t "$P_APR"
echo "Sesión $S lista: $MODO, $MAPA. Presiona R2+X para arrancar la navegación."
[ "$BASE" = mapping.launch.py ] && echo "Para guardar el mapa: scripts/guardar_mapa.sh <nombre> (antes de cerrar)"
echo "Para cerrar todo: tmux kill-session -t $S   (o scripts/laika_fase4_parar.sh)"
[ -n "$TMUX" ] && tmux switch-client -t $S || tmux attach -t $S
