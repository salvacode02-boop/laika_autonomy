#!/bin/bash
# Fase 4: arranca todo en tmux (sesión "laika4"). Corre en la Jetson externa.
#
#   scripts/laika_fase4.sh           ARMADO: el puente publica en /api/sport/request (solo con aprobación del usuario)
#   scripts/laika_fase4.sh --prueba  dry-run: el puente publica en /laika/sport_request_dry y se activa solo
#   scripts/laika_fase4_parar.sh     detiene todo en orden
#
# Al inicio solo corren: localización (mapa + AMCL), el puente en IDLE (no manda nada), RViz y la ventana de pose.
# Con R2+X el puente pasa a ACTIVE y entonces arrancan Nav2 (en el robot) y la terminal de aprobación (aquí).
# Ventanas: base, puente, nav, aprobar, pose, rviz.  Cambiar de ventana: Ctrl-b y el número, o Ctrl-b n.
set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT=unitree@192.168.123.18
S=laika4
ENV_R='source ~/laika_ros_env.sh && export PYTHONUNBUFFERED=1'
ENV_E="source $D/env_external.sh && export PYTHONUNBUFFERED=1"
# Pose de la marca de inicio (la de la caminata de la fase 2); si el robot está en otro lugar, corrígela en RViz.
MARCA='{header: {frame_id: map}, pose: {pose: {position: {x: 0.034, y: 0.027}, orientation: {z: -0.118, w: 0.993}}, covariance: [0.06,0,0,0,0,0, 0,0.06,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}'

if [ "$1" = "--prueba" ]; then
  SALIDA=/laika/sport_request_dry; EXTRA='-p dry_run_force_active:=true'; MODO='PRUEBA (dry-run)'
else
  SALIDA=/api/sport/request; EXTRA=''; MODO='ARMADO'
fi

tmux has-session -t $S 2>/dev/null && { echo "Ya existe la sesión $S. Entra con: tmux attach -t $S (o párala con scripts/laika_fase4_parar.sh)"; exit 1; }
ssh -o ConnectTimeout=5 $ROBOT true || { echo "No hay SSH al robot ($ROBOT)."; exit 1; }
# Regla de chrony: no lanzar si el robot no está sincronizado con la externa.
ssh $ROBOT 'chronyc tracking' | grep -q 'Reference ID.*192.168.123.50' || { echo "El robot no está sincronizado con la externa (chronyc tracking)."; exit 1; }
ssh $ROBOT 'tmux ls 2>/dev/null' | grep -q . && { echo "Hay sesiones tmux en el robot; ciérralas antes (ssh $ROBOT tmux ls)."; exit 1; }

# Espera a que el puente pase a ACTIVE (R2+X). En el robot (Foxy) y aquí (Humble).
ESPERA="until ros2 topic echo /laika/bridge_state 2>/dev/null | grep -m1 -q ACTIVE; do sleep 1; done"

tmux new-session -d -s $S -n base "ssh -t $ROBOT \"$ENV_R && ros2 launch laika_slam localization.launch.py\"; read -p 'base terminó. Enter para cerrar'"
tmux new-window -t $S -n puente "ssh -t $ROBOT \"$ENV_R && ros2 run laika_slam cmd_vel_to_sport --ros-args -r sport_request_out:=$SALIDA $EXTRA\"; read -p 'puente terminó. Enter para cerrar'"
tmux new-window -t $S -n nav "ssh -t $ROBOT \"$ENV_R && echo 'Esperando R2+X para arrancar Nav2...' && $ESPERA && ros2 launch laika_slam nav2_servers.launch.py\"; read -p 'nav terminó. Enter para cerrar'"
tmux new-window -t $S -n aprobar "$ENV_E && echo 'Esperando R2+X...' && $ESPERA && echo 'Activo: esta terminal es el hombre muerto (Ctrl-C = parar).' && python3 $D/aprobar_rutas.py; read -p 'aprobar terminó. Enter para cerrar'"
tmux new-window -t $S -n pose "$ENV_E && sleep 20 && ros2 topic pub -t 3 -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"$MARCA\" > /dev/null && echo 'Pose inicial en la marca publicada.' ; python3 $D/ver_pose.py"
tmux new-window -t $S -n rviz "$D/rviz_fase4.sh"
tmux select-window -t $S:aprobar
echo "Sesión $S lista en modo $MODO. Presiona R2+X para arrancar la navegación."
[ -n "$TMUX" ] && tmux switch-client -t $S || tmux attach -t $S
