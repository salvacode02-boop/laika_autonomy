#!/bin/bash
# Detiene la sesión de scripts/laika_fase4.sh en orden: primero el puente (manda StopMove al cerrar), luego lo demás.
# Con --sin-cable también para la sesión remota laika4r (Ctrl-C al puente, espera, kill-session).
S=laika4
R=unitree@192.168.123.18
# Modo sin cable: Ctrl-C al panel del puente remoto (StopMove si estaba en RUN), luego Nav2/base, luego la sesión.
if ssh -o ConnectTimeout=5 $R 'tmux has-session -t laika4r 2>/dev/null'; then
  ssh $R 'tmux send-keys -t laika4r:puente C-c; sleep 2; tmux send-keys -t laika4r:nav C-c; sleep 3; tmux send-keys -t laika4r:base C-c; sleep 3; tmux kill-session -t laika4r'
  echo "Sesión remota laika4r detenida."
fi
if tmux has-session -t $S 2>/dev/null; then
  for w in puente aprobar nav lio base pose rviz; do
    id=$(tmux list-panes -t $S -F '#{pane_id} #{pane_title}' | awk -v w=$w '$2==w{print $1}')
    [ -n "$id" ] && tmux send-keys -t "$id" C-c; sleep 1
  done
  sleep 3
  tmux kill-session -t $S
else
  echo "No hay sesión $S."
fi
# Por si algo quedó vivo en el robot (ssh cortado): mismo orden.
ssh $R 'pkill -INT -f "lib/laika_slam/[c]md_vel_to_sport"; sleep 2; pkill -INT -f "ros2 launch [l]aika_slam"; sleep 3; pgrep -af "[l]aika_slam|[n]av2_|[a]mcl|[f]astlio" | grep -v "pgrep -af" || echo "Robot limpio."'
