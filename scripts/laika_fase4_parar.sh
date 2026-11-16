#!/bin/bash
# Detiene la sesión de scripts/laika_fase4.sh en orden: primero el puente (manda StopMove al cerrar), luego lo demás.
S=laika4
tmux has-session -t $S 2>/dev/null || { echo "No hay sesión $S."; exit 0; }
for w in puente aprobar nav base pose rviz; do
  id=$(tmux list-panes -t $S -F '#{pane_id} #{pane_title}' | awk -v w=$w '$2==w{print $1}')
  [ -n "$id" ] && tmux send-keys -t "$id" C-c; sleep 1
done
sleep 3
tmux kill-session -t $S
# Por si algo quedó vivo en el robot (ssh cortado): mismo orden.
ssh unitree@192.168.123.18 'pkill -INT -f "lib/laika_slam/[c]md_vel_to_sport"; sleep 2; pkill -INT -f "ros2 launch [l]aika_slam"; sleep 3; pgrep -af "[l]aika_slam|[n]av2_|[a]mcl" | grep -v "pgrep -af" || echo "Robot limpio."'
