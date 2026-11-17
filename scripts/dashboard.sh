#!/bin/bash
# Autonomy Dashboard (paquete de ~/RRL-2026-Robot) con los tópicos de Laika. Corre en la Jetson externa.
# Necesita que en el robot corra la localización o el SLAM (scripts/laika_fase4.sh) para tener /map y la TF map ← odom.
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$D/env_external.sh"
source /home/robotica/RRL-2026-Robot/install/setup.bash
mkdir -p "$D/../maps/dashboard"
exec ros2 run autonomy_dashboard_pkg autonomy_dashboard --ros-args --params-file "$D/../config/dashboard_laika.yaml"
