#!/bin/bash
# Autonomy Dashboard (copia en ws_externa/, adaptada de ~/RRL-2026-Robot) con los tópicos de Laika. Corre en la Jetson externa.
# Necesita que en el robot corra la localización o el SLAM (scripts/laika_fase4.sh) para tener /map y la TF map ← odom.
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$D/env_external.sh"
source "$D/../ws_externa/install/setup.bash"  # copia de Laika del dashboard (no la de RRL)
mkdir -p "$D/../maps/dashboard"
exec ros2 run autonomy_dashboard_pkg autonomy_dashboard --ros-args --params-file "$D/../config/dashboard_laika.yaml"
