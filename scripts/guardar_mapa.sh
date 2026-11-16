#!/bin/bash
# Uso: scripts/guardar_mapa.sh [nombre]  -> maps/<nombre>.pgm + .yaml
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$D/env_external.sh"
mkdir -p "$D/../maps"
ros2 run nav2_map_server map_saver_cli -f "$D/../maps/${1:-mapa}"
