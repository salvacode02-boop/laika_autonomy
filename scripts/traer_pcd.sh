#!/bin/bash
# Uso: scripts/traer_pcd.sh <nombre> [ruta_origen_en_robot]  -> maps/<nombre>.pcd
[ -n "$1" ] || { echo "Uso: $0 <nombre> [ruta_en_robot]" >&2; exit 1; }
D="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$D/maps"
exec scp "unitree@192.168.123.18:${2:-laika_maps/map3d.pcd}" "$D/maps/$1.pcd"
