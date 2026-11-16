#!/bin/bash
# Uso: scripts/traer_pcd.sh <nombre>  -> maps/<nombre>.pcd
[ -n "$1" ] || { echo "Uso: $0 <nombre>" >&2; exit 1; }
D="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$D/maps"
exec scp unitree@192.168.123.18:laika_ws/src/point_lio_unilidar/PCD/scans.pcd "$D/maps/$1.pcd"
