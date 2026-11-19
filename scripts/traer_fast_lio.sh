#!/bin/bash
# Trae FAST_LIO_ROS2 (rama ros2) a ~/laika_vendor, aplica el parche de Foxy y lo copia al robot (sin internet).
set -euo pipefail
SHA=2fffc570a25d0df172720bac034fbdb6a13d2162   # HEAD de la rama ros2 al 2026-11-19
DIR=~/laika_vendor/FAST_LIO_ROS2
ROBOT=unitree@192.168.123.18
REPO=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p ~/laika_vendor
[ -d "$DIR" ] || git clone --recursive -b ros2 https://github.com/Ericsii/FAST_LIO_ROS2.git "$DIR"
cd "$DIR"
git checkout -q -f "$SHA" && git submodule update --init --recursive
[ -f "$REPO/patches/fast_lio_foxy.patch" ] && git apply "$REPO/patches/fast_lio_foxy.patch"
rsync -a --delete --exclude .git --exclude 'include/ikd-Tree/.git' "$DIR/" "$ROBOT:laika_ws/src/FAST_LIO_ROS2/"
rsync -a --delete "$REPO/ros2_ws/src/livox_ros_driver2/" "$ROBOT:laika_ws/src/livox_ros_driver2/"
echo "OK: FAST_LIO_ROS2 @ $SHA + livox_ros_driver2 copiados al robot"
