#!/bin/bash
# Uso: ~/reproducir_bag.sh <nombre> [args de ros2 bag play, p. ej. -r 0.5]
# Remapea a /bag/... para no chocar con los tópicos en vivo (dominio 0).
[ -n "$1" ] || { echo "Uso: $0 <nombre-bag> [args extra de ros2 bag play]" >&2; exit 1; }
source ~/laika_ros_env.sh
bag=$1; shift
exec ros2 bag play ~/laika_bags/$bag "$@" \
  --remap /utlidar/cloud:=/bag/utlidar/cloud /utlidar/imu:=/bag/utlidar/imu /utlidar/robot_odom:=/bag/utlidar/robot_odom
