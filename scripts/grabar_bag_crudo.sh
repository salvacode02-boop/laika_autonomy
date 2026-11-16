#!/bin/bash
# Uso: ~/grabar_bag_crudo.sh [nombre]   (cloud + imu + odom crudos del L1, para Point-LIO)
source ~/laika_ros_env.sh
mkdir -p ~/laika_bags
exec ros2 bag record -o ~/laika_bags/${1:-crudo-$(date +%Y%m%d-%H%M%S)} \
  /utlidar/cloud /utlidar/imu /utlidar/robot_odom
