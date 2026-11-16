#!/bin/bash
# Uso: ~/grabar_bag.sh [nombre]   (cloud_deskewed ~5 MB/s)
source ~/laika_ros_env.sh
mkdir -p ~/laika_bags
exec ros2 bag record -o ~/laika_bags/${1:-$(date +%Y%m%d-%H%M%S)} \
  /utlidar/cloud_deskewed /utlidar/robot_odom /scan /tf /tf_static /map
