# Uso: source scripts/env_external.sh  (Jetson externa, ROS 2 Humble)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP8p1s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
