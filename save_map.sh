#!/bin/bash
# 지금까지 만든 지도 저장: ./save_map.sh 이름  → ~/mecanum_slam_ws/maps/이름.{pgm,yaml,pbstream}
source ~/mecanum_slam_ws/env.sh
NAME=${1:-map_$(date +%m%d_%H%M%S)}; D=~/mecanum_slam_ws/maps; mkdir -p $D
if [ -e "$D/$NAME.yaml" ]; then echo "이미 있음: $D/$NAME.yaml (다른 이름 쓰기)"; exit 1; fi
ros2 run nav2_map_server map_saver_cli -f "$D/$NAME" --ros-args -p map_subscribe_transient_local:=true
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: /.pbstream, include_unfinished_submaps: true}"
ls -l $D/$NAME.*
