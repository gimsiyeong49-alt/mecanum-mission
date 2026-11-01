#!/bin/bash
# 라이다만 (구동·위치추정 없음) + base_link→laser TF, ROS 통신은 USB-C(l4tbr0) — 노트북 RViz 확인용 (10-21)
source ~/mecanum_slam_ws/env.sh
export ROS_LOCALHOST_ONLY=0 CYCLONEDDS_URI=file://$HOME/mecanum_slam_ws/cyclonedds_usb_lidar.xml
ros2 run tf2_ros static_transform_publisher --x 0.11 --y 0 --z 0.16 --yaw 1.641 --frame-id base_link --child-frame-id laser &
TFP=$!
trap "kill $TFP" EXIT
ros2 run sllidar_ros2 sllidar_node --ros-args -p channel_type:=serial -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_c4efdaad6cdf46439fd83d2dd53ef425-if00-port0 -p serial_baudrate:=256000 -p frame_id:=laser -p inverted:=false -p angle_compensate:=true -p scan_mode:=${LIDAR_MODE:-Boost}
