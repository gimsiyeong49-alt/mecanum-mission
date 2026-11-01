#!/bin/bash
# 메카넘 지도 만들기 시작 (하위 구동 + IMU + EKF + 라이다 + Cartographer). Ctrl+C 로 종료.
source ~/mecanum_slam_ws/env.sh
ros2 launch mecanum_slam mapping.launch.py lidar_yaw:=1.641 "$@"
