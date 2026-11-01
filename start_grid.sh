#!/bin/bash
# 위치추정 (SLAM 없음): 구동+IMU+EKF+라이다+laser TF  +  격자 맞춤 위치추정(map→odom). Ctrl+C 로 둘 다 종료.
source ~/mecanum_slam_ws/env.sh
ros2 launch mecanum_slam robot.launch.py &
L=$!
sleep 6                                    # EKF(odom→base_link)·라이다가 뜰 때까지
python3 ${LOCALIZER:-$HOME/mecanum_slam_ws/mission1/grid_localizer.py} "$@" &
G=$!
trap 'kill -INT $G $L 2>/dev/null; wait' INT TERM
wait
