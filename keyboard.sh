#!/bin/bash
# 키보드 조종 (w/x 앞뒤, a/d 옆, r/f 회전, s 정지, + / - 속도)
source ~/mecanum_slam_ws/env.sh
ros2 run mecanum_bringup mecanum_keyboard
