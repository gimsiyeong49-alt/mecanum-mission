#!/bin/bash
# 키보드 칸 이동: w/s 앞뒤 1칸 · a/d 옆 1칸 · q/e 90° 회전 · 스페이스 비상정지 · g 계속 · x 종료
# 로봇을 (1,9) 칸 중심, 정면 = (1,8) 쪽으로 놓고 실행. 다른 키보드 조종(keyboard.sh)은 끌 것.
source ~/mecanum_slam_ws/env.sh
D=~/mecanum_slam_ws/mission1_field; mkdir -p $D/logs
python3 $D/mission1_node.py --teleop --log $D/logs/teleop_$(date +%m%d_%H%M%S).log --front 0.191 --back 0.161 --reverse-cost-go 1.3 --strafe-cost 1.6 --reverse-cost 1.05 --front-target-shift 0.02 --turn-plan table --turn-spot --grab-mode station --prealign 0 "$@"
