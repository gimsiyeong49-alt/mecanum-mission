#!/bin/bash
# 1임무 dry-run: 바퀴 안 움직임. 칸 판정·계획만 출력. 로봇을 (1,9) 칸 중심에 남쪽(정면=(1,8) 쪽)으로 놓고 실행.
source ~/mecanum_slam_ws/env.sh
D=~/mecanum_slam_ws/mission1_field; mkdir -p $D/logs
python3 $D/mission1_node.py --log $D/logs/dryrun_$(date +%m%d_%H%M%S).log --front 0.191 --back 0.161 --reverse-cost-go 1.3 --strafe-cost 1.6 --reverse-cost 1.05 --front-target-shift 0.02 --turn-plan table --turn-spot --grab-mode station --prealign 0 "$@"
