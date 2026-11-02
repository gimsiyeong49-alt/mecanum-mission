#!/bin/bash
# 2임무 dry-run: 바퀴·그리퍼 안 움직임. 1 초마다 위치·라이다·큐브를 찍음. Ctrl+C 로 끝.
source ~/mecanum_slam_ws/env.sh
D=~/mecanum_slam_ws/mission2; mkdir -p $D/logs
python3 $D/battle_node.py --log $D/logs/dryrun_$(date +%m%d_%H%M%S).log "$@"
