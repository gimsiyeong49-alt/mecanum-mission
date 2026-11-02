#!/bin/bash
# 2임무 실제 주행. 기반·잡기 스테이션이 켜져 있어야 함 (field_battle.sh 가 확인). Ctrl+C 로 바퀴 멈춤, 그리퍼는 그대로.
source ~/mecanum_slam_ws/env.sh
D=~/mecanum_slam_ws/mission2; mkdir -p $D/logs
python3 $D/battle_node.py --execute --log $D/logs/battle_$(date +%m%d_%H%M%S).log "$@"
