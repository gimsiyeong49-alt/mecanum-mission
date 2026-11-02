#!/bin/bash
# 현장판(10-29): 실험판 + 새 몸 치수 + 회전 표 길찾기 + 비켜서 돌기 + 정면 멈춤 잡기(잡기 스테이션). 시뮬레이션·가상 시험 통과 판.
# 1임무 실제 주행: (1,9) → (5,5) → (1,9). 키보드 조종은 먼저 끌 것. Ctrl+C 로 즉시 정지.
source ~/mecanum_slam_ws/env.sh
D=~/mecanum_slam_ws/mission1_field; mkdir -p $D/logs
python3 $D/mission1_node.py --execute --log $D/logs/run_$(date +%m%d_%H%M%S).log --front 0.191 --back 0.161 --reverse-cost-go 1.3 --strafe-cost 1.6 --reverse-cost 1.05 --front-target-shift 0 --wall-prealign 0.015 --turn-plan table --turn-spot --grab-mode station --release-at-home --prealign 0 --symmetric --turn-known-r 1 "$@"
