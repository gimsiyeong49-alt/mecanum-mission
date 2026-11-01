#!/bin/bash
# ① 키보드 칸 이동 시험: 위치추정(오돔+격자 벽 보정, SLAM 없음) 켜기 → 키보드 칸 이동 (w/s/a/d 1칸, q/e 90°, 스페이스 비상정지, g 계속, x 종료)
# 실행 전: 로봇을 (1,9) 칸 중심, 정면(라이다) = (1,8) 쪽으로 놓을 것.
#   ssh -t uoeee@<IP> '~/mecanum_slam_ws/field_keyboard.sh'                 # 기본
#   ssh -t uoeee@<IP> '~/mecanum_slam_ws/field_keyboard.sh --max-v 0.15'    # 느리게
source ~/mecanum_slam_ws/field_common.sh
check_no_keyboard
ensure_base
echo "[임무] 키보드 칸 이동 시작 (끝내려면 x)"
$W/${MISSION_DIR:-mission1}/run_teleop.sh "$@"
echo "[끝] 키보드 칸 이동 끝. 기록: $(ls -t $W/${MISSION_DIR:-mission1}/logs/teleop_*.json 2>/dev/null | head -1)"
ask_stop_slam
