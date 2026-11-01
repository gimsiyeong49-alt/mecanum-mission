#!/bin/bash
# ② 1임무 자율 주행: 위치추정(오돔+격자 벽 보정, SLAM 없음) 켜기 → (1,9) → (5,5) 큐브 앞 → (1,9)
# 실행 전: 로봇을 (1,9) 칸 중심, 정면(라이다) = (1,8) 쪽으로 놓을 것.
#   ssh -t uoeee@<IP> '~/mecanum_slam_ws/field_auto.sh --dry'   # 먼저: 바퀴 안 움직이고 칸 판정·계획만 (x 로 끝)
#   ssh -t uoeee@<IP> '~/mecanum_slam_ws/field_auto.sh'         # 실제 주행 (스페이스 비상정지, g 계속, x 종료)
#   그 밖의 옵션은 그대로 노드로 넘어감: 예) --max-v 0.15
source ~/mecanum_slam_ws/field_common.sh
DRY=0; ARGS=()
for a in "$@"; do [ "$a" = "--dry" ] && DRY=1 || ARGS+=("$a"); done
check_no_keyboard
ensure_base
if [ $DRY = 1 ]; then
  echo "[임무] dry-run 시작 (바퀴 안 움직임, 끝내려면 x)"
  $W/${MISSION_DIR:-mission1}/run_dryrun.sh "${ARGS[@]}"
  K=dryrun
else
  echo "[임무] 자율 주행 시작. 스페이스 = 비상정지"

  $W/${MISSION_DIR:-mission1}/run_go.sh "${ARGS[@]}"
  K=run
fi
J=$(ls -t $W/${MISSION_DIR:-mission1}/logs/${K}_*.json 2>/dev/null | head -1)
echo "[끝] 기록: $J"
[ -n "$J" ] && python3 $W/${MISSION_DIR:-mission1}/draw_run.py "$J" --truth rulebook 2>/dev/null | sed 's/^/  그림: /'
ask_stop_slam
