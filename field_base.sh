#!/bin/bash
# ⓪ 기반 켜기 (준비): 구동·IMU·EKF·라이다 + 격자 위치추정. 바퀴는 안 움직임. 한 번 켜 두면 임무(①②)가 2~3 초 만에 시작.
#   ssh uoeee@<IP> '~/mecanum_slam_ws/field_base.sh'
source ~/mecanum_slam_ws/field_common.sh
ensure_base
source $W/env.sh
timeout 4 ros2 topic echo --once /grid_loc/status 2>/dev/null | sed -n 1p | sed 's/^data: /[기반] 상태: /'
echo "[기반] 준비 끝. 이제 키보드시험 / 자율확인 / 자율주행 을 켜면 됩니다 (로봇은 (1,9) 칸 중심, 정면 (1,8))."
