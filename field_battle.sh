#!/bin/bash
# 2임무(배틀럼블) 시작. 출발 칸은 묻지 않음: 네 칸 모두 중심이 왼쪽 앞 45° 라 같은 동작 (규정 그림 3 방향으로 놓을 것).
#   ssh uoeee@<IP> '~/mecanum_slam_ws/field_battle.sh ready'    # 준비 시간에: 기반·잡기 스테이션 켜고 확인 (약 30 초)
#   ssh -t uoeee@<IP> '~/mecanum_slam_ws/field_battle.sh dry'   # 바퀴 안 움직임: 위치·라이다·큐브 확인 (Ctrl+C 로 끝)
#   ssh uoeee@<IP> '~/mecanum_slam_ws/field_battle.sh go'       # 심판 시작 신호에: 로봇 안(screen 'mission')에서 출발
# 기반이 미리 켜져 있으면 go 뒤 약 3 초에 출발, 꺼져 있으면 기반이 켜지는 시간만큼 늦음 (field_autostart.sh 참고).
# 끄기: ~/mecanum_slam_ws/mission2/stop.sh (그리퍼는 그대로) · 진행: tail -f /tmp/mission_screen.log
# go 뒤의 인수는 노드로 넘어감: 예) --start-mode turn45 --no-wall-fix
source ~/mecanum_slam_ws/field_common.sh
LIDAR_WANT=Sensitivity     # 2임무는 벽에 바짝 붙지 않고 먼 벽을 봄
CMD=${1:-}; shift
station_up() { python3 -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8090/status', timeout=2)" 2>/dev/null; }
ready() {
  check_no_keyboard
  ensure_base
  station_up || { echo "[잡기 스테이션] 켜는 중"; $W/start_station.sh; for i in $(seq 1 30); do station_up && break; sleep 1; done; }
  station_up && echo "[잡기 스테이션] 켜짐" || { echo "잡기 스테이션이 안 켜짐: /tmp/cube_station.log 확인"; exit 1; }
}
case "$CMD" in
  ready) ready; echo "준비 끝. 출발 칸에 놓고 심판 신호에 go";;
  dry)   ready; $W/mission2/run_dry.sh "$@";;
  go)
    if screen -ls | grep -q '\.mission[[:space:]]'; then echo "이미 임무 화면이 있음: ~/mecanum_slam_ws/mission2/stop.sh 뒤 다시"; exit 1; fi
    # 기반이 꺼져 있어도 거부하지 않음 (경기 중이므로): 스테이션을 켜고 기반을 기다린 뒤 바로 출발. 전부 로봇 안에서 하므로 접속이 끊겨도 진행됨
    station_up || $W/start_station.sh
    screen -dmS mission bash -c "source $W/field_common.sh; LIDAR_WANT=Sensitivity; ensure_base; $W/mission2/run_go.sh $* 2>&1 | tee /tmp/mission_screen.log"
    sleep 0.5
    screen -ls | grep -q '\.mission[[:space:]]' && echo "출발함. 기록: ~/mecanum_slam_ws/mission2/logs/" || { echo "시작 실패: /tmp/mission_screen.log"; exit 1; };;
  *) echo "사용: field_battle.sh ready | dry | go"; exit 1;;
esac
