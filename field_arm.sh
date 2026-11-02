#!/bin/bash
# 라운드 준비와 출발 (엔터 한 번 규칙용). 로봇 안 screen 'mission' 에서 돌아 접속이 끊겨도 진행됨.
#   field_arm.sh 1 [auto|right|left]   # 1임무 준비: 기반·스테이션 확인, 노드 켜고 '준비 끝' 까지 (바퀴 안 움직임). auto = 출발 직전 라이다로 방향 판단
#   field_arm.sh 2                # 2임무 준비
#   field_arm.sh go [1|2]         # 출발 (준비돼 있으면 약 0.25 초 뒤 움직임, 안 돼 있으면 처음부터 켜고 출발)
#   field_arm.sh recheck          # 1임무 대기 중: 지금 자리로 원점 다시 잡고 칸 판정 새로 모음 (버튼 출발이 출발 직전에 부름)
#   field_arm.sh stop             # 준비 취소 (기반·스테이션은 그대로)
#   field_arm.sh status           # 준비됐는지
ARM=/tmp/mission_armed; LOG=/tmp/mission_screen.log; W=~/mecanum_slam_ws
# 출발은 ROS 환경을 불러오지 않음 (env.sh 만 약 1 초): 준비돼 있으면 screen 키 하나로 끝
if [ "${1:-}" = go ]; then
  if [ -f $ARM ]; then
    M=$(cat $ARM); rm -f $ARM
    if [ "$M" = 1 ]; then screen -S mission -X stuff g; else touch /tmp/battle_go; fi
    echo "출발 보냄 ($M 임무)"; exit 0
  fi
  M=${2:-$(cat $W/.autostart 2>/dev/null)}     # 준비가 안 돼 있어도 경기이므로 멈추지 않음: 처음부터 켜고 바로 출발
  echo "준비 안 돼 있음 → 처음부터 시작 ($M 임무, 몇 초 더 걸림)"
  case "$M" in 1) exec $W/field_compete.sh auto;; 2) exec $W/field_battle.sh go;; *) echo "임무 번호 없음: field_arm.sh go 1"; exit 1;; esac
fi
if [ "${1:-}" = recheck ]; then
  [ "$(cat $ARM 2>/dev/null)" = 1 ] && screen -S mission -X stuff r
  exit 0
fi
source $W/field_common.sh
node_up() { for d in /proc/[0-9]*; do a1=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 2p); case "${a1##*/}" in mission1_node.py|battle_node.py) return 0;; esac; done; return 1; }
station_up() { python3 -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8090/status', timeout=1)" 2>/dev/null; }
CMD=${1:-}; SIDE=${2:-auto}
case "$CMD" in
  1|2)
    if node_up; then echo "주행 노드가 이미 켜져 있음: 끝내려면 field_stop.sh 또는 mission2/stop.sh"; exit 1; fi
    screen -S mission -X quit 2>/dev/null; rm -f $ARM $LOG /tmp/battle_go
    T0=$(date +%s.%N)
    wait_devices
    station_up || $W/start_station.sh
    if [ "$CMD" = 1 ]; then
      screen -dmS mission bash -c "MISSION_DIR=mission1_field $W/field_auto.sh --start-side $SIDE 2>&1 | tee $LOG"
    else
      screen -dmS mission bash -c "source $W/field_common.sh; ensure_base; $W/mission2/run_go.sh --wait-go 2>&1 | tee $LOG"
    fi
    for i in $(seq 1 240); do
      grep -q "준비 끝" $LOG 2>/dev/null && break
      screen -ls | grep -q '\.mission[[:space:]]' || { echo "임무가 준비 중 끝남:"; tail -8 $LOG; exit 1; }
      sleep 0.25
    done
    grep -q "준비 끝" $LOG 2>/dev/null || { echo "60 초 안에 준비 안 됨:"; tail -8 $LOG; exit 1; }
    echo "$CMD" > $ARM
    printf "준비 끝 (%.1f 초). 신호에 출발\n" "$(echo "$(date +%s.%N) - $T0" | bc)";;
  stop)
    kill_nodes() { for d in /proc/[0-9]*; do a1=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 2p); case "${a1##*/}" in mission1_node.py|battle_node.py) kill -$1 ${d#/proc/};; esac; done; }
    kill_nodes INT
    for i in 1 2 3 4 5 6 7 8; do node_up || break; sleep 0.25; done
    node_up && kill_nodes KILL      # 안 꺼지면 다음 준비가 막힘. 바퀴는 명령이 끊기면 펌웨어가 0.3 초 안에 멈춤
    curl -s -m 1 -o /dev/null "http://127.0.0.1:8090/do?a=abort"     # 임무가 맡긴 잡기가 돌고 있으면 그것도 멈춤
    screen -S mission -X quit 2>/dev/null; rm -f $ARM /tmp/battle_go; echo "준비 취소";;
  status) [ -f $ARM ] && echo "$(cat $ARM) 임무 준비됨" || echo "준비 안 됨"; node_up && echo "노드 켜짐"; tail -16 $LOG 2>/dev/null;;
  *) sed -n 3,8p "$0"; exit 1;;
esac
