#!/bin/bash
# 대회용 시작: 명령 한 번 → 로봇 안(screen 'mission')에서 현장판 자율 주행. 출발 키 없이 라이다 판정(1 초) 뒤 바로 출발.
# 로봇 안에서 돌므로 노트북·폰 연결이 끊겨도 끝까지 감 (10-30: ssh 안에서 돌던 임무가 와이파이 끊김으로 같이 꺼짐).
#   ssh uoeee@<IP> '~/mecanum_slam_ws/field_compete.sh right'
#   right = 출발 칸 옆 바깥벽이 로봇 오른쪽 ((1,9) 남쪽 보기 = (9,1) 북쪽 보기)
#   left  = 옆 바깥벽이 로봇 왼쪽 ((1,9) 동쪽 보기 = (9,1) 서쪽 보기)
# 라이다 판단과 지정이 다르면 기록에 '주의' 가 남고 지정한 쪽으로 감.
# 기반(구동·센서·위치추정)이 꺼져 있으면 field_auto 가 먼저 켬(약 15 초) → 대회 전 준비 시간에 미리 켜 둘 것 (field_base.sh).
SIDE=${1:-}
case "$SIDE" in right|left|auto) ;; *) echo "사용: field_compete.sh right|left|auto   (출발 칸 옆 바깥벽이 로봇 오른쪽/왼쪽, auto = 라이다로 판단)"; exit 1;; esac
if screen -ls | grep -q '\.mission[[:space:]]'; then
  echo "이미 임무 화면(screen mission)이 있음 — 끝난 임무면 'screen -S mission -X quit' 뒤 다시"; exit 1
fi
# 잡기 스테이션이 꺼져 있으면 켬 (재부팅 뒤 바로 돌리면 큐브 앞에서 2 번 실패하고 멈췄음).
# 켜지길 기다리지 않음: 큐브에 닿을 때까지 30 초 넘게 걸리고, 그동안 출발을 늦출 이유가 없음
python3 -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8090/status', timeout=1)" 2>/dev/null || ~/mecanum_slam_ws/start_station.sh
screen -dmS mission bash -c "MISSION_DIR=mission1_field ~/mecanum_slam_ws/field_auto.sh --no-wait-key --start-side $SIDE 2>&1 | tee /tmp/mission_screen.log"
sleep 0.5
if screen -ls | grep -q '\.mission[[:space:]]'; then
  echo "시작함 (옆 바깥벽 $SIDE). 진행: tail -f /tmp/mission_screen.log · 기록: ~/mecanum_slam_ws/mission1_field/logs/"
else
  echo "시작 실패 — /tmp/mission_screen.log 확인"; exit 1
fi
