#!/bin/bash
# 잡기 스테이션 켜기 (카메라 + 큐브 잡기, 웹 :8090). 임무가 큐브 앞에서 여기에 잡기를 맡긴다.
# field_auto.sh / field_compete.sh 는 이걸 켜 주지 않으므로 임무 전에 한 번 켤 것.
#   ssh uoeee@<IP> '~/mecanum_slam_ws/start_station.sh'
if screen -ls 2>/dev/null | grep -q "\.station[[:space:]]"; then
  echo "잡기 스테이션이 이미 켜져 있음"; exit 0
fi
screen -dmS station bash -c 'source ~/mecanum_slam_ws/env.sh >/dev/null 2>&1; cd ~/mecanum_slam_ws/calib && python3 cube_station.py > /tmp/cube_station.log 2>&1'
sleep 3
if curl -s -m 3 http://127.0.0.1:8090/status > /dev/null; then echo "잡기 스테이션 켜짐"; else echo "응답 없음 (아직 뜨는 중일 수 있음) — /tmp/cube_station.log 확인"; fi
