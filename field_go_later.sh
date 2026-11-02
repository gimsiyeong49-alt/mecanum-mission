#!/bin/bash
# 선 뽑고 출발 (버튼 달기 전 시험용): 준비돼 있는 임무를 N 초 뒤에 출발시킴. 그동안 USB-C 를 뽑고 로봇에서 물러남.
# 1임무는 버튼과 같은 순서: 출발 0.5 초 전에 지금 자리로 원점 다시 잡기.
#   ~/mecanum_slam_ws/field_arm.sh 1 && ~/mecanum_slam_ws/field_go_later.sh 15
N=${1:-15}; W=~/mecanum_slam_ws
[ -f /tmp/mission_armed ] || { echo "준비된 임무 없음: 먼저 field_arm.sh 1 또는 field_arm.sh 2"; exit 1; }
screen -dmS later bash -c "sleep $N; $W/field_arm.sh recheck; sleep 0.5; $W/field_arm.sh go"
echo "$(cat /tmp/mission_armed) 임무: $N 초 뒤 출발. 지금 선을 뽑으세요. 취소: screen -S later -X quit"
