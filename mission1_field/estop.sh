#!/bin/bash
# 비상정지(다른 터미널용): 주행 노드를 끄고 영속도 명령을 보낸다.
# 노드가 꺼지면 0.3 초 뒤 컨트롤러·펌웨어 timeout 으로 모터 출력도 꺼진다 (브레이크가 아니라 바퀴가 풀림).
#   ssh uoeee@<IP> '~/mecanum_slam_ws/mission1/estop.sh'          # 주행 노드만
#   ssh uoeee@<IP> '~/mecanum_slam_ws/mission1/estop.sh all'      # 키보드 조종(mecanum_keyboard)까지
pids=""
for d in /proc/[0-9]*; do
  a0=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 1p); a1=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 2p)
  case "${a0##*/}:${a1##*/}" in
    python3:mission1_node.py) pids="$pids ${d#/proc/}";;
    python3:mecanum_keyboard) [ "$1" = "all" ] && pids="$pids ${d#/proc/}";;
  esac
done
if [ -n "$pids" ]; then echo "정지 신호 → PID$pids"; kill -INT $pids 2>/dev/null; sleep 1; kill -KILL $pids 2>/dev/null; else echo "실행 중인 주행 노드 없음"; fi
source ~/mecanum_slam_ws/env.sh
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
echo "영속도 명령 보냄. 바퀴가 멈췄는지 눈으로 확인할 것."
