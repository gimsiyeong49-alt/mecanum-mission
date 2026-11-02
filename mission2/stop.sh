#!/bin/bash
# 비상정지: 2임무 노드를 끄고 영속도를 보냄. 그리퍼는 그대로 (쥔 큐브를 놓지 않음).
pids=""
for d in /proc/[0-9]*; do
  a1=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 2p)
  case "$a1" in */battle_node.py) pids="$pids ${d#/proc/}";; esac
done
if [ -n "$pids" ]; then echo "정지 신호 → PID$pids"; kill -INT $pids 2>/dev/null; sleep 1; kill -KILL $pids 2>/dev/null; else echo "실행 중인 2임무 노드 없음"; fi
screen -S mission -X quit 2>/dev/null
source ~/mecanum_slam_ws/env.sh
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
echo "영속도 명령 보냄. 바퀴가 멈췄는지 눈으로 확인할 것."
