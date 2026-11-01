#!/bin/bash
# ③ 전부 끄기: 주행 노드·키보드 조종 강제 종료 + 영속도 + 위치추정·구동 끄기. 다른 터미널에서 비상용으로도 씀.
#   ssh uoeee@<IP> '~/mecanum_slam_ws/field_stop.sh'
source ~/mecanum_slam_ws/field_common.sh
$W/mission1/estop.sh all
stop_slam
for d in /proc/[0-9]*; do
  c=$(tr '\0' ' ' < $d/cmdline 2>/dev/null)
  case "$c" in *"ros2 launch mecanum_slam"*|*sllidar_node*|*cartographer_node*|*ros2_control_node*|*grid_localizer.py*) echo "  남은 프로세스 ${d#/proc/}: ${c:0:80}";; esac
done
echo "끝. 남은 프로세스가 위에 없으면 다 꺼진 것."
