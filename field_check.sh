#!/bin/bash
# 대회 전 점검: 바퀴 안 움직임, 아무것도 안 바꿈.  ~/mecanum_slam_ws/field_check.sh
source ~/mecanum_slam_ws/field_common.sh
ok() { printf "  %-16s %s\n" "$1" "$2"; }
echo "[통신]"
[ -f $W/.dds_local ] && ok "ROS 방식" "local (로봇 안에서만) ← 대회용" || ok "ROS 방식" "wifi (대회 전 field_dds.sh local 권장)"
ok "와이파이" "$(nmcli -t -f GENERAL.CONNECTION device show wlP1p1s0 2>/dev/null | cut -d: -f2-) / $(ip -4 -o addr show wlP1p1s0 2>/dev/null | awk '{print $4}')"
ok "ROS 환경" "LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-?} DOMAIN=${ROS_DOMAIN_ID:-?} 묶임=${MECANUM_DDS_IFS:-?}"
echo "[실행 상태]"
slam_running && ok "기반" "켜짐" || ok "기반" "꺼짐"
python3 -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8090/status', timeout=2)" 2>/dev/null && ok "잡기 스테이션" "켜짐" || ok "잡기 스테이션" "꺼짐"
screen -ls 2>/dev/null | grep -q '\.mission[[:space:]]' && ok "임무 화면" "있음 (끝난 임무면 지우기: screen -S mission -X quit)" || ok "임무 화면" "없음"
for d in /proc/[0-9]*; do a1=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 2p); case "${a1##*/}" in mission1_node.py|battle_node.py|mecanum_keyboard) ok "주행 노드" "켜짐 PID ${d#/proc/} ${a1##*/}";; esac; done
echo "[장치]"
for p in /dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0 /dev/serial/by-path/platform-3610000.usb-usb-0:2.3:1.0-port0 /dev/serial/by-id/usb-Silicon_Labs_CP2102*; do [ -e "$p" ] && ok "있음" "${p##*/}" || ok "없음" "$p"; done
ok "카메라" "$(ls /dev/v4l/by-path/ 2>/dev/null | grep -c video-index0) 대 (2 대여야 함)"
ok "디스크 여유" "$(df -h ~ | awk 'NR==2{print $4}')"
ok "온도" "$(cat /sys/devices/virtual/thermal/thermal_zone0/temp 2>/dev/null | awk '{printf "%.0f°C", $1/1000}')"
echo "[코드]"
(cd $W && python3 - <<'P'
import hashlib, os
bad, n = [], 0
for l in open("MANIFEST_robot_md5.txt", encoding="utf-8"):
    p = l.split()
    if len(p) >= 2 and os.path.exists(p[-1]):
        n += 1
        if hashlib.md5(open(p[-1], "rb").read()).hexdigest() != p[0]: bad.append(p[-1])
print("  매니페스트 %d 개 중 다른 파일: %s (env.sh, field_compete.sh, mission1_field/mission1_node.py 뿐이어야 함)" % (n, bad))
P
)
ok "라이다 TF" "$(grep -A0 -n 'lidar_x' $W/src/mecanum_slam/launch/robot.launch.py | grep -o 'default_value="[0-9.]*"' | head -1)"
if slam_running; then echo "[위치]"; python3 $W/tools/loc_check.py --once; fi
if slam_running; then echo "[토픽 6 초]"; python3 $W/tools/topic_watch.py 6 | tail -7; fi
