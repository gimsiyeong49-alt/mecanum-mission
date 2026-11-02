#!/bin/bash
# 로봇 전원을 켤 때 할 일 정하기 (이 사용자 crontab 한 줄만 건드림, sudo 안 씀)
#   field_autostart.sh on button [auto|right|left]   # 대회 로컬 방식: 로컬 통신 + 버튼 프로그램만 켬. 기반·임무는 1번 버튼이 켬 (field_button.py)
#   field_autostart.sh on 1 | on 2 | on         # 기반·잡기 스테이션 켜기, 임무 번호를 주면 그 임무 노드까지 준비
#   field_autostart.sh off | status
W=~/mecanum_slam_ws
LINE="@reboot $W/field_boot.sh > /tmp/field_boot.log 2>&1"
case "${1:-status}" in
  on)  (crontab -l 2>/dev/null | grep -v field_boot.sh; echo "$LINE") | crontab -
       if [ "${2:-}" = button ]; then
         : > $W/.autostart; echo "${3:-auto}" > $W/.autostart_button; touch $W/.dds_local
         echo "켬: 다음 부팅부터 버튼 방식 (로컬 통신, 옆 바깥벽 ${3:-auto})"
       else
         echo "${2:-}" > $W/.autostart; rm -f $W/.autostart_button
         echo "켬: 다음 부팅부터 (임무 ${2:-준비 안 함})"
       fi;;
  off) crontab -l 2>/dev/null | grep -v field_boot.sh | crontab -; rm -f $W/.autostart $W/.autostart_button; echo "끔 (로컬 통신은 그대로: field_dds.sh 로 확인)";;
  status) crontab -l 2>/dev/null | grep -q field_boot.sh && [ -f $W/.autostart ] && echo "켜짐" || echo "꺼짐"; [ -f $W/.autostart_button ] && echo "버튼 방식 (옆 바깥벽 $(cat $W/.autostart_button))"; [ -f /tmp/field_boot.log ] && tail -4 /tmp/field_boot.log;;
  *) sed -n 2,5p "$0"; exit 1;;
esac
