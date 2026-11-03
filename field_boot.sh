#!/bin/bash
# 전원 켤 때 할 일 (field_autostart.sh on 일 때 crontab @reboot 이 부름). 기록: /tmp/field_boot.log
#   버튼 방식: 버튼 프로그램만 켬. 기반·위치추정·임무는 1번 버튼이 켬
#   그 밖: 기반·잡기 스테이션 켜기, 임무 번호가 있으면 그 임무까지 준비
W=~/mecanum_slam_ws
[ -f $W/.autostart ] || exit 0
echo "$(date '+%H:%M:%S') 부팅 시작"
if [ -f $W/.autostart_button ]; then
  mkdir -p $W/run_logs
  SIDE=$(cat $W/.autostart_button); SIDE=${SIDE:-auto}
  screen -dmS button bash -c "python3 $W/field_button.py --side $SIDE --gpio 7 29 31 22 --gpio-low --hold 0.05 --hold-long 0.05 --on-release >> $W/run_logs/button_$(date +%m%d_%H%M%S).log 2>&1"
  echo "$(date '+%H:%M:%S') 버튼 프로그램 켬"
  exit 0
fi
source $W/field_common.sh
wait_devices
$W/start_station.sh
ensure_base
M=$(cat $W/.autostart)
if [ "$M" = 1 ] || [ "$M" = 2 ]; then $W/field_arm.sh $M; fi
echo "$(date '+%H:%M:%S') 준비 끝"
