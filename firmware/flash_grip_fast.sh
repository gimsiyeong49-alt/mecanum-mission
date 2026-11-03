#!/bin/bash
# 출력 올리기 판(mecanum_grip_fast) 빌드 → 지금 보드 펌웨어 백업 → 굽기. 로봇에서, 기반을 끈 뒤 (포트를 놓아야 함).
# 되돌리기: firmware/flash_grip_pi.sh (build_grip_pi 를 다시 구움) 또는 아래 백업 파일을 avrdude 로.
set -e
W=~/mecanum_slam_ws/firmware
P=/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0
AV=/home/uoeee/.arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1
CLI=~/.local/bin/arduino-cli
$CLI compile --fqbn arduino:avr:uno --build-path $W/build_grip_fast $W/mecanum_grip_fast
B=$W/backup_before_fast_$(date +%m%d_%H%M%S).hex
$AV/bin/avrdude -C $AV/etc/avrdude.conf -p atmega328p -c arduino -P $P -b 115200 -U flash:r:$B:i
echo "백업: $B"
$CLI upload -p $P --fqbn arduino:avr:uno --input-dir $W/build_grip_fast
echo "굽기 끝. 시험: field_battle.sh go --test start --pwm 228"
