#!/bin/bash
# 1) 지금 보드 펌웨어 읽어 백업  2) 그리퍼+적분 판(build_grip_pi) 굽기.  되돌리기: flash_grip.sh (그리퍼 판, 10-29)
P=/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0
AV=/home/uoeee/.arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1
set -x
$AV/bin/avrdude -C $AV/etc/avrdude.conf -p atmega328p -c arduino -P $P -b 115200 -U flash:r:/home/uoeee/mecanum_slam_ws/firmware/backup_before_grip_pi_1031.hex:i
echo "백업 종료코드 $?"
~/.local/bin/arduino-cli upload -p $P --fqbn arduino:avr:uno --input-dir /home/uoeee/mecanum_slam_ws/firmware/build_grip_pi
echo "굽기 종료코드 $?"
