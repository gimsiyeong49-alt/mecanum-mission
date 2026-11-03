#!/usr/bin/env python3
"""펌웨어 출력 상한을 잠시 255 로 (firmware/mecanum_grip_fast 를 구운 뒤에만). 끄면 1 초 안에 저절로 기본 200 으로 돌아옴. 미션 코드는 안 건드림.
  python3 fast_pwm.py [초] [상한]    # 기본 240 초 뒤 끝남, 상한 255"""
import os, sys, time

PORT = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0"
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
pwm = int(sys.argv[2]) if len(sys.argv) > 2 else 255
t_end = time.monotonic() + sec
while time.monotonic() < t_end:
    fd = os.open(PORT, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        os.write(fd, b"P,%d\n" % pwm)
    except BlockingIOError:
        pass
    finally:
        os.close(fd)
    time.sleep(0.4)
