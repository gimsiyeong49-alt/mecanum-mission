#!/usr/bin/env python3
"""바퀴 잠금 손 시험: python3 tools/wheel_lock.py 30   → 30 초 동안 잠금을 켬. 그동안 로봇을 손으로 밀어 보고, 끝나면 풀림.
기반(구동)이 켜져 있어야 함 (영속도 명령이 들어와야 목표 0 상태). firmware/mecanum_grip_hold 를 구운 뒤에만."""
import os, sys, time
PORT = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0"
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


def send(b):
    fd = os.open(PORT, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        os.write(fd, b)
    finally:
        os.close(fd)


t_end = time.monotonic() + sec
print("잠금 켬 %.0f 초: 손으로 밀어 보세요" % sec, flush=True)
try:
    while time.monotonic() < t_end:
        send(b"H,1\n")
        time.sleep(0.4)
finally:
    send(b"H,0\n")
    print("잠금 끔", flush=True)
