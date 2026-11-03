#!/usr/bin/env python3
"""버튼 핀 실시간 보기 (아두이노 시리얼 모니터처럼). 핀 값을 읽기만 하고 임무·바퀴는 안 건드림. Ctrl+C 로 끝.
  python3 ~/mecanum_slam_ws/tools/pin_monitor.py              # 40핀 BOARD 번호 7 29 31 22 = 버튼 1~4
  python3 ~/mecanum_slam_ws/tools/pin_monitor.py 7 29 31 21   # 다른 핀으로
맨 아래 줄은 지금 값(0.05 초마다 새로 씀), 바뀔 때마다 그 위에 한 줄씩 남김. 핀 번호는 40핀 BOARD 번호."""
import shutil, sys, time
import Jetson.GPIO as G

HOLD, MAX_HOLD = 0.05, 3.0                   # field_button --on-release 와 같은 기준: 이 사이로 눌렀다 떼면 받음

PINS = [int(p) for p in sys.argv[1:]] or [7, 29, 31, 22]    # 버튼 1~4 = 빨 흰 노 파. 파랑은 32 가 칩 입력이 꺼져 있어 22 로 옮김 (11-03)
G.setwarnings(False)
G.setmode(G.BOARD)
ok = []
for p in PINS:                               # 못 쓰는 핀은 빼고 계속 (핀 후보를 한꺼번에 볼 때)
    try:
        G.setup(p, G.IN)
        ok.append(p)
    except Exception as e:
        print("핀 %d 못 씀: %s" % (p, e))
PINS = ok


def now_str():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)


def row(v):
    return "  ".join("핀%d=%d" % (p, x) for p, x in zip(PINS, v))


prev = [G.input(p) for p in PINS]
t_low = {}
print("핀 %s 읽는 중. 누르면 0↔1 이 바뀌어야 함. Ctrl+C 로 끝" % PINS)
print("%s  처음: %s" % (now_str(), row(prev)))
n, t_draw = 0, 0.0
try:
    while True:
        cur = [G.input(p) for p in PINS]
        for i, (a, b) in enumerate(zip(prev, cur)):
            if a != b:
                n += 1
                print("\r\033[K%s  핀 %d: %d → %d" % (now_str(), PINS[i], a, b))
                if b == 0:
                    t_low[i] = time.monotonic()
                elif i in t_low:
                    held = time.monotonic() - t_low.pop(i)
                    ok = "받음" if HOLD <= held <= MAX_HOLD else "무시 (%.2f~%.1f 초만 받음)" % (HOLD, MAX_HOLD)
                    print("\r\033[K%s  핀 %d: %.2f 초 눌렀다 뗌 → %s" % (now_str(), PINS[i], held, ok))
        prev = cur
        if time.monotonic() - t_draw > 0.05:
            t_draw = time.monotonic()
            w = shutil.get_terminal_size((80, 20)).columns - 1          # 줄이 넘치면 \r 로 못 덮어 화면이 쌓임
            sys.stdout.write("\r\033[K" + ("지금 %s  (바뀜 %d)" % (row(cur), n))[:w])
            sys.stdout.flush()
        time.sleep(0.005)
except KeyboardInterrupt:
    print("\n끝. 바뀜 %d 번" % n)
