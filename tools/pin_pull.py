#!/usr/bin/env python3
"""버튼 핀(BOARD 7·29·31·32)의 칩 설정을 읽기만 함. 쓰지 않음. /dev/mem 이 필요해 sudo 로 실행.
  sudo python3 pin_pull.py        # 지금 설정
  sudo python3 pin_pull.py hold   # 프로그램이 입력으로 잡았을 때의 설정과 읽히는 값까지
주소: 장치트리 pinmux@2430000(크기 0x19100) + pinctrl-tegra234.c 의 그룹 오프셋. 각 레지스터는 32 비트로만 읽음.
비트: 1:0 기능, 3:2 풀(0 없음, 1 풀다운, 2 풀업), 4 트라이스테이트, 6 입력 켬, 10 GPIO/SFIO 선택"""
import ctypes, os, sys, time

BASE, SIZE = 0x02430000, 0x19100
PINS = [(7, "빨강", "PAC.06", 0x18030), (29, "흰색", "PQ.05", 0x0068), (31, "노랑", "PQ.06", 0x0070), (32, "파랑", "PG.06", 0x4080)]
PULL = {0: "없음", 1: "풀다운", 2: "풀업", 3: "예약값"}
PAGE = 4096

libc = ctypes.CDLL(None, use_errno=True)
libc.mmap.restype = ctypes.c_void_p
libc.mmap.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_long]
PROT_READ, MAP_SHARED = 1, 1


def read32(fd, addr):
    page = addr & ~(PAGE - 1)
    p = libc.mmap(None, PAGE, PROT_READ, MAP_SHARED, fd, page)
    if p in (None, ctypes.c_void_p(-1).value):
        raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))
    return ctypes.c_uint32.from_address(p + addr - page).value


def main():
    try:
        fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
    except OSError as e:
        sys.exit("/dev/mem 을 못 엶: %s (sudo 로 실행)" % e)
    show(fd)
    if len(sys.argv) > 1 and sys.argv[1] == "hold":     # 프로그램이 핀을 입력으로 잡고 있을 때의 칩 설정과 읽히는 값
        import Jetson.GPIO as G
        pins = [p[0] for p in PINS]
        G.setwarnings(False)
        G.setmode(G.BOARD)
        G.setup(pins, G.IN)
        time.sleep(0.2)
        print("--- 입력으로 잡은 상태")
        show(fd)
        for _ in range(3):
            print("읽힌 값 (7 29 31 32):", [G.input(p) for p in pins])
            time.sleep(0.5)
        G.cleanup()


def show(fd):
    for pin, color, name, off in PINS:
        if off >= SIZE:
            sys.exit("범위 밖 오프셋 0x%x" % off)
        addr = BASE + off
        try:
            v = read32(fd, addr)
        except OSError as e:
            sys.exit("읽기 실패 0x%08x: %s" % (addr, e))
        print("핀 %2d %s %-6s 0x%08x = 0x%08x | 풀 %s, 입력 %s, 트라이스테이트 %s, GPIO선택비트 %d, 기능 %d" % (
            pin, color, name, addr, v, PULL[(v >> 2) & 3], "켬" if (v >> 6) & 1 else "꺼짐", "켬" if (v >> 4) & 1 else "꺼짐", (v >> 10) & 1, v & 3))


if __name__ == "__main__":
    main()
