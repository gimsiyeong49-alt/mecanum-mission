#!/usr/bin/env python3
"""라우터·노트북 없이 로봇에 단 버튼 4개로 임무를 준비하고 출발. 누르는 순간 네트워크가 필요 없음.
  1번 = 구동·센서·위치추정·1임무 노드 켜기 (바퀴 안 움직임). 끝나면 그리퍼가 한 번 여닫힘
  2번 = 1임무 출발 (누르고 --delay1 초 뒤)
  3번 = 1임무 끄고 2임무 준비 (--hold-long 초 눌러야 함, 달리는 중이어도 끔). 끝나면 그리퍼가 두 번 여닫힘 (1임무와 구별)
  4번 = 2임무 출발 (누르고 --delay2 초 뒤)
준비 실패면 그리퍼가 세 번 여닫힘 → 그 버튼을 다시 누름. 이미 준비된 임무의 1·3번은 그리퍼만 여닫아 상태를 알려 줌 (1임무 한 번, 2임무 두 번).
준비 안 된 임무의 출발 버튼, 달리는 중의 1·2·4번, 준비 중의 모든 버튼은 무시. 2임무 준비 상태에서 1번은 --hold-long 초 눌러야 1임무로 돌아감.
  python3 field_button.py [--side auto|right|left] [--dev 장치...] [--gpio 핀1 핀2 핀3 핀4]
  python3 field_button.py --watch     # 눌러 보기만 (임무 안 건드림)
  python3 field_button.py --learn     # USB 장치의 버튼을 1~4번 순서로 눌러 .button_map 에 저장 (기본은 키 1~4)
  python3 field_button.py --fifo /tmp/field_btn   # 버튼 없이 시험: echo 2 > /tmp/field_btn 이 2번 누름과 같음 (받아들여질 때까지 누른 셈)
USB 는 /dev/input 읽기 권한(input 그룹), GPIO 는 gpio 그룹이 필요. 배선은 class Gpio 설명 참고. 전원 켤 때 자동 시작: field_autostart.sh on button"""
import argparse, glob, json, os, select, struct, subprocess, threading, time, urllib.request

EV_KEY = 1
KEYS = {2: 1, 79: 1, 3: 2, 80: 2, 4: 3, 81: 3, 5: 4, 75: 4}   # 윗줄 숫자 1~4, 숫자패드 1~4
FMT = "llHHi"                                # struct input_event (64 비트)
SIZE = struct.calcsize(FMT)
W = os.path.expanduser("~/mecanum_slam_ws")
ARMED = "/tmp/mission_armed"
KEYMAP = W + "/.button_map"


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


def edge(down, b, pressed):
    if pressed:
        down[b] = time.monotonic()
    else:
        down.pop(b, None)


class Evdev:
    """USB 키보드·숫자패드·마우스 등. 장치를 새로 꽂아도 3 초 안에 잡음, 빠져도 안 죽음."""

    def __init__(self, paths, keys):
        self.paths, self.keys, self.fds, self.t_scan, self.bad = paths, keys, {}, 0.0, set()

    def reset(self):                         # 준비하는 동안 눌린 것은 버림
        for fd in self.fds:
            os.close(fd)
        self.fds, self.t_scan = {}, 0.0

    def events(self):
        """(코드, 값) 들. 값 1 = 눌림, 0 = 뗌, 2 = 자동 반복."""
        now = time.monotonic()
        if now - self.t_scan > 3.0:
            self.t_scan = now
            known = set(self.fds.values())
            for p in (self.paths or glob.glob("/dev/input/event*")):
                if p in known:
                    continue
                try:
                    self.fds[os.open(p, os.O_RDONLY | os.O_NONBLOCK)] = p
                except OSError as e:
                    if p not in self.bad:
                        self.bad.add(p)
                        log("못 엶 %s: %s" % (p, e))
        if not self.fds:
            time.sleep(0.2)
            return []
        out = []
        for fd in select.select(list(self.fds), [], [], 0.02)[0]:
            try:
                buf = os.read(fd, SIZE * 32)
            except BlockingIOError:
                continue
            except OSError:
                os.close(fd)
                del self.fds[fd]
                continue
            for i in range(0, len(buf) - SIZE + 1, SIZE):
                _, _, typ, code, val = struct.unpack_from(FMT, buf, i)
                if typ == EV_KEY:
                    out.append((code, val))
        return out

    def poll(self, down):
        for code, val in self.events():
            if code in self.keys and val in (0, 1):
                edge(down, self.keys[code], val == 1)


class Gpio:
    """Jetson 40핀 (BOARD 번호, 3.3 V 전용, 5 V 금지). 배선: 핀 → 버튼 → GND, 눌림 = LOW (--gpio-low).
    아직 못 씀 (11-02 실측, tools/pin_pull.py): 7·29 는 눌러 LOW 로 떨어져도 뗀 뒤 HIGH 로 안 돌아옴 (칩 풀업이 켜져 있으나 약한 것으로 추정),
    31 은 처음부터 LOW (선 위치 확인 필요), 32 는 칩 입력이 꺼져 있음. 후보는 핀마다 외부 풀업 약 1 kΩ → 3.3 V 와 32 대신 입력이 켜진 핀. Jetson.GPIO 의 소프트웨어 풀업은 무시됨."""

    def __init__(self, pins, low=False):
        import Jetson.GPIO as G
        self.G, self.pins, self.on = G, pins, G.LOW if low else G.HIGH
        G.setwarnings(False)
        G.setmode(G.BOARD)
        G.setup(pins, G.IN)
        self.reset()

    def levels(self):
        return [self.G.input(p) == self.on for p in self.pins]

    def reset(self):                         # 지금 눌려 있는 것은 뗄 때까지 안 셈
        self.prev = self.levels()

    def poll(self, down):
        time.sleep(0.01)
        now = self.levels()
        for i, (a, b) in enumerate(zip(self.prev, now)):
            if a != b:
                edge(down, i + 1, b)
        self.prev = now


class Fifo:
    """버튼 달기 전 시험용: 파일에 쓴 숫자 1~4 를 누름으로 봄. 뗌이 없어 필요한 시간만큼 눌린 셈."""

    def __init__(self, path, tap=None):
        if not os.path.exists(path):
            os.mkfifo(path)
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self.tap, self.t_down = tap, {}          # tap 초 뒤 뗀 셈 (--on-release 시험용)

    def reset(self):
        pass

    def poll(self, down):
        if select.select([self.fd], [], [], 0.02)[0]:
            for c in os.read(self.fd, 64).decode(errors="ignore"):
                if c in "1234":
                    edge(down, int(c), True)
                    self.t_down[int(c)] = time.monotonic()
        if self.tap is not None:
            for b, t in list(self.t_down.items()):
                if time.monotonic() - t >= self.tap:
                    edge(down, b, False)
                    del self.t_down[b]


def wait_press(src, hold_for, on_release=False, max_hold=3.0):
    """on_release 면 눌렀다 뗄 때 받음: 누른 시간이 hold_for(번호)~max_hold 초일 때만 (0 에 붙은 핀·빠진 선은 안 받음). 돌려줌 (번호, 뗀 순간).
    아니면 hold_for(번호) 초 넘게 누른 버튼 (번호, 누른 순간). 기다리는 사이 임무 상태가 바뀔 수 있어 필요한 시간은 눌린 순간에 정함."""
    down, need = {}, {}
    src.reset()
    while True:
        before = dict(down)
        src.poll(down)
        now = time.monotonic()
        for b, t0 in down.items():
            if (b, t0) not in need:
                need[(b, t0)] = hold_for(b)
        for b, t0 in before.items():
            if down.get(b) != t0:                # 뗌
                held = now - t0
                if on_release and need[(b, t0)] <= held <= max_hold:
                    return b, now
                log("%d번 %.2f 초 누름: 무시 (%.2f~%.1f 초만 받음)" % (b, held, need[(b, t0)], max_hold) if on_release else "%d번 %.2f 초 누름: 짧아서 무시" % (b, held))
        if on_release:
            continue
        for b, t0 in down.items():
            if now - t0 >= need[(b, t0)]:
                return b, t0


def node_up():
    for d in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            argv = open(d, "rb").read().split(b"\0")
        except OSError:
            continue
        if len(argv) > 1 and os.path.basename(argv[1]) in (b"mission1_node.py", b"battle_node.py"):
            return True
    return False


def state():
    """1·2 = 그 임무 준비돼 출발 기다림, "run" = 달리는 중, 0 = 노드 없음 (준비했던 노드가 죽은 경우 포함)."""
    if not node_up():
        return 0
    try:
        return int(open(ARMED).read())
    except (OSError, ValueError):
        return "run"


def sign_blocking(times):
    """그리퍼로 알림: 화면도 통신도 없이 눈으로 확인. 스테이션이 없으면 그냥 넘어감."""
    for _ in range(times):
        for act in ("close", "open"):
            try:
                urllib.request.urlopen("http://127.0.0.1:8090/do?a=" + act, timeout=2).read()
            except Exception as e:
                log("그리퍼 알림 실패: %s" % e)
                return
            time.sleep(0.7)


def sign(times=1):
    threading.Thread(target=sign_blocking, args=(times,), daemon=True).start()   # 알리는 동안에도 버튼을 받음


def prepare(m, side):
    log("%d임무 준비 시작 (바퀴 안 움직임)" % m)
    t0 = time.monotonic()
    if subprocess.call([W + "/field_arm.sh", str(m), side]) == 0:
        log("%d임무 준비 끝 (%.1f 초)" % (m, time.monotonic() - t0))
        sign(m)                              # 1임무 한 번, 2임무 두 번 (실패는 세 번)
    else:
        log("%d임무 준비 실패 (%.1f 초)" % (m, time.monotonic() - t0))
        sign(3)


def launch(m, t_press, delay, lead):
    def until(t):
        wait = t - time.monotonic()
        if wait > 0:
            time.sleep(wait)
    if m == 1:                               # 손이 떨어진 뒤 지금 자리를 출발 칸으로 다시 잡음 (준비 뒤 들어서 옮긴 경우)
        until(t_press + delay - lead)
        subprocess.call([W + "/field_arm.sh", "recheck"])
    until(t_press + delay)
    log("%d임무 출발 (누르고 %.2f 초)" % (m, time.monotonic() - t_press))
    subprocess.call([W + "/field_arm.sh", "go"])


def stop_all():
    subprocess.call([W + "/field_arm.sh", "stop"])


def press(b, st, t_press, a):
    if b == 1:
        if st == 1:
            log("1임무는 이미 준비됨: 그리퍼로 알림")
            sign()
        elif st == "run":
            log("달리는 중: 1번 무시")
        else:
            if st == 2:
                stop_all()
            prepare(1, a.side)
    elif b == 2:
        if st == 1:
            launch(1, t_press, a.delay1, a.recheck_lead)
        else:
            log("1임무 준비 안 됨 (상태 %s): 2번 무시" % st)
    elif b == 3:
        if st == 2:
            log("2임무는 이미 준비됨: 그리퍼로 알림")
            sign(2)
        else:
            if st != 0:
                log("켜져 있는 임무 끔 (상태 %s)" % st)
            stop_all()
            prepare(2, a.side)
    elif b == 4:
        if st == 2:
            launch(2, t_press, a.delay2, a.recheck_lead)
        else:
            log("2임무 준비 안 됨 (상태 %s): 4번 무시" % st)


def learn(paths):
    src, keys = Evdev(paths, {}), {}
    for b in (1, 2, 3, 4):
        print("%d번 버튼을 누르세요" % b, flush=True)
        while True:
            hit = [c for c, v in src.events() if v == 1 and c not in keys]
            if hit:
                keys[hit[0]] = b
                print("  코드 %d = %d번" % (hit[0], b), flush=True)
                time.sleep(0.3)
                src.events()
                break
    json.dump(keys, open(KEYMAP, "w"))
    print("저장: %s" % KEYMAP)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="auto", choices=["auto", "right", "left"], help="1임무 출발 칸 옆 바깥벽. auto = 출발 직전 놓인 자세를 라이다로 판단")
    ap.add_argument("--hold", type=float, default=0.05, help="이보다 짧은 입력은 스침으로 무시 [s]")
    ap.add_argument("--hold-long", type=float, default=0.05, help="켜진 임무를 끄는 누름 (3번, 2임무 준비 상태의 1번) [s]. 11-03 사용자 결정으로 다른 버튼과 같게")
    ap.add_argument("--delay1", type=float, default=1.0, help="2번 누르고 1임무 출발까지 [s]")
    ap.add_argument("--delay2", type=float, default=0.0, help="4번 누르고 2임무 출발까지 [s]")
    ap.add_argument("--recheck-lead", type=float, default=0.5, help="1임무 출발 이만큼 전에 지금 자리로 원점 다시 잡기 [s]")
    ap.add_argument("--dev", nargs="*", default=None, help="읽을 장치 (기본: /dev/input/event* 전부)")
    ap.add_argument("--gpio", nargs=4, type=int, default=None, metavar="핀", help="USB 대신 Jetson 40핀 BOARD 번호 1~4번 순서 (빨 흰 노 파 = 7 29 31 22)")
    ap.add_argument("--gpio-low", action="store_true", help="눌림 = LOW 배선 (버튼 → GND + 외부 풀업). 없으면 눌림 = HIGH")
    ap.add_argument("--fifo", default=None, help="버튼 없이 시험: 이 파일에 쓴 숫자를 누름으로 봄")
    ap.add_argument("--on-release", action="store_true", help="눌렀다 뗄 때 실행 (누른 시간 --hold~--max-hold 초만). 없으면 누르고 --hold 초 지나면 바로")
    ap.add_argument("--max-hold", type=float, default=3.0, help="--on-release: 이보다 오래 눌린 건 붙은 핀으로 보고 무시 [s]")
    ap.add_argument("--watch", action="store_true", help="눌러 보기만")
    ap.add_argument("--learn", action="store_true", help="USB 버튼을 1~4번 순서로 눌러 저장")
    a = ap.parse_args()
    if a.learn:
        return learn(a.dev)
    keys = dict(KEYS)
    if os.path.exists(KEYMAP):
        keys = {int(k): v for k, v in json.load(open(KEYMAP)).items()}
    src = Fifo(a.fifo, tap=max(a.hold, a.hold_long) + 0.05 if a.on_release else None) if a.fifo else Gpio(a.gpio, a.gpio_low) if a.gpio else Evdev(a.dev, keys)
    log("버튼 대기: %s, %s, 지금 상태 %s" % (a.fifo or ("GPIO %s" % a.gpio if a.gpio else "USB 코드 %s" % sorted(keys)), "뗄 때 실행" if a.on_release else "누르면 실행", state()))
    hold_for = lambda b: a.hold_long if b == 3 or (b == 1 and state() == 2) else a.hold
    while True:
        b, t_press = wait_press(src, hold_for, a.on_release, a.max_hold)
        if a.watch:
            log("%d번 눌림" % b)
            continue
        press(b, state(), t_press, a)


if __name__ == "__main__":
    main()
