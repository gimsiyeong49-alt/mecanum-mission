"""그리퍼 시험용: 아두이노 포트를 한 번만 열어 두고(열 때마다 리셋되므로) /tmp/grip_cmd 에 쓴 줄을 그대로 보냄.
응답 중 OK/ERR/READY 줄만 /tmp/grip_log 에 남김. 주행 스택이 꺼져 있을 때만 사용."""
import os, serial, time, select
P = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0"
FIFO = "/tmp/grip_cmd"
if not os.path.exists(FIFO): os.mkfifo(FIFO)
log = open("/tmp/grip_log", "a", buffering=1)
s = serial.Serial(P, 115200, timeout=0.05)
time.sleep(2.5); log.write("%s 열림 %s\n" % (time.strftime("%T"), s.read(200).decode(errors="ignore").splitlines()[:1]))
fd = os.open(FIFO, os.O_RDONLY | os.O_NONBLOCK); buf = b""
while True:
    r, _, _ = select.select([fd], [], [], 0.05)
    if r:
        d = os.read(fd, 256)
        if not d:
            os.close(fd); fd = os.open(FIFO, os.O_RDONLY | os.O_NONBLOCK); continue
        for line in d.decode().split("\n"):
            line = line.strip()
            if line: s.write((line + "\n").encode()); log.write("%s 보냄 %s\n" % (time.strftime("%T"), line))
    buf += s.read(512)
    while b"\n" in buf:
        L, buf = buf.split(b"\n", 1); L = L.decode(errors="ignore").strip()
        if L.startswith(("OK", "ERR", "READY")): log.write("%s 받음 %s\n" % (time.strftime("%T"), L))
