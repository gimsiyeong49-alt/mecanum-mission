"""초점 맞추기 화면 (브라우저 http://<로봇IP>:8090). 카메라 설정은 바꾸지 않음(읽기만).
가운데 초록 네모 안 선명도(라플라시안 분산, 최근 5장 평균)를 크게, 최고값(PEAK)과 함께 표시. /reset 으로 최고값 초기화."""
import cv2, threading, time, numpy as np, collections
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
frames, peak, hist = {}, {0: 0.0, 4: 0.0}, {0: collections.deque(maxlen=5), 4: collections.deque(maxlen=5)}
def grab(dev):
    c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    while True:
        ok, f = c.read()
        if not ok: time.sleep(0.05); continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY); h, w = g.shape
        hist[dev].append(cv2.Laplacian(g[h // 3:2 * h // 3, w // 3:2 * w // 3], cv2.CV_64F).var())
        frames[dev] = f
        v = float(np.mean(hist[dev])); peak[dev] = max(peak[dev], v)
SIZES = [(7, 4), (4, 7), (9, 6), (6, 9), (8, 5), (5, 8), (7, 5), (5, 7), (6, 4), (4, 6), (10, 7), (7, 10), (8, 6), (6, 8), (11, 8), (8, 11)]
board = {0: None, 4: None}
def board_worker(dev):
    last = None
    while True:
        f = frames.get(dev)
        if f is None: time.sleep(0.3); continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY); small = cv2.resize(g, (640, 360))
        found = None
        for sz in ([last] if last else []) + [z for z in SIZES if z != last]:
            if not cv2.checkChessboard(small, sz): continue
            ok, corners = cv2.findChessboardCornersSB(g, sz)
            if ok:
                found = (sz, cv2.estimateChessboardSharpness(g, sz, corners)[0][0]); last = sz; break
        board[dev] = (found, time.time()) if found else None
        time.sleep(0.2)
for d in (0, 4):
    threading.Thread(target=grab, args=(d,), daemon=True).start()
    threading.Thread(target=board_worker, args=(d,), daemon=True).start()
cube = {0: None, 4: None}
def cube_sharp(f):
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    m = (((hsv[..., 0] < 15) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 140) & (hsv[..., 2] > 70)).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2: return None
    # 큐브다운 덩어리만: 가로세로 비 0.6~1.7, 상자 채움 0.65 이상 (전선 릴 같은 빨간 고리 제외), 그중 가장 큰 것
    ok = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= 3000
          and 0.4 <= st[i, cv2.CC_STAT_WIDTH] / st[i, cv2.CC_STAT_HEIGHT] <= 2.5
          and st[i, cv2.CC_STAT_AREA] / (st[i, cv2.CC_STAT_WIDTH] * st[i, cv2.CC_STAT_HEIGHT]) >= 0.8]
    if not ok: return None
    k = max(ok, key=lambda i: st[i, cv2.CC_STAT_AREA])
    x, y, w, h = st[k, :4]
    edge = cv2.dilate(m * (lab == k), np.ones((9, 9), np.uint8)) - cv2.erode(m * (lab == k), np.ones((9, 9), np.uint8))
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype(np.float64)
    gx = cv2.Sobel(g, cv2.CV_64F, 1, 0, ksize=3); gy = cv2.Sobel(g, cv2.CV_64F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)[edge > 0]
    if mag.size < 100: return None
    contrast = np.percentile(g[edge > 0], 95) - np.percentile(g[edge > 0], 5) + 1.0
    return float(np.percentile(mag, 90) / contrast * 100), (x, y, w, h)
cube_hist = {0: collections.deque(maxlen=12), 4: collections.deque(maxlen=12)}
def cube_worker(dev):
    while True:
        f = frames.get(dev)
        if f is not None:
            r = cube_sharp(f)
            if r: cube_hist[dev].append(r[0]); r = (float(np.median(cube_hist[dev])), r[1])
            cube[dev] = (r, time.time())
        time.sleep(0.15)
for d in (0, 4): threading.Thread(target=cube_worker, args=(d,), daemon=True).start()
def panel(dev, name):
    f = frames.get(dev)
    if f is None: return np.zeros((405, 720, 3), np.uint8)
    p = cv2.resize(f, (720, 405)); h, w = p.shape[:2]
    cv2.rectangle(p, (w // 3, h // 3), (2 * w // 3, 2 * h // 3), (0, 255, 0), 2)
    v = float(np.mean(hist[dev])) if hist[dev] else 0.0; pk = peak[dev]
    cv2.rectangle(p, (0, 0), (w, 62), (0, 0, 0), -1)
    cv2.putText(p, "%s  SHARP %.0f   PEAK %.0f" % (name, v, pk), (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA)
    c = cube.get(dev)
    if c and c[0] and time.time() - c[1] < 1:
        cs, (x, y, bw, bh) = c[0]; sx = w / f.shape[1]
        cv2.rectangle(p, (int(x * sx), int(y * sx)), (int((x + bw) * sx), int((y + bh) * sx)), (255, 0, 255), 2)
        cv2.rectangle(p, (0, h - 100), (w, h - 52), (0, 0, 0), -1)
        cv2.putText(p, "CUBE SHARP %.0f (2s median, bigger = sharper)" % cs, (8, h - 66), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 0, 255), 2, cv2.LINE_AA)
    b = board.get(dev)
    if b and time.time() - b[1] < 2:
        (sz, ew), _ = b
        cv2.rectangle(p, (0, h - 50), (w, h), (0, 0, 0), -1)
        cv2.putText(p, "BOARD %dx%d  EDGE %.2f px (smaller = sharper)" % (sz[0], sz[1], ew), (8, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(p, "no board", (8, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2, cv2.LINE_AA)
    frac = 0 if pk <= 0 else min(1.0, v / pk)
    cv2.rectangle(p, (8, 40), (8 + int((w - 16) * frac), 56), (0, 200, 0) if frac > 0.9 else (0, 160, 255), -1)
    cv2.rectangle(p, (8, 40), (w - 8, 56), (200, 200, 200), 1)
    return p
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path.startswith("/reset"):
            for d in peak: peak[d] = 0.0
            self.send_response(302); self.send_header("Location", "/"); self.end_headers(); return
        if self.path == "/":
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.end_headers()
            self.wfile.write(("<html><body style=\"background:#222;color:#eee;font-family:sans-serif\">"
                "<h3>초점 맞추기 · 왼쪽 = video0 · 오른쪽 = video4 · 초록 네모 안에 격자판(또는 글씨)을 두고 링을 돌려 SHARP 를 최대로 · "
                "<a style=\"color:#6cf\" href=\"/reset\">[최고값 지우기]</a></h3><img id=\"v\" style=\"width:100%\">"
                "<script>const v=document.getElementById(\"v\");function next(){const im=new Image();"
                "im.onload=()=>{v.src=im.src;setTimeout(next,80)};im.onerror=()=>setTimeout(next,500);"
                "im.src=\"/frame.jpg?\"+Date.now()}next()</script></body></html>").encode()); return
        if self.path.startswith("/frame.jpg"):
            img = np.hstack([panel(0, "LEFT"), panel(4, "RIGHT")])
            ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80]); b = j.tobytes()
            self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Content-Length", str(len(b)))
            self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(b); return
        self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f"); self.end_headers()
        try:
            while True:
                img = np.hstack([panel(0, "LEFT"), panel(4, "RIGHT")])
                ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + j.tobytes() + b"\r\n"); time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError): pass
ThreadingHTTPServer(("0.0.0.0", 8090), H).serve_forever()
