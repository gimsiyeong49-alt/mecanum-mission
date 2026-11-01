"""두 카메라 실시간 보기 (브라우저 http://<로봇IP>:8090). 카메라 설정은 바꾸지 않음(읽기만).
화면 위 숫자: 평균 밝기 · SHARP=가운데 선명도(클수록 선명, 초점 맞출 때 봄) · 하얗게 날아간 픽셀(>=250) % · 까맣게 묻힌 픽셀(<=5) %."""
import cv2, threading, time, numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
frames, stats = {}, {}
def grab(dev):
    c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    n, t0 = 0, time.time()
    while True:
        ok, f = c.read()
        if not ok: time.sleep(0.05); continue
        frames[dev] = f; n += 1
        if time.time() - t0 > 1: stats[dev + 100] = n / (time.time() - t0); n, t0 = 0, time.time()
for d in (0, 4): threading.Thread(target=grab, args=(d,), daemon=True).start()
def panel(dev, name):
    f = frames.get(dev)
    if f is None: return np.zeros((360, 640, 3), np.uint8)
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    h, w = g.shape; sharp = cv2.Laplacian(g[h // 4:3 * h // 4, w // 4:3 * w // 4], cv2.CV_64F).var()
    s = "%s mean %.0f white %.1f%% black %.1f%% SHARP %.0f %.0ffps" % (name, g.mean(), 100 * (g >= 250).mean(), 100 * (g <= 5).mean(), sharp, stats.get(dev + 100, 0))
    p = cv2.resize(f, (640, 360)); cv2.rectangle(p, (0, 0), (640, 26), (0, 0, 0), -1)
    cv2.putText(p, s, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    return p
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path == "/":
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.end_headers()
            self.wfile.write("<html><body style=\"background:#222;color:#eee;font-family:sans-serif\"><h3>왼쪽 = video0 · 오른쪽 = video4 (로봇 뒤에서 볼 때)</h3><img src=\"/stream\" style=\"width:100%\"></body></html>".encode()); return
        self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f"); self.end_headers()
        try:
            while True:
                img = np.hstack([panel(0, "LEFT video0"), panel(4, "RIGHT video4")])
                ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + j.tobytes() + b"\r\n"); time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError): pass
ThreadingHTTPServer(("0.0.0.0", 8090), H).serve_forever()
