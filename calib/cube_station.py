"""큐브 잡기 스테이션 (로봇에서, 브라우저 http://<로봇IP>:8090).
카메라 두 대를 이 프로그램 하나가 계속 잡고 있고(껐다 켜지 않음 → 묵은 사진 없음), 웹 화면과 잡기 단계를 같이 한다.
잡기 단계 (항상 멈춤 → 보기 → 움직이기, 움직이는 중엔 안 잼, 판단 못 하면 옛날 값으로 가지 않고 멈춤):
  보기   : 멈춘 뒤 찍힌 새 사진 8장 → 5 cm 큐브 모형을 두 화면에 겹쳐 위치·각도 (cube_fit.py), 겹침 0.85↑·흔들림 5 mm 안일 때만 믿음
  찾기   : 한쪽 카메라에만 보이면 그쪽으로 4 cm 옆 이동 (최대 2번)
  돌기   : 큐브가 8~25° 돌아가 있으면 제자리 회전 → 보기 (25° 넘으면 안 돎: 그리퍼가 닫히며 바로잡음)
  옆맞춤 : 그리퍼 가운데로 옆 이동 → 보기 (5 mm 안 될 때까지 최대 3번)
  들어가기: 마지막 보기 값으로 곧게 (큐브가 사각지대라 안 봄), 마지막 10 cm 천천히, --push 더 밀기
  쥐기   : G,<grip-us>
바퀴 명령(/cmd_vel)은 잡기 단계 동안만 내고, 다른 발행자가 있으면 시작 안 함. 기록: ~/mecanum_slam_ws/calib/grab_runs/<시각>/"""
import argparse, glob, json, math, os, sys, threading, time, datetime, functools
import cv2, numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cube_face import mask_of, EntryWatch
print = functools.partial(print, flush=True)

ap = argparse.ArgumentParser()
ap.add_argument("--calib", default=os.path.expanduser("~/mecanum_slam_ws/calib/calib_current.npz"))
ap.add_argument("--rig-forward", type=float, default=0.186, help="바퀴 가운데 → 두 렌즈 가운데 앞 [m] (어림; grip-x 를 같은 기준으로 잼)")
ap.add_argument("--grip-x", type=float, default=0.191, help="쥘 때 큐브 가운데 앞 [m] (10-29 그리퍼 재기)")
ap.add_argument("--grip-left", type=float, default=0.010, help="옆맞춤 목표: 몸 가운데에서 왼쪽 [m] (10-29 잰 값 0.005. 11-01 그리퍼가 왼쪽으로 약 2~3° 꺾임 → 잡힌 기록 범위 안쪽인 0.010)")
ap.add_argument("--push", type=float, default=0.008)
ap.add_argument("--push-battle", type=float, default=0.015, help="2임무: 큐브에 닿은 뒤 더 미는 거리 [m]. 덜 들어가 앞쪽만 물면 큐브가 꺾임 (11-03), 막대에 붙여 밀면 반듯해지고 큐브만 조금 밀림. 11-03 21시 사용자: 더 밀어 확실히 → 8 → 15 mm")
ap.add_argument("--grip-us", type=int, default=2100); ap.add_argument("--open-us", type=int, default=1500)
ap.add_argument("--min-w", type=float, default=0.75)
# 옆맞춤 기준: 맞추는 목표는 5 mm 그대로 (10-31 19:49: 10 mm 로 풀었더니 오른쪽 7.2 mm 에서 놓침. 왼쪽은 13.5 mm 도 잡힘 →
# 그리퍼 실제 가운데가 grip-left 값보다 더 왼쪽, 양옆 여유가 같지 않음). 실패로 칠 기준만 15 mm (8 mm 로 두니 몇 mm 모자라 임무가 멈춤 — 18:50)
ap.add_argument("--side-tol", type=float, default=0.005, help="이 안이면 옆맞춤 없이 들어감 [m]")
ap.add_argument("--side-tries", type=int, default=5, help="옆맞춤 최대 횟수")
ap.add_argument("--coast-max", type=float, default=0.04, help="옆맞춤에서 더 간 양을 다음 명령에서 빼는 한도 [m]")
ap.add_argument("--side-max", type=float, default=0.015, help="옆맞춤 3 번 뒤에도 이보다 벗어나면 멈춤 [m]")
ap.add_argument("--min-iou", type=float, default=0.85, help="큐브 모형과 빨간 영역 겹침이 두 카메라 모두 이 이상일 때만 믿음")
ap.add_argument("--turn-min", type=float, default=8.0); ap.add_argument("--turn-max", type=float, default=25.0)
ap.add_argument("--v", type=float, default=0.16); ap.add_argument("--v-slow", type=float, default=0.10); ap.add_argument("--v-fast", type=float, default=0.25)
ap.add_argument("--min-v", type=float, default=0.10); ap.add_argument("--min-v-side", type=float, default=0.16)  # 주행 코드와 같은 최소 속도
ap.add_argument("--max-forward", type=float, default=0.40); ap.add_argument("--max-side", type=float, default=0.15)
ap.add_argument("--port", default="/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0")
ap.add_argument("--http", type=int, default=8090)
a = ap.parse_args()

c = np.load(a.calib)
KL, DL, KR, DR, R, T, CM, A = c["K_L"], c["D_L"], c["K_R"], c["D_R"], c["R_LR"], c["T_LR"], c["C_M_in_L"], c["A_ML"]
SIDE = 50.0

# ---------------- 카메라 (계속 잡고 있음) ----------------
frames = {}


def cam_loop(k, tag):
    dev = os.path.realpath(glob.glob("/dev/v4l/by-path/*%s-video-index0" % tag)[0])
    cp = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cp.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); cp.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cp.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cp.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while True:
        ok, f = cp.read()
        if ok: frames[k] = (f, time.time())
        else: time.sleep(0.05)


for k, tag in (("L", "usb-0:2.1.2:1.0"), ("R", "usb-0:2.1.4:1.0")):
    threading.Thread(target=cam_loop, args=(k, tag), daemon=True).start()


def fresh_pair(after):  # after 시각 뒤에 찍힌 두 사진 (멈춘 뒤 사진만)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        if "L" in frames and "R" in frames and frames["L"][1] > after and frames["R"][1] > after:
            return frames["L"][0], frames["R"][0]
        time.sleep(0.01)
    return None, None


# ---------------- 큐브 위치 (모형 겹치기 한 가지) ----------------
from cube_fit import Fitter
FIT = Fitter(c)


def to_body(Mmm):  # 리그 M(mm, x 오른쪽 z 앞) → 바퀴 가운데 기준 (m, x 앞, y 왼쪽)
    return a.rig_forward + Mmm[2] / 1000.0, -Mmm[0] / 1000.0


# ---------------- 상태·기록 ----------------
st = dict(busy=False, state="대기", log=[], last=None, overlay={}, coast=0.005)
run_dir = [None]


def log(msg):
    line = time.strftime("%H:%M:%S ") + msg; print(line)
    st["log"] = (st["log"] + [line])[-14:]
    if run_dir[0]:
        with open(os.path.join(run_dir[0], "log.txt"), "a") as fh: fh.write(line + "\n")


def snap(tag, L, Rr, res=None):
    if not run_dir[0]: return
    vis = []
    for img, f in ((L, res and res["fl"]), (Rr, res and res["fr"])):
        v = img.copy()
        if f is not None: cv2.polylines(v, [f.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 3)
        vis.append(cv2.resize(v, (960, 540)))
    cv2.imwrite(os.path.join(run_dir[0], "%s_%s.jpg" % (time.strftime("%H%M%S"), tag)), np.hstack(vis))


class Stop(Exception): pass


def look(tag, n=4):
    """멈춘 뒤 새 사진 n 장 → 큐브 모형 겹치기. 믿을 만하면 (dict, 0), 아니면 (None, 보이는 쪽 ±1 / 0)."""
    time.sleep(0.15); t_req = time.time(); res, last = [], None  # 멈춘 뒤 0.15 s (흔들림 1 mm 안이라 4장이면 충분, 10-29 재기 1 s → 0.5 s)
    st["measuring"] = True; t_m = time.time()
    try:
        for _ in range(n):
            L, Rr = fresh_pair(t_req)
            if L is None: raise Stop("카메라 사진이 안 들어옴")
            t_req = time.time(); last = (L, Rr)
            # 둘째 장부터 직전 답에서 이어 찾음 (149 → 37 ms, 같은 답, 11-03 로봇 실측)
            r_ = FIT.fit(L, Rr, min_iou=a.min_iou, init=[*res[-1]["M"], res[-1]["yaw"]] if st.get("quick") and res else None)
            if r_ is not None and r_["ok"]: res.append(r_)
    finally:
        st["measuring"] = False
    st["t_look"] = time.time() - t_m
    if run_dir[0]:  # 원본 한 쌍 보관 (나중에 노트북에서 다시 계산할 수 있게) — 저장은 뒤에서 (0.13 s)
        stamp, dir_, pair = time.strftime("%H%M%S"), run_dir[0], (last[0].copy(), last[1].copy())
        threading.Thread(target=lambda: [cv2.imwrite(os.path.join(dir_, "%s_%s_%s.png" % (stamp, tag, k)), im) for k, im in zip("LR", pair)], daemon=True).start()
    if len(res) >= n // 2 + 1:
        P = np.median([r_["M"] for r_ in res], axis=0)
        res = [r_ for r_ in res if np.linalg.norm(r_["M"] - P) < 10.0]
    if len(res) >= n // 2 + 1:
        P = np.median([r_["M"] for r_ in res], axis=0); yaw = float(np.median([r_["yaw"] for r_ in res]))
        spread = float(np.max(np.linalg.norm(np.array([r_["M"] for r_ in res]) - P, axis=1)))
        iou = float(np.median([min(r_["iou"]) for r_ in res]))
        if spread <= 5.0:
            x, y = to_body(P)
            m = dict(x=x, y=y, yaw=yaw, spread=spread, n=len(res), iou=iou)
            st["pred"] = dict(x=x, y=y, yaw=yaw, t=time.time())      # 2임무: 이후 한쪽 카메라가 가려지면 이 값을 움직인 만큼 옮겨 씀
            st["last"] = m; st["overlay"] = dict(fl=res[-1]["hull_l"], fr=res[-1]["hull_r"], t=time.time())
            log("[%s] 큐브: 앞 %.1f cm, %s %.1f cm, 각도 %.1f°, 겹침 %.3f, %d장 흔들림 %.1f mm (재기 %.2f 초)"
                % (tag, 100 * x, "왼쪽" if y > 0 else "오른쪽", 100 * abs(y), yaw, iou, len(res), spread, st.get("t_look", 0)))
            snap(tag, last[0], last[1], dict(fl=res[-1]["hull_l"], fr=res[-1]["hull_r"])); return m, 0
        log("[%s] 측정 흔들림 큼 (%.1f mm)" % (tag, spread))
    else:
        log("[%s] 두 카메라에 맞는 큐브 없음 (믿을 장 %d/%d)" % (tag, len(res), n))
    # 다음 행동 (10-29 버그: 두 카메라 다 보는데 측정만 흔들린 것을 '한쪽만 보임'으로 보고 엉뚱한 쪽으로 4 cm 감)
    #   두 카메라 다 보이고 안 잘림 → 'retry'(움직이지 않고 다시 잼) / 화면 끝에 잘림 → 그 끝 쪽 / 한 카메라만 → 그 카메라 쪽 / 둘 다 없음 → 0
    snap(tag + "_fail", last[0], last[1])
    vis, cut = [], []
    for img in last:
        mk = mask_of(img)
        vis.append(mk is not None)
        if mk is not None:
            xs = np.nonzero(mk)[1]
            cut.append(-1 if xs.min() <= 3 else (1 if xs.max() >= mk.shape[1] - 4 else 0))   # 화면 왼쪽 끝 = 로봇 왼쪽(+), 오른쪽 끝 = 오른쪽(−)
    if not any(vis): return None, 0
    if all(vis) and not any(c != 0 for c in cut): return None, "retry"
    # 얼마나 옆으로: 보이는 카메라의 빨간 영역 가운데 방향 × 큐브까지 깊이(직전 측정, 없으면 정면 멈춤 거리) → 큐브 좌우 어림
    k = 0 if vis[0] else 1
    mk = mask_of(last[k]); u = float(np.nonzero(mk)[1].mean()); v_ = float(np.nonzero(mk)[0].mean())
    Kc, Dc = (KL, DL) if k == 0 else (KR, DR)
    xn = float(cv2.undistortPoints(np.array([[[u, v_]]], np.float32), Kc, Dc)[0, 0, 0])
    lm = st.get("last"); depth = (lm["x"] - a.rig_forward) if lm else 0.23
    cam_y = 0.0926 if k == 0 else -0.0926
    y_est = cam_y - depth * xn
    if any(c != 0 for c in cut): y_est += 0.025 * (1 if [c for c in cut if c][0] < 0 else -1)      # 잘린 만큼 더 바깥
    return None, ("move", max(-0.10, min(0.10, y_est - a.grip_left)))


# ---------------- 움직이기 ----------------
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init(); node = Node("cube_station"); pub = node.create_publisher(Twist, "/cmd_vel", 10); pose = {}


def _odom(msg):
    q = msg.pose.pose.orientation
    pose["p"] = (msg.pose.pose.position.x, msg.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    pose["t"] = time.time()


node.create_subscription(Odometry, "/odometry/filtered", _odom, 10)
# ── 라이다로 동작 전 확인 (10-30): 잡기 동작(찾기·회전·옆맞춤·들어가기)이 몸이 지나갈 자리를 벽에서 CLEAR 이상 두는지.
#    몸 모양 = 현장판 turnspot.PARTS (시뮬레이션·가상 시험과 같은 몸). 라이다는 16 cm 높이라 5 cm 큐브는 벽으로 안 잡힘.
import shapely
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
sys.path.insert(0, os.path.expanduser("~/mecanum_slam_ws/mission1_field"))
import turnspot as TS
LASER = (0.095, 0.0, 1.641)                                   # base_link → laser (robot.launch 정적 TF)
CLEAR = 0.01
scan = {}
_FOOTBUF = TS.FOOT.buffer(0.005)


def _scan(m):
    r = np.asarray(m.ranges, float); ang = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER[2]
    ok = np.isfinite(r) & (r > 0.15) & (r < 1.5)
    P = np.column_stack([LASER[0] + r[ok] * np.cos(ang[ok]), LASER[1] + r[ok] * np.sin(ang[ok])])
    if len(P): P = P[~shapely.contains_xy(_FOOTBUF, P[:, 0], P[:, 1])]       # 몸에 맞은 점(자기 자신) 버림
    scan.update(pts=P, t=time.time())


node.create_subscription(LaserScan, "/scan", _scan, qos_profile_sensor_data)


def _clear_of(geom):
    t0 = time.time()       # 수신이 잠깐 밀렸으면(계산 중) 새 값을 1 초까지 기다림 (10-30 03:35 '라이다 없음'으로 첫 시도 실패)
    while ("pts" not in scan or time.time() - scan["t"] > 0.6) and time.time() - t0 < 1.0: time.sleep(0.02)
    if "pts" not in scan or time.time() - scan["t"] > 0.6: raise Stop("라이다 없음 — 1 초 기다려도 새 값 없음")
    P = scan["pts"]
    return 9.0 if len(P) == 0 else float(shapely.distance(geom, shapely.multipoints(P)))


def _limit():
    """지켜야 할 간격: CLEAR, 단 지금 서 있는 자리가 이미 그보다 가까우면 지금 간격(−2 mm 흔들림 여유).
    → 벽 쪽으로 더 가까워지는 움직임만 막고, 나란히 가거나 멀어지는 움직임은 허용
    (10-30 02:27: 정면 멈춤 자리에서 뒤 상자까지 1 cm 남짓 → 왼쪽 1.1 cm 옆맞춤을 '공간 없음'으로 막아 임무 중단)."""
    return min(CLEAR, _clear_of(TS.FOOT) - 0.002)


def safe_fraction(dx, dy):
    """몸 기준 (dx, dy) 직선 이동 중 지켜야 할 간격(_limit) 이상인 최대 비율 (0~1)."""
    lim = _limit()
    if _clear_of(TS.sweep_trans(0, 0, dx, dy, 0.0)) >= lim: return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        if _clear_of(TS.sweep_trans(0, 0, dx * mid, dy * mid, 0.0)) >= lim: lo = mid
        else: hi = mid
    return lo


def _nearest_txt(dx, dy):
    """막은 점이 어디인지 기록용: 이동 전체 자리에서 가장 가까운 라이다 점 (몸 기준 앞/왼 cm, 몸까지 cm)."""
    P = scan.get("pts")
    if P is None or len(P) == 0: return "가까운 점 없음"
    d = shapely.distance(TS.sweep_trans(0, 0, dx, dy, 0.0), shapely.points(P)); i = int(np.argmin(d))
    d0 = float(shapely.distance(TS.FOOT, shapely.points(P[i])))
    return "가장 가까운 점 앞 %+.1f 왼 %+.1f cm (지금 몸에서 %.1f cm, 명령대로 다 가면 %.1f cm)" % (100 * P[i, 0], 100 * P[i, 1], 100 * d0, 100 * float(d[i]))


def safe_move(dx, dy, v, label, min_v, **kw):
    f = safe_fraction(dx, dy)
    if f < 1.0:
        log("  %s: 라이다로 보니 벽 %.0f cm 안 → %.1f cm 만 감 (명령 %.1f cm) · %s" % (label, 100 * _limit(), 100 * f * math.hypot(dx, dy), 100 * math.hypot(dx, dy), _nearest_txt(dx, dy)))
        if f * math.hypot(dx, dy) < 0.003: return 0.0
    move_rel(dx * f, dy * f, v, label, min_v, **kw)
    p = st.get("pred")
    if p is not None:                                   # 우리가 움직인 만큼 큐브는 몸 기준으로 반대로 옮겨 보임. 짧은 옆걸음은 명령보다 약 side_gain 배 감 (11-03 실측)
        side = dy * f * (st.get("side_gain", 2.0) if st.get("quick") and dx == 0.0 else 1.0)
        p["x"] -= dx * f; p["y"] -= side
    return f


def safe_turn(dyaw):
    st["pred"] = None                                  # 돌면 예측값 버림 (2임무는 안 돎)
    c_ = _clear_of(TS.sweep_rot(0, 0, 0.0, dyaw))
    if c_ < _limit():
        log("  회전 %.1f°: 몸이 도는 자리가 벽에서 %.1f cm → 돌지 않음" % (math.degrees(dyaw), 100 * c_)); return False
    turn_rel(dyaw); return True
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
pub_img = node.create_publisher(CompressedImage, "/cube_station/view/compressed", 2)
from sensor_msgs.msg import Image as RosImage
pub_raw = node.create_publisher(RosImage, "/cube_station/view_small", 2)   # 노트북 RViz 에 압축 이미지 플러그인이 없어 작은 일반 이미지도 (320×360 세로, 3 Hz)
_raw_t = [0.0]
pub_mk = node.create_publisher(MarkerArray, "/cube_station/cube", 2)
_ros_t = [0.0]


def _cube_marker(i, x, y, yaw_deg, rgba, ns):
    m = Marker(); m.header.frame_id = "base_link"; m.header.stamp = node.get_clock().now().to_msg()
    m.ns, m.id, m.type, m.action = ns, i, Marker.CUBE, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.0
    th = -math.radians(yaw_deg) / 2.0                      # 큐브 각도(위에서 시계 +) → ROS yaw(반시계 +)
    m.pose.orientation.z, m.pose.orientation.w = math.sin(th), math.cos(th)
    m.scale.x = m.scale.y = m.scale.z = SIDE / 1000.0
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.lifetime.nanosec = 500_000_000
    return m


def ros_publish():
    """RViz: 큐브 상자(노랑 = 실시간 추적, 초록 = 멈춰서 잰 값, base_link 기준) + 카메라 화면 5 Hz."""
    ma = MarkerArray()
    if live.get("ok"): ma.markers.append(_cube_marker(0, live["x"], live["y"], live["yaw"], (1.0, 0.85, 0.0, 0.8), "live"))
    lm = st.get("last")
    if lm and time.time() - st["overlay"].get("t", 0) < 3: ma.markers.append(_cube_marker(1, lm["x"], lm["y"], lm["yaw"], (0.1, 0.9, 0.2, 0.9), "measured"))
    if ma.markers: pub_mk.publish(ma)


pub_v = node.create_publisher(CompressedImage, "/cube_station/view_v/compressed", 2)   # 세로(왼쪽 위·오른쪽 아래) JPEG, RViz 압축 이미지용


def rviz_image_loop():
    """RViz 카메라 화면은 따로 (10-30: 추적 반복 안에 있어 재는 동안 화면이 멈춤). 받는 쪽이 있을 때만 만듦.
    view_v = 세로 JPEG 10 Hz (노트북에 compressed-image-transport 설치 뒤), view_small = 원본 3 Hz (설치 전), view = 가로 JPEG 5 Hz."""
    tv = ts = tc = 0.0
    while True:
        now = time.time(); want_v = pub_v.get_subscription_count() > 0 and now - tv > 0.1
        want_s = pub_raw.get_subscription_count() > 0 and now - ts > 0.33
        want_c = pub_img.get_subscription_count() > 0 and now - tc > 0.2
        if want_v or want_s or want_c:
          try:
            v_ = view(); stamp = node.get_clock().now().to_msg()
            if want_c:
                tc = now; msg = CompressedImage(); msg.header.stamp = stamp; msg.header.frame_id = "cube_cameras"; msg.format = "jpeg"
                msg.data = cv2.imencode(".jpg", v_, [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes(); pub_img.publish(msg)
            if want_v or want_s:
                h_ = v_.shape[1] // 2
                vert = np.vstack([v_[:, :h_], v_[:, h_:2 * h_]])
                if want_v:
                    tv = now; sm = cv2.resize(vert, (400, 450), interpolation=cv2.INTER_AREA)
                    msg = CompressedImage(); msg.header.stamp = stamp; msg.header.frame_id = "cube_cameras"; msg.format = "jpeg"
                    msg.data = cv2.imencode(".jpg", sm, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes(); pub_v.publish(msg)
                if want_s:
                    ts = now; sm = cv2.resize(vert, (320, 360), interpolation=cv2.INTER_AREA)
                    im = RosImage(); im.header.stamp = stamp; im.header.frame_id = "cube_cameras"
                    im.height, im.width, im.encoding, im.step = sm.shape[0], sm.shape[1], "bgr8", sm.shape[1] * 3; im.data = sm.tobytes(); pub_raw.publish(im)
          except Exception as e:                      # 화면 보내기 실패로 이 반복이 죽지 않게
            print("RViz 화면 보내기 오류:", repr(e), flush=True); time.sleep(0.5)
        time.sleep(0.02)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()


def stop():
    for _ in range(5): pub.publish(Twist()); time.sleep(0.02)


def check_motion():
    if "p" not in pose or time.time() - pose["t"] > 0.5: raise Stop("/odometry/filtered 없음 — 기반을 먼저 켤 것")
    # 임무 노드가 맡긴 잡기(owner=mission)는 임무 노드가 기다리며 바퀴 명령을 안 냄 → 발행자 수 검사 생략
    if not st.get("owner_mission") and node.count_publishers("/cmd_vel") > 1: raise Stop("/cmd_vel 에 다른 발행자 — 키보드·임무 끌 것")


def move_rel(dx, dy, v, label, min_v, tol=0.005, slow_last=0.0, v_slow=None, timeout=15.0, dyaw=0.0, kw=1.5):
    check_motion(); x0, y0, h0 = pose["p"]; t0 = time.time(); d = 0.0
    st["overlay"] = {}                     # 초록 = 멈춰서 잰 값 → 움직이기 시작하면 지움 (남겨 두면 뒤처져 따라오는 것처럼 보임)
    while True:
        if st.get("abort"): stop(); raise Stop("멈춤 버튼")
        if time.time() - pose["t"] > 0.5: stop(); raise Stop("오돔 끊김")
        x, y, h = pose["p"]; ex, ey = x - x0, y - y0
        px = ex * math.cos(h0) + ey * math.sin(h0); py = -ex * math.sin(h0) + ey * math.cos(h0)
        rx, ry = dx - px, dy - py; d = math.hypot(rx, ry)
        if d < tol: break
        if time.time() - t0 > timeout: stop(); raise Stop("%s 시간 초과" % label)
        if st.get("watch") is not None and st["watch"].vanished(time.time()): stop(); raise Stop("큐브가 그리퍼로 들어오기 전에 사라짐")
        sp = v_slow if (v_slow and d < slow_last) else v; sp = max(min_v, min(sp, 1.5 * d))
        ch, sh = math.cos(h - h0), math.sin(h - h0)
        tw = Twist(); tw.linear.x = sp * (rx * ch + ry * sh) / d; tw.linear.y = sp * (-rx * sh + ry * ch) / d
        tw.angular.z = kw * math.atan2(math.sin(h0 + dyaw - h), math.cos(h0 + dyaw - h)); pub.publish(tw); time.sleep(0.05)
    stop(); log("  %s 끝 (남은 %.1f mm)" % (label, 1000 * d))


def turn_rel(dyaw, timeout=10.0, tol_deg=2.5):  # 왼쪽 +, 도는 동안 제자리 유지
    check_motion(); x0, y0, h0 = pose["p"]; t0 = time.time(); ht = h0 + dyaw; ew = dyaw
    st["overlay"] = {}
    while True:
        if st.get("abort"): stop(); raise Stop("멈춤 버튼")
        if time.time() - pose["t"] > 0.5: stop(); raise Stop("오돔 끊김")
        x, y, h = pose["p"]; ew = math.atan2(math.sin(ht - h), math.cos(ht - h))
        if abs(ew) < math.radians(tol_deg): break
        if time.time() - t0 > timeout: stop(); raise Stop("회전 시간 초과 (남은 %.1f°)" % math.degrees(ew))
        # 최소 0.75 rad/s: 주행 코드 0.45 는 90° 회전용 — 15° 같은 작은 회전은 0.54 rad/s 로 10 초에 2.8° 만 돎(10-29, 바퀴 출력 25~40%)
        w = max(-1.2, min(1.2, 2.0 * ew)); w = math.copysign(max(abs(w), a.min_w), ew)
        ex, ey = x0 - x, y0 - y; bx = ex * math.cos(h) + ey * math.sin(h); by = -ex * math.sin(h) + ey * math.cos(h)
        tw = Twist(); tw.linear.x = max(-0.08, min(0.08, 3.0 * bx)); tw.linear.y = max(-0.08, min(0.08, 3.0 * by)); tw.angular.z = w
        pub.publish(tw); time.sleep(0.05)
    stop(); log("  회전 끝 (남은 %.1f°)" % math.degrees(ew))


def gripper(us):
    fd = os.open(a.port, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        # 한 번에 한 줄 (주행 프로그램 명령과 안 섞임, 열린 포트라 재시작 없음).
        # 주행 프로그램이 50 Hz 로 같은 포트에 쓰므로 출력 대기열이 차 있으면 EAGAIN → 잠깐 뒤 다시 (10-29 그리퍼 열기 실패)
        for i in range(400):
            try: os.write(fd, ("G,%d\n" % us).encode()); break
            except BlockingIOError: time.sleep(0.005)
        else: raise Stop("그리퍼 명령을 2 초 동안 못 씀 (포트가 계속 막힘)")
    finally: os.close(fd)
    st["grip_us"] = us
    log("  그리퍼 G,%d" % us)


# ---------------- 잡기 단계 ----------------
def predicted(tag, why):
    """2임무: 발견한 뒤 한쪽(또는 둘 다) 카메라가 상대 차체에 가려지면, 마지막으로 잰 자리를 우리가 움직인 만큼 옮긴 예측값으로 계속 잡음 (11-03 사용자)."""
    p = st.get("pred")
    if not st.get("battle") or p is None or time.time() - p["t"] > 15.0:
        return None
    log("[%s] %s → 예측값으로 계속: 앞 %.1f cm, %s %.1f cm (%.1f 초 전에 잰 값에서)" % (tag, why, 100 * p["x"], "왼쪽" if p["y"] > 0 else "오른쪽", 100 * abs(p["y"]), time.time() - p["t"]))
    return dict(x=p["x"], y=p["y"], yaw=p["yaw"], spread=0.0, n=0, iou=0.0, pred=True)


def look_or_seek(tag):
    moves, blind = 0, 0
    for i in range(5):
        m, side = look(tag if i == 0 else "%s 다시%d" % (tag, i))
        if m is not None: return m
        if side != "retry":
            q = predicted(tag, "두 카메라 모두 가려짐" if side == 0 else "한쪽 카메라 가려짐")
            if q is not None: return q
        if side == 0:
            # 한 번 못 본 것(사람이 가림·사진 한 장 늦음)으로 바로 끝내지 않음: 두 번까지 다시 봄
            blind += 1
            if blind > (0 if st.get("battle") else 2): raise Stop("큐브가 두 카메라 모두에 안 보임 (%d번 봄)" % blind)   # 2임무: 기록 225 개에서 다시 봐서 살린 적 0 번, 바로 순회로
            log("  두 카메라 모두 큐브 없음 → 다시 봄 (%d/2)" % blind); time.sleep(0.3); continue
        if side == "retry":
            log("  두 카메라 다 보임 → 움직이지 않고 다시 잼"); continue
        if moves == 2: break
        moves += 1
        need = side[1]
        log("  찾기: 큐브가 한쪽 카메라에만(잘림) → 보이는 방향으로 계산하면 %s %.1f cm" % ("왼쪽" if need > 0 else "오른쪽", 100 * abs(need)))
        if safe_move(0.0, need, a.v, "찾기 옆으로", a.min_v_side) == 0.0: raise Stop("찾으러 옆으로 갈 공간이 없음")
    raise Stop("다시 재거나 옆으로 가도 두 카메라에 같이 안 잡힘")


def grab(dry):
    st["state"] = "잡기 (계획만)" if dry else "잡기"
    st["pred"] = None
    if st.get("battle") and live.get("ok") and time.time() - live.get("t", 0) < 1.0:     # 막 멈춘 자리에서 실시간으로 본 큐브 = 첫 예측값
        st["pred"] = dict(x=live["x"], y=live["y"], yaw=live.get("yaw", 0.0), t=time.time())
    m = look_or_seek("보기 1") if not dry else look("보기 1")[0]
    if m is None: raise Stop("큐브를 두 카메라에서 못 찾음")
    fwd = m["x"] - a.grip_x + a.push; dy = m["y"] - a.grip_left
    log("계획: %s옆 %s %.1f cm → 앞 %.1f cm → G,%d" % ("돌기 %.1f° → " % -m["yaw"] if a.turn_min <= abs(m["yaw"]) <= a.turn_max else "",
                                                 "왼쪽" if dy > 0 else "오른쪽", 100 * abs(dy), 100 * fwd, a.grip_us))
    if not (0.0 < fwd < a.max_forward and abs(m["y"]) < a.max_side): raise Stop("시험 범위 밖 (앞 %.2f m, 옆 %.2f m)" % (fwd, m["y"]))
    if dry: return
    was_open = st.get("grip_us") == a.open_us
    gripper(a.open_us); time.sleep(0.3 if st.get("quick") and was_open else 0.5)   # 이미 열려 있으면 덜 기다림
    if a.turn_min <= abs(m["yaw"]) <= a.turn_max:
        yaw0 = m["yaw"]
        if st.get("owner_mission"):
            log("  임무 중엔 잡기 회전 안 함 (정면 멈춤 칸 뒤 여유 2.4 cm·라이다 뒤쪽 비스듬한 방향 가림) → 막대로 밀어 반듯하게")
        elif safe_turn(-math.radians(yaw0)):            # 돌 공간이 없으면 안 돌고 아래 '막대로 밀어 반듯하게'로
            m = look_or_seek("돈 뒤")
            if abs(m["yaw"]) > abs(yaw0) + 3: raise Stop("돈 뒤 각도가 오히려 커짐 (%.1f° → %.1f°) — 회전 방향 확인 필요" % (yaw0, m["yaw"]))
    # 옆맞춤: 목표 2.5 mm (10-29: 5 mm 로 두니 딱 5 mm 오른쪽으로 쏠려 들어감). 메카넘은 몇 mm 미는 게 어려워
    # 3번 안에 2.5 mm 가 안 되면 5 mm 안이면 들어감, 그것도 아니면 멈춤
    # 짧은 옆 이동은 멈춘 뒤 굴러가 매번 약 5 mm 더 감(10-29: 0.6 cm 명령 → 1.1 cm, 목표를 사이에 두고 왔다 갔다) →
    # 카메라로 '명령 vs 실제 간 거리'를 재서 더 간 양(coast)을 배우고 다음 명령에서 그만큼 덜 감 (바닥·배터리가 달라도 따라감)
    tries = a.side_tries
    for i in range(tries + 1):
        dy = m["y"] - a.grip_left
        # 5 mm 안이면 끝: 짧게 밀면 약 5 mm 더 가서(굴러감) 그보다 작은 오차는 바퀴로 못 고침 — 고치려다 왔다 갔다·시간만 씀(10-29)
        if abs(dy) <= a.side_tol or i == tries: break
        if st.get("quick") and -0.005 <= dy <= 0.010: break         # 왼쪽은 13.5 mm 도 잡힘 (10-31). 7 mm 를 맞추려다 5 번 왕복 7 초 (11-03)
        cmd = dy - math.copysign(min(st["coast"], 0.8 * abs(dy)), dy)
        if st.get("quick"):     # 굴러감을 빼는 대신 '명령 대비 실제로 간 비율'을 배움. 이 바닥 실측 5.5→14, 8.3→24, 17→43, 22→54 mm 라 약 2.5 배 (11-03)
            cmd = dy / st.get("side_gain", 2.0)
            if abs(cmd) < 0.004: cmd = math.copysign(0.004, dy)   # 3 mm 안쪽 명령은 아예 안 움직임
        y_before = m["y"]
        # 라이다가 조금 줄였으면(덜 감) 다시 재서 판단 — 남은 오차가 허용 안이면 그대로 들어감. 전혀 못 움직일 때만 멈춤
        # (10-30 02:27: 1.1 cm 중 1.096 cm 가서 남은 1.9 mm 인데 '100% 아니면 실패'로 임무 중단)
        if safe_move(0.0, cmd, a.v, "옆맞춤 %d (명령 %.1f mm, 굴러감 %.1f mm 뺌)" % (i + 1, 1000 * cmd, 1000 * st["coast"]), a.min_v_side, tol=0.003) == 0.0:
            raise Stop("옆맞춤할 공간이 없음 (벽)")
        m = look_or_seek("옆맞춤 뒤 %d" % (i + 1))
        actual = y_before - m["y"]                                  # 큐브가 옆으로 옮겨 보인 만큼 = 로봇이 실제로 간 거리
        over = (actual - cmd) * math.copysign(1.0, cmd)             # 더 간 양 (+)
        if st.get("quick"): st["side_gain"] = max(1.0, min(3.5, 0.5 * st.get("side_gain", 2.0) + 0.5 * actual / cmd))
        st["coast"] = max(0.0, min(a.coast_max, 0.5 * st["coast"] + 0.5 * over))   # 적분 펌웨어 뒤 3 cm 넘게 더 가기도 함 (11-01 17:32: 3.4 cm 명령 → 6.7 cm)
    dy = m["y"] - a.grip_left
    if abs(dy) > a.side_max: raise Stop("옆맞춤 %d번 해도 %.0f mm 안에 안 들어옴 (%.1f mm)" % (a.side_tries, 1000 * a.side_max, 1000 * dy))
    log("옆맞춤 결과: 그리퍼 가운데에서 %s %.1f mm" % ("왼쪽" if dy > 0 else "오른쪽", 1000 * abs(dy)))
    fwd = m["x"] - a.grip_x + (a.push_battle if st.get("battle") else a.push)
    th = math.radians(min(45.0, abs(m["yaw"])))
    if abs(m["yaw"]) >= a.turn_min:
        # 돌아간 큐브: 대각선으로 물면 모서리가 튀어나옴(10-29 사진, 26°) → 모서리가 먼저 막대에 닿는 만큼 + 1 cm 더 밀어
        # 흰 막대가 큐브를 돌려 면을 붙이게 한 뒤 쥠
        extra = SIDE / 1000.0 / 2 * (math.cos(th) + math.sin(th) - 1) + 0.010
        fwd += extra; log("큐브 %.1f° 돌아감 → 막대로 밀어 반듯하게: %.1f cm 더 밀기" % (m["yaw"], 100 * extra))
    if not (0.0 < fwd < a.max_forward): raise Stop("들어갈 거리 이상 (%.2f m)" % fwd)
    log("들어가기 %.1f cm (마지막 10 cm 천천히)" % (100 * fwd))
    if st.get("battle"): st["watch"] = EntryWatch()    # 2임무: 큐브가 화면 아래로 빠져 들어오는지 지켜봄 (live_loop 가 사진을 넣음)
    if safe_move(fwd, 0.0, a.v, "들어가기", a.min_v, slow_last=0.10, v_slow=a.v_slow) < 1.0:
        log("  들어가기가 벽 때문에 덜 감 — 쥐기는 시도")
    time.sleep(0.3)
    if st.get("watch") is not None and st["watch"].vanished(time.time()): raise Stop("큐브가 그리퍼로 들어오기 전에 사라짐")
    gripper(a.grip_us); time.sleep(0.8)
    if st.get("watch") is not None:
        st["went_in"] = st["watch"].went_in(); log("  큐브가 그리퍼로 들어옴: %s" % st["went_in"])
    log("완료: 쥠")


def grab_fast():
    """2임무용 빠른 잡기: 2장씩만 재고, 몸을 큐브 쪽으로 살짝 돌리며 곧게 들어가 바로 쥠. 못 쥐면 battle.py 쥠 확인에서 걸려 다음엔 grab() 으로. 1임무는 안 씀.
    옆걸음으로 mm 를 맞추지 않음: 짧게 밀면 이 바닥에서 1 cm 넘게 더 가고 작은 명령은 아예 안 움직임 (11-03 03:48: 8.4 mm 치우친 채 들어가 큐브를 밂).
    앞으로 가면서 옆까지 가게 해도 옆이 덜 감 (03:35). 몸을 돌리면 바퀴는 앞으로만 굴러 옆 미끄러짐이 없음. 3 cm 넘게 어긋나면 옆걸음 한 번으로 줄임."""
    st["state"] = "잡기"
    gripper(a.open_us)
    see = lambda tag: look(tag, n=2)[0] or look(tag + " 다시")[0]
    m = see("빠른 보기")
    if m is None: raise Stop("큐브가 안 보임")
    if not (0.0 < m["x"] - a.grip_x + a.push < a.max_forward and abs(m["y"]) < a.max_side): raise Stop("시험 범위 밖 (앞 %.2f m, 옆 %.2f m)" % (m["x"], m["y"]))
    if abs(m["y"] - a.grip_left) > 0.03:
        if safe_move(0.0, m["y"] - a.grip_left, a.v, "옆걸음", a.min_v_side, tol=0.003) == 0.0: raise Stop("옆걸음할 공간이 없음 (벽)")
        m = see("옆걸음 뒤")
        if m is None: raise Stop("옆걸음 뒤 큐브가 안 보임")
    gx = a.grip_x - a.push
    if abs(m["yaw"]) >= a.turn_min:   # 돌아간 큐브: grab() 과 같이 막대로 밀어 반듯하게
        th = math.radians(min(45.0, abs(m["yaw"])))
        gx -= SIDE / 1000.0 / 2 * (math.cos(th) + math.sin(th) - 1) + 0.010
    d = 0.0                           # 끝에서 큐브가 몸 기준 (gx, grip_left) 에 오는 방향 d 와 자리. 자리가 그 방향의 직선 위가 되게 몇 번 되풀이
    for _ in range(40):               # 한 번에 약 0.7 배로 줄어드는 반복이라 넉넉히 (4 번이면 5 mm 남음)
        fwd = m["x"] - (gx * math.cos(d) - a.grip_left * math.sin(d))
        dy = m["y"] - (gx * math.sin(d) + a.grip_left * math.cos(d))
        d = math.atan2(dy, fwd)
    if not (0.0 < fwd < a.max_forward): raise Stop("들어갈 거리 이상 (%.2f m)" % fwd)
    log("빠른 들어가기: 앞 %.1f cm, 옆 %+.1f mm, 몸 돌림 %+.1f°" % (100 * fwd, 1000 * dy, math.degrees(d)))
    if safe_move(fwd, dy, a.v_fast, "빠른 들어가기", a.min_v, slow_last=0.10, v_slow=a.v_slow, dyaw=d, kw=3.0) < 1.0:
        log("  들어가기가 벽 때문에 덜 감 — 쥐기는 시도")
    time.sleep(0.1); gripper(a.grip_us); time.sleep(0.5)
    log("완료: 쥠")


def run_action(name, fn, owner=None, battle=False, quick=False):
    if st["busy"]: return "다른 동작 중"
    st["busy"] = True; st["abort"] = False; st["owner_mission"] = owner == "mission"; st["battle"] = battle; st["quick"] = battle or quick; st["result"] = None; st["went_in"] = None
    run_dir[0] = os.path.expanduser("~/mecanum_slam_ws/calib/grab_runs/" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") + name)
    os.makedirs(run_dir[0], exist_ok=True)

    def work():
        try:
            fn(); st["result"] = "ok"
        except Stop as e:
            log("멈춤: %s" % e); st["result"] = "멈춤: %s" % e
        except Exception as e:
            log("오류: %r" % e); st["result"] = "오류: %r" % e
        finally:
            stop(); st["watch"] = None; st["owner_mission"] = False; st["busy"] = False; st["state"] = "대기"
    threading.Thread(target=work, daemon=True).start()
    if name.startswith("grab"):  # 포트폴리오용: 잡는 동안 웹 화면(카메라 두 대 + 선·측정값)을 10 fps 영상으로 저장
        threading.Thread(target=record_view, args=(os.path.join(run_dir[0], "카메라화면.mp4"),), daemon=True).start()
    return "시작"


def record_view(path, fps=15):
    """실제 시간과 같은 속도로 재생되게: 1/fps 마다 한 장, 계산이 밀리면 같은 장을 채워 넣음 (고정 fps 로 쓰고 실제론 덜 찍혀 빨리 감기처럼 되는 것 방지)."""
    vw = None; t_end = None; t0 = time.time(); k = 0; img = None
    while True:
        img = view()
        if vw is None: vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (img.shape[1], img.shape[0]))
        due = int((time.time() - t0) * fps) + 1
        while k < due: vw.write(img); k += 1
        if not st["busy"] and t_end is None: t_end = time.time() + 1.5   # 끝난 뒤 1.5 s 더
        if t_end is not None and time.time() > t_end: break
        time.sleep(max(0.0, t0 + k / fps - time.time()))
    vw.release(); log("  카메라 화면 영상 저장: %s" % os.path.basename(os.path.dirname(path)))


import subprocess, glob as _glob
WS = os.path.expanduser("~/mecanum_slam_ws")
MDIR = os.environ.get("MISSION_DIR", "mission1_field")


def mission_running():
    return subprocess.run(["bash", "-c", "screen -ls | grep -q '\\.mission[[:space:]]'"]).returncode == 0


def node_running():
    for d in _glob.glob("/proc/[0-9]*/cmdline"):
        try:
            if b"mission1_node.py" in open(d, "rb").read(): return True
        except OSError: pass
    return False


def close_finished(wait=0.0):
    """임무가 끝난 화면은 '기반도 끌까요? [y/N]' 에서 엔터를 기다림 (폰 버튼엔 엔터가 없음) → 엔터(=기반 켜 둠)로 닫음.
    물음이 화면 기록에 떠 있을 때만 (준비 중·주행 중엔 건드리지 않음). wait = 물음이 뜨길 기다릴 최대 초."""
    def asking():
        try:
            with open("/tmp/mission_screen.log", errors="replace") as fh: return "끌까요" in fh.read()[-300:]
        except OSError: return False
    t0 = time.time()
    while time.time() - t0 < wait and mission_running() and not asking(): time.sleep(0.3)
    if not mission_running() or node_running() or not asking(): return
    subprocess.run(["screen", "-S", "mission", "-X", "stuff", "\n"])
    t0 = time.time()
    while time.time() - t0 < 5 and mission_running(): time.sleep(0.2)


def _quit_then_close():
    t0 = time.time()
    while time.time() - t0 < 20 and node_running(): time.sleep(0.3)
    close_finished(wait=15.0)


def mission_cmd(x):
    if x == "m_start":
        close_finished()
        if mission_running(): return "이미 켜져 있음"
        subprocess.run(["bash", "-c", "screen -dmS mission bash -c 'MISSION_DIR=%s %s/field_auto.sh 2>&1 | tee /tmp/mission_screen.log'" % (MDIR, WS)])
        return "자율 준비 시작 (기반 확인 → 라이다 칸 판정 → '출발' 기다림)"
    keys = {"m_go": "g", "m_estop": " ", "m_quit": "x"}
    if x in keys:
        if not mission_running(): return "임무가 안 켜져 있음"
        subprocess.run(["screen", "-S", "mission", "-X", "stuff", keys[x]])
        if x == "m_quit": threading.Thread(target=_quit_then_close, daemon=True).start()   # 노드가 끝나면 물음에 엔터
        return "키 '%s' 보냄" % keys[x].replace(" ", "스페이스")
    return "?"


def mission_log(n=12):
    fs = sorted(_glob.glob(os.path.join(WS, MDIR, "logs", "run_*.log")), key=os.path.getmtime)
    src = fs[-1] if fs and time.time() - os.path.getmtime(fs[-1]) < 600 else "/tmp/mission_screen.log"
    try:
        with open(src, errors="replace") as fh: return [l.rstrip() for l in fh.readlines()[-n:]]
    except OSError:
        return []


ACTIONS = {
    "grab": lambda: grab(False), "plan": lambda: grab(True), "grabfast": grab_fast,
    "open": lambda: gripper(a.open_us), "close": lambda: gripper(a.grip_us),
    "back20": lambda: (gripper(a.open_us), time.sleep(0.6), move_rel(-0.20, 0.0, a.v, "뒤로 20 cm", a.min_v)),
}

# ---------------- 웹 화면 ----------------
PAGE = """<html><head><meta charset="utf-8"><title>큐브 잡기</title><style>
body{background:#1e1e1e;color:#eee;font-family:sans-serif;margin:10px}button{font-size:18px;margin:3px;padding:9px 16px;border-radius:6px;border:1px solid #888;background:#333;color:#eee}
.go{background:#2a7}.stop{background:#b33}#st{font-size:17px;line-height:1.5;white-space:pre-wrap;background:#111;padding:8px;border-radius:6px}</style></head><body>
<div style="margin-bottom:6px">임무: <button class=go onclick="a('m_start')">자율 준비</button><button class=go onclick="a('m_go')">출발 / 계속</button>
<button class=stop onclick="a('m_estop')">비상정지</button><button onclick="a('m_quit')">임무 종료</button> <span id=ms></span></div>
<div>잡기: <button class=go onclick="a('plan')">계획만 보기</button><button class=go onclick="a('grab')">잡기 시작</button><button class=stop onclick="a('abort')">멈춤</button>
 &nbsp; <button onclick="a('open')">그리퍼 열기</button><button onclick="a('close')">그리퍼 닫기</button><button onclick="a('back20')">열고 뒤로 20 cm</button></div>
<div id=st></div><div id=ml style="font-size:14px;white-space:pre-wrap;background:#102;padding:6px;border-radius:6px;margin-top:4px;max-height:220px;overflow:auto"></div><img id=v style="width:100%;margin-top:6px">
<script>
function a(x){fetch('/do?a='+x).then(u)}
function u(){fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('st').textContent='잡기 상태: '+j.state+(j.busy?' (움직이는 중 — 건드리지 마세요)':'')+'\\n'+j.log.join('\\n');
 document.getElementById('ms').textContent=j.mission?'● 임무 켜짐':'○ 임무 꺼짐';document.getElementById('ml').textContent='임무 기록\\n'+(j.mlog||[]).join('\\n')})}
setInterval(u,500);u();
const v=document.getElementById('v');function n(){const im=new Image();im.onload=()=>{v.src=im.src;setTimeout(n,30)};im.onerror=()=>setTimeout(n,500);im.src='/frame.jpg?'+Date.now()}n();
</script></body></html>"""


live = {}


def live_loop():
    """실시간 추적(후보): 멈춰 있지 않아도 최신 사진으로 계속 모형 겹치기 → 노란 선·대략 위치. 잡기 판단엔 안 씀(움직이는 중엔
    두 카메라 시각 차·지연·흔들림으로 cm 오차). 잡기 단계가 도는 동안은 계산 자원을 양보."""
    while True:
        if "L" not in frames or "R" not in frames: time.sleep(0.1); continue
        if st.get("measuring"): time.sleep(0.03); continue   # 멈춰서 재는 동안 양보 (10-30: 재기 2~3 초·라이다 수신 밀림 → 큐브 미션 15 초)
        t0 = time.time()
        fl, fr = frames["L"][0], frames["R"][0]
        if st.get("watch") is not None: st["watch"].feed((fl, fr), time.time())
        init = [*live["M"], live["yaw"]] if live.get("ok") and time.time() - live["t"] < 0.5 else None  # 직전 위치에서 이어 찾기
        r_ = FIT.fit(fl, fr, min_iou=a.min_iou, init=init, fast=True)   # 빨간 영역도 절반 크기 (같은 답, 18 → 8 ms)
        # 선을 계산한 바로 그 사진에 그려 보여 줌 (최신 사진 + 조금 전 선이면 선이 뒤따라오는 것처럼 보임, 10-29)
        if r_ is not None and r_["ok"]:
            x, y = to_body(r_["M"]); live.update(ok=True, M=r_["M"], x=x, y=y, yaw=r_["yaw"], iou=min(r_["iou"]), hl=r_["hull_l"], hr=r_["hull_r"],
                                                 t=time.time(), dt=time.time() - t0, imgs=(fl, fr))
        else:
            live.update(ok=False, cut=bool(r_ and r_.get("cut")), t=time.time(), imgs=(fl, fr))
        ros_publish()
        time.sleep(max(0.0, 0.066 - (time.time() - t0)))  # 초당 15번 안팎


threading.Thread(target=live_loop, daemon=True).start()


def view():
    out = []
    ov = st["overlay"] if time.time() - st["overlay"].get("t", 0) < 3 else {}
    lv = live if time.time() - live.get("t", 0) < 0.5 else {}
    for i_, (k, kk, lk) in enumerate((("L", "fl", "hl"), ("R", "fr", "hr"))):
        if k not in frames: out.append(np.zeros((360, 640, 3), np.uint8)); continue
        src = lv["imgs"][i_] if lv.get("imgs") is not None else frames[k][0]          # 선을 계산한 그 사진
        p = cv2.resize(src, (640, 360), interpolation=cv2.INTER_AREA)   # 먼저 줄이고 그림 (화면 끊김 줄이기)
        if lv.get("ok"): cv2.polylines(p, [(lv[lk] * 0.5).astype(np.int32).reshape(-1, 1, 2)], True, (0, 220, 255), 2)   # 노랑 = 실시간 후보
        if kk in ov: cv2.polylines(p, [(ov[kk] * 0.5).astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)          # 초록 = 멈춰서 잰 값
        cv2.putText(p, "LEFT" if k == "L" else "RIGHT", (8, 24), 0, 0.7, (0, 255, 255), 2)
        out.append(p)
    img = np.hstack(out)
    if lv:
        txt = ("LIVE  fwd %.1f cm  %s %.1f cm  yaw %.0f  iou %.2f" % (100 * lv["x"], "L" if lv["y"] > 0 else "R", 100 * abs(lv["y"]), lv["yaw"], lv["iou"])
               if lv.get("ok") else ("LIVE  cube cut at edge" if lv.get("cut") else "LIVE  no cube"))
        if lv.get("ok"): txt += "  %.0f ms" % (1000 * lv.get("dt", 0))
        cv2.putText(img, txt, (8, 350), 0, 0.7, (0, 220, 255), 2)
    return img


class Hd(BaseHTTPRequestHandler):
    def log_message(self, *x): pass

    def send(self, body, ct):
        self.send_response(200); self.send_header("Content-Type", ct); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path); q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if u.path == "/": return self.send(PAGE.encode(), "text/html; charset=utf-8")
        if u.path == "/frame.jpg": return self.send(cv2.imencode(".jpg", view(), [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes(), "image/jpeg")
        if u.path == "/do":
            x = q.get("a")
            if x == "abort": st["abort"] = True; stop(); log("멈춤 버튼")
            elif x and x.startswith("m_"): log("임무: %s → %s" % (x, mission_cmd(x)))
            elif x in ACTIONS: log("요청: %s%s → %s" % (x, " (임무)" if q.get("owner") == "mission" else "", run_action(x, ACTIONS[x], q.get("owner"), q.get("battle") == "1", q.get("quick") == "1")))
        lv = {k: v for k, v in live.items() if k in ("ok", "x", "y", "yaw", "iou", "cut")} if time.time() - live.get("t", 0) < 1.0 else {}
        return self.send(json.dumps(dict(state=st["state"], busy=st["busy"], log=st["log"], last=st["last"], live=lv, result=st.get("result"), went_in=st.get("went_in"), mission=mission_running(), mlog=mission_log()), ensure_ascii=False).encode(), "application/json")


threading.Thread(target=rviz_image_loop, daemon=True).start()   # view() 정의 뒤에 시작
log("큐브 잡기 스테이션 시작 (카메라 계속 켜 둠, 웹 %d)" % a.http)
ThreadingHTTPServer(("0.0.0.0", a.http), Hd).serve_forever()
