#!/usr/bin/env python3
"""1임무 실주행 노드 — 전성환 planner.py(v1.0, 수정 없음)를 메카넘 로봇에 붙인다.

입력  : Cartographer TF map→base_link (위치), /scan (칸 판정)
출력  : /cmd_vel (칸 중심까지 평행 이동·제자리 회전), /mission1/markers (RViz)

좌표  : 노드를 켤 때 로봇이 서 있는 자리 = (1,9) 칸 중심, 정면 = 남쪽(S, (1,8) 쪽).
        칸 (x, y): x = 1..9 왼→오, y = 1..9 아래→위. 칸 크기는 --cell (기본 0.40 m).
        planner 는 400 mm 기준 좌표를 내지만 여기서는 칸 번호만 쓰고 중심은 --cell 로 다시 계산한다.

모드  : 기본은 dry-run (바퀴 안 움직임, 칸 판정·계획만 출력).
        --execute 를 줘야 실제로 움직인다.

  python3 mission1_node.py                 # dry-run
  python3 mission1_node.py --execute       # 실제 주행 (키보드 조종은 먼저 끌 것)
"""
import argparse
import copy
import math
import json
import os
import queue
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool, Empty
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner as P                      # noqa: E402  전성환 planner (수정 없음)
from robot_loop_example import run       # noqa: E402  전성환 제어 루프 (수정 없음)

N = P.N_CELLS
HEADING_YAW = {P.N: 90.0, P.E: 0.0, P.S: -90.0, P.W: 180.0}   # 경기장 기준 yaw [deg]


class MissionAbort(Exception):
    pass


class MoveTimeout(MissionAbort):
    """한 동작이 시간 안에 안 끝남. 자율 주행은 중단, 키보드 모드는 그 동작만 멈추고 계속."""
    pass


class MirrorGrid(P.GridMap):
    """점대칭 경기장 지도. 직접 본 값(cells)과 미러로 아는 값을 나눠 둠 (planner.py 의 symmetric 은 안 씀 — 양쪽에 덮어써서
    가짜 벽·가짜 빈칸 하나가 반대편에서 직접 본 칸까지 지움).
    - 미러 값은 어디에도 써 넣지 않고 그때그때 계산: 원본 칸이 바뀌면 바로 따라 바뀌고, 직접 본 칸은 절대 안 덮임.
    - 칸 안쪽에 점이 찍힌 칸(not_box)은 상자가 아님(상대 로봇 등) → 미러 근거로 안 씀.
    - 길이 막혀 지운 칸(banned)은 직접 볼 때까지 미러 값을 안 씀. mirror_on=False 면 원래 GridMap 과 똑같음."""

    def __init__(self, mirror_on=False, dyn_ttl=3.0):
        super().__init__(symmetric=False, dyn_ttl=dyn_ttl)
        self.mirror_on = mirror_on
        self.not_box, self.banned = set(), set()
        self.dyn_first = {}                            # 빈 칸으로 본 곳에 물체가 처음 보인 시각

    def inferred(self, c):
        """c 를 직접 본 적 없을 때 미러로 아는 값. 없으면 None."""
        if not self.mirror_on or c in self.cells or c in self.banned or not P.inside(c):
            return None
        m = P.mirror(c)
        st = self.cells.get(m)
        return None if st is None or (st == P.OCC and m in self.not_box) else st

    def state(self, c):
        if not P.inside(c):
            return P.OCC
        st = self.cells.get(c)
        if st is not None:
            return st
        st = self.inferred(c)
        return P.UNKNOWN if st is None else st

    def direct_state(self, c):
        return self.cells.get(c, P.UNKNOWN) if P.inside(c) else P.OCC

    def observe(self, c, occupied, t=0.0):
        if not P.inside(c):
            return
        self.direct.add(c)
        self.banned.discard(c)
        if not occupied:
            self.dyn_first.pop(c, None)
        elif self.cells.get(c) == P.FREE:              # 직접 빈 칸으로 본 곳에 물체 → 상대 로봇 (원래 규칙). 미러로만 빈 칸이면 벽으로 믿음
            if c != P.TARGET:
                self.block(c, t + self.dyn_ttl)
                # dyn_ttl 지나서도 또 보이면 그 칸은 막힌 것 (안 그러면 안 보이는 자리로 물러날 때마다 풀려 두 칸을 끝없이 왕복).
                # 나중에 빈 칸으로 보이면 바로 풀림. 비었다 막힌 칸은 상자가 아니므로 미러로는 안 넘김
                t0 = self.dyn_first.setdefault(c, t)
                if t - t0 >= self.dyn_ttl and c not in self.fixed_free:
                    self.not_box.add(c)
                    self._set(c, P.OCC)
            return
        self._set(c, P.OCC if occupied else P.FREE)

    def forget(self, c):
        """길이 막혀 지도를 의심할 때: 이 칸은 직접 다시 볼 때까지 모름."""
        self.cells.pop(c, None)
        self.banned.add(c)


class GuardStop(Exception):
    """충돌 방지 정지: 이 동작만 취소하고 계속(자율 = 다시 계획, 키보드 = 다음 키 대기)."""
    pass




def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Mission1Robot(Node):
    """planner 의 RealRobot 자리에 들어가는 연결부."""

    def __init__(self, a):
        super().__init__("mission1")
        self.a = a
        self.c = a.cell
        self.execute = a.execute
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mk_pub = self.create_publisher(MarkerArray, "/mission1/markers", 1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.occ_scans = np.zeros((N + 1, N + 1))         # [x][y], 1-기반. 스캔마다 decay 배로 줄어드는 증거
        self.free_scans = np.zeros((N + 1, N + 1))
        self.scan_k = 0
        self.prev_scan_yaw = None
        self.start = None              # map 기준 출발 자세 (x0, y0, yaw0)
        self.psi = -math.pi / 2        # 출발 때 정면의 경기장 방향: −90° 남 (바깥벽 오른쪽) / 0° 동 (바깥벽 왼쪽). detect_start_side 가 정함
        self.last_cell = P.HOME
        self.at_grab_point = False
        self.last_pose = None
        self.path = []
        self.grid = None               # planner GridMap (표시용)
        self.mp = None                 # 움직이는 중 재계획용 (복사해서만 씀)
        self.mirror_known, self.mirror_checked, self.map_hist = {}, {}, []   # 미러로 안 칸 · 직접 확인 결과 · 칸 지도 이력
        self.replan_obs = None         # 마지막으로 재계획한 라이다 판정
        self.log_f = open(a.log, "a") if a.log else None
        self.rec_path = (a.log[:-4] if a.log.endswith(".log") else a.log) + ".json" if a.log else ""
        self.trail = []                # 경기장 좌표 이동 기록 [(t, X, Y, yaw)]
        self.events = []               # [(t, 문자열)]
        self.t0 = time.monotonic()
        self.result = "진행 중"
        self._last_dump = 0.0
        self.estop = False             # 비상정지 (스페이스 / /mission1/estop)
        self.quit_req = False          # x 키
        self.keys = queue.Queue()      # 키보드 모드 명령
        self.go_event = threading.Event()   # 자율: 출발 키(엔터 / g)
        self.recheck_req = False
        self.wait_go = False
        self.guard_pts = None          # 몸 기준 라이다 점 (x, y) — 충돌 방지용, 매 스캔
        self.laser_base = None         # base_link 기준 라이다 자세
        self.forced_occ = {}           # 충돌 방지로 막은 칸 → 시각
        self.guard_streak = 0
        self.slip_log = []
        self.wall_log = []
        self.occ_debug = []
        # 제동 거리: 정지 명령 뒤에도 바퀴를 풀어 굴러가는 거리 = 속도 × brake_tb [s]. 앞뒤·옆 따로, 칸 이동마다 다시 잼
        # 능동 제동을 켜면 굴러가는 거리가 훨씬 짧다 → 값을 따로 둔다. 지난 주행에서 배운 값을 파일에서 이어 씀
        self.brake_key = "active" if a.active_brake > 0 else "coast"
        tb0 = a.brake_tb if a.brake_tb is not None else (0.13 if a.active_brake > 0 else 0.30)
        self.brake_tb = {"long": tb0, "lat": tb0}
        self.brake_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "brake_learned.json")
        try:
            saved = json.load(open(self.brake_file)).get(self.brake_key, {})
            for k in ("long", "lat"):
                if a.brake_tb is None and 0.0 <= float(saved.get(k, -1)) <= a.brake_max:
                    self.brake_tb[k] = float(saved[k])
        except Exception:
            pass
        self.last_return_v = 0.0
        self.last_return_why = ""
        self.last_return_odom = None
        self.brake_log = []
        self.straight_run = 0          # 서지 않고 이어 간 칸 수
        self.go_dir = None             # 서지 않고 지나가던 방향
        self.yaw_ref_odom = None       # 이번 직진의 방향 기준 (오돔 yaw)
        self.grabbed = False           # 큐브 집기 끝 (돌아가는 중)
        self.gaps_seen = {}
        self.face_skip = {}                        # 마주 본 벽이 아니라고 버린 앞·뒤 벽 거리 (수, 최소 폭) — 실기 확인용
        self.calib_log = []            # 키보드 측정: 벽 기준 실제 이동 vs 오돔·위치추정
        self.last_cmd = (0.0, 0.0)     # 마지막 이동 명령 (몸 기준) — 능동 제동 방향
        self._wait_t0 = self._last_wait = 0.0   # 자율: 길 막힘(WAIT) 이 이어진 시간
        self.pcorr = (0.0, 0.0)        # 서서 잰 벽 거리로 위치추정에 더하는 보정 [m, 경기장 좌표]
        self.pcorr_at = None           # 보정을 정한 때의 (보정 전) 위치 — 멀어질수록 보정이 줄어듦
        self.create_subscription(Bool, "/mission1/estop", self._on_estop_topic, 10)
        # IMU 자이로 직접: EKF 오돔 방향은 바퀴 회전 속도와 섞여, 큐브가 끌려 도는 걸 실제보다 작게 봄 (실기 10-30 22:07)
        self.imu = None                # (마지막 시각, 회전 속도 z, 적분한 방향) — 자이로 z 는 로봇 z 와 같은 방향(10-21 녹화 기울기 0.98)
        self.create_subscription(Imu, "/imu/data", lambda m: self.imu_feed(m.angular_velocity.z), qos_profile_sensor_data)
        self.create_timer(0.2, self._markers_timer)       # RViz 를 나중에 켜도 바로 보이게 2 Hz 로 다시 보냄

    # ── 기록 ──
    def log(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        print(line, flush=True)
        if hasattr(self, "events"):
            self.events.append((round(time.monotonic() - self.t0, 2), msg))
        if self.log_f:
            self.log_f.write(line + "\n")
            self.log_f.flush()

    # ── 좌표 변환 ──
    def home_xy(self):
        return (P.HOME[0] - 0.5) * self.c, (P.HOME[1] - 0.5) * self.c

    def map_to_arena(self, mx, my):
        x0, y0, th0 = self.start
        dx, dy = mx - x0, my - y0
        fwd = math.cos(th0) * dx + math.sin(th0) * dy
        left = -math.sin(th0) * dx + math.cos(th0) * dy
        hx, hy = self.home_xy()
        c, s_ = math.cos(self.psi), math.sin(self.psi)    # 출발 시 정면 = 경기장 방향 psi (남 −90°: 정면 −Y·왼쪽 +X / 동 0°: 정면 +X·왼쪽 +Y)
        return hx + c * fwd - s_ * left, hy + s_ * fwd + c * left

    def arena_to_map(self, X, Y):
        x0, y0, th0 = self.start
        hx, hy = self.home_xy()
        c, s_ = math.cos(self.psi), math.sin(self.psi)
        fwd, left = c * (X - hx) + s_ * (Y - hy), -s_ * (X - hx) + c * (Y - hy)
        return (x0 + math.cos(th0) * fwd - math.sin(th0) * left,
                y0 + math.sin(th0) * fwd + math.cos(th0) * left)

    def center(self, cell):
        return (cell[0] - 0.5) * self.c, (cell[1] - 0.5) * self.c

    def lookup(self, child, stamp=None):
        when = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
        t = self.tf.lookup_transform("map", child, when)
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation), age

    def tell_start_yaw(self):
        """격자 위치추정기에 출발 방향을 알림 (원점 다시 잡기 전에). 옛 위치추정기(받는 쪽 없음)면 남향만 가능."""
        from std_msgs.msg import Float32
        pub = self.create_publisher(Float32, "/grid_loc/start_yaw", 10)
        t0 = time.monotonic()
        while self.count_subscribers("/grid_loc/start_yaw") == 0 and time.monotonic() - t0 < 2.0: time.sleep(0.05)
        if self.count_subscribers("/grid_loc/start_yaw") == 0:
            if abs(wrap(self.psi + math.pi / 2)) > 0.1:
                raise MissionAbort("위치추정기가 출발 방향을 못 받는 옛 판 — 기반을 다시 켜야 동향 출발 가능 (~/mecanum_slam_ws/field_stop.sh 뒤 다시)")
            return
        for _ in range(5):
            pub.publish(Float32(data=float(math.degrees(self.psi)))); time.sleep(0.05)

    def reset_origin(self, timeout=3.0):
        """기반(격자 위치추정)이 켜져 있으면 지금 자리를 원점으로 다시 잡게 한다. 원점이 바뀐 것(map→base_link ≈ 0)을 확인."""
        self.grid_origin = False
        pub = self.create_publisher(Empty, "/grid_loc/reset", 10)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if pub.get_subscription_count() > 0:
                break
            time.sleep(0.05)
        else:
            self.log("격자 위치추정이 없음 (원점 다시 잡기 생략)")
            return False
        pub.publish(Empty())                         # 한 번만 보낸다
        self.grid_origin = True                      # 격자 위치추정의 map 원점 = (1,9) 칸 가운데·남향 (놓인 자리가 조금 틀어져도 곧 격자에 맞춤)
        while time.monotonic() - t0 < timeout:
            try:
                x, y, th, _ = self.lookup("base_link")
                if math.hypot(x, y) < 0.04 and abs(th) < math.radians(5.0):    # 곧바로 벽에 맞춰 몇 cm·몇 도 고쳐도 원점 잡힌 것
                    self.log("원점 다시 잡음: 지금 자리 = 칸 %s 중심" % (P.HOME,))
                    return True
            except Exception:
                pass
            time.sleep(0.05)
        self.log("원점 다시 잡기 확인 못 함 (계속 진행)")
        return False

    def recheck_origin(self, force=False):
        """출발 키 직후 확인: 대기 중엔 안 움직였으니 지금 자리가 출발 칸이어야 함. 위치추정이 끊겼거나 칸 중심·출발 방향에서 벗어나 있으면 그 자리로 다시 잡음.
        (11-01 18:18: 2 분 대기 중 91.7 초 끊겼다 돌아오며 40 cm 튀어 지도가 한 행 밀림. 대기 중에 로봇을 옮겨 놓은 경우도 여기서 맞춰짐.)
        정상이면 조회 한 번뿐이라 출발이 늦어지지 않음.
        force = r 키 (버튼 출발이 출발 직전에 보냄): 들어서 옮기면 바퀴가 안 돌아 오돔으로는 안 움직인 걸로 보이고 준비하던 자리에서 쌓은 칸 판정이 남으므로 무조건 다시 잡음."""
        if not getattr(self, "grid_origin", False):
            return
        why = "r 키" if force else None
        if why is None:
            try:
                x, y, th, age = self.lookup("base_link")
                if age > 0.5:
                    why = "%.1f 초째 안 들어옴" % age
                elif math.hypot(x, y) > self.a.recheck_dist:
                    why = "칸 중심에서 %.1f cm" % (100 * math.hypot(x, y))
                elif abs(th) > math.radians(self.a.recheck_yaw):
                    why = "방향이 %.1f° 틀어짐" % math.degrees(th)
            except Exception as e:
                why = "조회 실패 (%s)" % e
        if why is None:
            return
        self.log("출발 직전 위치 확인: %s → 지금 자리를 출발 칸으로 다시 잡음" % why)
        if self.a.start_side == "auto":                # 준비한 뒤 놓은 방향이 다를 수 있으므로 놓인 자세로 다시 판단. 못 정해도 경기이므로 멈추지 않고 전에 정한 방향으로
            if self.detect_start_side("auto") is None:
                self.log("  주의: 출발 방향이 분명하지 않음 → %s 로 감" % self.start_name())
            self.tell_start_yaw()
        self.reset_origin()
        self.wait_start()
        self.last_pose = None
        with self.lock:                                # 어긋난 위치로 쌓은 칸 판정은 버리고 새로 봄
            self.occ_scans[:] = 0
            self.free_scans[:] = 0
        time.sleep(self.a.recheck_collect)

    def start_name(self):
        return "바깥벽 오른쪽·뒤 출발" if abs(wrap(self.psi + math.pi / 2)) < 0.1 else "바깥벽 왼쪽·뒤 출발"

    def detect_start_side(self, want="auto", wait=3.0):
        """출발 칸 바깥벽이 몸 오른쪽인지 왼쪽인지 → psi. 출발 칸은 구석이라 뒤와 한쪽 옆이 바깥벽.
        구별: 바깥벽 너머는 경기장 밖이라 라이다 점이 거의 없고, 안쪽은 상자 사이·앞쪽으로 먼 경기장이 보임 → 양옆 35 cm 너머 점 수 비교
        (10-30 가상: '옆 줄이 길게 이어지나'만 보면 안쪽에 상자가 줄지어 있을 때 52 대 129 로 애매). 점대칭이라 (9,1) 북향 = 남향, 서향 = 동향.
        want = right/left 면 그대로 (라이다 판단은 기록만), auto 면 라이다로 — 분명하지 않으면 None (임무가 멈추고 지정 요청)."""
        t0, scans = time.monotonic(), []
        while time.monotonic() - t0 < wait and len(scans) < 8:
            g = self.guard_pts
            if g is not None and (not scans or g[0] != scans[-1][0]): scans.append(g)
            time.sleep(0.05)
        far = {"right": 0, "left": 0}; line = {"right": 0, "left": 0}
        for _, px, py in scans:
            rng = (px > -0.30) & (px < 2.50)
            far["left"] += int(np.sum(rng & (py > 0.35))); far["right"] += int(np.sum(rng & (py < -0.35)))
            along = (px > -0.10) & (px < 1.40)
            line["left"] += int(np.sum(along & (np.abs(py - 0.20) < 0.06))); line["right"] += int(np.sum(along & (np.abs(py + 0.20) < 0.06)))
        k = max(1, len(scans))
        fr, fl, lr, ll = far["right"] / k, far["left"] / k, line["right"] / k, line["left"] / k
        seen = None
        if fl >= 20 and fr <= 0.1 * fl and lr >= 5: seen = "right"      # 오른쪽 너머가 비고 오른쪽 옆 벽이 보임 → 바깥벽 오른쪽
        elif fr >= 20 and fl <= 0.1 * fr and ll >= 5: seen = "left"
        side = want if want in ("right", "left") else seen
        self.log("출발 방향 판단: 옆 35 cm 너머 점 (스캔당) 오른쪽 %.0f · 왼쪽 %.0f, 옆 벽 점 오른쪽 %.0f · 왼쪽 %.0f → 라이다 %s%s" % (
            fr, fl, lr, ll, {"right": "바깥벽 오른쪽", "left": "바깥벽 왼쪽", None: "분명하지 않음"}[seen],
            "" if want == "auto" else " · 지정 %s 사용" % {"right": "오른쪽", "left": "왼쪽"}[want]))
        if want in ("right", "left") and seen is not None and seen != want:
            self.log("  주의: 지정한 쪽과 라이다 판단이 다름 — 로봇 놓은 방향 확인")
        if side is None:
            return None
        self.psi = -math.pi / 2 if side == "right" else 0.0
        return side

    def wait_start(self, timeout=10.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                x, y, th, age = self.lookup("base_link")
                if age < 0.5:
                    if getattr(self, "grid_origin", False):
                        # 격자 위치추정은 map 을 격자에 맞춘다 → 경기장 기준은 map 원점 그대로. 지금 위치추정 값(놓인 자리 어긋남)을
                        # 기준으로 잡으면 그만큼 경기장 전체가 틀어짐 (필드 10-21 14:30: −1.6° 로 고정돼 지도 어긋난 채 출발)
                        self.start = (0.0, 0.0, 0.0)
                        self.log("출발 자세: 경기장 기준 = 격자 위치추정 map 원점 ((1,9) 가운데·%s). 지금 로봇은 map (%.3f, %.3f, %.1f°)"
                                 % (self.start_name(), x, y, math.degrees(th)))
                    else:
                        self.start = (x, y, th)
                        self.log("출발 자세 고정: map (%.3f, %.3f, %.1f°) = 칸 %s 중심, %s, 칸 크기 %.2f m"
                                 % (x, y, math.degrees(th), P.HOME, self.start_name(), self.c))
                    return
            except Exception:
                pass
            time.sleep(0.1)
        raise MissionAbort("map→base_link TF 없음 (SLAM 이 켜져 있는지 확인)")

    def pose(self):
        """경기장 좌표 (X, Y [m], yaw [rad])."""
        # 위치추정이 잠깐 늦거나 조회가 한 번 실패해도 바로 임무를 끝내지 않음: 바퀴 세우고 loc_wait 초까지 기다렸다 이어 감
        # (10-21 키보드 시험 3번이 '1.2~3.1 초째 안 들어옴'으로 통째로 중단)
        t_wait = None
        while True:
            try:
                x, y, th, age = self.lookup("base_link"); why = None if age <= 1.0 else "%.1f 초째 안 들어옴" % age
            except Exception as e:
                why = "조회 실패 (%s)" % e
            if why is None: break
            if t_wait is None:
                t_wait = time.monotonic(); self.stop()
                self.log("  위치추정 %s → 바퀴 세우고 최대 %.0f 초 기다림" % (why, self.a.loc_wait))
            if time.monotonic() - t_wait > self.a.loc_wait:
                raise MissionAbort("위치추정 %s — %.0f 초 기다려도 안 돌아옴 (기반 확인)" % (why, self.a.loc_wait))
            self.send(0.0, 0.0, 0.0); time.sleep(0.05)
        if t_wait is not None:
            self.log("  위치추정 다시 들어옴 (%.1f 초 기다림) → 이어 감" % (time.monotonic() - t_wait))
            self.last_pose = None                       # 기다리는 사이 바뀐 만큼을 '튐'으로 보지 않음
        X, Y = self.map_to_arena(x, y)
        yaw = wrap(th - self.start[2] + self.psi)
        if self.last_pose is not None:
            jump = math.hypot(X - self.last_pose[0], Y - self.last_pose[1])
            dt = time.monotonic() - self.last_pose[3]
            if jump > 0.25 and dt < 0.5:
                raise MissionAbort("SLAM 위치가 %.2f m 튐 → 정지" % jump)
        self.last_pose = (X, Y, yaw, time.monotonic())
        if self.pcorr_at is not None and self.a.pcorr_fade > 0:
            kx, ky = self.pcorr_keep(X, Y)
            X, Y = X + kx * self.pcorr[0], Y + ky * self.pcorr[1]
        tn = time.monotonic() - self.t0
        if not self.trail or tn - self.trail[-1][0] > 0.2:
            self.trail.append((round(tn, 2), round(X, 4), round(Y, 4), round(math.degrees(yaw), 1)))
        return X, Y, yaw

    def dump(self, force=False):
        """주행 기록 JSON (draw_run.py 로 그림). 1 초마다, 끝날 때 한 번 더."""
        if not self.rec_path or (not force and time.monotonic() - self._last_dump < 1.0):
            return
        self._last_dump = time.monotonic()
        import json
        with self.lock:
            o, f = self.occ_scans.tolist(), self.free_scans.tolist()
        g = {"%d,%d" % (x, y): int(self.grid.state((x, y))) for x in range(1, N + 1) for y in range(1, N + 1)} if self.grid else {}
        d = {"cell": self.c, "mode": "execute" if self.execute else "dry-run", "result": self.result, "map_hist": self.map_hist,
             "sec": round(time.monotonic() - self.t0, 1), "trail": self.trail, "grid": g, "path": self.path,
             "occ_scans": o, "free_scans": f, "events": self.events[-400:], "start_map": self.start, "start_psi_deg": round(math.degrees(self.psi), 1),
             "states": {"UNKNOWN": P.UNKNOWN, "FREE": P.FREE, "OCC": P.OCC}, "slip": self.slip_log, "walls": self.wall_log, "occ_debug": self.occ_debug,
             "brake": self.brake_log, "calib": self.calib_log}
        tmp = self.rec_path + ".tmp"
        with open(tmp, "w") as fp:
            json.dump(d, fp, ensure_ascii=False)
        os.replace(tmp, self.rec_path)

    def _on_estop_topic(self, m):
        self.estop = bool(m.data)
        self.log("[비상정지] 토픽으로 %s" % ("정지" if self.estop else "해제"))
        if self.estop:
            self.send_zero()

    def get_laser_base(self):
        if self.laser_base is None:
            t = self.tf.lookup_transform("base_link", "laser", rclpy.time.Time()).transform
            self.laser_base = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        return self.laser_base

    def lookup_odom(self):
        t = self.tf.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    # ── 칸 판정 (라이다) ──
    def _on_scan(self, m):
        # 충돌 방지용: 매 스캔, 몸 기준 점 (칸 판정보다 먼저, 거르지 않고)
        try:
            bx0, by0, bth = self.get_laser_base()
            rr = np.asarray(m.ranges, float)
            aa = m.angle_min + m.angle_increment * np.arange(len(rr))
            ok = np.isfinite(rr) & (rr > 0.12) & (rr < 2.5)
            # 벽까지 거리 캘리브레이션 (11-01 실측): 양옆 합이 40 cm 에서 1.05 cm 모자람 → 거리 +0.5 cm, 앞뒤 가운데 놓고 뒤 벽 1.9 cm 짧음 → 라이다 위치 −1.4 cm
            bx0 = bx0 + self.a.laser_dx
            rc_ = rr + self.a.range_bias
            self.guard_pts = (time.monotonic(), bx0 + rc_[ok] * np.cos(bth + aa[ok]), by0 + rc_[ok] * np.sin(bth + aa[ok]))
        except Exception:
            pass
        if self.start is None:
            return
        self.scan_k += 1
        if self.scan_k % self.a.scan_every:
            return
        stamp = getattr(getattr(m, "header", None), "stamp", None)
        try:
            lx, ly, lth, age = self.lookup("laser", stamp)      # 스캔 찍은 시각의 라이다 자세
        except Exception:
            try:
                lx, ly, lth, age = self.lookup("laser")
            except Exception:
                return
        now = time.monotonic()
        prev, self.prev_scan_yaw = self.prev_scan_yaw, (now, lth)
        if prev is not None and abs(wrap(lth - prev[1])) / max(now - prev[0], 1e-3) > self.a.rot_gate:
            return                                              # 도는 중인 스캔은 번져서 칸 판정에 안 씀
        if stamp is None and age > 0.5:
            return
        r = np.asarray(m.ranges, float)
        ang = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > 0.15)          # 무효(inf)는 검은 면에서도 생기므로 빈 칸 증거로 안 씀
        r, ang = r[ok][::2], ang[ok][::2]
        if r.size == 0:
            return
        far = r >= self.a.far_range               # far_range 너머에 맞은 광선: far_range 까지는 비어 있다는 증거만
        mid = (r >= self.a.obs_range) & ~far       # obs_range~far_range: 면을 정면에 가깝게 맞은 점만 장애물 증거
        r_free = np.minimum(r, self.a.far_range + self.a.free_gap)
        th0 = self.start[2]
        # 라이다 원점과 광선 방향을 경기장 좌표로
        OX, OY = self.map_to_arena(lx, ly)
        a_map = lth + ang
        a_ar = a_map - th0 + self.psi           # map→경기장 회전
        ux, uy = np.cos(a_ar), np.sin(a_ar)
        c = self.c
        inner = lambda X, Y, mg: (((X % c) > mg) & ((X % c) < c - mg) & ((Y % c) > mg) & ((Y % c) < c - mg))
        occ = np.zeros((N + 1, N + 1), int)
        free = np.zeros((N + 1, N + 1), int)
        # 맞은 점 → 장애물 칸. 장애물은 격자에 맞춰 놓이므로 맞은 점은 격자선(면) 위에 있다.
        # 가장 가까운 격자선을 면으로 보고 라이다 반대편 칸을 장애물로 친다.
        # 세로·가로 격자선이 둘 다 가까운 점(모서리)은 어느 면인지 애매해서 버린다.
        HX, HY = OX + r * ux, OY + r * uy
        near = ~far & ~mid
        gx, gy = np.round(HX / c) * c, np.round(HY / c) * c
        dx, dy = np.abs(HX - gx), np.abs(HY - gy)
        tol, corner = self.a.face_tol, self.a.corner_tol
        graze = math.sin(math.radians(self.a.graze_deg))  # 면과 이 각도보다 나란히 스친 광선은 잡음에 약해 버린다
        # 두 격자선이 다 가까운 점(모서리 근처)은 광선이 더 정면으로 맞는 쪽 면만 인정한다.
        # (필드 10-21 11:44: (7,3) 세로 면을 15° 로 비스듬히 본 점이 5 cm 어긋나 가로 면으로 분류 → 빈 칸 (6,3) 이 장애물, 유일한 길이 막힘)
        near_v, near_h = dx < tol, dy < tol
        ax_, ay_ = np.abs(ux), np.abs(uy)
        # 모서리 근처는 버린다. 멀수록 몸 방향 몇 도 오차로 점이 더 밀리므로 버리는 범위를 거리에 비례해 넓힘
        # (필드 10-21 12:50: 1 m 앞 (2,5) 모서리 점이 5 cm 밀려 (1,5) 를 장애물로 → 1열 막다른 길에서 길 막힘)
        ce = corner + r * math.tan(math.radians(self.a.corner_yaw))
        on_v = near_v & (dy > ce) & (ax_ > graze) & ~(near_h & (ay_ > ax_))      # 세로 면 (x = gx)
        on_h = near_h & (dx > ce) & (ay_ > graze) & ~(near_v & (ax_ >= ay_)) & ~on_v   # 가로 면 (y = gy)
        ix = np.where(on_v, np.round(gx / c).astype(int) + np.where(OX < gx, 1, 0), np.floor(HX / c).astype(int) + 1)
        iy = np.where(on_h, np.round(gy / c).astype(int) + np.where(OY < gy, 1, 0), np.floor(HY / c).astype(int) + 1)
        # 칸 한가운데 쪽 점 (상대 로봇 등): 들어 있는 칸. 위치 오차로 밀린 벽 점이 끼지 않게 가까운 점·격자선에서 먼 점만
        off = inner(HX, HY, self.a.off_margin) & (r < self.a.off_range)
        headon = math.cos(math.radians(self.a.far_incidence))
        sel = ((on_v | on_h | off) & near) | (mid & ((on_v & (np.abs(ux) > headon)) | (on_h & (np.abs(uy) > headon))))
        kind = np.where(on_v, 1, np.where(on_h, 2, 3))[sel]       # 1 세로면 2 가로면 3 칸 안 물체
        dbg_xy, dbg_r, dbg_a = np.c_[HX[sel], HY[sel]], r[sel], ang[sel]
        ix, iy = ix[sel], iy[sel]
        hit_ins = (ix >= 1) & (ix <= N) & (iy >= 1) & (iy <= N)
        np.add.at(occ, (ix[hit_ins], iy[hit_ins]), 1)
        # 광선이 지나간 곳: 맞은 점 앞 free_gap 까지, 칸 가장자리 띠는 빼고 = 빈 칸 증거
        steps = np.arange(0.05, self.a.far_range, 0.03)
        S_ = steps[None, :]
        valid = S_ < (r_free[:, None] - self.a.free_gap)
        FX, FY = OX + S_ * ux[:, None], OY + S_ * uy[:, None]
        sel = valid & inner(FX, FY, 0.04)
        jx, jy = np.floor(FX[sel] / c).astype(int) + 1, np.floor(FY[sel] / c).astype(int) + 1
        ins = (jx >= 1) & (jx <= N) & (jy >= 1) & (jy <= N)
        np.add.at(free, (jx[ins], jy[ins]), 1)
        # 장애물 증거가 처음 생기는 칸: 어떤 점 때문인지 기록 (잘못 찍힘 원인 찾기용, 칸마다 처음 몇 번만)
        try:
            X0, Y0 = self.map_to_arena(*self.lookup("base_link")[:2])
            for (cx_, cy_) in zip(*np.where(occ >= self.a.min_pts)):
                if self.occ_scans[cx_, cy_] < 0.5 and len(self.occ_debug) < 300:
                    m_ = hit_ins & (ix == cx_) & (iy == cy_)
                    idx = np.where(m_)[0][:6]
                    self.occ_debug.append({"t": round(time.monotonic() - self.t0, 2), "cell": [int(cx_), int(cy_)],
                                           "robot": [round(X0, 3), round(Y0, 3)], "lidar": [round(OX, 3), round(OY, 3)],
                                           "n": int(m_.sum()),
                                           "pts": [[round(float(dbg_xy[i, 0]), 3), round(float(dbg_xy[i, 1]), 3), round(float(dbg_r[i]), 3),
                                                    round(math.degrees(float(dbg_a[i])), 1), int(kind[i])] for i in idx]})
        except Exception:
            pass
        inner_hits = np.zeros((N + 1, N + 1), int)
        k3_ = hit_ins & (kind == 3)
        np.add.at(inner_hits, (ix[k3_], iy[k3_]), 1)
        with self.lock:
            self.occ_scans *= self.a.decay
            self.free_scans *= self.a.decay
            self.occ_scans += (occ >= self.a.min_pts)
            self.free_scans += ((free >= 8) & (occ == 0))
            g_ = getattr(self, "grid", None)
            if g_ is not None and hasattr(g_, "not_box"):
                g_.not_box.update((int(x_), int(y_)) for x_, y_ in zip(*np.where(inner_hits >= self.a.min_pts)))

    def observed_cells(self):
        with self.lock:
            o, f = self.occ_scans.copy(), self.free_scans.copy()
        out = {}
        for x in range(1, N + 1):
            for y in range(1, N + 1):
                if o[x, y] >= self.a.occ_need and o[x, y] * 2 > f[x, y]:
                    out[(x, y)] = True
                elif f[x, y] >= self.a.free_need and f[x, y] > 4 * o[x, y]:
                    out[(x, y)] = False
        return out

    # ── planner 가 부르는 메서드 ──
    def current_cell(self):
        X, Y, _ = self.pose()
        if self.at_grab_point:                     # 큐브 앞 정지점: planner 에는 (5,5) 도착으로 알림
            return P.TARGET
        cand = (int(math.floor(X / self.c)) + 1, int(math.floor(Y / self.c)) + 1)
        cx, cy = self.center(cand)
        if cand != self.last_cell and math.hypot(X - cx, Y - cy) < 0.4 * self.c:   # 칸 경계 흔들림 방지
            self.last_cell = cand
        return self.last_cell

    def current_heading(self):
        _, _, yaw = self.pose()
        return int(round((90.0 - math.degrees(yaw)) / 90.0)) % 4

    def opponent_cells(self):
        """충돌 방지로 멈춘 칸은 planner 에 '임시 차단'(3 초)으로만 알린다. 관측으로 넣으면 모르던 칸이 영구 장애물이 돼 길이 막힘."""
        now = time.monotonic()
        out = set()
        for cell, t in list(self.forced_occ.items()):
            if now - t < 3.0:
                out.add(cell)
            else:
                del self.forced_occ[cell]
        return out

    def now(self):
        return time.monotonic()

    def wait(self, seconds):
        self.stop()
        self.hold_if_estop()
        now = time.monotonic()
        if now - self._last_wait > 1.0:
            self._wait_t0 = now                    # 대기가 끊겼다 다시 시작
        self._last_wait = now
        if (self.a.stuck_reset > 0 and not self.a.teleop and self.grid is not None
                and now - self._wait_t0 > self.a.stuck_reset and not self.opponent_cells()):   # 충돌 방지로 잠깐 막은 칸 때문이면 지도를 의심할 일이 아님
            self.relook_map(now - self._wait_t0)
            self._wait_t0 = self._last_wait = time.monotonic()
            return
        time.sleep(seconds)

    def relook_map(self, waited):
        """길이 계속 막히면 지도를 의심한다: 장애물로 본 칸을 지우고 서서 다시 본다.
        진짜 장애물은 곧 다시 잡히고, 잘못 찍힌 칸은 풀린다. 다시 안 보이는 칸은 '모름' → 확인 없이 들어가지 않는 규칙이 지킴."""
        # 로봇 주변 2 칸 안만 의심한다. 다 지우면 여기서 안 보이는 진짜 장애물까지 '모름'이 돼 엉뚱한 길로 왔다 갔다 함 (필드 10-21 12:40)
        rc = self.last_cell
        occ = sorted((x, y) for x in range(1, N + 1) for y in range(1, N + 1)
                     if self.grid.state((x, y)) == P.OCC and max(abs(x - rc[0]), abs(y - rc[1])) <= 2)
        self.log("  길이 %.0f 초째 막힘 → 장애물로 본 칸 %d 개를 지우고 %.1f 초 다시 봄" % (waited, len(occ), self.a.collect + 0.5))
        with self.lock:
            for c in occ:
                self.occ_scans[c[0], c[1]] = 0
        for c in occ:
            self.grid.forget(c)
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.a.collect + 0.5:
            self.hold_if_estop()
            self.send_zero()
            time.sleep(0.05)
        back = sorted(c for c, o in self.observed_cells().items() if o)
        self.log("  다시 보니 장애물 %d 칸 (풀린 칸: %s)" % (len(back), [c for c in occ if c not in back][:12]))

    def send(self, vx, vy, wz):
        if not self.execute:
            return
        if self.estop or self.quit_req:
            vx = vy = wz = 0.0
        a = self.a
        if a.wheel_vmax > 0 and abs(wz) >= a.wheel_room_w and abs(vy) > abs(vx):
            # 옆걸음일 때만: 큐브 끌림(회전을 못 하게 하는 힘)은 옆으로 갈 때만 생김. 앞뒤 이동까지 줄이고 천천히 되돌리니
            # 돈 직후 몸이 몇 도 틀어진 채 출발할 때마다 칸마다 멈칫거림(실기 10-31 19:21·19:29)
            # 평소 방향 유지의 작은 회전(0.01~0.05 rad/s)에는 안 씀 — 그것까지 깎으니 모든 이동이 0.33 → 0.26 m/s 로 느려짐(실기 10-31 18:37)
            # 바퀴 출력 상한: 바퀴 테 속도 = |vx|+|vy|+L|w| (메카넘). 펌웨어 PWM = 25×바퀴속도, 상한 200 → 옆 0.32 m/s 면 이미 최대라
            # 섞은 회전이 사라짐 (실기 10-31 18:29: 큐브 물고 옆 0.32 m/s, 회전 −0.30 보냈는데 IMU 0.00 → (6,5) 에 걸림).
            # 회전을 먼저 보장하고 남는 만큼만 이동 (상한 = (200 − 적분 몫 60)/25 × 0.04 = 0.224 m/s)
            room = a.wheel_vmax - a.wheel_l * abs(wz)
            mv = abs(vx) + abs(vy)
            if mv > max(room, 0.0) and mv > 1e-6:
                k_need = max(room, 0.0) / mv
            else:
                k_need = 1.0
        else:
            k_need = 1.0
        # 줄일 땐 바로, 되돌릴 땐 천천히 (wheel_k_up 비율/초) — 회전이 켜졌다 꺼질 때마다 옆 속도가 반 토막 났다 튀어 움찔거림
        now_ = time.monotonic()
        k_prev, t_prev = getattr(self, "_wheel_k", (1.0, now_))
        lx_, ly_ = getattr(self, "last_cmd", (0.0, 0.0))
        if abs(vx) + abs(vy) < 0.02 or vx * lx_ + vy * ly_ < 0:
            k_prev = 1.0                                   # 멈춤·제동(반대 명령)은 줄이지 않음 — 능동 제동이 약해지면 지나침
        k_now = k_need if k_need < k_prev else min(k_need, k_prev + a.wheel_k_up * min(0.2, now_ - t_prev))
        self._wheel_k = (k_now, now_)
        if k_now < 0.999:
            vx, vy = vx * k_now, vy * k_now
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        try:
            self.cmd_pub.publish(t)
        except Exception:
            pass                       # 종료 중이면 0.3 초 timeout 으로 모터가 꺼진다

    def send_zero(self):
        self.send(0.0, 0.0, 0.0)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0, 0.0)
            time.sleep(0.02)

    def hold_if_estop(self):
        """비상정지 중이면 정지 명령을 계속 내며 대기. 해제되면 True(시간 초과 계산 다시 시작)."""
        if self.quit_req:
            self.stop()
            raise MissionAbort("x 키로 종료")
        if not self.estop:
            return False
        self.log("[비상정지] 정지 중 — g: 계속, x: 종료")
        while self.estop and not self.quit_req:
            self.send_zero()
            time.sleep(0.05)
        if self.quit_req:
            self.stop()
            raise MissionAbort("x 키로 종료")
        self.log("[비상정지] 해제 → 계속")
        return True

    def wall_gap(self, bx, by):
        """몸 기준 방향 (bx, by)(축 방향)으로 차체 면에서 벽까지 거리 (라이다 직접, 띠 안 점들의 아래 20%). 없으면 None."""
        g = self.guard_pts
        if g is None or time.monotonic() - g[0] > 0.4:
            return None
        a = self.a
        px, py = g[1], g[2]
        if abs(bx) >= abs(by):
            along, lat_ok = (px if bx > 0 else -px), np.abs(py) < a.side - 0.04
            half = a.front if bx > 0 else a.back
        else:
            along, lat_ok = (py if by > 0 else -py), (px > -a.back + 0.04) & (px < a.front - 0.04)
            half = a.side
        m = lat_ok & (along > 0) & (along < half + 0.5)
        if m.sum() < 5:
            return None
        return float(np.percentile(along[m], 20)) - half

    def guard_check(self, bx, by, remaining):
        """몸 기준 이동 방향 (bx, by) 로 remaining [m] 더 갈 때, 차체 앞면이 라이다 점에 clear 보다 가까워지면 점 수."""
        g = self.guard_pts
        if g is None or time.monotonic() - g[0] > 0.5:
            return 0, None
        n = math.hypot(bx, by)
        if n < 1e-6:
            return 0, None
        ux, uy = bx / n, by / n
        px, py = g[1], g[2]
        along = px * ux + py * uy
        a = self.a
        # 실제 차체 크기 (base_link = 네 바퀴 가운데 기준): 앞 front, 뒤 back(Jetson·USB 선), 좌우 side. 앞뒤가 다르다.
        if abs(ux) >= abs(uy):                       # 앞뒤로 움직임
            half_along = a.front if ux > 0 else a.back
            in_band = np.abs(py) < a.side - 0.03
        else:                                        # 옆으로 움직임
            half_along = a.side
            in_band = (px > -a.back + 0.03) & (px < a.front - 0.03)
        gap = along - half_along                                   # 차체 면에서 점까지
        lat_ok = in_band
        # 목표 칸 너머 벽은 목표 중심에 섰을 때 차체 면에서 5.5~7.5 cm, 목표 칸 안 장애물 면은 목표 중심보다 20 cm 앞.
        # 둘 사이 중간쯤(guard_slack)을 기준으로 잡아 잡음·지연 몇 cm 에 흔들리지 않게 한다. 아주 가까운 점(guard_min)은 무조건 정지.
        hit = (along > 0) & lat_ok & ((gap < remaining + self.a.guard_clear - self.a.guard_slack) | (gap < self.a.guard_min))
        k = int(hit.sum())
        return k, (float(gap[hit].min()) if k else None)

    def _drive(self, Xt, Yt, yaw_t, pos_tol, yaw_tol, label, watch_cell=None, guard=True, move_dir=None, timeout=None,
               min_v=None, stop_gap=None, brake=None, hold_tol=None, pass_pt=None, late_go=None, slot=False, vmax=None,
               yaw_ref_odom=None):
        """brake = "long"/"lat": 굴러갈 거리(속도 × 제동 시간)만큼 미리 멈춤. stop_gap = (경기장 방향, 벽 여유):
        그 방향으로 가는 중 라이다로 잰 벽까지 거리 − 굴러갈 거리가 여유에 닿으면 멈춤(위치추정보다 우선)."""
        a = self.a
        min_v = a.min_v if min_v is None else min_v
        timeout = a.move_timeout if timeout is None else timeout
        t0 = time.monotonic()
        tb = self.brake_tb.get(brake, 0.0) if brake else 0.0
        v_meas, prev_o = 0.0, None
        vbody = [0.0, 0.0]                          # 몸 기준 실제 속도 (오돔) [앞, 왼]
        pid_s = None                                # 옆걸음 몸 방향 PID 상태 [기준(오돔), 적분, 직전 방향, 직전 시각]
        n_loop = 0
        gaps_seen = self.gaps_seen                 # 옆 이동 중 잰 앞·뒤 벽 간격 (최소, 최대) [m] — 서지 않고 이어 간 칸들 합쳐서
        self.last_return_why = ""
        clamp_logged = False
        stall_t, stall_logged = None, False
        self.last_return_v = 0.0
        while True:
            if self.hold_if_estop():
                t0 = time.monotonic()
            if watch_cell is not None and watch_cell != P.TARGET and self.observed_cells().get(watch_cell):
                self.stop()
                raise GuardStop("가려던 칸 %s 가 장애물로 판정됨" % (watch_cell,))
            if time.monotonic() - t0 > timeout:
                self.stop()
                raise MoveTimeout("%s 시간 초과 (%.0f s)" % (label, timeout))
            X, Y, yaw = self.pose()
            ex, ey = Xt - X, Yt - Y
            ew = wrap(yaw_t - yaw)
            dist = math.hypot(ex, ey)
            # 실제 속도 (오돔, 위치추정 보정에 안 흔들림)
            try:
                ox, oy, oth = self.lookup_odom()
                tn = time.monotonic()
                if prev_o is not None and tn - prev_o[2] > 0.02:
                    vi = math.hypot(ox - prev_o[0], oy - prev_o[1]) / (tn - prev_o[2])
                    v_meas = 0.5 * v_meas + 0.5 * vi
                    dvx, dvy = (ox - prev_o[0]) / (tn - prev_o[2]), (oy - prev_o[1]) / (tn - prev_o[2])
                    vbody[0] = 0.5 * vbody[0] + 0.5 * (math.cos(oth) * dvx + math.sin(oth) * dvy)
                    vbody[1] = 0.5 * vbody[1] + 0.5 * (-math.sin(oth) * dvx + math.cos(oth) * dvy)
                prev_o = (ox, oy, tn)
            except Exception:
                pass
            lead = v_meas * tb
            n_loop += 1
            if late_go is not None and n_loop % 3 == 0 and dist > lead + 0.03 and late_go():
                # 서기로 했던 칸인데, 가는 동안 다음 칸이 빈 칸으로 확인됨 → 멈추지 않고 이어 감
                self.last_return_v, self.last_return_why, self.last_return_odom = v_meas, "late", prev_o
                return
            if pass_pt is not None and (X - pass_pt[0]) * pass_pt[2] + (Y - pass_pt[1]) * pass_pt[3] >= -0.02:
                return                                        # 이어 가기: 이 칸 가운데를 지났으니 멈추지 않고 다음 칸으로
            if dist < pos_tol and abs(ew) < yaw_tol:
                self.last_return_v, self.last_return_why, self.last_return_odom = v_meas, "pos", prev_o
                return
            if brake and v_meas > 0.05 and dist <= lead + 0.005 and abs(ew) < yaw_tol:
                self.halt(v_meas)                             # 이대로 굴러가면 목표쯤에서 선다
                self.last_return_v, self.last_return_why, self.last_return_odom = v_meas, "brake", prev_o
                return
            if stop_gap is not None:
                gdx, gdy = P.DXY[stop_gap[0]]
                if ex * gdx + ey * gdy > 0:                   # 벽 쪽으로 가는 중일 때만 (벽에서 떨어지는 건 막지 않음)
                    gbx = math.cos(yaw) * gdx + math.sin(yaw) * gdy
                    gby = -math.sin(yaw) * gdx + math.cos(yaw) * gdy
                    gbx, gby = (math.copysign(1.0, gbx), 0.0) if abs(gbx) >= abs(gby) else (0.0, math.copysign(1.0, gby))
                    gap = self.wall_gap(gbx, gby)
                    if gap is not None and gap - v_meas * max(tb, self.brake_tb["long"]) <= stop_gap[1]:
                        self.halt()
                        self.last_return_v, self.last_return_why, self.last_return_odom = v_meas, "wall", prev_o
                        self.log("  벽까지 %.1f cm, 굴러갈 거리 %.1f cm (벽 여유 목표 %.1f) → 여기서 정지"
                                 % (100 * gap, 100 * v_meas * max(tb, self.brake_tb["long"]), 100 * stop_gap[1]))
                        return
            bx = math.cos(yaw) * ex + math.sin(yaw) * ey          # 몸 기준 오차
            by = -math.sin(yaw) * ex + math.cos(yaw) * ey
            vcap = a.max_v if vmax is None else min(a.max_v, vmax)
            v = min(vcap, a.kp * dist) if pass_pt is None else vcap
            move_tol = pos_tol if hold_tol is None else hold_tol    # 회전 중엔 더 작게 벗어나도 바로잡음
            if dist >= move_tol:
                v = max(v, min_v)
                # 메카넘은 옆으로 갈 때 0.10 m/s 로는 잘 안 밀림 → 옆이 주된 이동은 맞춤용 최소 속도로.
                # 명령을 주는데 1 초 넘게 안 움직이면 조금 더 세게 (필드 10-30 02:02: 큐브를 문 채 (6,5) 5 cm 앞에서 바퀴만 울고 멈춤)
                if abs(by) > abs(bx):
                    v = max(v, a.settle_min_v)
                if v_meas < 0.01 and n_loop > 4:
                    if stall_t is None: stall_t = time.monotonic()
                    elif time.monotonic() - stall_t > 1.0:
                        v = max(v, a.settle_min_v + 0.04)
                        if not stall_logged:
                            stall_logged = True; self.log("  %s: 명령을 줘도 1 초 넘게 안 움직임 → 최소 속도 %.2f m/s 로" % (label, v))
                else:
                    stall_t = None
            else:
                v = 0.0
            vx, vy = (bx / dist * v, by / dist * v) if dist > 1e-6 else (0.0, 0.0)
            if move_dir is not None and a.perp_k > 0:
                # 가는 방향과 수직 방향을 따로: 목표점까지 직선으로만 가면 긴 이동 중 수직 몇 cm 는 거의 안 고쳐짐
                # (필드 10-21 12:36: 옆으로 1.2 m 가며 앞뒤 2 cm 가 남아 좁은 칸 (6,3) 모서리에 걸림)
                gdx, gdy = P.DXY[move_dir]
                mx = math.cos(yaw) * gdx + math.sin(yaw) * gdy
                my = -math.sin(yaw) * gdx + math.cos(yaw) * gdy
                mx, my = (math.copysign(1.0, mx), 0.0) if abs(mx) >= abs(my) else (0.0, math.copysign(1.0, my))
                px_, py_ = -my, mx                                   # 몸 기준 수직 방향
                along = bx * mx + by * my
                perp = bx * px_ + by * py_
                # 벽이 보일 때만 따로 제어한다. 위치추정 값으로 수직을 세게 쫓으면 직선에서 흔들림 (필드 10-21 12:39)
                w_ = None
                if mx == 0.0:                                        # 옆으로 감: 실제 앞·뒤 벽 간격 기록 (확인용)
                    for k_, g_ in (("앞", self.wall_gap(1.0, 0.0)), ("뒤", self.wall_gap(-1.0, 0.0))):
                        if g_ is not None and abs(g_ - self.want_long()) < 0.08:
                            lo_, hi_ = gaps_seen.get(k_, (9.0, -9.0))
                            gaps_seen[k_] = (min(lo_, g_), max(hi_, g_))
                if mx == 0.0 and a.wall_follow_lat:                  # 옆으로 감 → 앞뒤는 앞·뒤 벽 거리로
                    # 앞은 라이다 사각지대(7 cm)라 출렁임이 앞으로 나가면 못 봄 → 보이는 뒤 벽 쪽으로 조금 당겨 잡음
                    w_ = self.wall_center_err(1, a.slot_back_bias if slot else a.lat_back_bias)
                    if w_ is not None:
                        perp = w_ * px_
                elif mx != 0.0 and a.wall_follow:                    # 앞뒤로 감 → 좌우를 옆벽으로 (기본 끔)
                    w_ = self.wall_center_err(0)
                    if w_ is not None:
                        perp = w_ * py_
                if w_ is not None:
                    v_main = math.copysign(v, along) if abs(along) > 0.003 else 0.0
                    if a.perp_damp:
                        # 명령을 줄여도 바퀴는 천천히만 느려져(필드 10-21) 목표를 1~2 cm 지나침 → 지금 수직 속도로 서기까지 갈 거리를 미리 뺌.
                        # 지나칠 것 같으면 반대 명령 = 능동 제동 (서는 시간 = 이 로봇에서 잰 앞뒤 제동 학습값)
                        perp -= (vbody[0] * px_ + vbody[1] * py_) * self.brake_tb["long"]
                    if abs(perp) < a.perp_dead:
                        v_perp = 0.0
                    else:   # 작은 수직 명령은 옆 이동 중 바퀴가 못 따라감 (필드 10-21 13:03: 1 cm 앞으로 나온 채 유지) → 최소 속도
                        v_perp = math.copysign(max(a.perp_vmin, min(a.perp_vmax, a.perp_k * abs(perp))), perp)
                    vx, vy = v_main * mx + v_perp * px_, v_main * my + v_perp * py_
            if brake == "lat" and a.lat_gain != 1.0:
                vy *= a.lat_gain          # 메카넘은 옆으로 갈 때 실제 속도가 명령의 60~75% (필드 10-21) → 옆 칸 이동만 명령을 키움
            if guard and move_dir is not None and v > 0 and dist > 0.03:
                # 칸 이동 축(경기장 N/E/S/W)을 몸 기준으로 바꿔 가장 가까운 축으로 맞춘다. 순간 속도 방향(보정 성분 포함)은 쓰지 않음
                gdx, gdy = P.DXY[move_dir]
                gbx = math.cos(yaw) * gdx + math.sin(yaw) * gdy
                gby = -math.sin(yaw) * gdx + math.cos(yaw) * gdy
                gbx, gby = (math.copysign(1.0, gbx), 0.0) if abs(gbx) >= abs(gby) else (0.0, math.copysign(1.0, gby))
                remaining = max(0.0, ex * gdx + ey * gdy)
                k, gap = self.guard_check(gbx, gby, remaining)
                if k >= a.guard_pts:
                    self.halt()
                    raise GuardStop("가는 방향 차체 앞 %.0f cm 에 라이다 점 %d 개" % (100 * gap, k))
            if a.wall_clamp > 0:
                # 몸 네 방향 중 벽이 설계 여유보다 wall_clamp 넘게 가까운 쪽으로는 속도를 내지 않는다 (위치추정이 틀려도)
                vfull = math.hypot(vx, vy)
                for (ux, uy) in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
                    comp = vx * ux + vy * uy
                    if comp <= 0:
                        continue
                    gap = self.wall_gap(ux, uy)
                    lim = (self.want_long() if ux else self.want_side()) - a.wall_clamp
                    if gap is None or gap >= lim:
                        continue
                    vx, vy = vx - comp * ux, vy - comp * uy
                    if not clamp_logged:
                        clamp_logged = True
                        self.log("  %s 벽까지 %.1f cm → 그쪽 속도 막음" % ({1.0: "앞", -1.0: "뒤"}.get(ux, "왼") if ux else ("왼" if uy > 0 else "오른"), 100 * gap))
                    if comp > 0.7 * vfull and v > 0 and abs(ew) < yaw_tol:
                        self.halt()                          # 가려던 방향 자체가 막힘 → 여기서 멈춤
                        self.last_return_v, self.last_return_why, self.last_return_odom = v_meas, "wall", prev_o
                        return
            if slot:
                yw_ = self.wall_yaw_err()
                if yw_ is not None:
                    ew = yw_                                   # 좁은 칸: 몸 방향은 뒤 벽 각도로 (위치추정 방향은 2~3° 틀림)
            gyro_fix = False
            if yaw_ref_odom is not None and not slot:
                # 직진 중 방향은 '직진 시작 때 정한 기준'을 자이로(오돔)로 유지 (위치추정 방향이 이동 중 튀어도 안 따라감).
                # 예전엔 8° 넘어야 제대로 돌려서 내려가며 3° 돌아간 채 왼쪽으로 4.5 cm 붙음 (필드 10-21 15:20)
                try:
                    ew = wrap(yaw_ref_odom - self.lookup_odom()[2])
                    gyro_fix = abs(ew) > math.radians(a.gyro_hold_tol)
                except Exception:
                    pass
            w = max(-a.max_w, min(a.max_w, a.kw * ew))
            if abs(ew) >= yaw_tol and abs(w) < a.min_w:
                w = math.copysign(a.min_w, ew)
            elif slot and abs(ew) > math.radians(1.0) and abs(w) < a.slot_min_w:
                w = math.copysign(a.slot_min_w, ew)            # 작은 회전 명령은 바퀴가 못 따라감
            elif gyro_fix and abs(w) < a.gyro_hold_min_w:
                w = math.copysign(a.gyro_hold_min_w, ew)       # 움직이는 중엔 이 정도면 바퀴가 따라옴
            elif (a.strafe_yaw_tol > 0 and move_dir is not None and abs(ew) > math.radians(a.strafe_yaw_tol)
                  and abs(w) < a.strafe_yaw_w and dist > 0.02):
                # 옆으로 가는 중엔 몸 방향을 strafe_yaw_tol 로 계속 잡음 (서지 않고 회전만 섞음). 큐브가 바닥에 끌려 옆걸음 중 앞이
                # 뒤로 끌리며 몸이 돎 (실기 10-30: 40 cm 에 8~11°, 4° 허용 안이라 안 고쳐 앞 모서리가 큐브 옆 판에 걸림)
                gdx_, gdy_ = P.DXY[move_dir]
                if abs(-math.sin(yaw) * gdx_ + math.cos(yaw) * gdy_) > 0.7:     # 이동 방향이 몸 옆쪽 = 옆걸음
                    w = math.copysign(a.strafe_yaw_w, ew)           # 끌려 도는 빠르기(옆 0.3 m/s 에 약 0.15 rad/s)보다 세게
            if a.yaw_pid and move_dir is not None and not slot:
                gdx_, gdy_ = P.DXY[move_dir]
                if abs(-math.sin(yaw) * gdx_ + math.cos(yaw) * gdy_) > 0.7:     # 옆걸음: IMU(오돔 방향) PID 로 몸 방향 유지
                    # 기준 = 칸 축 방향(위치추정, 벽 기준과 중앙 0.3° 차이)을 오돔 방향으로 옮긴 값 — 출발 때 한 번만 정함
                    # (10-21 자이로 유지는 틀어진 회전 직후 방향을 기준으로 삼아 쏠렸음). 되먹임은 빠른 오돔(IMU) 방향·회전 속도로.
                    # P: 틀어진 만큼 / I: 큐브 끌림처럼 계속 미는 힘을 점점 세게 맞받음 / D: 출발 순간 확 도는 걸 바로 누름
                    try:
                        tn_ = time.monotonic()
                        imu_ok = self.imu is not None and tn_ - self.imu[0] < 0.2
                        oth_ = self.imu[2] if imu_ok else self.lookup_odom()[2]     # IMU 자이로 적분 방향 (없으면 오돔)
                        if pid_s is None:
                            pid_s = [oth_ + max(-math.radians(3), min(math.radians(3), wrap(yaw_t - yaw))), 0.0, oth_, tn_, 0.0]
                        ref_, I_, prev_, tp_ = pid_s[:4]
                        dt_ = max(1e-3, tn_ - tp_)
                        e_ = wrap(ref_ - oth_)
                        rate_ = self.imu[1] if imu_ok else wrap(oth_ - prev_) / dt_
                        I_ = max(-a.yaw_pid_imax, min(a.yaw_pid_imax, I_ + e_ * dt_))
                        if a.yaw_pid == 2:
                            # IMU 로 보고, 몸 잡기처럼 strafe_yaw_tol(1°) 넘을 때만 strafe_yaw_w 로 바로잡음. 그 안에선 안 건드림
                            # (PID 는 0.5° 부터 최소 0.15 rad/s 로 계속 건드려 옆걸음 흔들림 ±2.1° — 실기 10-30 22:52, 몸 잡기는 ±1.1°)
                            # 회전 명령이 실제 회전으로 나오기까지 약 0.2 초 늦음(실측 10-31: 무반응 0.1~0.17 초, 63% 0.25~0.3 초)
                            # → 1° 안에서 멈춰도 몸이 계속 돌아 반대로 +3.5° 넘어감(실기 10-31 18:39). 지금 회전 속도로 그만큼 뒤를 예측해 판단.
                            e_pred = e_ - rate_ * a.yaw_lag                     # e = 기준 − 방향 → 몸이 +로 돌면 e 가 줄어듦
                            # 세기는 틀어진 만큼 (1° ≈ 0.16, 2° 넘으면 0.3 rad/s) — 늘 0.3 으로 돌리면 그때마다 옆 속도가 반으로 떨어져 움찔거림(실기 10-31 19:07)
                            if abs(e_pred) > math.radians(a.strafe_yaw_tol):
                                w = math.copysign(min(a.strafe_yaw_w, max(a.gyro_hold_min_w, 3.0 * a.yaw_pid_kp * abs(e_pred))), e_pred)
                            else:
                                w = 0.0
                        else:
                            w = a.yaw_pid_kp * e_ + a.yaw_pid_ki * I_ - a.yaw_pid_kd * rate_
                            w = max(-a.max_w, min(a.max_w, w))
                            if abs(e_) > math.radians(0.5) and abs(w) < a.gyro_hold_min_w:
                                w = math.copysign(a.gyro_hold_min_w, w if w != 0 else e_)   # 작은 회전 명령은 바퀴가 못 따라감
                        pid_s[1:4] = [I_, oth_, tn_]
                        if tn_ - pid_s[4] > 0.25:                  # 4 번/초: 옆걸음 중 방향 PID 가 실제로 뭘 냈는지 (회전 명령이 먹히는지 판정용)
                            pid_s[4] = tn_
                            self.log("    [방향PID] 틀어짐(IMU) %+.1f° · 회전 %+.2f rad/s(IMU) · 보낸 회전 %+.2f · 보낸 옆 %+.2f 앞 %+.2f m/s · 위치추정 방향 %+.1f°" % (
                                math.degrees(e_), rate_, w, vy * (a.lat_gain if brake == "lat" else 1.0), vx, math.degrees(wrap(yaw - yaw_t))))
                    except Exception:
                        pass
            self.send(vx, vy, w)
            self.last_cmd = (vx, vy)
            time.sleep(0.05)

    def _drive_odom(self, dx, dy, tol, timeout, min_v, label):
        """오돔 기준으로 몸 기준 (dx 앞, dy 왼) 만큼 짧게 이동. 위치추정이 그 사이 바뀌어도 휘둘리지 않음(벽 맞춤용)."""
        a = self.a
        ox, oy, oth = self.lookup_odom()
        tx = ox + math.cos(oth) * dx - math.sin(oth) * dy
        ty = oy + math.sin(oth) * dx + math.cos(oth) * dy
        t0 = time.monotonic()
        while True:
            if self.hold_if_estop():
                t0 = time.monotonic()
            if time.monotonic() - t0 > timeout:
                self.stop()
                raise MoveTimeout("%s 시간 초과 (%.0f s)" % (label, timeout))
            cx, cy, cth = self.lookup_odom()
            ex, ey = tx - cx, ty - cy
            dist = math.hypot(ex, ey)
            if dist < tol:
                self.stop()
                return
            bx = math.cos(cth) * ex + math.sin(cth) * ey
            by = -math.sin(cth) * ex + math.cos(cth) * ey
            v = max(min(a.max_v, a.kp * dist), min_v)
            w = max(-a.max_w, min(a.max_w, a.kw * wrap(oth - cth)))
            self.send(bx / dist * v, by / dist * v, w)
            time.sleep(0.05)

    def measure_walls(self, off):
        """지금 스캔으로 이 칸을 둘러싼 벽(뒤·앞·왼·오른)까지 거리를 재서, 몸통이 칸 가운데에서 얼마나 벗어났는지(몸 기준 앞 cx, 왼 cy) 계산.
        벽이 없는 쪽은 None. 움직이지 않는다."""
        g = self.guard_pts
        if g is None or time.monotonic() - g[0] > 0.5:
            return None
        px, py = g[1], g[2]
        half = self.c / 2.0

        def wall(sel, vals):
            v = vals[sel]
            if v.size >= 8 and np.std(v) < 0.015:
                return float(np.median(v))
            return None
        band = 0.6 * half                       # 몸 옆 띠 (모서리 근처 점 제외)
        d_back = wall((px < -0.10) & (px > -half - 0.12) & (np.abs(py) < band), -px)
        d_front = wall((px > 0.10) & (px < half + 0.12) & (np.abs(py) < band), px)
        d_left = wall((py > 0.10) & (py < half + 0.12) & (np.abs(px) < band), py)
        d_right = wall((py < -0.10) & (py > -half - 0.12) & (np.abs(px) < band), -py)
        cx_, cy_ = [], []
        if d_back is not None and abs(d_back - (half + off)) < 0.08:
            cx_.append(-(d_back - (half + off)))
        if d_front is not None and abs(d_front - (half - off)) < 0.08:
            cx_.append(d_front - (half - off))
        if d_left is not None and abs(d_left - half) < 0.08:
            cy_.append(d_left - half)
        if d_right is not None and abs(d_right - half) < 0.08:
            cy_.append(-(d_right - half))
        return {"d": (d_back, d_front, d_left, d_right),
                "cx": float(np.mean(cx_)) if cx_ else None, "cy": float(np.mean(cy_)) if cy_ else None}

    def pcorr_keep(self, X, Y):
        """벽으로 고친 보정이 얼마나 남았나 (경기장 x, y 따로): 그 축으로 움직인 거리만큼만 줄어든다.
        같은 벽을 따라 옆으로 가는 동안엔 그 벽 쪽 보정이 그대로 맞기 때문 (필드 10-21 12:20 (6,5)→(6,2) 에서 보정이 사라져 뒤 벽에 닿음)."""
        f = self.a.pcorr_fade
        return (max(0.0, 1.0 - abs(X - self.pcorr_at[0]) / f), max(0.0, 1.0 - abs(Y - self.pcorr_at[1]) / f))

    def wall_fix(self, off, Xt, Yt):
        """서 있을 때 벽까지 거리로 잰 위치가 위치추정과 다르면, 벽 쪽을 믿고 위치추정에 보정을 더한다 (로봇은 안 움직임).
        필드 10-21: (6,x) 에서 위치추정이 벽 실측보다 약 5 cm 어긋나, 위치추정은 벽 쪽으로 밀고 안전장치는 막는 싸움이 남."""
        if self.a.pcorr_fade <= 0:
            return
        time.sleep(0.15)
        m = self.measure_walls(off)
        if m is None or (m["cx"] is None and m["cy"] is None):
            return
        X, Y, yaw = self.pose()
        ex, ey = X - Xt, Y - Yt
        ef = math.cos(yaw) * ex + math.sin(yaw) * ey          # 위치추정이 본 목표에서의 어긋남 (몸 기준 앞)
        el = -math.sin(yaw) * ex + math.cos(yaw) * ey         # (몸 기준 왼)
        # measure_walls 의 cx, cy 는 '가운데로 가려면 움직일 양'(앞·왼) → 지금 어긋난 위치는 그 반대 부호
        df = (-m["cx"] - ef) if m["cx"] is not None else 0.0
        dl = (-m["cy"] - el) if m["cy"] is not None else 0.0
        if math.hypot(df, dl) < 0.008:
            return
        lim = self.a.pcorr_max
        # 지금 보정(줄어든 만큼 포함)을 다시 계산해 새 기준점에서 합친다
        kx = ky = 0.0
        if self.pcorr_at is not None:
            kx, ky = self.pcorr_keep(self.last_pose[0], self.last_pose[1])
        cx_ = kx * self.pcorr[0] + math.cos(yaw) * df - math.sin(yaw) * dl
        cy_ = ky * self.pcorr[1] + math.sin(yaw) * df + math.cos(yaw) * dl
        cx_, cy_ = max(-lim, min(lim, cx_)), max(-lim, min(lim, cy_))
        self.pcorr = (cx_, cy_)
        self.pcorr_at = (self.last_pose[0], self.last_pose[1])
        self.log("  벽으로 잰 위치가 위치추정과 다름: 앞뒤 %+.1f 좌우 %+.1f cm → 벽 쪽을 믿고 위치추정에 보정 (경기장 x %+.1f y %+.1f cm)"
                 % (100 * df, 100 * dl, 100 * cx_, 100 * cy_))

    def check_walls(self, off, cell, heading):
        """도착 후 실제 위치 확인(기록만): 벽까지 거리로 본 칸 가운데와의 차이. 위치추정이 실제로 얼마나 맞는지 보는 용도."""
        time.sleep(0.2)
        m = self.measure_walls(off)
        if m is None or (m["cx"] is None and m["cy"] is None):
            return
        ds = " ".join("%s %s" % (n, ("%.1f" % (100 * d)) if d is not None else "-")
                      for n, d in zip(("뒤", "앞", "왼", "오른"), m["d"]))
        ex = "앞뒤 %+.1f cm" % (100 * m["cx"]) if m["cx"] is not None else "앞뒤 -"
        ey = "좌우 %+.1f cm" % (100 * m["cy"]) if m["cy"] is not None else "좌우 -"
        yc = self.wall_yaw_any()
        _, _, yaw_ = self.pose()
        loc_off = math.degrees(wrap(yaw_ - math.radians(HEADING_YAW[heading])))
        yw = "· 격자에서 몸 틀어짐: 벽 기준 %s / 위치추정 %+.1f°" % (
            "%+.1f°" % -math.degrees(yc) if yc is not None else "못 잼", loc_off)
        self.log("  실제 위치 확인(벽 거리, 움직이지 않음): %s → 칸 가운데에서 %s, %s %s" % (ds, ex, ey, yw))
        self.wall_log.append({"cell": list(cell), "heading": P.HEADING_NAME[heading], "d": m["d"], "cx": m["cx"], "cy": m["cy"]})

    def fine_approach(self, Xt, Yt, yaw_t, label):
        """마지막 몇 cm 를 '톡톡' 맞춤: 짧게 밀고 → 멈춰 위치 확인 → 다시. 계속 밀다 튀어 나가는 걸 막는다(옵션 --fine)."""
        a = self.a
        for k in range(a.fine_pulses):
            time.sleep(a.fine_pause)                         # 서 있는 동안 위치추정이 맞춰지게
            X, Y, yaw = self.pose()
            ex, ey = Xt - X, Yt - Y
            dist = math.hypot(ex, ey)
            if dist <= a.fine_tol:
                if k:
                    self.log("  톡톡 맞춤 끝 (%d 번): 목표까지 %.1f cm" % (k, 100 * dist))
                return
            if dist > 0.08:
                return                                       # 너무 멀면 톡톡 맞춤 대상 아님
            bx = math.cos(yaw) * ex + math.sin(yaw) * ey
            by = -math.sin(yaw) * ex + math.cos(yaw) * ey
            v = a.settle_min_v
            t_end = time.monotonic() + a.fine_pulse
            while time.monotonic() < t_end:
                self.hold_if_estop()
                self.send(bx / dist * v, by / dist * v, 0.0)
                time.sleep(0.02)
            self.stop()
        X, Y, _ = self.pose()
        self.log("  톡톡 맞춤 %d 번 뒤 목표까지 %.1f cm" % (a.fine_pulses, 100 * math.hypot(Xt - X, Yt - Y)))

    def next_same_dir(self, cell, mdir):
        """planner 의 지금 경로에서 cell 다음 칸도 같은 방향(mdir)이면 True. (자율 주행만, 키보드 모드는 False)"""
        if self.a.teleop or mdir is None or not self.path or cell == P.TARGET:
            return False
        try:
            i = self.path.index(cell)
        except ValueError:
            return False
        if i + 1 >= len(self.path):
            return False
        nxt = self.path[i + 1]
        if nxt == P.TARGET:
            return False                                  # 큐브 앞은 서서 맞춤
        if self.straight_run >= self.a.straight_max:
            return False                                  # 오돔 진행 방향 오차가 쌓이지 않게 몇 칸마다 한 번은 섬
        if self.cell_known(nxt) is not False:
            return False                                  # 다음 칸이 빈 칸으로 확인된 때만 이어 감 (모름이면 서서 봄)
        return abs(nxt[0] - cell[0]) + abs(nxt[1] - cell[1]) == 1 and P.direction(cell, nxt) == mdir

    def late_same_dir(self, cell, mdir, keep_heading):
        """서러 가는 중: 라이다 판정이 바뀌었으면 '도착하면 어떤 길이 될까'를 복사본 지도로 미리 짬. 같은 방향으로 이어지면 안 서고 감.
        실기 10-31: 꺾으려던 길 끝이 막혔다고 도착 15~40 cm 전에 이미 보면서도, 계획을 안 새로 짜 일단 서고 나서 직진함."""
        if self.next_same_dir(cell, mdir):
            return True
        if not self.a.replan_moving or self.mp is None:
            return False
        obs = self.observed_cells()
        if (cell, obs) == self.replan_obs:
            return False                                  # 같은 칸·같은 판정이면 계획도 그대로
        self.replan_obs = (cell, obs)
        with self.lock:
            mp2 = copy.deepcopy(self.mp)                  # not_box 를 라이다 콜백이 고치므로 잠금 안에서 복사
        cmd = P.MissionPlanner.update(mp2, cell, keep_heading, obs, t=self.now(), moving=mdir)
        if cmd.action != "MOVE" or cmd.next_heading != keep_heading:
            return False
        old = self.path
        self.path = list(cmd.path or [])
        if self.next_same_dir(cell, mdir):
            self.log("  가는 중 라이다로 길이 바뀜 → %s 에 서지 않음 (새 길 %s)" % (cell, self.path[:4]))
            return True
        self.path = old
        return False

    def mirror_report(self):
        """미러로 새로 안 칸, 그 칸을 직접 본 뒤 맞았는지 기록 + 칸별 지도 이력(그림용). 주행 동작에는 영향 없음."""
        g = self.grid
        if not getattr(g, "mirror_on", False):
            return
        name = {P.OCC: "벽", P.FREE: "빈 칸"}
        chars = []
        for y in range(N, 0, -1):
            for x in range(1, N + 1):
                c = (x, y)
                v = g.inferred(c)
                if v is not None and c not in self.mirror_known:
                    self.mirror_known[c] = v
                    self.log("  [미러] %s 를 %s 으로 봄 → %s 도 %s 으로 앎" % (P.mirror(c), name[v], c, name[v]))
                d = g.cells.get(c)
                if d is not None and c in self.mirror_known and c not in self.mirror_checked:
                    self.mirror_checked[c] = (d == self.mirror_known[c])
                    if d != self.mirror_known[c]:
                        self.log("  [미러] 틀림: %s 를 %s 으로 알았는데 직접 보니 %s" % (c, name[self.mirror_known[c]], name[d]))
                chars.append({P.OCC: "#", P.FREE: "."}.get(d) or {P.OCC: "%", P.FREE: ","}.get(v, "?"))
        self.map_hist.append((round(time.monotonic() - self.t0, 2), list(self.last_cell) if self.last_cell else None, "".join(chars)))

    def mirror_summary(self):
        g = self.grid
        if not getattr(g, "mirror_on", False):
            return
        wrong = sorted(c for c, ok in self.mirror_checked.items() if not ok)
        self.log("[미러] 미러로 안 칸 %d 개 · 그중 직접 확인 %d 개 · 틀린 칸 %d 개 %s · 상자 아님으로 뺀 칸 %s" % (
            len(self.mirror_known), len(self.mirror_checked), len(wrong), wrong, sorted(g.not_box)))

    def cell_known(self, cell):
        """True = 장애물, False = 빈 칸, None = 모름 (지금 라이다 판정 + planner 지도)."""
        o = self.observed_cells().get(cell)
        if o is not None:
            return o
        if not self.grid:
            return None
        if self.grid.state(cell) == P.OCC:
            return True                                   # 벽은 미러로 안 것까지 (앞뒤 벽 맞춤·서는 거리·회전 자리에 씀)
        return False if self.grid.direct_state(cell) == P.FREE else None   # 빈 칸은 직접 본 것만 (속도 내고 들어갈지 판단)

    def peek_ahead(self, from_cell, cell, heading):
        """정면 앞 칸이 모름일 때: 뒤로 peek m 물러서서 잠깐 보고 돌아간다. 장애물이면 False(이동 취소, planner 가 다시 계획)."""
        a = self.a
        hx_, hy_ = P.DXY[heading]
        cx, cy = self.center(from_cell)
        behind = (from_cell[0] - hx_, from_cell[1] - hy_)
        if not P.inside(behind) or self.cell_known(behind) is not False:
            if not a.teleop:
                # 자율: 물러설 수 없으면 안 보이는 칸에 들어가지 않는다 → 3 초 임시 차단, planner 가 다른 길(또는 몸을 돌려 보는 길)
                self.log("  앞 칸 %s 은 모름이고 뒤 칸 %s 이 빈 칸이 아니라 물러서 볼 수 없음 → 들어가지 않음, 다시 계획" % (cell, behind))
                self.forced_occ[cell] = time.monotonic()
                return False
            self.log("  앞 칸 %s 은 모름인데 뒤 칸 %s 이 빈 칸이 아니라 물러서지 않음 → 천천히 감시하며 진행" % (cell, behind))
            return True
        self.log("  앞 칸 %s 은 모름(라이다 바로 앞 사각지대) → %.0f cm 물러서서 봄" % (cell, 100 * a.peek))
        try:
            self._drive(cx - hx_ * a.peek, cy - hy_ * a.peek, math.radians(HEADING_YAW[heading]), 0.02,
                        math.radians(a.yaw_tol * 2), "물러서서 보기", guard=True, move_dir=(heading + 2) % 4, timeout=6.0,
                        brake="long")
        except (GuardStop, MoveTimeout) as e:
            self.stop()
            self.log("  물러서기 못 함 (%s)" % e)
        self.stop()
        t0 = time.monotonic()
        res = None
        while time.monotonic() - t0 < a.peek_wait:
            time.sleep(0.1)
            res = self.cell_known(cell)
            if res is not None:
                break
        if res is True:
            self.log("  앞 칸 %s 은 장애물 → 이동 취소, 다시 계획" % (cell,))
            if self.grid is not None:
                self.grid.observe(cell, True, self.now())
            return False
        if res is None and not a.teleop:
            # 물러서서도 안 보이면 면이 라이다 최소 거리(실측 약 15.6 cm) 안이거나 가려진 것 → 들어가지 않음.
            # 충돌 방지도 같은 사각지대라 못 막는다(필드 10-21 05:24 (5,3)). 나중에 빈 칸으로 보이면 지도가 다시 바뀜
            self.log("  앞 칸 %s 은 물러서서도 모름 → 들어가지 않음, 장애물로 치고 다시 계획" % (cell,))
            if self.grid is not None:
                self.grid.observe(cell, True, self.now())
            return False
        self.log("  앞 칸 %s 은 %s → 진행" % (cell, "빈 칸" if res is False else "여전히 모름(가까이 가며 충돌 방지로 감시)"))
        return True

    def pulse_center(self, off, label):
        """벽까지 거리를 직접 재서 칸 가운데로 '톡톡' 맞춤 (짧게 밀고 → 멈춰 다시 잼). 회전 전 모서리가 벽에 닿지 않게."""
        a = self.a
        for k in range(a.turn_pulses + 1):
            time.sleep(a.fine_pause if k else 0.1)        # 처음엔 이미 서 있음 (실제 정지 확인 뒤), 민 뒤에만 기다림
            m = self.measure_walls(off)
            if m is None:
                return
            cx = m["cx"] or 0.0
            cy = m["cy"] or 0.0
            if math.hypot(cx, cy) <= a.turn_pulse_tol or k == a.turn_pulses:
                if k:
                    self.log("  회전 전 벽 기준 맞춤 끝 (%d 번): 앞뒤 %+.1f 좌우 %+.1f cm" % (k, 100 * cx, 100 * cy))
                return
            if math.hypot(cx, cy) > 0.08:
                return
            n = math.hypot(cx, cy)
            v = a.settle_min_v
            t_end = time.monotonic() + min(a.fine_pulse, 0.05 + n / max(v * 0.35, 1e-3))
            while time.monotonic() < t_end:
                self.hold_if_estop()
                self.send(cx / n * v, cy / n * v, 0.0)
                time.sleep(0.02)
            self.stop()
        m = self.measure_walls(off)
        if m:
            self.log("  회전 전 벽 기준 맞춤: 남은 앞뒤 %s 좌우 %s cm" % (
                "%+.1f" % (100 * m["cx"]) if m["cx"] is not None else "-", "%+.1f" % (100 * m["cy"]) if m["cy"] is not None else "-"))

    def pulse_along(self, Xt, Yt, hx, hy):
        """몸 앞뒤로만 '톡톡' 맞춤: 짧게 밀고 → 멈춰 위치를 다시 봄. 한 번에 밀면 굴러가는데 앞은 라이다 사각지대라 못 막음
        (실기 11-02 23:57: (6,4) 에서 2.9 cm 맞추다 4.6 cm 가서 앞 (6,5) 까지 0.5 cm)."""
        a = self.a
        for k in range(a.turn_pulses + 1):
            time.sleep(a.fine_pause if k else 0.1)        # 민 뒤에는 실제로 선 다음에 잼
            X_, Y_, _ = self.pose()
            e = (Xt - X_) * hx + (Yt - Y_) * hy           # + 면 몸 앞쪽으로 더 가야 함
            if abs(e) <= a.turn_pulse_tol or k == a.turn_pulses:
                self.log("  옆걸음 전 앞뒤 맞춤 끝 (%d 번 밂): 남은 %+.1f cm" % (k, 100 * e))
                return
            v = a.settle_min_v
            t_end = time.monotonic() + min(a.fine_pulse, 0.05 + abs(e) / max(v * 0.35, 1e-3))
            while time.monotonic() < t_end:
                self.hold_if_estop()
                self.send(math.copysign(v, e), 0.0, 0.0)
                time.sleep(0.02)
            self.stop()

    def wall_align(self, off, label):
        """서 있을 때 이 칸을 둘러싼 벽(뒤·왼·오른, 보이면 앞)까지 거리를 라이다로 직접 재서 몸통을 칸 가운데로.
        off = 바퀴 가운데가 칸 중심보다 앞에 있어야 할 거리. 벽이 없는 쪽은 건드리지 않음. 터틀봇 미로 코드의 벽거리 정렬과 같은 생각."""
        a = self.a
        if not a.wall_align:
            return
        for _ in range(2):
            time.sleep(0.25)
            g = self.guard_pts
            if g is None or time.monotonic() - g[0] > 0.5:
                return
            px, py = g[1], g[2]
            half = self.c / 2.0

            def wall(sel, vals):
                v = vals[sel]
                if v.size >= 8 and np.std(v) < 0.015:
                    return float(np.median(v))
                return None
            band = 0.6 * half                       # 몸 옆 띠 (모서리 근처 점 제외)
            d_back = wall((px < -0.10) & (px > -half - 0.12) & (np.abs(py) < band), -px)
            d_front = wall((px > 0.10) & (px < half + 0.12) & (np.abs(py) < band), px)
            d_left = wall((py > 0.10) & (py < half + 0.12) & (np.abs(px - off * 0) < band), py)
            d_right = wall((py < -0.10) & (py > -half - 0.12) & (np.abs(px) < band), -py)
            cx_, cy_ = [], []
            if d_back is not None and abs(d_back - (half + off)) < 0.08:
                cx_.append(-(d_back - (half + off)))
            if d_front is not None and abs(d_front - (half - off)) < 0.08:
                cx_.append(d_front - (half - off))
            if d_left is not None and abs(d_left - half) < 0.08:
                cy_.append(d_left - half)
            if d_right is not None and abs(d_right - half) < 0.08:
                cy_.append(-(d_right - half))
            cx = float(np.mean(cx_)) if cx_ else 0.0
            cy = float(np.mean(cy_)) if cy_ else 0.0
            if math.hypot(cx, cy) < a.wall_tol:
                if cx_ or cy_:
                    self.log("  벽 맞춤 OK (%s): 뒤 %s 앞 %s 왼 %s 오른 %s cm" % (label, *(("%.1f" % (100 * d)) if d is not None else "-"
                                                                                    for d in (d_back, d_front, d_left, d_right))))
                return
            self.log("  벽 맞춤 (%s): 뒤 %s 앞 %s 왼 %s 오른 %s cm → 몸 기준 앞 %+.1f 왼 %+.1f cm 이동" % (
                label, *(("%.1f" % (100 * d)) if d is not None else "-" for d in (d_back, d_front, d_left, d_right)),
                100 * cx, 100 * cy))
            try:
                self._drive_odom(cx, cy, a.wall_tol * 0.7, a.settle_timeout, a.settle_min_v, "벽 맞춤")
            except MoveTimeout:
                self.stop()
                return
            self.stop()

    def turn_to(self, h):
        if self.a.turn_spot:
            return self.turn_to_spot(h)
        self.stop()
        time.sleep(self.a.turn_wait)                     # 칸 이동 끝에 이미 '실제로 멈춤'을 확인했으므로 짧게
        X, Y, yaw = self.pose()
        # 축 = 바퀴 가운데를 칸 가운데에: 실제 모양(바퀴 몸통 ±13.6 cm + 좁은 라이다·USB 선)에서 가장 멀리 도는 점이 18.8 cm 로 가장 작다.
        # (몸통 가운데를 축으로 돌면 바퀴 앞 모서리가 19.6 cm 로 커진다 — 가상 시험 10-21 에서 닿음)
        cx, cy = (X, Y) if self.at_grab_point else self.center(self.last_cell)   # 큐브 앞에선 그 자리에서 회전
        h_now = int(round((90.0 - math.degrees(yaw)) / 90.0)) % 4
        if not self.at_grab_point:
            off_ = (self.a.back - self.a.front) / 2.0
            hx_, hy_ = P.DXY[h_now]
            self.wall_fix(off_, cx + off_ * hx_, cy + off_ * hy_)
            self.keep_off_walls(h_now)
            X, Y, yaw = self.pose()
        if not self.at_grab_point and math.hypot(X - cx, Y - cy) > self.a.turn_center_tol:
            self.log("  회전 전 칸 중심 맞춤 (%.1f cm 벗어남)" % (100 * math.hypot(X - cx, Y - cy)))
            try:
                self._drive(cx, cy, math.radians(HEADING_YAW[h_now]), self.a.turn_center_tol, math.radians(self.a.yaw_tol * 2),
                            "회전 전 중심 맞춤", guard=False, min_v=self.a.settle_min_v, timeout=self.a.settle_timeout,
                            brake="long")
            except (GuardStop, MoveTimeout):
                self.stop()
        if not self.at_grab_point and self.a.turn_wall_align:
            self.pulse_center(0.0, "회전 전")
        self.log("  제자리 회전 → %s" % P.HEADING_NAME[h])
        if self.a.turn_odom_hold and not self.at_grab_point:
            self.rotate_in_place(math.radians(HEADING_YAW[h]))
        else:
            # 회전 여유가 1.2~1.7 cm 라 도는 동안 밀린 위치를 되돌린다 (위치추정 기준)
            self._drive(cx, cy, math.radians(HEADING_YAW[h]), self.a.turn_center_tol * 1.5, math.radians(self.a.yaw_tol), "회전",
                        guard=False, hold_tol=self.a.turn_hold_tol)
        self.stop()

    def rest_xy(self, cell, h):
        """칸 안 자세: 몸통 가운데를 칸 가운데에 (바퀴 가운데 = 칸 중심 + (뒤−앞)/2 × 몸 방향)."""
        cx, cy = self.center(cell); off = (self.a.back - self.a.front) / 2.0; hx, hy = P.DXY[h]
        return cx + off * hx, cy + off * hy

    def turn_to_spot(self, h):
        """시뮬레이션 합친 판과 같은 규칙: 회전 원(22.6 cm)이 벽에서 5 cm 남는 자리를 두 칸 안에서 먼저, 없으면 1 cm 를 한 칸 안에서,
        그것도 없으면 칸 둘레 모양 회전 표의 자리. 표에서도 못 돌면 돌지 않고 그 회전을 기억해 다시 계획(다음 translate_to 가 취소)."""
        import turnspot as TS
        self.stop()
        time.sleep(self.a.turn_wait)
        X, Y, yaw = self.pose()
        h_now = int(round((90.0 - math.degrees(yaw)) / 90.0)) % 4
        cell = P.TARGET if self.at_grab_point else self.last_cell
        if not self.at_grab_point:                     # 원래 회전 전 준비 그대로: 벽으로 위치 고치기·벽에서 떨어지기
            off_ = (self.a.back - self.a.front) / 2.0
            hx_, hy_ = P.DXY[h_now]; cx, cy = self.center(cell)
            self.wall_fix(off_, cx + off_ * hx_, cy + off_ * hy_)
            self.keep_off_walls(h_now)
            X, Y, yaw = self.pose()
        allc = [(x, y) for x in range(1, N + 1) for y in range(1, N + 1)]
        occ = {c for c in allc if self.grid is not None and self.grid.state(c) == P.OCC}
        okc = {c for c in allc if self.grid is not None and self.grid.direct_state(c) == P.FREE} | {cell}   # 벽은 미러까지 피하고, 서는 자리는 직접 본 칸만
        yaw1 = math.radians(HEADING_YAW[h])
        cmd = getattr(self, "pending_cmd", None)
        yaw_move = yaw1                                    # 돈 다음 가는 방향 (기본 = 새 몸 방향, 앞으로)
        if cmd is not None and cmd.action == "MOVE" and cmd.next_heading == h:     # 돈 다음 갈 곳 (시뮬레이션 TableExec 와 같게)
            nc = cmd.next_cell
            if cmd.move_dir is not None: yaw_move = math.radians(HEADING_YAW[cmd.move_dir])
            if nc == P.TARGET and self.a.grab_mode == "station": tx, ty = self.rest_xy(cell, h)          # 큐브 옆 칸 정면 멈춤 자세
            elif nc == P.TARGET:
                ux, uy = P.DXY[h]; cx_, cy_ = self.center(nc); tx, ty = cx_ - ux * self.a.grab_standoff, cy_ - uy * self.a.grab_standoff
            else: tx, ty = self.rest_xy(nc, h)
        else:
            tx, ty = self.rest_xy(cell, h)
        OB = TS.obstacles(occ, self.c)
        t_s = time.monotonic()
        sp = TS.turn_spot(X, Y, yaw, cell, yaw1, tx, ty, OB, okc, self.c, yaw_move)
        how = "넓은 자리"
        if sp is None:
            off = TS.table_offset(cell, h_now, h, occ, P)
            if off is None:
                if getattr(self, "tp", None) is not None:
                    self.tp.forbid.add((cell, h_now, h))
                self.log("  %s 에서 %s→%s 회전 자리 없음 → 돌지 않고 기억, 다시 계획" % (cell, P.HEADING_NAME[h_now], P.HEADING_NAME[h]))
                return
            cx, cy = self.center(cell); sp = (0.0, cx + off[0], cy + off[1]); how = "표 자리"
        px, py = sp[1], sp[2]; cx, cy = self.center(cell)
        self.log("  회전 자리(%s): %s 중심에서 (%+.1f, %+.1f) cm, 회전 원 벽까지 %.1f cm, 찾는 데 %.0f ms"
                 % (how, cell, 100 * (px - cx), 100 * (py - cy), 100 * (OB.distance(TS.Point(px, py)) - TS.R_SWEEP), 1000 * (time.monotonic() - t_s)))
        if math.hypot(X - px, Y - py) > 0.008:
            try:
                self._drive(px, py, math.radians(HEADING_YAW[h_now]), 0.012, math.radians(self.a.yaw_tol * 2), "회전 자리로",
                            guard=False, min_v=self.a.settle_min_v, timeout=self.a.settle_timeout * 2, brake="long")
            except (GuardStop, MoveTimeout):
                self.stop()
        self.log("  제자리 회전 → %s" % P.HEADING_NAME[h])
        if self.a.turn_odom_hold:
            self.rotate_in_place(yaw1)
        else:
            self._drive(px, py, yaw1, self.a.turn_center_tol * 1.5, math.radians(self.a.yaw_tol), "회전", guard=False, hold_tol=self.a.turn_hold_tol)
        self.stop()
        # 다음 길(다음 목표를 지나는 새 방향 직선)에서 옆으로 벗어난 만큼 먼저 옆으로만 맞춤 (회전 자리를 고를 때 이 'ㄱ자' 길을 벽 검사함).
        # 앞으로 가며 비스듬히 맞추면 옆 성분이 작아 메카넘이 옆으로 거의 안 감 → 비켜선 쪽 상자를 긁음 (필드 10-30 01:34 (6,2)→(6,3))
        X, Y, _ = self.pose(); fx, fy = TS.foot(X, Y, tx, ty, yaw_move)
        if math.hypot(X - fx, Y - fy) > 0.01:
            side_ = abs(wrap(yaw_move - yaw1)) < 0.1                 # 앞으로 갈 길이면 옆으로 맞춤, 옆걸음 길이면 앞뒤로 맞춤
            self.log("  %s 줄 맞춤 %.1f cm (다음 길 위로, 그다음 곧게)" % ("옆으로" if side_ else "앞뒤로", 100 * math.hypot(X - fx, Y - fy)))
            try:   # 최소 속도: 앞뒤는 0.10 (0.16 이면 굴러가 앞벽에 닿음, 앞은 라이다 사각), 옆은 _drive 가 알아서 0.16 이상
                self._drive(fx, fy, yaw1, 0.01, math.radians(self.a.yaw_tol * 2), "줄 맞춤",
                            guard=False, min_v=self.a.min_v, timeout=self.a.settle_timeout, brake="lat" if side_ else "long")
            except (GuardStop, MoveTimeout):
                self.stop()
            self.stop()

    def release_cube(self):
        """출발점 도착 뒤 그리퍼 열기 (잡기 스테이션의 '그리퍼 열기'와 같은 명령, 바퀴는 안 움직임)."""
        if getattr(self, "_front_with_cube", False):
            self._front_with_cube = False
            self.a.front -= self.a.cube_out
        import urllib.request
        try:
            with urllib.request.urlopen(self.a.station_url.rstrip("/") + "/do?a=open", timeout=3) as r:
                r.read()
            time.sleep(0.8)
            self.log("출발점에서 그리퍼 열어 큐브 놓음")
        except Exception as e:
            self.log("그리퍼 열기 실패 (%s) — 폰 [그리퍼 열기]로 여세요" % e)

    def open_gripper_key(self):
        """o 키: 잡기 스테이션에 그리퍼 열기 요청 (중간에 멈췄을 때 큐브 놓기, 10-30 사용자 요청)."""
        import urllib.request
        try:
            with urllib.request.urlopen(self.a.station_url.rstrip("/") + "/do?a=open", timeout=3) as r:
                r.read()
            self.log("o 키: 그리퍼 열기 보냄")
        except Exception as e:
            self.log("o 키: 그리퍼 열기 실패 (%s) — 노트북에서 그리퍼열기.sh" % e)

    def station_grab(self):
        """잡기 스테이션에 재기·맞추기·들어가기·쥐기를 맡기고 끝날 때까지 바퀴 명령을 내지 않고 기다림. (성공, 결과 글)"""
        import json as _json, urllib.request
        base = self.a.station_url.rstrip("/")

        def get(path):
            with urllib.request.urlopen(base + path, timeout=3) as r:
                return _json.loads(r.read().decode())
        try:
            st = get("/status")
        except Exception as e:
            return False, "잡기 스테이션 연결 안 됨 (%s)" % e
        if st.get("busy"):
            return False, "잡기 스테이션이 다른 동작 중"
        self.stop()
        self.log("  큐브 옆 칸 정면에서 멈춤 → 잡기 스테이션: 재기·맞추기·들어가기·쥐기")
        get("/do?a=grab&owner=mission&quick=1")
        t0, busy_seen = time.monotonic(), False
        while time.monotonic() - t0 < self.a.station_timeout:
            if self.quit_req or self.estop:
                try: get("/do?a=abort")
                except Exception: pass
                if self.quit_req: raise MissionAbort("x 키로 종료 (잡기 중)")
                return False, "비상정지로 잡기 멈춤"
            time.sleep(0.2)
            st = get("/status")
            if st.get("busy"): busy_seen = True; continue
            if busy_seen or st.get("result"): break
        else:
            try: get("/do?a=abort")
            except Exception: pass
            return False, "잡기 시간 초과 (%.0f s)" % self.a.station_timeout
        for line in (st.get("log") or [])[-8:]:
            self.log("    [스테이션] " + line)
        return st.get("result") == "ok", st.get("result") or "결과 없음"

    def align_heading_wall(self, label):
        """서 있을 때 벽 직선 각도(정확도 약 0.3°)로 몸을 격자에 맞춘다. 제자리 회전은 0.45 rad/s 보다 약하면 바퀴가 안 돌아서
        (10-21 14:37 실패) 실제 회전 세기(min_w)로 짧게 돌리고 → 멈춰 다시 재기를 최대 3 번."""
        a = self.a
        if a.wall_heading_tol <= 0:
            return
        first = None
        for k in range(3):
            time.sleep(0.12)
            yc = self.wall_yaw_any()
            if yc is None or abs(yc) > math.radians(8.0):
                break
            if first is None:
                first = yc
            if abs(yc) <= math.radians(a.wall_heading_tol if k == 0 else 0.6):
                break
            # 짧은 펄스는 바퀴 반응이 늦어 명령의 일부만 돎(칸 맞춤 펄스와 같은 생각) → 그만큼 길게, 다음에 다시 재서 고침
            dur = min(0.4, 0.05 + abs(yc) / (a.min_w * a.heading_pulse_eff))
            t_end = time.monotonic() + dur
            while time.monotonic() < t_end:
                self.hold_if_estop()
                self.send(0.0, 0.0, math.copysign(a.min_w, yc))
                time.sleep(0.02)
            self.stop()
        if first is not None and abs(first) > math.radians(a.wall_heading_tol):
            y2 = self.wall_yaw_any()
            self.log("  %s: 격자에서 몸 틀어짐 %+.1f° → 제자리에서 맞춤 → 남은 %s" % (
                label, -math.degrees(first), "%+.1f°" % -math.degrees(y2) if y2 is not None else "-"))

    def rotate_in_place(self, yaw_t):
        """제자리 회전 + 오돔으로 제자리 유지: 시작 자리(오돔)에서 밀린 만큼 바로 되밀며 돈다.
        필드 10-21 키보드 측정: 제자리 회전마다 1~4 cm 밀림 (회전 모서리 여유 1.2~1.7 cm 보다 큼).
        위치추정은 짧은 순간 흔들려 쫓으면 더 밀리므로(10-21 앞선 시도), 짧은 시간엔 매끄럽고 정확한 오돔(±1%)을 기준으로."""
        a = self.a
        try:
            ox0, oy0, _ = self.lookup_odom()
        except Exception:
            ox0 = oy0 = None
        t0 = time.monotonic()
        max_d = 0.0
        w_meas, prev_th = 0.0, None
        while True:
            if self.hold_if_estop():
                t0 = time.monotonic()
            if time.monotonic() - t0 > a.move_timeout:
                self.stop()
                raise MoveTimeout("회전 시간 초과 (%.0f s)" % a.move_timeout)
            _, _, yaw = self.pose()
            ew = wrap(yaw_t - yaw)
            try:                                              # 실제 도는 속도 (오돔·자이로)
                th_, tn_ = self.lookup_odom()[2], time.monotonic()
                if prev_th is not None and tn_ - prev_th[1] > 0.02:
                    w_meas = 0.5 * w_meas + 0.5 * wrap(th_ - prev_th[0]) / (tn_ - prev_th[1])
                prev_th = (th_, tn_)
            except Exception:
                pass
            # 회전도 멈춘 뒤 굴러감 → 도는 속도 × 굴러가는 시간만큼 미리 멈춤 (예전: 4° 안이면 멈춰 1.2 rad/s 에서 4~5° 어긋남, 10-21 15:07)
            lead = abs(w_meas) * a.turn_brake_t if a.turn_brake_t > 0 else 0.0
            if abs(ew) < math.radians(a.yaw_tol) if a.turn_brake_t <= 0 else abs(ew) < max(math.radians(a.turn_end_tol), lead):
                break
            w = max(-a.max_w, min(a.max_w, a.kw * ew))
            if abs(w) < a.min_w:
                w = math.copysign(a.min_w, ew)
            vx = vy = 0.0
            if ox0 is not None:
                try:
                    ox, oy, oth = self.lookup_odom()
                    dx, dy = ox - ox0, oy - oy0
                    d = math.hypot(dx, dy)
                    max_d = max(max_d, d)
                    if d > a.turn_hold_dead:
                        bx = math.cos(oth) * dx + math.sin(oth) * dy     # 몸 기준 밀린 양
                        by = -math.sin(oth) * dx + math.cos(oth) * dy
                        sp = max(a.turn_hold_vmin, min(a.turn_hold_vmax, a.turn_hold_k * d))
                        vx, vy = -bx / d * sp, -by / d * sp
                except Exception:
                    pass
            self.send(vx, vy, w)
            time.sleep(0.05)
        if a.turn_brake_t > 0 and abs(w_meas) > 0.15 and a.active_brake > 0:
            # 반대로 아주 짧게 돌려 각도 굴러감을 줄임 (직진 능동 제동과 같은 생각)
            t_b = time.monotonic()
            sgn = math.copysign(1.0, w_meas)
            while time.monotonic() - t_b < 0.15:
                self.send(0.0, 0.0, -sgn * a.min_w)
                time.sleep(0.02)
                try:
                    th_ = self.lookup_odom()[2]
                    if prev_th is not None and (th_ - prev_th[0]) * sgn <= 0.002:
                        break                                 # 멈췄거나 반대로 돌기 시작
                    prev_th = (th_, time.monotonic())
                except Exception:
                    break
        self.stop()
        if a.turn_brake_t > 0:
            self.wait_stopped(0.15, longest=0.8)
            _, _, yaw = self.pose()
            ew = wrap(yaw_t - yaw)
            if abs(ew) > math.radians(a.turn_end_fix):           # 그래도 남으면 한 번 짧게 마저 돌림
                t_end = time.monotonic() + min(0.3, 0.05 + abs(ew) / (a.min_w * 0.6))
                while time.monotonic() < t_end:
                    self.hold_if_estop()
                    self.send(0.0, 0.0, math.copysign(a.min_w, ew))
                    time.sleep(0.02)
                self.stop()
                time.sleep(0.15)
                _, _, yaw2 = self.pose()
                self.log("  회전 끝 %+.1f° 남아 한 번 더 맞춤 → %+.1f°" % (math.degrees(ew), math.degrees(wrap(yaw_t - yaw2))))
            else:
                self.log("  회전 끝 오차 %+.1f°" % math.degrees(ew))
        self.log("  회전 중 오돔 기준 최대 밀림 %.1f cm (제자리 유지 켬)" % (100 * max_d))

    def tight_strafe(self, from_cell, cell, mdir, keep_heading):
        """옆걸음인데 지나갈 두 칸의 앞쪽·뒤쪽에 모두 장애물(경기장 밖 포함)이 있으면 = 1칸 틈 (앞뒤 여유 합 4.8 cm 고정).
        옆 속도를 낮춰 바퀴 속도에 여유를 남김 — 옆걸음 중 회전 명령이 안 먹힘(실기 10-30 22:07·22:31: IMU PID 로도 3° 에서 못 되돌림)."""
        if self.a.gap_v <= 0 or mdir is None or mdir in (keep_heading, (keep_heading + 2) % 4):
            return False
        hx_, hy_ = P.DXY[keep_heading]
        blocked = lambda c: (not P.inside(c)) or self.cell_known(c) is True
        fr = any(blocked((c[0] + hx_, c[1] + hy_)) for c in (from_cell, cell))
        bk = any(blocked((c[0] - hx_, c[1] - hy_)) for c in (from_cell, cell))
        if fr and bk:
            self.log("  1칸 틈 옆걸음 (앞·뒤 다 막힘) → 천천히 %.2f m/s" % self.a.gap_v)
        return fr and bk

    def facing_gap(self, bx, by):
        """wall_gap 과 같은 거리지만, 그 거리 근처(1.5 cm) 점들이 몸 폭 방향으로 face_span 넘게 퍼져 있을 때만 = 정면으로 마주 본 벽.
        라이다는 앞 7 cm 안을 못 봐서, 옆걸음으로 다가가는 상자의 옆면을 비스듬히 본 점(폭 방향으로 한 줄)이 '앞 벽 8 cm' 로 잡힘
        → 앞뒤 가운데로 가려고 앞으로 밀다 상자 모서리에 닿음 (가상 10-30 대회형_14: 실제 앞 2.0 cm)."""
        g = self.wall_gap(bx, by)
        if g is None or self.a.face_span <= 0:
            return g
        a = self.a
        px, py = self.guard_pts[1], self.guard_pts[2]
        if abs(bx) >= abs(by):
            along, lat, half, lat_ok = (px if bx > 0 else -px), py, (a.front if bx > 0 else a.back), np.abs(py) < a.side - 0.04
        else:
            along, lat, half, lat_ok = (py if by > 0 else -py), px, a.side, (px > -a.back + 0.04) & (px < a.front - 0.04)
        # 그쪽 차체 면이 라이다에서 최소 거리(face_blind) 밖이면 가까운 진짜 면이 숨을 수 없음 → 부분만 보인 면도 진짜 → 그대로 씀.
        # (실기 10-30 19:54: 뒤(라이다에서 27 cm)까지 거르니 (5,3) 상자가 뒤에 막 걸릴 때 폭이 좁다고 버려 뒤 간격 −0.4 cm 까지 붙음.
        #  앞은 라이다에서 8 cm 라 사각 안 → 비스듬한 옆면만 보이므로 거름)
        try:
            lx_, ly_, _ = self.get_laser_base()
        except Exception:
            lx_, ly_ = 0.0, 0.0
        lo = lx_ if abs(bx) >= abs(by) else ly_
        face_from_lidar = half - (lo if (bx > 0 or by > 0) else -lo)
        if face_from_lidar >= a.face_blind:
            return g
        m = lat_ok & (along > half) & (along < half + g + 0.015)
        if m.sum() < 5 or np.ptp(lat[m]) < a.face_span:
            if abs(bx) >= abs(by) and abs(g - self.want_long()) < 0.08:
                k_ = "앞" if bx > 0 else "뒤"
                n_, w_ = self.face_skip.get(k_, (0, 9.0))
                self.face_skip[k_] = (n_ + 1, min(w_, float(np.ptp(lat[m])) if m.sum() else 0.0))
            return None
        return g

    def wall_center_err(self, axis, back_bias=0.0):
        """벽까지 거리로 본 '칸 가운데로 가려면 움직일 양' [m] (몸 기준). axis 0 = 좌우(왼 +), 1 = 앞뒤(앞 +).
        양쪽 벽이 보이면 가운데, 한쪽만이면 설계 여유 기준. 벽이 없거나 예상(±8 cm)을 벗어나면 None."""
        if axis == 0:
            gp, gn, want = self.facing_gap(0.0, 1.0), self.facing_gap(0.0, -1.0), self.want_side()
        else:
            gp, gn, want = self.facing_gap(1.0, 0.0), self.facing_gap(-1.0, 0.0), self.want_long()
        ok = lambda g: g is not None and abs(g - want) < 0.08
        if ok(gp) and ok(gn) and self.a.face_span > 0 and abs(gp + gn - 2 * want) > 0.015:
            # 벽은 칸 경계에만 있으니 양쪽 다 진짜 벽이면 간격 합 = 칸 - 몸 길이. 안 맞으면 먼 쪽이 비스듬히 본 다른 면
            if gp > gn: gp = None
            else: gn = None
        if ok(gp) and ok(gn):
            return (gp - gn) / 2.0 - (back_bias if axis == 1 else 0.0)
        if ok(gp):
            return gp - want
        if ok(gn):
            return (want - (back_bias if axis == 1 else 0.0)) - gn
        return None

    def wall_yaw_err(self):
        """뒤 벽 점들로 직선을 맞춰 몸 방향이 벽과 얼마나 틀어졌는지 [rad] (돌아야 할 양, 반시계 +). 못 재면 None."""
        g = self.guard_pts
        if g is None or time.monotonic() - g[0] > 0.4:
            return None
        px, py = g[1], g[2]
        sel = (px < -(self.a.back + 0.003)) & (px > -(self.a.back + self.want_long() + 0.08)) & (np.abs(py) < 0.15)
        if sel.sum() < 15:
            return None
        X_, Y_ = py[sel], px[sel]
        for _ in range(2):                                  # 몸 기준 px = a·py + b, 1 cm 넘게 벗어난 점은 빼고 다시
            A = np.c_[X_, np.ones_like(X_)]
            coef, *_ = np.linalg.lstsq(A, Y_, rcond=None)
            res = np.abs(Y_ - A @ coef)
            keep = res < 0.01
            if keep.sum() < 15:
                return None
            X_, Y_ = X_[keep], Y_[keep]
        if np.ptp(X_) < 0.10:                               # 벽이 10 cm 넘게 보여야 각도를 믿음
            return None
        return -math.atan(coef[0])

    def wall_yaw_any(self):
        """보이는 벽(뒤·앞·왼·오른) 중 점이 가장 많은 쪽에 직선을 맞춰, 몸이 격자와 얼마나 틀어졌는지 [rad] (돌아야 할 양). 없으면 None."""
        g = self.guard_pts
        if g is None or time.monotonic() - g[0] > 0.4:
            return None
        px, py = g[1], g[2]
        a = self.a
        wl, ws = self.want_long(), self.want_side()
        cands = [  # (선택, 독립변수, 종속변수, 부호): 앞뒤 벽은 px=a·py+b → -atan(a), 옆 벽은 py=a·px+b → +atan(a)
            ((px < -(a.back + 0.003)) & (px > -(a.back + wl + 0.08)) & (np.abs(py) < 0.15), py, px, -1.0),
            ((px > a.front + 0.003) & (px < a.front + wl + 0.08) & (np.abs(py) < 0.15), py, px, -1.0),
            ((py > a.side + 0.003) & (py < a.side + ws + 0.08) & (np.abs(px) < 0.15), px, py, 1.0),
            ((py < -(a.side + 0.003)) & (py > -(a.side + ws + 0.08)) & (np.abs(px) < 0.15), px, py, 1.0)]
        best = None
        for sel, X_, Y_, sg in cands:
            if sel.sum() < 15:
                continue
            X_, Y_ = X_[sel], Y_[sel]
            ok_ = True
            for _ in range(2):
                A = np.c_[X_, np.ones_like(X_)]
                coef, *_ = np.linalg.lstsq(A, Y_, rcond=None)
                keep = np.abs(Y_ - A @ coef) < 0.01
                if keep.sum() < 15:
                    ok_ = False
                    break
                X_, Y_ = X_[keep], Y_[keep]
            if not ok_ or np.ptp(X_) < 0.10:
                continue
            if best is None or len(X_) > best[0]:
                best = (len(X_), sg * math.atan(coef[0]))
        return None if best is None else best[1]

    def wall_dist(self, bx, by, rng=2.4):
        """몸 기준 축 방향(bx, by)으로 가장 가까운 벽까지 거리 [m, 바퀴 가운데 기준] — 좁은 띠(±6 cm) 안 가장 앞쪽 점 무리의 중앙값."""
        g = self.guard_pts
        if g is None or time.monotonic() - g[0] > 0.4:
            return None
        px, py = g[1], g[2]
        along = px * bx + py * by
        lat = -px * by + py * bx
        sel = (np.abs(lat) < 0.06) & (along > 0.12) & (along < rng)
        if sel.sum() < 6:
            return None
        v = np.sort(along[sel])
        near = v[v < v[min(len(v) - 1, max(2, len(v) // 10))] + 0.03]
        return float(np.median(near)) if near.size >= 4 else None

    def calib_snapshot(self):
        """서 있을 때: 오돔 자세 + 벽으로 잰 칸 가운데에서의 어긋남(몸 기준) + 벽 각도로 잰 몸 방향 어긋남."""
        self.wait_stopped(0.3, longest=1.5)
        time.sleep(0.1)
        off = (self.a.back - self.a.front) / 2.0
        m = self.measure_walls(off)
        try:
            od = self.lookup_odom()
        except Exception:
            od = None
        yc = self.wall_yaw_any()
        X, Y, yaw = self.pose()
        dists = {k: self.wall_dist(*v) for k, v in (("f", (1.0, 0.0)), ("b", (-1.0, 0.0)), ("l", (0.0, 1.0)), ("r", (0.0, -1.0)))}
        return {"od": od, "cx": None if m is None else m["cx"], "cy": None if m is None else m["cy"],
                "yawc": yc, "pose": (X, Y, yaw), "cell": self.last_cell, "dist": dists}

    def calib_compare(self, kind, b, a_, step_dir_body=None, turn_sign=0):
        """동작 앞뒤 스냅샷으로 실제 이동(벽 기준)과 오돔·위치추정 이동 비교 → 기록. 바닥마다 달라 나중에 주행 중 자동 보정에 씀."""
        rec = {"kind": kind}
        if b["od"] is None or a_["od"] is None:
            return
        th0 = b["od"][2]
        dxo, dyo = a_["od"][0] - b["od"][0], a_["od"][1] - b["od"][1]
        if kind == "move" and a_["cell"] != b["cell"]:
            bx, by = step_dir_body                                     # 몸 기준 이동 축 (1,0)/(0,1) 등
            od_along = (math.cos(th0) * dxo + math.sin(th0) * dyo) * bx + (-math.sin(th0) * dxo + math.cos(th0) * dyo) * by
            # 실제 이동 = 가는 쪽 벽까지 거리가 준 만큼, 뒤쪽 벽까지 거리가 는 만큼 (둘 다 보이면 평균, 한 칸 ±12 cm 안만)
            ahead = {(1, 0): "f", (-1, 0): "b", (0, 1): "l", (0, -1): "r"}[(bx, by)]
            behind = {"f": "b", "b": "f", "l": "r", "r": "l"}[ahead]
            ests = []
            if b["dist"][ahead] is not None and a_["dist"][ahead] is not None:
                ests.append(b["dist"][ahead] - a_["dist"][ahead])
            if b["dist"][behind] is not None and a_["dist"][behind] is not None:
                ests.append(a_["dist"][behind] - b["dist"][behind])
            ests = [e for e in ests if abs(e - self.c) < 0.12]
            if ests:
                true_along = float(np.mean(ests))
                rec.update(axis="long" if bx else "lat", true=true_along, odom=od_along)
                sl = math.hypot(a_["pose"][0] - b["pose"][0], a_["pose"][1] - b["pose"][1])
                rec["loc"] = sl
                self.log("  [측정] %s 이동: 벽 기준 실제 %.1f cm · 오돔 %.1f cm (%+.0f%%) · 위치추정 %.1f cm (%+.0f%%)" % (
                    "앞뒤" if bx else "옆", 100 * true_along, 100 * od_along, 100 * (od_along / true_along - 1),
                    100 * sl, 100 * (sl / true_along - 1)))
        elif kind == "turn":
            if b["yawc"] is None or a_["yawc"] is None:
                self.log("  [측정] 회전: 벽 각도를 못 잼 (회전 전 %s, 회전 뒤 %s) → 이번 회전은 측정 빠짐" % (
                    "있음" if b["yawc"] is not None else "없음", "있음" if a_["yawc"] is not None else "없음"))
            if b["yawc"] is not None and a_["yawc"] is not None:
                true_rot = turn_sign * math.pi / 2 - a_["yawc"] + b["yawc"]
                od_rot = wrap(a_["od"][2] - b["od"][2])
                lo_rot = wrap(a_["pose"][2] - b["pose"][2])
                rec.update(true=true_rot, odom=od_rot, loc=lo_rot)
                self.log("  [측정] 회전: 벽 기준 실제 %+.1f° · 오돔 %+.1f° (%+.1f°) · 위치추정 %+.1f° (%+.1f°) · 회전 뒤 벽과 틀어짐 %+.1f°" % (
                    math.degrees(true_rot), math.degrees(od_rot), math.degrees(od_rot - true_rot),
                    math.degrees(lo_rot), math.degrees(lo_rot - true_rot), -math.degrees(a_["yawc"])))
        if len(rec) > 1:
            self.calib_log.append({k: (round(v, 4) if isinstance(v, float) else v) for k, v in rec.items()})

    def is_slot(self, cell, heading):
        """cell 의 앞·뒤(몸 방향 기준) 칸이 다 막혔나 = 옆으로 지날 때 앞뒤 여유가 3 cm 뿐인 좁은 칸."""
        hx_, hy_ = P.DXY[heading]
        blocked = lambda c: (not P.inside(c)) or self.cell_known(c) is True
        return blocked((cell[0] + hx_, cell[1] + hy_)) and blocked((cell[0] - hx_, cell[1] - hy_))

    def slot_align(self, cell):
        """좁은 칸에 옆으로 들어가기 전: 뒤 벽 거리·각도로 앞뒤 위치와 몸 방향을 맞춘다. 이미 맞으면 서지 않는다."""
        a = self.a
        e = self.wall_center_err(1, a.slot_back_bias)
        y = self.wall_yaw_err()
        if (e is None or abs(e) <= a.slot_pos_tol) and (y is None or abs(y) <= math.radians(a.slot_yaw_tol)):
            return
        self.halt()
        self.wait_stopped(0.2, longest=1.5)
        for k in range(3):
            time.sleep(0.15)
            y = self.wall_yaw_err()
            if y is not None and abs(y) > math.radians(a.slot_yaw_tol):
                X, Y, yaw = self.pose()
                try:
                    self._drive(X, Y, yaw + y, 0.06, math.radians(0.8), "좁은 칸 앞 방향 맞춤", guard=False, timeout=2.0)
                except (GuardStop, MoveTimeout):
                    pass
                self.stop()
                time.sleep(0.15)
            e = self.wall_center_err(1, a.slot_back_bias)
            if e is None or abs(e) <= a.slot_pos_tol:
                break
            v = a.settle_min_v
            t_end = time.monotonic() + min(0.3, 0.05 + abs(e) / (v * 0.35))
            while time.monotonic() < t_end:
                self.hold_if_estop()
                self.send(math.copysign(v, e), 0.0, 0.0)
                time.sleep(0.02)
            self.stop()
        e2, y2 = self.wall_center_err(1, a.slot_back_bias), self.wall_yaw_err()
        self.log("  좁은 칸 %s 앞 정렬 (뒤 벽 기준): 앞뒤 %s, 방향 %s" % (
            cell, "%+.1f cm" % (100 * e2) if e2 is not None else "-", "%+.1f°" % math.degrees(y2) if y2 is not None else "-"))

    def want_long(self):
        """칸 가운데(몸통)에 섰을 때 앞·뒤 차체 면과 벽 사이 여유 (3.15 cm)."""
        return self.c / 2.0 - (self.a.front + self.a.back) / 2.0

    def want_side(self):
        return self.c / 2.0 - self.a.side

    def wall_want(self, mdir, heading):
        """칸 가운데(몸통 기준)에 섰을 때 mdir 쪽 차체 면과 그 칸 벽 사이 여유 [m]. 앞뒤 3.15 cm, 옆 7.75 cm."""
        off = (self.a.back - self.a.front) / 2.0
        if mdir == heading:
            return self.c / 2.0 - (self.a.front + off)
        if mdir == (heading + 2) % 4:
            return self.c / 2.0 - (self.a.back - off)
        return self.c / 2.0 - self.a.side

    def halt(self, v_now=None):
        """멈추기. 펌웨어는 0 명령이면 모터를 풀어 굴러간다(0.3 m/s 에서 14 cm, 0.38 m/s 에서 21 cm).
        active_brake > 0 이면 가던 반대로 짧게 명령해 바퀴를 세운다. 가던 방향 속도가 0.05 m/s 아래로 떨어지거나
        반대로 가기 시작하면 바로 끊는다 (세게·오래 걸면 차체가 뒤로 끌려감 — 필드 10-21 12:4x).
        제동 시간 상한은 속도에 비례 (0.3 m/s 에서 active_brake_t 초)."""
        a = self.a
        cx, cy = self.last_cmd
        n = math.hypot(cx, cy)
        if a.active_brake <= 0 or n < 0.05 or self.estop:
            self.stop()
            return
        ux, uy = cx / n, cy / n
        try:
            ox, oy, oth = self.lookup_odom()
        except Exception:
            self.stop()
            return
        dux, duy = math.cos(oth) * ux - math.sin(oth) * uy, math.sin(oth) * ux + math.cos(oth) * uy   # 가던 방향 (오돔 좌표)
        v0 = v_now if v_now is not None else n
        t_max = min(a.active_brake_max, a.active_brake_t * max(v0, 0.05) / 0.3)
        prev = (ox, oy, time.monotonic())
        t0 = time.monotonic()
        while time.monotonic() - t0 < t_max:
            self.send(-ux * a.active_brake, -uy * a.active_brake, 0.0)
            time.sleep(0.02)
            try:
                ox, oy, _ = self.lookup_odom()
            except Exception:
                break
            now = time.monotonic()
            if now - prev[2] >= 0.04:
                v_along = ((ox - prev[0]) * dux + (oy - prev[1]) * duy) / (now - prev[2])
                if v_along < 0.05:
                    break
                prev = (ox, oy, now)
        self.last_cmd = (0.0, 0.0)
        self.stop()

    def wait_stopped(self, at_least, longest=2.0):
        """정지 명령 뒤 실제로 멈출 때까지(오돔 속도 1.5 cm/s 미만이 0.15 초) 기다림. 최소 at_least 초 (서서 격자 보정 시간).
        빠르게 가면 굴러가는 시간이 0.6 초보다 길어, 고정 시간만 기다리면 아직 굴러가는 중에 재고 맞춰서 벽에 닿음 (필드 10-21 12:15 (6,2))."""
        t0 = time.monotonic()
        still_since, prev = None, None
        while True:
            now = time.monotonic()
            try:
                ox, oy, _ = self.lookup_odom()
                if prev is not None and now - prev[2] > 0.04:
                    v = math.hypot(ox - prev[0], oy - prev[1]) / (now - prev[2])
                    if v < 0.015:
                        still_since = still_since or now
                    else:
                        still_since = None
                    prev = (ox, oy, now)
                elif prev is None:
                    prev = (ox, oy, now)
            except Exception:
                still_since = still_since or now
            if now - t0 >= at_least and still_since is not None and now - still_since >= 0.15:
                return
            if now - t0 >= longest:
                return
            self.send_zero()
            time.sleep(0.03)

    def learn_brake(self, brake, v0, why, odom0):
        """멈춘 뒤 실제로 굴러간 거리(오돔)로 제동 시간 다시 잼: brake_tb = 굴러간 거리 / 멈출 때 속도."""
        a = self.a
        if not brake or not a.brake_learn or odom0 is None or v0 < 0.12 or why not in ("brake", "pos", "wall"):
            return
        try:
            ox, oy, _ = self.lookup_odom()
        except Exception:
            return
        coast = math.hypot(ox - odom0[0], oy - odom0[1])
        tb_obs = coast / v0
        old = self.brake_tb[brake]
        self.brake_tb[brake] = min(a.brake_max, max(0.0, 0.5 * old + 0.5 * tb_obs))
        self.brake_log.append({"axis": brake, "v": round(v0, 3), "coast": round(coast, 4), "tb": round(self.brake_tb[brake], 3)})
        try:
            data = json.load(open(self.brake_file)) if os.path.exists(self.brake_file) else {}
            data[self.brake_key] = {k: round(v, 3) for k, v in self.brake_tb.items()}
            tmp = self.brake_file + ".tmp"
            json.dump(data, open(tmp, "w"))
            os.replace(tmp, self.brake_file)
        except Exception:
            pass
        self.log("  제동: %.2f m/s 에서 멈춤 명령 뒤 %.1f cm 굴러감 → %s 미리 멈춤 %.2f 초 (전 %.2f)"
                 % (v0, 100 * coast, "앞뒤" if brake == "long" else "옆", self.brake_tb[brake], old))

    def keep_off_walls(self, heading):
        """도착 후 라이다로 잰 벽이 설계 여유보다 가까우면 벽 반대쪽으로만 짧게 민다 (벽 쪽으로는 절대 안 감)."""
        a = self.a
        if a.repel_tol <= 0:
            return
        for k in range(3):
            time.sleep(0.15)
            worst = None
            for mdir in range(4):
                gdx, gdy = P.DXY[mdir]
                _, _, yaw = self.pose()
                gbx = math.cos(yaw) * gdx + math.sin(yaw) * gdy
                gby = -math.sin(yaw) * gdx + math.cos(yaw) * gdy
                gb = (math.copysign(1.0, gbx), 0.0) if abs(gbx) >= abs(gby) else (0.0, math.copysign(1.0, gby))
                gap = self.wall_gap(*gb)
                if gap is None:
                    continue
                short = self.wall_want(mdir, heading) - gap
                if short > a.repel_tol and (worst is None or short > worst[0]):
                    worst = (short, mdir, gap, gb)
            if worst is None:
                return
            short, mdir, gap, gb = worst
            opp = (mdir + 2) % 4
            og = self.wall_gap(-gb[0], -gb[1])
            push = short
            if og is not None:
                push = min(push, max(0.0, og - (self.wall_want(opp, heading) - 0.005)))
            if push < 0.006:
                return
            self.log("  %s 벽까지 %.1f cm (여유 %.1f cm 보다 가까움) → 반대로 %.1f cm 떨어짐"
                     % (P.HEADING_NAME[mdir], 100 * gap, 100 * self.wall_want(mdir, heading), 100 * push))
            v = a.settle_min_v
            t_end = time.monotonic() + min(a.fine_pulse * 1.5, 0.05 + push / max(v * 0.35, 1e-3))
            while time.monotonic() < t_end:
                self.hold_if_estop()
                self.send(-gb[0] * v, -gb[1] * v, 0.0)
                time.sleep(0.02)
            self.stop()

    def translate_to(self, x_mm, y_mm, keep_heading):
        cell = P.mm_to_cell(x_mm, y_mm)            # planner 는 400 mm 기준 → 칸 번호로 되돌림
        if self.straight_run == 0:
            self.gaps_seen = {}
            self.face_skip = {}                        # 마주 본 벽이 아니라고 버린 앞·뒤 벽 거리 (수, 최소 폭) — 실기 확인용
            self.yaw_ref_odom = None
            if self.a.gyro_hold:
                try:
                    _, _, yl = self.pose()
                    corr = wrap(math.radians(HEADING_YAW[keep_heading]) - yl)
                    lim = math.radians(self.a.gyro_ref_max)
                    corr = max(-lim, min(lim, corr))          # 위치추정 방향이 크게 틀렸을 때 따라 돌지 않게 (10-21 (6,2) 5° 사례)
                    self.yaw_ref_odom = self.lookup_odom()[2] + corr
                except Exception:
                    self.yaw_ref_odom = None
        if self.a.turn_spot and self.current_heading() != keep_heading:
            self.log("  몸 방향이 %s 가 아님(회전 못 함) → 이 이동 취소, 다시 계획" % P.HEADING_NAME[keep_heading])
            return False
        from_cell = self.last_cell
        from_grab = self.at_grab_point
        if from_grab:
            gx_, gy_, _ = self.pose()
        mdir = P.direction(from_cell, cell) if abs(from_cell[0] - cell[0]) + abs(from_cell[1] - cell[1]) == 1 else None
        if self.straight_run > 0 and mdir != self.go_dir:
            # 서지 않고 지나가던 중(앞 칸 방향으로 속도가 남음)에 다시 계획되어 방향이 바뀜 → 바로 새 방향으로 가면 남은 속도로 모서리를
            # 질러 대각선 상자에 닿음 (가상 10-30 지그재그_d35: 북으로 지나가다 동으로 바뀌어 (4,4) 모서리). 서서 지금 칸 가운데로 맞춘 뒤 감
            self.log("  지나가던 중 길이 바뀜 (%s → %s) → 서서 칸 %s 가운데로 맞춘 뒤 이동" % (
                P.HEADING_NAME[self.go_dir], "?" if mdir is None else P.HEADING_NAME[mdir], from_cell))
            self.halt()
            self.straight_run = 0
            bx_, by_ = self.center(from_cell)
            try:
                self._drive(bx_, by_, math.radians(HEADING_YAW[keep_heading]), self.a.pos_tol, math.radians(self.a.yaw_tol * 2),
                            "길 바뀜 전 칸 가운데", guard=False, min_v=self.a.min_v, timeout=self.a.settle_timeout)
            except MoveTimeout:
                pass
            self.stop()
        Xt, Yt = self.center(cell)
        # 자율: 라이다로 빈 칸이 확인된 칸에만 들어간다. 뒤쪽은 배터리 쪽에 가려(몸 기준 120~163°, 193~260°) 뒤·옆으로 갈 때
        # 가는 쪽 칸이 안 보일 수 있다 → 서서 잠깐 보고, 그래도 모르면 그쪽으로 몸을 돌려 직접 본 뒤 다시 계획
        if (not self.a.teleop and cell != P.TARGET and mdir is not None and not self.at_grab_point
                and self.a.unknown_wait > 0 and self.cell_known(cell) is None):
            self.stop()
            t0 = time.monotonic()
            while time.monotonic() - t0 < self.a.unknown_wait and self.cell_known(cell) is None:
                self.hold_if_estop()
                time.sleep(0.1)
            st = self.cell_known(cell)
            if st is True:
                self.log("  가려던 칸 %s 은 서서 보니 장애물 → 이동 취소, 다시 계획" % (cell,))
                if self.grid is not None:
                    self.grid.observe(cell, True, self.now())
                return False
            if st is None and mdir != keep_heading:
                self.log("  가려던 칸 %s 이 %s 쪽이라 라이다로 안 보임 → 그쪽으로 몸을 돌려 보고 다시 계획" % (cell, P.HEADING_NAME[mdir]))
                self.turn_to(mdir)
                return False
        # 정면 바로 앞 칸은 라이다 사각지대(면이 라이다에서 약 8 cm, 최소 측정거리 15 cm) → 모르는 칸이면 물러서서 본다
        if (cell != P.TARGET and mdir is not None and mdir == keep_heading and self.a.peek > 0
                and not self.at_grab_point and self.cell_known(cell) is None):
            if not self.peek_ahead(from_cell, cell, keep_heading):
                return False
        self.at_grab_point = False
        if cell == P.TARGET and self.a.grab_mode == "station":
            # 큐브 옆 칸 가운데에서 큐브 정면으로 멈춤(카메라 41 cm) → 스테이션이 재기·맞추기·곧게 들어가기·쥐기 (10-29 실기 시험 방식)
            lx, ly = self.rest_xy(from_cell, keep_heading)
            X_, Y_, _ = self.pose()
            if math.hypot(X_ - lx, Y_ - ly) > 0.02:    # 칸 도착 맞춤이 이미 끝남 — 앞뒤 여유 2.4 cm 칸이라 1 cm 고치려다 스침(가상 시험), 마지막 맞춤은 카메라
                try:
                    self._drive(lx, ly, math.radians(HEADING_YAW[keep_heading]), 0.012, math.radians(self.a.yaw_tol * 2), "정면 멈춤 자세로",
                                guard=False, min_v=self.a.settle_min_v, timeout=self.a.settle_timeout * 2, brake="long")
                except (GuardStop, MoveTimeout):
                    self.stop()
            ok, msg = self.station_grab()
            if not ok and not self.quit_req and not self.estop and "비상정지" not in msg and "멈춤 버튼" not in msg:
                # 스테이션은 매번 지금 자리에서 다시 재므로 한 번 더 맡겨 봄 (한 번의 판정 실패로 임무 전체를 끝내지 않음)
                self.log("  잡기 실패 (%s) → 지금 자리에서 다시 재서 한 번 더" % msg)
                time.sleep(0.5)
                ok, msg = self.station_grab()
            if not ok:
                raise MissionAbort("큐브 잡기 실패: %s" % msg)
            self.last_cell = cell
            self.at_grab_point = True
            self.last_pose = None                   # 잡는 동안 20 cm 움직임 → 위치 튐 검사 다시 시작
            return True
        if cell == P.TARGET:
            # 큐브는 (5,5) 칸 중심에 있다. 칸 중심까지 가면 큐브를 밀어 버리므로
            # 이동 방향(= 정면, planner 가 정면 진입만 계획) 반대로 grab_standoff 만큼 앞에서 멈춘다.
            ux, uy = P.DXY[keep_heading]
            Xt, Yt = Xt - ux * self.a.grab_standoff, Yt - uy * self.a.grab_standoff
            self.log("  큐브 앞 정지점 → (%.2f, %.2f) m (큐브 중심에서 %.2f m 앞), 몸 %s"
                     % (Xt, Yt, self.a.grab_standoff, P.HEADING_NAME[keep_heading]))
        else:
            # 차체가 앞뒤로 비대칭(뒤가 더 김)이라 바퀴 가운데를 칸 중심보다 앞으로 옮겨 몸통을 칸 가운데에 둔다
            off = (self.a.back - self.a.front) / 2.0
            hx_, hy_ = P.DXY[keep_heading]
            Xt, Yt = Xt + off * hx_, Yt + off * hy_
            Xd, Yd = Xt, Yt                                       # 설계 목표 (벽 측정 비교 기준)
            if mdir is not None and mdir == (keep_heading + 2) % 4 and self.a.back_target_shift > 0:
                # 뒤로 가서 뒤가 벽인 칸: 평균 0.4 cm 앞에서 섬 (필드 10-21 뒤 간격 평균 3.5 cm, 설계 3.15) → 목표를 그만큼 뒤로.
                # 너무 붙는 건 벽 보고 멈춤(2.85 cm)이 막음
                bc = (cell[0] - hx_, cell[1] - hy_)
                if not P.inside(bc) or self.cell_known(bc) is True:
                    Xt, Yt = Xt - self.a.back_target_shift * hx_, Yt - self.a.back_target_shift * hy_
            if mdir is not None and mdir == keep_heading and self.a.front_target_shift > 0:
                # 앞으로 가서 앞이 벽인 칸(막다른 칸): 그리퍼를 달아 앞 여유가 6 → 2.4 cm 라 멈출 때 굴러가 닿음(가상 시험 10-29)
                # → 목표를 그만큼 덜 감. 정면 바로 앞 칸은 라이다 사각지대라 벽 보고 멈춤이 없음
                fc = (cell[0] + hx_, cell[1] + hy_)
                if not P.inside(fc) or self.cell_known(fc) is True:
                    Xt, Yt = Xt - self.a.front_target_shift * hx_, Yt - self.a.front_target_shift * hy_
            self.log("  이동 → 칸 %s (몸통 가운데 기준, 바퀴 중심 %+.0f cm 앞) (%.2f, %.2f) m, 몸 %s 유지"
                     % (cell, 100 * off, Xt, Yt, P.HEADING_NAME[keep_heading]))
        if (cell != P.TARGET and mdir is not None and mdir not in (keep_heading, (keep_heading + 2) % 4)
                and self.straight_run == 0 and (self.a.prealign > 0 or self.a.wall_prealign > 0) and not from_grab):   # 큐브 잡은 자리(칸 안쪽)에서 나갈 땐 안 함 (at_grab_point 는 위에서 이미 지워짐 — 10-30 가상: 잡은 뒤 앞으로 9.4 cm 더 감)
            # 옆걸음 전 앞뒤 맞춤: 앞뒤가 칸 기준 자세에서 어긋난 채 옆으로 가면 옆 칸 상자를 앞(그리퍼)·뒤로 스침 — 옆으로 가는 중엔
            # 앞뒤를 천천히만 고침 (가상 10-30: 출발·도착 맞춤 허용 3 cm 로 그리퍼 앞 0.9~2 cm 인 채 옆걸음 → 닿음 16 번)
            X_, Y_, _ = self.pose(); hx_, hy_ = P.DXY[keep_heading]
            along = (X_ - Xt) * hx_ + (Y_ - Yt) * hy_
            # 벽 쪽으로 치우쳤을 때만 (wall_prealign): 가는 칸의 앞(뒤) 칸이 장애물·경기장 밖인데 로봇이 그쪽으로 넘게 나가 있으면 먼저 반대로 맞춤.
            # 실기 10-30 19:54·20:12: (6,3)→(6,4) 앞으로 가다 2~4 cm 덜 서서, 뒤가 (5,3) 상자인 (5,4) 로 옆걸음하자마자 뒤 −0.4 cm.
            # 벽에서 멀어지는 쪽으로만 움직이므로 맞추다 그 벽에 닿지 않음 (모든 옆걸음·양쪽으로 맞추던 prealign 은 시뮬에서 오히려 나빴음)
            side_c = (cell[0] + hx_, cell[1] + hy_) if along > 0 else (cell[0] - hx_, cell[1] - hy_)
            wall_side = (not P.inside(side_c)) or self.cell_known(side_c) is True
            lim = self.a.prealign if self.a.prealign > 0 else 9.0
            if self.a.wall_prealign > 0 and wall_side:
                lim = min(lim, self.a.wall_prealign)
            if abs(along) > lim:
                self.log("  옆걸음 전 앞뒤 맞춤 %+.1f cm (몸 %s)" % (-100 * along, P.HEADING_NAME[keep_heading]))
                self.pulse_along(X_ - along * hx_, Y_ - along * hy_, hx_, hy_)
        if (from_grab and cell != P.TARGET and mdir is not None and mdir not in (keep_heading, (keep_heading + 2) % 4)
                and self.a.grab_exit_align):
            # 큐브 잡은 자리는 칸 가운데보다 15 cm 큐브 쪽(두 줄에 걸침) → 거기서 옆 칸 가운데로 바로 가면 대각선으로 두 칸을 쓸고
            # 몸이 8~11° 돌아감 (실기 10-30 새벽·저녁 같음). 먼저 앞뒤로만 다음 칸 가운데 줄에 맞추고 옆으로 곧게.
            X_, Y_, _ = self.pose(); hx_, hy_ = P.DXY[keep_heading]
            along = (X_ - Xt) * hx_ + (Y_ - Yt) * hy_
            if abs(along) > 0.01:
                self.log("  잡은 자리에서 옆으로 나가기 전 앞뒤 맞춤 %+.1f cm (몸 %s)" % (-100 * along, P.HEADING_NAME[keep_heading]))
                try:
                    self._drive(X_ - along * hx_, Y_ - along * hy_, math.radians(HEADING_YAW[keep_heading]), 0.008,
                                math.radians(self.a.yaw_tol * 2), "잡은 자리 앞뒤 맞춤", guard=False, min_v=self.a.min_v,
                                timeout=self.a.settle_timeout, brake="long")
                except (GuardStop, MoveTimeout):
                    self.stop()
                self.stop()
        go_on = cell != P.TARGET and self.a.straight and self.next_same_dir(cell, mdir)
        pass_pt = None
        if go_on:
            ux, uy = P.DXY[mdir]
            pass_pt = (Xt, Yt, ux, uy)
            Xt, Yt = Xt + ux * self.c, Yt + uy * self.c          # 다음 칸 가운데를 겨눠 속도 유지
            self.log("  다음 칸도 같은 방향·빈 칸 → 서지 않고 지나감 (연속 %d 칸째)" % (self.straight_run + 1))
        stop_gap = None
        brake = None
        if mdir is not None and not go_on:
            brake = "long" if mdir in (keep_heading, (keep_heading + 2) % 4) else "lat"
        if mdir is not None and cell != P.TARGET and not go_on:
            # 가는 방향 벽은 지도에 있든 없든(뒤는 배터리에 가려 모를 때가 많다) 라이다로 보이면 지킨다.
            # 목표 칸 너머가 비어 있으면 가장 가까운 면이 40 cm 더 멀어 걸리지 않는다.
            extra = self.a.back_stop_extra if mdir == (keep_heading + 2) % 4 else 0.0   # 뒤로 가서 뒤 벽에 설 때만 더 붙음
            if mdir == (keep_heading + 2) % 4:
                extra -= self.a.back_stop_margin             # 뒤로 벽에 설 때 1 cm 일찍 (앞은 front_target_shift 2 cm, 뒤는 여유가 없어 제동 예측 2 cm 틀리면 닿음 — 가상 10-30 큐브 잡고 뒤로 두 칸)
            stop_gap = (mdir, max(0.012, self.wall_want(mdir, keep_heading) - 0.003 - extra))
        self.cur_stop_gap = stop_gap
        slot = (mdir is not None and mdir not in (keep_heading, (keep_heading + 2) % 4) and cell != P.TARGET
                and self.a.slot_v > 0 and self.is_slot(cell, keep_heading))
        if slot:
            self.slot_align(cell)
        late_go = None
        if not go_on and self.a.straight and mdir is not None and cell != P.TARGET and not self.a.teleop:
            late_go = lambda: self.late_same_dir(cell, mdir, keep_heading)
        try:
            for _ in (1, 2):
                self._drive(Xt, Yt, math.radians(HEADING_YAW[keep_heading]), self.a.pos_tol, math.radians(self.a.yaw_tol * 2),
                            "칸 %s 이동" % (cell,), watch_cell=cell, move_dir=mdir, stop_gap=stop_gap, brake=brake, pass_pt=pass_pt,
                            late_go=late_go, slot=slot, vmax=(self.a.slot_v if slot else (self.a.gap_v if self.tight_strafe(from_cell, cell, mdir, keep_heading) else None)),
                            yaw_ref_odom=self.yaw_ref_odom)
                if go_on or self.last_return_why != "late":
                    break
                # 서기로 했던 칸인데 가는 중 이어 가도 된다고 나옴 → 이 칸 가운데까지 그대로 달림.
                # 바로 돌아가면 몸이 아직 앞 칸이라 앞 칸에서 다시 계획하고 연속 칸 수도 두 번 셈 (실기 10-31 20:50 (1,3) 에 섬)
                go_on, late_go, stop_gap, brake = True, None, None, None
                self.cur_stop_gap = None
                ux, uy = P.DXY[mdir]
                pass_pt = (Xt, Yt, ux, uy)
                Xt, Yt = Xt + ux * self.c, Yt + uy * self.c
                self.log("  가는 중 다음 칸이 빈 칸으로 확인됨 → 서지 않고 이어 감 (연속 %d 칸째)" % (self.straight_run + 1))
        except GuardStop as e:
            self.stop()
            self.straight_run = 0
            self.guard_streak += 1
            # 칸 중간에 서 있으면 다음 이동이 대각선이 되므로, 지나온(안전한) 출발 칸 중심으로 되돌아간다
            bx_, by_ = (gx_, gy_) if from_grab else self.center(from_cell)   # 큐브 앞에서 출발했으면 큐브 앞으로
            self.log("  [충돌 방지 정지] %s → 출발 칸 %s 중심으로 되돌아감" % (e, from_cell))
            try:
                self._drive(bx_, by_, math.radians(HEADING_YAW[keep_heading]), self.a.pos_tol, math.radians(self.a.yaw_tol * 2),
                            "출발 칸 복귀", guard=False)
                if not from_grab and mdir is not None and mdir not in (keep_heading, (keep_heading + 2) % 4):
                    # 옆걸음이 막힌 건 대개 앞뒤로 2~3 cm 어긋나서임 (복도 여유 2.4 cm). 복귀 허용(pos_tol)으로는 그대로라 같은 곳에서 또 막힘 → 정지한 뒤에만 정확히 맞춤
                    self.stop()
                    self._drive(bx_, by_, math.radians(HEADING_YAW[keep_heading]), 0.008, math.radians(self.a.yaw_tol * 2),
                                "막힌 뒤 칸 가운데 맞춤", guard=False, min_v=self.a.min_v, timeout=self.a.settle_timeout, brake="long")
            except MoveTimeout:
                pass
            finally:
                self.stop()
            self.last_cell = from_cell
            self.at_grab_point = from_grab
            if cell != P.TARGET and not self.a.teleop:
                self.forced_occ[cell] = time.monotonic()      # 자율: 3 초간 막힌 칸으로 → planner 가 다른 길
            self.log("  [충돌 방지 정지] 칸 %s 이동 취소 (연속 %d 회)" % (cell, self.guard_streak))
            if self.guard_streak >= self.a.guard_abort:
                raise MissionAbort("충돌 방지 정지가 %d 회 연속 → 임무 중단" % self.guard_streak)
            time.sleep(0.5)
            return False
        self.guard_streak = 0
        if go_on:
            self.straight_run += 1
            self.go_dir = mdir
            self.last_cell = cell
            return True                                   # 같은 방향 계속: 서지 않고 다음 칸으로 (run() 이 바로 다음 이동)
        self.straight_run = 0
        ret_v, ret_why, ret_odom = self.last_return_v, self.last_return_why, self.last_return_odom
        if self.gaps_seen:
            self.log("  옆 이동 중 라이다로 잰 벽 간격 (목표 %.1f cm): %s" % (100 * self.want_long(), ", ".join(
                "%s %.1f~%.1f cm" % (k, 100 * v[0], 100 * v[1]) for k, v in sorted(self.gaps_seen.items()))))
        if self.face_skip:
            self.log("  옆 이동 중 마주 본 벽이 아니라 버린 거리: %s (점 폭 %.0f cm 넘어야 씀)" % (", ".join(
                "%s %d 번(폭 최소 %.1f cm)" % (k, v[0], 100 * v[1]) for k, v in sorted(self.face_skip.items())), 100 * self.a.face_span))
        if (self.a.quick_finish and self.grabbed and cell == P.HOME and not self.a.teleop
                and math.hypot(*(a_ - b_ for a_, b_ in zip(self.pose()[:2], self.center(cell)))) < 0.10):
            self.halt()
            self.log("  출발점 칸 안에 들어옴 → 맞춤 없이 끝")
            self.last_cell = cell
            return True
        if self.a.settle > 0:
            # 서 있을 때 격자 보정이 진행 방향 거리까지 맞추므로, 잠깐 기다렸다가 어긋난 만큼 마저 간다 (최대 2 번).
            # 자율 주행도 칸마다 선다: 오돔 진행 방향 오차가 반 칸(20 cm) 넘게 쌓이면 격자 보정이 옆 격자선에 맞춰 버리기 때문
            for k_settle in range(2):
                if k_settle == 0:
                    self.halt()                   # 칸 가운데 도착으로 끝난 경우에도 아직 속도가 있으면 능동 제동
                else:
                    self.stop()
                self.wait_stopped(self.a.settle)
                if k_settle == 0:
                    self.learn_brake(brake, ret_v, ret_why, ret_odom)
                    if cell != P.TARGET:
                        # 벽 측정은 설계 가운데 기준 → 뒤로 옮긴 목표가 아니라 설계 목표와 비교
                        self.wall_fix((self.a.back - self.a.front) / 2.0, Xd, Yd)
                X1, Y1, _ = self.pose()
                err = math.hypot(Xt - X1, Yt - Y1)
                if err <= self.a.settle_tol:
                    break
                if stop_gap is not None:
                    gdx, gdy = P.DXY[stop_gap[0]]
                    _, _, yw = self.pose()
                    gb = (math.cos(yw) * gdx + math.sin(yw) * gdy, -math.sin(yw) * gdx + math.cos(yw) * gdy)
                    gb = (math.copysign(1.0, gb[0]), 0.0) if abs(gb[0]) >= abs(gb[1]) else (0.0, math.copysign(1.0, gb[1]))
                    gp = self.wall_gap(*gb)
                    toward = (Xt - X1) * gdx + (Yt - Y1) * gdy > 0
                    if gp is not None and gp <= stop_gap[1] + 0.01 and toward:
                        self.log("  벽까지 %.1f cm 라 더 밀지 않음 (위치추정은 %.1f cm 더 가라고 함)" % (100 * gp, 100 * err))
                        break
                self.log("  서서 다시 보니 목표까지 %.1f cm → 마저 맞춤" % (100 * err))
                try:
                    self._drive(Xt, Yt, math.radians(HEADING_YAW[keep_heading]), self.a.settle_tol,
                                math.radians(self.a.yaw_tol * 2), "칸 %s 맞춤" % (cell,), watch_cell=cell, move_dir=mdir,
                                timeout=self.a.settle_timeout, min_v=self.a.settle_min_v, stop_gap=stop_gap, brake=brake,
                                yaw_ref_odom=self.yaw_ref_odom)
                except (GuardStop, MoveTimeout) as e:
                    self.stop()
                    self.log("  맞춤 그만 (%s) — 지금 자리로 둠" % e)   # 몇 cm 는 다음 이동에서 다시 맞춰짐
                    break
        if self.a.fine and cell != P.TARGET:
            self.fine_approach(Xt, Yt, math.radians(HEADING_YAW[keep_heading]), "도착")
        if self.a.settle > 0 and cell != P.TARGET:
            self.align_heading_wall("도착")
            self.keep_off_walls(keep_heading)
            self.wall_align((self.a.back - self.a.front) / 2.0, "도착")
            self.check_walls((self.a.back - self.a.front) / 2.0, cell, keep_heading)
        self.last_cell = cell
        if cell == P.TARGET:
            self.stop()
            self.at_grab_point = True
        return True

    def imu_feed(self, wz, now=None):
        now = time.monotonic() if now is None else now
        if self.imu is None or now - self.imu[0] > 0.2:
            self.imu = (now, wz, 0.0)
        else:
            t0, w0, yaw0 = self.imu
            self.imu = (now, wz, yaw0 + 0.5 * (w0 + wz) * (now - t0))

    def grab_cube(self):
        self.grabbed = True
        if self.grid is not None:
            # 돌아가는 길에 (5,5) 를 지나면 '큐브 칸 도착'으로 처리돼 스테이션이 다시 잡기 시작함 (시작하자마자 그리퍼를 엶)
            # 온 길이 있으니 (5,5) 를 빼도 길은 있음. 충돌 방지로 잠깐 막히면 풀릴 때까지 기다렸다 같은 길로 감
            self.grid.block(P.TARGET, math.inf)
        if self.a.cube_out > 0 and not getattr(self, "_front_with_cube", False):
            self._front_with_cube = True
            self.a.front += self.a.cube_out        # 물린 큐브가 그리퍼 앞으로 나온 만큼 몸 앞 길이에 더함 (사용자 실측 0.5~0.7 cm)
        if self.a.grab_mode == "station":
            self.log("  잡기 스테이션이 쥐기까지 끝냄 → 돌아가기")
            return True
        self.log("  [집게 없음] 큐브 앞 정지점 도착을 집기 성공으로 처리, %.1f 초 정지" % self.a.grab_wait)
        self.stop()
        time.sleep(self.a.grab_wait)
        return True

    def find_cube_cell(self):
        return P.TARGET

    # ── 키 입력 (스페이스 = 비상정지, g = 계속, x = 종료, 키보드 모드면 w/s/a/d/q/e) ──
    def key_loop(self):
        while not self.quit_req:
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
            except Exception:
                return
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch == " ":
                self.estop = True
                self.send_zero()
                print("\n*** 비상정지 (g = 계속, x = 종료) ***", flush=True)
            elif ch in ("g", "\n", "\r"):
                if self.estop and ch == "g":
                    self.estop = False
                    print("\n*** 비상정지 해제 ***", flush=True)
                elif self.wait_go and not self.estop:
                    self.go_event.set()
            elif ch == "r":                                     # 출발 대기 중에만: 지금 자리로 원점 다시 잡고 칸 판정 새로 모음 (버튼 출발)
                self.recheck_req = self.wait_go
            elif ch == "x":
                self.quit_req = True
                self.send_zero()
                print("\n*** 종료 요청 ***", flush=True)
            elif ch == "o":                                     # 그리퍼 열기 (큐브 놓기) — 바퀴는 안 움직임, 비상정지 중에도 됨
                print("\n*** 그리퍼 열기 ***", flush=True)
                threading.Thread(target=self.open_gripper_key, daemon=True).start()
            elif ch in "wsadqe":
                self.keys.put(ch)

    def teleop(self, grid):
        """키 한 번 = 칸 한 칸(앞·뒤·옆) 또는 90° 회전. 이동은 SLAM 기준 다음 칸 중심까지(오차가 쌓이지 않음).
        동작마다 오돔(바퀴+자이로) 이동량과 SLAM 이동량을 비교해 미끄러짐을 기록한다."""
        names = {"w": "앞", "s": "뒤", "a": "왼쪽 옆", "d": "오른쪽 옆", "q": "왼쪽 90° 회전", "e": "오른쪽 90° 회전"}
        self.log("키보드 칸 이동: w/s 앞뒤 1칸 · a/d 옆 1칸 · q/e 90° 회전 · 스페이스 비상정지 · g 계속 · x 종료")
        while not self.quit_req:
            self.hold_if_estop()
            try:
                k = self.keys.get(timeout=0.5)
            except queue.Empty:
                # 키를 안 눌러도 칸 판정을 지도에 계속 반영 (RViz·화면에 바로 보이게)
                try:
                    cell = self.current_cell()
                    for c_, o_ in self.observed_cells().items():
                        grid.observe(c_, o_, self.now())
                    grid.observe(cell, False, self.now())
                except MissionAbort:
                    raise
                except Exception:
                    pass
                continue
            if self.estop:
                continue
            cell, h = self.current_cell(), self.current_heading()
            for c_, o_ in self.observed_cells().items():
                grid.observe(c_, o_, self.now())
            grid.observe(cell, False, self.now())
            snap0 = self.calib_snapshot() if self.a.calib else None
            X0, Y0, yaw0 = self.pose()
            try:
                od0 = self.lookup_odom()
            except Exception:
                od0 = None
            if k in "wsad":
                d = {"w": h, "s": (h + 2) % 4, "a": (h + 3) % 4, "d": (h + 1) % 4}[k]
                nxt = (cell[0] + P.DXY[d][0], cell[1] + P.DXY[d][1])
                if not P.inside(nxt):
                    self.log("[%s] %s → 경기장 밖이라 무시" % (names[k], cell))
                    continue
                if grid.state(nxt) == P.OCC:
                    self.log("[%s] %s → %s 는 장애물로 판정된 칸이라 무시" % (names[k], cell, nxt))
                    continue
                self.log("[%s] %s → %s (몸 %s 유지, %s 쪽 이동)" % (names[k], cell, nxt, P.HEADING_NAME[h], P.HEADING_NAME[d]))
                try:
                    ok = self.translate_to(*P.cell_center_mm(nxt), keep_heading=h)
                except MoveTimeout as e:
                    self.stop()
                    self.log("  [%s] %s — 이 동작만 취소, 다음 키 받음" % (names[k], e))
                    ok = False
                self.stop()
                X1, Y1, yaw1 = self.pose()
                dX, dY = X1 - X0, Y1 - Y0
                along = dX * P.DXY[d][0] + dY * P.DXY[d][1]
                lat = -dX * P.DXY[d][1] + dY * P.DXY[d][0]
                msg = "  결과 %s: 위치추정 이동 %.1f cm (옆으로 샘 %+.1f cm)" % ("도착" if ok else "취소", 100 * along, 100 * lat)
                if od0 is not None:
                    od1 = self.lookup_odom()
                    d_od = math.hypot(od1[0] - od0[0], od1[1] - od0[1])
                    d_sl = math.hypot(dX, dY)
                    slip = 100.0 * (d_od - d_sl) / d_sl if d_sl > 0.05 else float("nan")
                    msg += " · 오돔 이동 %.1f cm · 오돔이 위치추정보다 더 잰 비율 %+.0f%%" % (100 * d_od, slip)
                    self.slip_log.append({"key": k, "dir": P.HEADING_NAME[d], "slam": round(d_sl, 4), "odom": round(d_od, 4),
                                          "lat": round(lat, 4), "ok": ok})
                off = (self.a.back - self.a.front) / 2.0
                cx, cy = self.center(self.last_cell) if ok else (X1, Y1)
                if ok:
                    cx, cy = cx + off * P.DXY[h][0], cy + off * P.DXY[h][1]      # 몸통 가운데 기준 목표
                msg += " · 목표까지 %.1f cm" % (100 * math.hypot(X1 - cx, Y1 - cy))
                self.log(msg)
                if snap0 is not None and ok:
                    body = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1)}[k]
                    self.calib_compare("move", snap0, self.calib_snapshot(), step_dir_body=body)
            else:
                nh = (h + 3) % 4 if k == "q" else (h + 1) % 4
                self.log("[%s] 칸 %s, 몸 %s → %s" % (names[k], cell, P.HEADING_NAME[h], P.HEADING_NAME[nh]))
                try:
                    self.turn_to(nh)
                except MoveTimeout as e:
                    self.stop()
                    self.log("  [%s] %s — 이 동작만 취소, 다음 키 받음" % (names[k], e))
                X1, Y1, yaw1 = self.pose()
                msg = "  결과: 위치추정 회전 %+.1f°" % math.degrees(wrap(yaw1 - yaw0))
                if od0 is not None:
                    od1 = self.lookup_odom()
                    msg += " · 오돔 회전 %+.1f°" % math.degrees(wrap(od1[2] - od0[2]))
                msg += " · 위치 이동 %.1f cm" % (100 * math.hypot(X1 - X0, Y1 - Y0))
                self.log(msg)
                if snap0 is not None:
                    self.calib_compare("turn", snap0, self.calib_snapshot(), turn_sign=(1 if k == "q" else -1))
            cell = self.current_cell()
            for c_, o_ in self.observed_cells().items():
                grid.observe(c_, o_, self.now())
            print("\n" + self.ascii_map(cell), flush=True)
            self.publish_markers()
            self.dump(force=True)

    # ── 표시 ──
    def ascii_map(self, robot_cell=None):
        g = self.grid
        rows = []
        for y in range(N, 0, -1):
            s = "%d " % y
            for x in range(1, N + 1):
                c = (x, y)
                if c == robot_cell:
                    ch = "R"
                elif c == P.TARGET:
                    ch = "T"
                elif c in self.path:
                    ch = "*"
                else:
                    st = g.state(c) if g else P.UNKNOWN
                    ch = "#" if st == P.OCC else ("." if st == P.FREE else "?")
                s += ch + " "
            rows.append(s)
        rows.append("  " + " ".join(str(x) for x in range(1, N + 1)))
        return "\n".join(rows)

    def _markers_timer(self):
        try:
            if self.a.teleop and self.grid is not None and self.start is not None:
                # 키보드 모드: 움직이는 중에도 칸 판정을 지도에 바로 반영 (원래는 도착 뒤에야 반영돼 늦게 보였음)
                for c_, o_ in self.observed_cells().items():
                    self.grid.observe(c_, o_, self.now())
            self.publish_markers()
        except Exception:
            pass

    def publish_markers(self):
        """RViz 표시 — 규정 그림 1 과 같은 색: 검은 바탕 · 연회색 칸 · 검은 장애물 · 초록 출발점 · 빨간 (5,5) (노란 2임무 경계선은 1임무에 필요 없어 안 그림).
        아직 모르는 칸은 어두운 회색, 계획 경로는 파란 선."""
        if self.start is None:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        yaw_a = self.start[2] - self.psi                 # 경기장 축의 map 기준 방향
        ox, oy = self.arena_to_map(0.0, 0.0)             # 경기장 좌하단 모서리의 map 좌표

        def base(ns, mid, typ):
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", now
            m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
            m.pose.position.x, m.pose.position.y = ox, oy
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw_a / 2), math.cos(yaw_a / 2)
            return m

        L = N * self.c
        floor = base("floor", 0, Marker.CUBE)            # 검은 바탕(바깥벽 포함)
        floor.pose.position.x, floor.pose.position.y = self.arena_to_map(L / 2, L / 2)
        floor.scale.x = floor.scale.y = L + 0.08
        floor.scale.z = 0.005
        floor.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        ma.markers.append(floor)

        cells = base("cells", 1, Marker.CUBE_LIST)       # 칸 사이 1 cm 틈 = 검은 칸 선
        cells.scale.x = cells.scale.y = self.c - 0.012
        cells.scale.z = 0.01
        for x in range(1, N + 1):
            for y in range(1, N + 1):
                st = self.grid.state((x, y)) if self.grid else P.UNKNOWN
                X, Y = self.center((x, y))
                if (x, y) == P.TARGET:
                    col = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)          # 빨강 (요구조자)
                elif (x, y) in (P.HOME, P.OPP_HOME):
                    col = ColorRGBA(r=0.31, g=0.66, b=0.20, a=1.0)       # 초록 (출발점)
                elif self.grid and self.grid.inferred((x, y)) == P.OCC:
                    col = ColorRGBA(r=0.20, g=0.15, b=0.55, a=1.0)       # 남색 (미러로 아는 장애물)
                elif self.grid and self.grid.inferred((x, y)) == P.FREE:
                    col = ColorRGBA(r=0.72, g=0.80, b=0.98, a=1.0)       # 연파랑 (미러로 아는 빈 칸)
                elif st == P.OCC:
                    col = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)          # 검정 (장애물)
                elif st == P.FREE:
                    col = ColorRGBA(r=0.91, g=0.91, b=0.91, a=1.0)       # 연회색 (빈 칸)
                else:
                    col = ColorRGBA(r=0.30, g=0.30, b=0.30, a=1.0)       # 어두운 회색 (아직 모름)
                cells.points.append(Point(x=X, y=Y, z=0.008))
                cells.colors.append(col)
        ma.markers.append(cells)


        line = base("path", 3, Marker.LINE_STRIP)        # 계획 경로
        line.scale.x = 0.035
        line.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=1.0)
        for cell in self.path:
            X, Y = self.center(cell)
            line.points.append(Point(x=X, y=Y, z=0.03))
        if len(line.points) < 2:
            line.action = Marker.DELETE
        ma.markers.append(line)
        self.mk_pub.publish(ma)


def apply_phase_costs(mp, a):
    """단계별 이동 비용. planner 코드는 그대로 두고 비용 값만 바꾼다.
    중앙으로 갈 때: 카메라·그리퍼가 정면(라이다 쪽)이라 뒤·옆 걸음을 비싸게 → 정면이 가는 쪽을 봄.
    돌아올 때: 원래 메카넘 비용(뒤·옆 걸음 허용)."""
    if mp.phase == "TO_TARGET":
        mp.reverse_cost, mp.strafe_cost = a.reverse_cost_go, a.strafe_cost_go
    else:
        mp.reverse_cost, mp.strafe_cost = a.reverse_cost, a.strafe_cost


def main():
    ap = argparse.ArgumentParser(description="1임무 실주행 (전성환 planner + Cartographer + RPLIDAR)")
    ap.add_argument("--execute", action="store_true", help="자율 주행 (없으면 dry-run)")
    ap.add_argument("--collect", type=float, default=1.0, help="출발 전 라이다 칸 판정 수집 시간 [s]")
    ap.add_argument("--no-reset", action="store_true", help="시작할 때 위치추정 원점을 다시 잡지 않음")
    ap.add_argument("--teleop", action="store_true", help="키보드 칸 이동 모드 (w/s/a/d 1칸, q/e 90°)")
    ap.add_argument("--front", type=float, default=0.151, help="두 바퀴 축 가운데 → 차체 앞 끝(라이다) [m] (실측 10-21: 축간 19.2/2 + 5.5)")
    ap.add_argument("--back", type=float, default=0.186, help="두 바퀴 축 가운데 → 차체 뒤 끝(Jetson·USB 선 포함) [m] (실측 10-21: 19.2/2 + 9)")
    ap.add_argument("--peek", type=float, default=0.16,
                    help="정면 앞 칸이 모름이면 이만큼 물러서서 봄 [m] (0 = 안 함). 0.16 이면 앞 칸 면이 라이다에서 23 cm (최소 거리 약 15.6 cm)")
    ap.add_argument("--pcorr-fade", type=float, default=0.0,
                    help="서서 잰 벽 거리로 고친 위치 보정이 그 축으로 이 거리[m] 움직이면 0 이 됨 (0 = 벽으로 위치추정 고치기 끔)")
    ap.add_argument("--pcorr-max", type=float, default=0.10, help="벽으로 고치는 위치 보정 최대 [m]")
    ap.add_argument("--stuck-reset", type=float, default=3.0,
                    help="자율: 길 막힘이 이만큼 이어지면 장애물로 본 칸을 지우고 다시 봄 [s] (0 = 안 함)")
    ap.add_argument("--unknown-wait", type=float, default=1.0,
                    help="자율: 가려던 칸이 모름이면 서서 이만큼 봄 [s], 그래도 모르면 그쪽으로 돌아서 봄 (0 = 안 함, 모름 칸도 들어감)")
    ap.add_argument("--peek-wait", type=float, default=0.8, help="물러서서 보는 최대 시간 [s]")
    ap.add_argument("--fine", action="store_true", help="도착 후 마지막 몇 cm 를 짧게 밀고 멈춰 확인하며 맞춤")
    ap.add_argument("--fine-tol", type=float, default=0.01, help="톡톡 맞춤 목표 허용 [m]")
    ap.add_argument("--fine-pulse", type=float, default=0.10, help="한 번 미는 시간 [s]")
    ap.add_argument("--fine-pause", type=float, default=0.35, help="밀고 나서 서서 확인하는 시간 [s]")
    ap.add_argument("--fine-pulses", type=int, default=6, help="톡톡 최대 횟수")
    ap.add_argument("--no-turn-wall-align", dest="turn_wall_align", action="store_false",
                    help="회전 전 벽 거리 직접 맞춤 끔 (기본 켬: 회전 반지름 18.7 cm, 칸 반경 20 cm 라 1 cm 단위로 맞춰야 함)")
    ap.add_argument("--wall-align", dest="wall_align", action="store_true",
                    help="벽 거리 직접 맞추기 켬 (기본 끔: 필드 10-21 에서 짧은 보정 이동이 튀어 벽에 닿음)")
    ap.add_argument("--wall-tol", type=float, default=0.01, help="벽 맞춤 허용 [m]")
    ap.add_argument("--side", type=float, default=0.1225, help="두 바퀴 축 가운데 → 바퀴 바깥 [m] (실측 폭 24.5/2)")
    ap.add_argument("--guard-clear", type=float, default=0.03, help="차체 면과 장애물 최소 간격 [m], 이보다 가까워질 것 같으면 정지")
    ap.add_argument("--guard-slack", type=float, default=0.10, help="목표 칸 너머 벽을 장애물로 오인하지 않게 빼는 여유 [m]")
    ap.add_argument("--guard-min", type=float, default=0.015, help="차체 면에서 이보다 가까운 점은 무조건 정지 [m] (칸 가운데 섰을 때 벽 여유 3.15 cm 보다 작게)")
    ap.add_argument("--guard-pts", type=int, default=3, help="근접 정지에 필요한 라이다 점 수")
    ap.add_argument("--loc-wait", type=float, default=3.0, help="위치추정이 늦거나 조회 실패 시 바퀴 세우고 기다리는 최대 시간 [s] (넘으면 임무 중단)")
    ap.add_argument("--guard-abort", type=int, default=4, help="자율 모드에서 근접 정지가 연속 이만큼이면 임무 중단")
    ap.add_argument("--brake-tb", type=float, default=None,
                    help="처음 제동 시간 [s]: 멈춤 명령 뒤 굴러가는 거리 = 속도 × 이것 (10-21 필드: 0.28 m/s 에서 약 9 cm). 칸 이동마다 다시 잼")
    ap.add_argument("--brake-max", type=float, default=0.6, help="제동 시간 상한 [s]")
    ap.add_argument("--no-brake-learn", dest="brake_learn", action="store_false", help="제동 시간 다시 재지 않음")
    ap.add_argument("--repel-tol", type=float, default=0.012,
                    help="도착 후 벽이 설계 여유보다 이만큼 넘게 가까우면 반대로 떨어짐 [m] (0 = 안 함)")
    ap.add_argument("--front-target-shift", type=float, default=0.0,
                    help="앞으로 막다른 칸에 들어갈 때 목표를 이만큼 덜 감 [m] (그리퍼 달고 앞 여유 2.4 cm; 필드용 0.02)")
    ap.add_argument("--back-target-shift", type=float, default=0.004,
                    help="뒤로 가서 뒤가 벽인 칸에 설 때 목표를 이만큼 뒤로 [m] (10-21: 평균 0.4 cm 앞에서 섬)")
    ap.add_argument("--back-stop-margin", type=float, default=0.0,
                    help="뒤로 가서 뒤 벽에 설 때 벽 보고 멈춤을 이만큼 일찍 [m] (제동 예측 오차 여유, 가상 10-30)")
    ap.add_argument("--prealign", type=float, default=0.012,
                    help="옆걸음 시작 전 앞뒤가 칸 기준 자세에서 이만큼 넘게 어긋나면 먼저 앞뒤로만 맞춤 [m] (0 = 끔)")
    ap.add_argument("--back-stop-extra", type=float, default=0.0,
                    help="뒤로 가서 뒤 벽에 설 때 이만큼 더 붙어 섬 [m] (필드 10-21: 눈으로 보니 멀게 섬)")
    ap.add_argument("--wall-clamp", type=float, default=0.01,
                    help="벽이 설계 여유보다 이만큼 넘게 가까운 쪽으로는 속도를 내지 않음 [m] (0 = 끔)")
    ap.add_argument("--quick-finish", dest="quick_finish", action="store_true",
                    help="출발점 도착 때 맞춤 없이 바로 끝 (기본 끔: 10-21 마지막 도착은 서서 맞추기로)")
    ap.add_argument("--calib", action="store_true",
                    help="키보드 측정: 동작마다 앞뒤로 서서 벽을 재 실제 이동·회전을 오돔·위치추정과 비교해 기록")
    ap.add_argument("--no-turn-odom-hold", dest="turn_odom_hold", action="store_false",
                    help="회전 중 오돔으로 제자리 유지 끔 (기본 켬: 10-21 회전마다 1~4 cm 밀림)")
    ap.add_argument("--turn-hold-k", type=float, default=3.0, help="회전 중 제자리 유지 이득 [1/s]")
    ap.add_argument("--turn-hold-dead", type=float, default=0.004, help="회전 중 이 안의 밀림은 그대로 [m]")
    ap.add_argument("--turn-hold-vmin", type=float, default=0.03, help="회전 중 되밀기 최소 속도 [m/s]")
    ap.add_argument("--turn-hold-vmax", type=float, default=0.10, help="회전 중 되밀기 최대 속도 [m/s]")
    ap.add_argument("--heading-pulse-eff", type=float, default=0.6,
                    help="방향 맞춤 짧은 펄스에서 실제로 도는 비율 (짧으면 바퀴 반응이 늦음, 키보드로 보고 조정)")
    ap.add_argument("--wall-heading-tol", type=float, default=0.0,
                    help="설 때 벽 각도로 몸이 이만큼 넘게 틀어졌으면 제자리에서 짧게 돌려 맞춤 [deg] (0 = 끔)")
    ap.add_argument("--turn-brake-t", type=float, default=0.0,
                    help="회전 미리 멈춤: 도는 속도 × 이 시간 [s] 남았을 때 멈춤 (0 = 예전처럼 yaw-tol 안이면 멈춤)")
    ap.add_argument("--turn-end-tol", type=float, default=1.0, help="회전 끝 허용 [deg] (미리 멈춤과 함께)")
    ap.add_argument("--turn-end-fix", type=float, default=2.0, help="회전 끝나고 이만큼 넘게 남으면 한 번 더 짧게 맞춤 [deg]")
    ap.add_argument("--gyro-hold", dest="gyro_hold", action="store_true",
                    help="직진 중 자이로로 방향 유지 (기본 끔: 10-21 15:25 회전 6° 넘침이 기준에 남아 8 cm 쏠림)")
    ap.add_argument("--gyro-hold-tol", type=float, default=1.0, help="직진 중 기준에서 이만큼 넘게 돌면 바로잡음 [deg]")
    ap.add_argument("--gyro-hold-min-w", type=float, default=0.15, help="직진 중 방향 바로잡기 최소 회전 [rad/s]")
    ap.add_argument("--gyro-ref-max", type=float, default=2.0, help="직진 시작 때 위치추정 방향으로 고치는 최대 각 [deg]")
    ap.add_argument("--turn-wait", type=float, default=0.1, help="회전 전 서서 기다리는 시간 [s] (예전 0.4)")
    ap.add_argument("--turn-pulse-tol", type=float, default=0.012,
                    help="회전 전 벽 기준 맞춤: 이 안이면 안 밂 [m] (회전 여유 1.2~1.7 cm, 예전 0.008)")
    ap.add_argument("--turn-pulses", type=int, default=3, help="회전 전 벽 기준 맞춤 최대 횟수 (예전 8)")
    ap.add_argument("--turn-hold-tol", type=float, default=None, help="회전 중 칸 중심에서 이만큼 밀리면 바로 되돌림 [m] (기본 = turn-center-tol×1.5. 작게 하면 굴러가는 보정이 오히려 더 밂, 가상 10-21)")
    ap.add_argument("--turn-center-tol", type=float, default=0.02, help="회전 전 칸 중심 허용 [m]")
    ap.add_argument("--cell", type=float, default=0.40, help="칸 크기 [m] (대회·연습 0.40)")
    ap.add_argument("--recheck-dist", type=float, default=0.06, help="출발 키 때 칸 중심에서 이만큼 넘게 벗어나 있으면 원점을 다시 잡음 [m] (놓는 오차 2 cm 넘게, 칸 혼동 20 cm 보다 작게)")
    ap.add_argument("--recheck-yaw", type=float, default=3.0, help="출발 키 때 출발 방향에서 이만큼 넘게 틀어져 있으면 원점을 다시 잡음 [°] (격자 보정 탐색 ±4° 안)")
    ap.add_argument("--recheck-collect", type=float, default=0.5, help="다시 잡은 뒤 칸 판정을 새로 모으는 시간 [s]")
    ap.add_argument("--start-side", choices=["auto", "right", "left"], default="auto",
                    help="출발 칸 바깥벽 쪽: right = 오른쪽·뒤 ((1,9) 남향 / (9,1) 북향), left = 왼쪽·뒤 ((1,9) 동향 / (9,1) 서향), auto = 라이다로 판단")
    ap.add_argument("--symmetric", action="store_true", help="미러 맵핑: 본 칸의 (5,5) 점대칭 칸도 같다고 봄. 직접 본 값이 항상 우선 (MirrorGrid)")
    ap.add_argument("--wheel-vmax", type=float, default=0.224,
                    help="회전을 섞을 때 바퀴 테 속도 합 |vx|+|vy|+L|w| 상한 [m/s] (펌웨어 (MAX_PWM 200 − I_LIMIT 60)/KFF 25 × 반지름 0.04), 0 = 끔")
    ap.add_argument("--wheel-k-up", type=float, default=1.5, help="회전 여유 때문에 줄인 이동 속도를 되돌리는 빠르기 [비율/초] (반 → 전부에 약 0.3 초)")
    ap.add_argument("--wheel-room-w", type=float, default=0.1, help="이보다 센 회전을 섞을 때만 바퀴 여유를 위해 이동을 줄임 [rad/s]")
    ap.add_argument("--wheel-l", type=float, default=0.1995, help="메카넘 회전 팔 lx+ly [m] (controllers.yaml sum_of_robot_center_projection)")
    ap.add_argument("--max-v", type=float, default=0.32, help="최대 명령 속도 [m/s] (10-21: 0.25 → 0.32, 능동 제동과 함께)")
    ap.add_argument("--active-brake", type=float, default=0.08,
                    help="멈출 때 가던 반대로 이 속도[m/s]를 짧게 명령해 굴러가는 거리를 줄임 (0 = 끔, 예: 0.15)")
    ap.add_argument("--active-brake-max", type=float, default=0.35, help="능동 제동 최대 시간 [s]")
    ap.add_argument("--active-brake-t", type=float, default=0.25, help="0.3 m/s 에서 능동 제동 시간 상한 [s] (속도에 비례)")
    ap.add_argument("--wall-follow", dest="wall_follow", action="store_true",
                    help="앞뒤로 갈 때 좌우를 옆벽 거리로 맞추기 (기본 끔: 필드 10-21 12:30 직선에서 흔들림)")
    ap.add_argument("--no-wall-follow-lat", dest="wall_follow_lat", action="store_false",
                    help="옆으로 갈 때 앞뒤를 앞·뒤 벽 거리로 맞추기 끔 (기본 켬: 좁은 칸 (6,3) 모서리 걸림 방지)")
    ap.add_argument("--perp-k", type=float, default=3.0, help="이동 중 수직 오차를 고치는 이득 [1/s] (0 = 예전처럼 목표점 직선)")
    ap.add_argument("--slot-v", type=float, default=0.0,
                    help="앞뒤가 막힌 좁은 칸을 옆으로 지날 때 최대 속도 [m/s] (0 = 좁은 칸 처리 끔)")
    ap.add_argument("--slot-back-bias", type=float, default=0.005,
                    help="좁은 칸에선 보이는 뒤 벽 쪽으로 이만큼 붙음 [m] (앞은 라이다 사각지대)")
    ap.add_argument("--lat-back-bias", type=float, default=0.004,
                    help="옆으로 갈 때 뒤 벽만 보이면 목표를 뒤 벽 쪽으로 이만큼 [m] (앞 사각지대 쪽 여유 확보)")
    ap.add_argument("--slot-pos-tol", type=float, default=0.008, help="좁은 칸 들어가기 전 앞뒤 허용 [m]")
    ap.add_argument("--slot-yaw-tol", type=float, default=1.2, help="좁은 칸 들어가기 전 방향 허용 [deg]")
    ap.add_argument("--slot-min-w", type=float, default=0.15, help="좁은 칸 지날 때 방향 보정 최소 회전 [rad/s]")
    ap.add_argument("--perp-dead", type=float, default=0.006, help="이 안의 수직 오차는 안 고침 [m]")
    ap.add_argument("--perp-vmin", type=float, default=0.03, help="수직 보정 최소 속도 [m/s]")
    ap.add_argument("--perp-vmax", type=float, default=0.08, help="이동 중 수직 보정 최대 속도 [m/s]")
    ap.add_argument("--face-span", type=float, default=0.06,
                    help="앞뒤 벽으로 믿을 최소 폭 [m]: 그 거리 점들이 몸 폭 방향으로 이만큼 퍼져 있어야 마주 본 벽 (0 = 검사 안 함)")
    ap.add_argument("--grab-exit-align", type=int, default=0,
                    help="큐브 잡은 자리에서 옆으로 나갈 때 먼저 앞뒤로 다음 칸 가운데 줄에 맞춘 뒤 옆으로 곧게 (0 = 예전처럼 대각선)")
    ap.add_argument("--wall-prealign", type=float, default=0.0,
                    help="옆걸음 전: 가는 칸의 앞(뒤)이 벽인데 로봇이 그쪽으로 이만큼 넘게 치우쳐 있으면 먼저 반대로 맞춤 [m] (0 = 끔)")
    ap.add_argument("--face-blind", type=float, default=0.15,
                    help="라이다 최소 거리 [m]: 차체 면이 라이다에서 이보다 가까운 쪽(앞)만 마주 본 벽 검사 (가까운 진짜 면이 사각에 숨을 수 있음)")
    ap.add_argument("--perp-damp", type=int, default=1,
                    help="수직 보정에서 지금 수직 속도 × 제동 시간(학습값)만큼 미리 뺌 → 지나치기 전에 반대 명령으로 섬 (0 = 끔)")
    ap.add_argument("--lat-gain", type=float, default=1.0,
                    help="옆 칸 이동 때 옆 속도 명령 배율 (메카넘 옆 이동이 느려서, 예: 1.5). 앞뒤·회전·맞춤 이동엔 안 씀")
    ap.add_argument("--min-v", type=float, default=0.10, help="최소 명령 속도 [m/s] (바퀴가 서는 저속 방지)")
    ap.add_argument("--max-w", type=float, default=1.2, help="최대 회전 명령 [rad/s] (10-21: 회전 중 제자리 유지와 함께 1.0 → 1.2, 1.3 은 살짝 밀림)")
    ap.add_argument("--min-w", type=float, default=0.45, help="최소 회전 명령 [rad/s]")
    ap.add_argument("--kp", type=float, default=1.5)
    ap.add_argument("--kw", type=float, default=2.0)
    ap.add_argument("--pos-tol", type=float, default=0.04, help="칸 중심 도착 허용 [m]")
    ap.add_argument("--yaw-tol", type=float, default=4.0, help="방향 허용 [deg]")
    ap.add_argument("--strafe-yaw-tol", type=float, default=1.0,
                    help="옆걸음 중 몸 방향을 이만큼 넘게 틀어지면 회전을 섞어 계속 잡음 [deg] (0 = 끔, 예전엔 yaw-tol 4° 까지 안 고침)")
    ap.add_argument("--strafe-yaw-w", type=float, default=0.3, help="옆걸음 중 몸 방향 잡는 회전 [rad/s]")
    ap.add_argument("--yaw-pid", type=int, default=2,
                    help="옆걸음 중 몸 방향: 2 = IMU 로 보고 1° 넘을 때만 바로잡음(기본), 1 = IMU PID(흔들림 큼), 0 = 위치추정 방향으로 몸 잡기")
    ap.add_argument("--yaw-pid-kp", type=float, default=3.0, help="[rad/s per rad]")
    ap.add_argument("--yaw-lag", type=float, default=0.2, help="회전 명령 → 실제 회전 늦음 [s] (10-31 실측) — yaw-pid 2 에서 그만큼 뒤를 예측")
    ap.add_argument("--yaw-pid-ki", type=float, default=3.0, help="[rad/s per rad·s]")
    ap.add_argument("--yaw-pid-kd", type=float, default=0.3, help="[rad/s per rad/s] 회전 속도 되먹임")
    ap.add_argument("--gap-v", type=float, default=0.0, help="1칸 틈(지나갈 칸 앞·뒤 다 막힘) 옆걸음 최대 속도 [m/s] (0 = 끔)")
    ap.add_argument("--cube-out", type=float, default=0.007, help="물린 큐브가 그리퍼 앞으로 나온 길이 [m] (잡은 동안 몸 앞 길이에 더함)")
    ap.add_argument("--yaw-pid-imax", type=float, default=0.1, help="적분 상한 [rad·s] (ki 와 곱해 최대 0.3 rad/s)")
    ap.add_argument("--move-timeout", type=float, default=25.0, help="한 동작 최대 시간 [s]")
    ap.add_argument("--grab-standoff", type=float, default=0.20, help="큐브 중심에서 로봇 중심까지 정지 거리 [m] (앞면 ≈ 3 cm 앞)")
    ap.add_argument("--grab-wait", type=float, default=2.0)
    # 10-29 시뮬레이션(회전자리_20261028 합친 판) 옮김 — 기본은 꺼짐(예전 동작 그대로)
    ap.add_argument("--turn-plan", choices=["orig", "table"], default="orig",
                    help="table = 칸 둘레 모양 회전 표로 '돌 수 있는 칸'에서만 회전하게 길찾기 (planner.py 원본은 그대로, 이 프로세스에서만 바꿔 끼움)")
    ap.add_argument("--turn-spot", action="store_true",
                    help="회전 명령이면 넓은 자리(여유 5 cm, 두 칸 안 → 1 cm, 한 칸 안 → 표 자리)로 비켜서 돎. 카메라 받침 때문에 회전 반경 22.6 cm")
    ap.add_argument("--grab-mode", choices=["none", "station"], default="none",
                    help="station = 큐브 옆 칸 정면에서 멈추고 잡기 스테이션(cube_station.py)에 재기·맞추기·들어가기·쥐기를 맡김")
    ap.add_argument("--station-url", default="http://127.0.0.1:8090")
    ap.add_argument("--station-timeout", type=float, default=60.0)
    ap.add_argument("--release-at-home", action="store_true", help="잡기 스테이션 쓸 때: 출발점까지 완주하면 그리퍼를 열어 큐브를 놓음")
    ap.add_argument("--obs-range", type=float, default=1.6, help="칸 판정에 쓰는 라이다 거리 [m]")
    ap.add_argument("--face-tol", type=float, default=0.06, help="격자선에서 이 안이면 장애물 면 위의 점 [m]")
    ap.add_argument("--far-range", type=float, default=2.5, help="이 거리까지 벽을 봄 (obs-range 너머는 정면에 가깝게 맞은 점만) [m]")
    ap.add_argument("--far-incidence", type=float, default=30.0, help="먼 벽은 면에 수직에서 이 각도 안으로 맞은 점만 [deg]")
    ap.add_argument("--straight", dest="straight", action="store_true", default=True,
                    help="(실험판 기본 켬) 다음 칸도 같은 방향이고 빈 칸으로 확인됐으면 서지 않고 이어 감 (꺾는 곳·큐브 앞·끝·모르는 칸 앞에서 섬)")
    ap.add_argument("--no-straight", dest="straight", action="store_false", help="칸마다 섬")
    ap.add_argument("--no-wait-key", dest="wait_key", action="store_false",
                    help="자율 주행을 출발 키 없이 바로 시작 (기본: 엔터 또는 g 를 눌러야 출발)")
    ap.add_argument("--replan-moving", type=int, default=1, help="서러 가는 중 라이다 판정이 바뀌면 다시 계획해, 같은 방향으로 이어지면 서지 않음 (0 = 끔)")
    ap.add_argument("--turn-known-r", type=int, default=-1, help="큐브로 갈 때 큐브에서 이 칸 수 안에서는 둘레가 확인된 곳에서만 돌 계획 (1 = 3×3, 2 = 5×5, -1 = 끔)")
    ap.add_argument("--range-bias", type=float, default=0.0, help="라이다 거리 보정 [m] (벽 간격 계산에만, 실측 로봇 0.005)")
    ap.add_argument("--laser-dx", type=float, default=0.0, help="라이다 앞뒤 위치 보정 [m] (벽 간격 계산에만, 실측 로봇 -0.014)")
    ap.add_argument("--straight-max", type=int, default=6, help="서지 않고 이어 가는 최대 칸 수 (그다음 한 번 서서 위치 맞춤)")
    ap.add_argument("--decay", type=float, default=0.97, help="칸 증거가 판정 스캔마다 줄어드는 비율 (0.97 ≈ 6 초 반감)")
    ap.add_argument("--off-range", type=float, default=1.0, help="면 위가 아닌 점(상대 로봇 등)은 이 거리 안만 [m]")
    ap.add_argument("--settle-min-v", type=float, default=0.16, help="도착 후 맞춤 최소 명령 속도 [m/s] (메카넘 옆 몇 cm 는 느리면 안 밀림)")
    ap.add_argument("--settle-tol", type=float, default=0.03, help="도착 후 맞춤 허용 [m]")
    ap.add_argument("--settle-timeout", type=float, default=4.0, help="도착 후 맞춤 최대 시간 [s] (넘으면 그 자리로 둠)")
    ap.add_argument("--settle", type=float, default=0.6, help="칸 도착 후 서서 위치를 다시 맞추는 시간 [s] (0 = 안 섬)")
    ap.add_argument("--off-margin", type=float, default=0.10, help="면이 아닌 점은 격자선에서 이보다 안쪽일 때만 장애물 [m]")
    ap.add_argument("--graze-deg", type=float, default=15.0, help="면과 이 각도보다 나란히 스친 점은 장애물 증거에서 뺌 [deg]")
    ap.add_argument("--corner-yaw", type=float, default=2.5,
                    help="모서리 근처 점 버리는 범위 = corner-tol + 거리 × tan(이 각도) [deg] (몸 방향 오차만큼 먼 점이 밀림)")
    ap.add_argument("--corner-tol", type=float, default=0.04, help="다른 방향 격자선이 이 안이면 모서리라 버림 [m]")
    ap.add_argument("--free-gap", type=float, default=0.08, help="맞은 점 앞 이만큼은 빈 칸 증거에서 뺌 [m]")
    ap.add_argument("--min-pts", type=int, default=4, help="한 스캔에서 장애물 칸으로 칠 최소 점 수")
    ap.add_argument("--occ-need", type=int, default=3, help="장애물 확정에 필요한 스캔 수")
    ap.add_argument("--free-need", type=int, default=4, help="빈 칸 확정에 필요한 스캔 수")
    ap.add_argument("--rot-gate", type=float, default=0.25, help="이보다 빨리 돌 때 스캔은 칸 판정에서 뺌 [rad/s]")
    ap.add_argument("--scan-every", type=int, default=2, help="스캔 몇 개마다 한 번 판정 (11 Hz / 2 ≈ 6 Hz)")
    ap.add_argument("--strafe-cost", type=float, default=1.25, help="돌아올 때 옆 1칸 비용 (앞 1칸 = 1)")
    ap.add_argument("--reverse-cost", type=float, default=1.0, help="돌아올 때 뒤 1칸 비용")
    ap.add_argument("--strafe-cost-go", type=float, default=1.25, help="중앙으로 갈 때 옆 1칸 비용. 기본 = 원래 planner 값(회전 최소). 3 쯤으로 키우면 카메라가 가는 쪽을 봄")
    ap.add_argument("--reverse-cost-go", type=float, default=1.0, help="중앙으로 갈 때 뒤 1칸 비용. 기본 = 원래 planner 값")
    ap.add_argument("--turn-cost", type=float, default=1.0)
    ap.add_argument("--log", default="", help="기록 파일")
    a = ap.parse_args()
    if a.teleop:
        a.execute = True

    # Ctrl+C 를 rclpy 가 먼저 가로채면 정지 명령을 보내기 전에 ROS 가 내려간다 → 우리가 받아서 정지부터 보냄
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    robot = Mission1Robot(a)
    ex = SingleThreadedExecutor()
    ex.add_node(robot)
    spin_th = threading.Thread(target=ex.spin, daemon=True)
    spin_th.start()

    mp = P.MissionPlanner(grid=MirrorGrid(a.symmetric), strafe_cost=a.strafe_cost, turn_cost=a.turn_cost)
    robot.grid = mp.grid
    robot.mp = mp
    if a.turn_plan == "table":
        import turnspot as TS
        robot.tp = TS.TablePlanner(P)
        robot.tp.turn_known_r = a.turn_known_r
        P.plan_path_mecanum = robot.tp.plan_or_orig
        robot.log("길찾기: 회전 표 (돌 수 있는 칸에서만 회전)")
    orig_update = mp.update

    def update_and_show(*args, **kw):          # 계획 결과를 표시용으로 가로챈다 (planner 동작은 그대로)
        apply_phase_costs(mp, a)
        cmd = orig_update(*args, **kw)
        robot.pending_cmd = cmd                 # 회전 자리 고를 때 다음 목표를 알기 위해
        robot.path = list(cmd.path or [])
        robot.mirror_report()
        robot.publish_markers()
        robot.dump()
        return cmd
    mp.update = update_and_show

    old_tty = None
    if sys.stdin.isatty():
        old_tty = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())          # 키 한 개씩 바로 읽음 (Ctrl+C 는 그대로 동작)
        threading.Thread(target=robot.key_loop, daemon=True).start()
    try:
        robot.log("planner %s, 모드 %s, 칸 %.2f m, 점대칭 %s" % (
            P.__version__, "키보드 칸 이동" if a.teleop else ("자율 주행" if a.execute else "dry-run"),
            a.cell, "사용" if a.symmetric else "안 씀"))
        robot.log("비상정지: 스페이스 (g = 계속, x = 종료, o = 그리퍼 열기) · 다른 터미널: ~/mecanum_slam_ws/mission1/estop.sh")
        robot.log("실행 옵션: %s · 제동 시간 시작값 앞뒤 %.2f 옆 %.2f 초 (%s)" % (
            " ".join(sys.argv[1:]), robot.brake_tb["long"], robot.brake_tb["lat"], "능동 제동" if a.active_brake > 0 else "능동 제동 없음"))
        # 다른 임무·키보드 조종이 이미 돌고 있으면 원점을 건드리기 전에 그만둔다 (dry-run 도 원점을 다시 잡으므로 똑같이)
        time.sleep(1.0)
        # 잡기 스테이션(cube_station)은 /cmd_vel 발행자지만 임무가 맡길 때만 움직임(그동안 임무는 바퀴 명령 안 냄) → 세지 않음
        others = sum(1 for i in robot.get_publishers_info_by_topic("/cmd_vel")
                     if i.node_name not in (robot.get_name(), "cube_station"))
        if others > 0:
            raise MissionAbort("/cmd_vel 에 다른 발행자 %d 개 — 다른 임무/키보드 조종이 켜져 있음 (먼저 끌 것)" % others)
        if not a.no_reset:
            if robot.detect_start_side(a.start_side) is None:
                if not (a.execute and a.wait_key and old_tty is not None):
                    raise MissionAbort("출발 방향을 라이다로 못 정함 (옆 바깥벽이 분명하지 않음) — --start-side right(바깥벽 오른쪽)/left(왼쪽) 로 지정")
                robot.log("출발 방향을 아직 못 정함 (출발 칸에 놓기 전일 수 있음) → 출발 직전에 다시 봄")
            robot.tell_start_yaw()
            robot.reset_origin()
        robot.wait_start()
        robot.log("라이다 칸 판정 %.1f 초 수집..." % a.collect)
        time.sleep(a.collect)
        if a.teleop:
            robot.teleop(mp.grid)
            raise MissionAbort("x 키로 종료")
        if a.execute and a.wait_key and old_tty is not None:
            # 출발 키를 누를 때까지 그 자리에서 정지 명령만 내며 대기 (그동안에도 라이다 칸 판정은 쌓인다)
            print("\n" + robot.ascii_map(robot.last_cell))
            robot.log("준비 끝 — 엔터 또는 g 를 누르면 출발 (x = 종료, 스페이스 = 비상정지, o = 그리퍼 열기)")
            robot.wait_go = True
            while not robot.go_event.is_set():
                if robot.quit_req:
                    raise MissionAbort("출발 전 x 키로 종료")
                if robot.recheck_req:
                    robot.recheck_req = False
                    robot.recheck_origin(force=True)
                robot.send_zero()
                time.sleep(0.05)
            robot.wait_go = False
            robot.log("출발")
            robot.recheck_origin()
        if not a.execute:
            while not robot.quit_req:
                cell, h = robot.current_cell(), robot.current_heading()
                cmd = mp.update(cell, h, robot.observed_cells(), t=robot.now())
                X, Y, yaw = robot.pose()
                print("\n" + robot.ascii_map(cell))
                robot.log("[dry-run] 위치 (%.2f, %.2f) m, 칸 %s, 몸 %s(%.0f°) → %s %s %s" % (
                    X, Y, cell, P.HEADING_NAME[h], math.degrees(yaw), cmd.action, cmd.phase,
                    "다음 %s 몸 %s 이동 %s" % (cmd.next_cell, P.HEADING_NAME[cmd.next_heading],
                                            P.HEADING_NAME[cmd.move_dir]) if cmd.action == "MOVE" else ""))
                time.sleep(1.0)
            raise MissionAbort("x 키로 종료")
        ok = run(robot, mp, log=robot.log)
        robot.stop()
        robot.mirror_summary()
        robot.result = "완주" if ok else "미완료 (단계 수 초과)"
        robot.log("임무 %s" % robot.result)
        if ok and a.release_at_home and a.grab_mode == "station":
            robot.release_cube()
        print("\n" + robot.ascii_map(robot.last_cell))
    except MissionAbort as e:
        robot.stop()
        robot.result = "중단: %s" % e
        robot.log(robot.result)
    except KeyboardInterrupt:
        robot.stop()
        robot.result = "Ctrl+C 로 정지"
        robot.log(robot.result)
    finally:
        robot.stop()
        if old_tty is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
        robot.dump(force=True)
        if robot.rec_path:
            print("기록: %s  →  그림: python3 %s/draw_run.py %s" % (robot.rec_path, os.path.dirname(os.path.abspath(__file__)), robot.rec_path))
        ex.shutdown(timeout_sec=1.0)
        spin_th.join(timeout=2.0)
        robot.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
