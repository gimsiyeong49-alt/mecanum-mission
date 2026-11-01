#!/usr/bin/env python3
"""격자 맞춤 위치추정 (SLAM 없음) — EKF 오돔 + 라이다 벽을 40 cm 격자선에 맞춰 보정. [실험판: 칼만식 섞기]

실험판 (10-21 필드 측정 근거): 오돔은 칸당 ±1.5 cm(앞뒤 ±3%, 옆 +2.6%)로 정확한데, 예전 방식(스캔마다 벽 매칭의 50% 반영,
서 있으면 ±16 cm 찾아 한 번에 최대 8 cm 이동)은 위치추정을 칸당 −9~+12% 흔들었다. 그래서 오돔 오차(움직인 거리에 비례해 커짐)와
벽 매칭 오차(점 수·흩어짐·움직임에 따라)를 각각 들고 다니며 오차 크기에 맞게 섞는다. 오차에 비해 너무 큰 매칭은 버리되,
같은 값이 여러 번 계속 나오면 받아들인다.

경기장 바깥벽과 장애물 면은 모두 격자선(칸 경계) 위에 있다. 오돔이 틀리면 라이다 벽 점이 격자선에서 벗어나 보이므로,
매 스캔 (1) 벽 점이 격자선에 가장 잘 붙는 회전 (2) 세로 면 점으로 좌우, 가로 면 점으로 앞뒤 를 구해 조금씩 바로잡는다.
지도는 만들지 않는다. 미리 아는 것: 칸 크기, 출발 칸 (1,9) 중심·남향.

출력: TF map→odom (30 Hz). map 원점 = 켤 때 로봇 자리. mission1_node.py 는 map→base_link 만 본다(SLAM 때와 같음).

  python3 grid_localizer.py               # robot.launch.py(구동+IMU+EKF+라이다+laser TF) 가 켜진 뒤
"""
import argparse
import math
import threading
import time

import numpy as np

HOME = (1, 9)
N = 9


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """a∘b (2D 자세 x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


class GridCorrector:
    """ROS 없이 시험할 수 있는 본체. T = map→odom."""

    def __init__(self, cell=0.40, max_r=1.5, win=0.07, gain=0.5, yaw_win_deg=4.0, min_pts=12, rot_gate=0.3,
                 k_odom=0.03, k_yaw=0.02):
        self.c, self.max_r, self.win, self.gain = cell, max_r, win, gain
        self.yaw_win = math.radians(yaw_win_deg)
        self.min_pts, self.rot_gate = min_pts, rot_gate
        self.T = None
        self.hx, self.hy = (HOME[0] - 0.5) * cell, (HOME[1] - 0.5) * cell
        self.psi = -math.pi / 2           # 출발 때 정면의 경기장 방향 (−90° = 남, 0 = 동). 임무가 /grid_loc/start_yaw 로 알려 줌
        self.prev = None
        self.prev_pos = None
        self.moving_v = 0.03              # 이보다 빠르면 '움직이는 중' [m/s]
        self.still_search = 0.16          # 서 있을 때 위치를 찾아보는 범위 ±[m] (반 칸 20 cm 보다 작게 → 옆 격자선과 안 헷갈림)
        self.moving_along_gain = 0.15     # 움직이는 중 진행 방향 보정은 이 비율만
        self.max_pts = 400
        self.line_min = 12                # 같은 벽 줄(±7.5 mm)에 이만큼 점이 있어야 벽 점으로 봄
        self.near_r = 0.9                 # 가까운 벽 우선 거리 [m] (필드 칸 크기가 40 cm 에서 칸마다 ±1 cm, 전체 3 cm 어긋남 → 먼 벽은 타협만 만듦)
        self.lock = threading.Lock()
        self.stat = {"n": 0, "used": 0, "dyaw": 0.0, "dX": 0.0, "dY": 0.0, "inl": 0, "rej": 0}
        # 칼만식: 지금 보정값의 오차 분산 (경기장 X, Y [m²], yaw [rad²]). 오돔은 움직인 거리·회전에 비례해 오차가 커짐
        self.k_odom, self.k_yaw = k_odom, k_yaw
        self.P = [0.03 ** 2, 0.03 ** 2, math.radians(2.0) ** 2]
        self.r_floor = 0.006              # 벽 매칭 최소 오차 [m] (필드 칸 크기 칸마다 ±1 cm)
        self.r_yaw = math.radians(0.6)    # 회전 매칭 오차 (탐색 간격 0.25° + 잡음)
        self.t_sync = 0.08                # 움직이는 중 스캔·오돔 시각 어긋남 [s] → 진행 방향 오차 = 속도 × 이것
        self.rej = [[], [], []]           # 축마다 연속으로 버린 매칭 값 (같은 값이 계속 나오면 받아들임)
        self.last_odom = None

    def set_start(self, odom_base):
        """켤 때 로봇 자리 = map 원점, 정면 = map +x (= 경기장 남쪽)."""
        with self.lock:
            self.T = inverse(odom_base)
            self.P = [0.03 ** 2, 0.03 ** 2, math.radians(2.0) ** 2]
            self.rej = [[], [], []]
            self.last_odom = None

    def map_pose(self, odom_pose):
        with self.lock:
            return compose(self.T, odom_pose)

    # map ↔ 경기장 (출발: (1,9) 중심, map +x = 출발 때 정면 = 경기장 방향 psi; 기본 남(−Y), 왼쪽 = 동(+X))
    def to_arena(self, mx, my):
        c, s_ = math.cos(self.psi), math.sin(self.psi)
        return self.hx + c * mx - s_ * my, self.hy + s_ * mx + c * my

    def to_map_delta(self, dX, dY):
        """경기장 기준 이동량 → map 기준 이동량 (to_arena 의 회전을 거꾸로)."""
        c, s_ = math.cos(self.psi), math.sin(self.psi)
        return c * dX + s_ * dY, -s_ * dX + c * dY

    def update(self, odom_laser, r, ang, now=None):
        """odom_laser: 스캔 시각 odom 기준 라이다 자세. r, ang: 라이다 기준 거리·각도 배열."""
        now = time.monotonic() if now is None else now
        self.stat["n"] += 1
        lo, self.last_odom = self.last_odom, odom_laser
        if lo is not None:                       # 오돔 오차가 움직인 만큼 커짐
            ds = math.hypot(odom_laser[0] - lo[0], odom_laser[1] - lo[1])
            dth = abs(wrap(odom_laser[2] - lo[2]))
            self.P[0] += (self.k_odom * ds) ** 2 + (0.3 * ds * self.k_yaw * dth) ** 2
            self.P[1] += (self.k_odom * ds) ** 2 + (0.3 * ds * self.k_yaw * dth) ** 2
            self.P[2] += (self.k_yaw * dth) ** 2 + (math.radians(0.02)) ** 2
        prev, self.prev = self.prev, (now, odom_laser[2])
        pp, self.prev_pos = self.prev_pos, (now, odom_laser[0], odom_laser[1])
        if prev is not None and abs(wrap(odom_laser[2] - prev[1])) / max(now - prev[0], 1e-3) > self.rot_gate:
            return False                                         # 도는 중 스캔은 번져서 안 씀
        with self.lock:
            T = self.T
        lx, ly, lth = compose(T, odom_laser)
        ok = np.isfinite(r) & (r > 0.15) & (r < self.max_r)
        r, ang = r[ok], ang[ok]
        if r.size < 30:
            return False
        px, py = lx + r * np.cos(lth + ang), ly + r * np.sin(lth + ang)
        X, Y = self.to_arena(px, py)
        c, L = self.c, N * self.c
        arena = (X > -0.05) & (X < L + 0.05) & (Y > -0.05) & (Y < L + 0.05)   # 경기장 밖 점 제외
        px, py = px[arena], py[arena]
        if px.size < 30:
            return False
        # (1) 회전: 벽 방향만으로 잰다 (위치와 따로). 경기장 벽은 모두 동서·남북이라, 점들을 돌려 보며 x·y 로 투영한
        # 분포가 가장 뾰족해지는 각도 = 몸이 격자에서 틀어진 양. 격자선 위치를 몰라도 되고 가까운 옆 벽을 그대로 씀.
        # (필드 10-21 녹화: 예전 '격자선 1.5 cm 안 점 수' 방식은 가까운 벽 86% 를 분류에서 버리고, 먼 벽이 다른 격자선을
        #  스치는 엉뚱한 봉우리에 걸려 탐색 끝 ±4° 를 6 번 연속 내 받아들여짐 → (6,2) 에서 −4°. 새 방식은 스캔마다 0.2~0.4°)
        d_meas = self._wall_dir_yaw(lx, ly, px, py)
        # 세로면 점/가로면 점: 방향을 맞춘 뒤 같은 X 에 몰린 점 = 세로 벽, 같은 Y 에 몰린 점 = 가로 벽. 세로면 점은 좌우(X)만,
        # 가로면 점은 앞뒤(Y)만 알려 준다 (필드 10-21: 세로 벽만 보이는 곳에서 세로면 점이 앞뒤 찾기에 섞여 19 cm 엉뚱하게 당김).
        # 예전 '이웃 점 ±2 방향' 은 라이다 15~30 cm 벽에서 점 간격 1 mm < 거리 잡음 8 mm 라 14% 만 분류 → 바로 옆 벽을 거의 못 씀
        da_ = d_meas or 0.0
        ca_, sa_ = math.cos(da_), math.sin(da_)
        vert, horiz = self._wall_lines(*self.to_arena(lx + ca_ * (px - lx) - sa_ * (py - ly),
                                                      ly + sa_ * (px - lx) + ca_ * (py - ly)))
        ins = vert | horiz                                           # 모서리·방향 애매한 점 제외
        px, py, vert, horiz = px[ins], py[ins], vert[ins], horiz[ins]
        if px.size < 30:
            return False
        if px.size > self.max_pts:                                   # 계산량 제한 (Jetson)
            k = np.linspace(0, px.size - 1, self.max_pts).astype(int)
            px, py, vert, horiz = px[k], py[k], vert[k], horiz[k]
        rr = np.hypot(px - lx, py - ly)
        near = rr < self.near_r

        def pick(mask):
            # 가까운 벽 점이 충분하면 그것만, 아니면 전부
            m2 = mask & near
            return m2 if m2.sum() >= self.min_pts else mask

        def resid(qx, qy):
            QX, QY = self.to_arena(qx, qy)
            ex, ey = QX - np.round(QX / c) * c, QY - np.round(QY / c) * c
            return ex, ey

        d = 0.0
        if d_meas is not None:
            ky = self._kalman(2, d_meas, self.r_yaw ** 2)
            d = ky * d_meas
        cd, sd = math.cos(d), math.sin(d)
        qx = lx + cd * (px - lx) - sd * (py - ly)
        qy = ly + sd * (px - lx) + cd * (py - ly)
        moving, ux, uy = False, 0.0, 0.0
        if pp is not None and now - pp[0] > 1e-3:
            vox, voy = (odom_laser[0] - pp[1]) / (now - pp[0]), (odom_laser[1] - pp[2]) / (now - pp[0])
            sp = math.hypot(vox, voy)
            if sp > self.moving_v:
                cT, sT = math.cos(T[2]), math.sin(T[2])
                vmx, vmy = cT * vox - sT * voy, sT * vox + cT * voy      # map 기준 속도
                cp_, sp_psi = math.cos(self.psi), math.sin(self.psi)      # map → 경기장 회전 (출발 방향 psi)
                moving, ux, uy = True, (cp_ * vmx - sp_psi * vmy) / sp, (sp_psi * vmx + cp_ * vmy) / sp   # 경기장 기준 진행 방향
        shX = shY = 0.0
        if not moving:
            # 서 있을 때: ±still_search 안에서 1 cm 간격으로 옮겨 보며 격자선 1.5 cm 안에 드는 점이 가장 많은 위치.
            # 오돔 오차가 몇 cm 쌓여 좁은 창(win) 밖으로 나가도 다시 잡을 수 있게.
            # 좌우(X)·앞뒤(Y)를 따로 1차원으로 찾는다 (가벼움: 33×점수 두 번)
            QX, QY = self.to_arena(qx, qy)
            sh = np.arange(-self.still_search, self.still_search + 1e-9, 0.01)
            k0 = len(sh) // 2
            VX, HY = QX[pick(vert)], QY[pick(horiz)]
            if VX.size >= self.min_pts:          # 세로면 점이 충분할 때만 좌우를 넓게 찾음
                sx_ = (np.abs((VX[None, :] + sh[:, None]) - np.round((VX[None, :] + sh[:, None]) / c) * c) < 0.015).sum(1)
                i = int(np.argmax(sx_))
                if sx_[i] > sx_[k0] * 1.2 + 4 and sx_[i] >= self.min_pts:
                    shX = float(sh[i])
            if HY.size >= self.min_pts:          # 가로면 점이 충분할 때만 앞뒤를 넓게 찾음
                sy_ = (np.abs((HY[None, :] + sh[:, None]) - np.round((HY[None, :] + sh[:, None]) / c) * c) < 0.015).sum(1)
                j = int(np.argmax(sy_))
                if sy_[j] > sy_[k0] * 1.2 + 4 and sy_[j] >= self.min_pts:
                    shY = float(sh[j])
            if shX or shY:
                dqx, dqy = self.to_map_delta(shX, shY)             # 경기장 (X+shX, Y+shY) ↔ map
                qx, qy = qx + dqx, qy + dqy
        # (2) 위치: 세로 격자선 근처 점 → 좌우(X), 가로 격자선 근처 점 → 앞뒤(Y). 모서리 근처(두 선 다 가까움)는 뺌
        ex, ey = resid(qx, qy)
        on_v = pick(vert & (np.abs(ex) < self.win))
        on_h = pick(horiz & (np.abs(ey) < self.win))
        sp_ = 0.0
        if moving and pp is not None and now - pp[0] > 1e-3:
            sp_ = math.hypot(odom_laser[0] - pp[1], odom_laser[1] - pp[2]) / (now - pp[0])
        meas = []                                     # (축, 측정값, 측정 분산)
        for axis, on, e_, sh_ in ((0, on_v, ex, shX), (1, on_h, ey, shY)):
            n_ = int(on.sum())
            if n_ < self.min_pts:
                continue
            res = e_[on]
            med = float(np.median(res))
            mad = float(np.median(np.abs(res - med))) * 1.48
            R = max(mad, 0.004) ** 2 / n_ + self.r_floor ** 2
            if sh_:
                R += 0.02 ** 2                        # 넓게 찾아 옮긴 값은 덜 믿음
            if moving:                                 # 움직이는 중 진행 방향은 시각 어긋남만큼 덜 믿음
                u_ = ux if axis == 0 else uy
                R += (u_ * sp_ * self.t_sync) ** 2
            meas.append((axis, -med + sh_, R))
        dX = dY = 0.0
        for axis, z, R in meas:
            k_ = self._kalman(axis, z, R)
            if axis == 0:
                dX = k_ * z
            else:
                dY = k_ * z
        # 경기장 보정량 → map (남향 출발이면 X = hx + my → my += dX, Y = hy − mx → mx −= dY)
        (dmx, dmy), dd = self.to_map_delta(dX, dY), d
        with self.lock:
            tx, ty, tth = self.T
            cdd, sdd = math.cos(dd), math.sin(dd)
            ntx = cdd * (tx - lx) - sdd * (ty - ly) + lx + dmx
            nty = sdd * (tx - lx) + cdd * (ty - ly) + ly + dmy
            self.T = (ntx, nty, wrap(tth + dd))
        self.stat.update(used=self.stat["used"] + 1, dyaw=math.degrees(d), dX=dX, dY=dY, moving=moving,
                         inl=int(on_v.sum() + on_h.sum()), sx=math.sqrt(self.P[0]), sy=math.sqrt(self.P[1]),
                         syaw=math.degrees(math.sqrt(self.P[2])))
        return True

    def _wall_dir_yaw(self, lx, ly, px, py):
        """map 기준 점들을 라이다 중심으로 돌려 보며 x·y 투영 히스토그램(5 mm 칸)의 제곱합이 가장 큰 각도 [rad].
        가까운 점(near_r 안)이 충분하면 그것만 쓴다. 탐색 끝에 걸리면(범위 안에 봉우리 없음) None."""
        k = np.hypot(px - lx, py - ly) < self.near_r
        if k.sum() < 60:
            k = np.ones_like(k)
        if k.sum() < 60:
            return None
        x, y = px[k] - lx, py[k] - ly

        def peak(dl):
            cd, sd = math.cos(dl), math.sin(dl)
            hx = np.bincount(np.floor((cd * x - sd * y + 2.0) / 0.005).astype(int))
            hy = np.bincount(np.floor((sd * x + cd * y + 2.0) / 0.005).astype(int))
            return float(np.dot(hx, hx) + np.dot(hy, hy))

        coarse = np.arange(-self.yaw_win, self.yaw_win + 1e-9, math.radians(0.5))
        i = int(np.argmax([peak(dl) for dl in coarse]))
        if i == 0 or i == len(coarse) - 1:
            return None
        fine = coarse[i] + np.radians(np.arange(-0.5, 0.5001, 0.1))
        return float(fine[int(np.argmax([peak(dl) for dl in fine]))])

    def _wall_lines(self, X, Y):
        """방향을 맞춘 경기장 좌표 점들 → (세로 벽 점, 가로 벽 점). 자기 X 의 ±7.5 mm 안에 같은 X 인 점이 line_min 개 넘고
        같은 Y 쪽보다 2 배 넘게 많으면 세로 벽 (가로도 같은 식). 두 쪽 다 비슷하면 모서리라 뺌."""
        b = 0.005
        ix, iy = np.floor((X + 1.0) / b).astype(int), np.floor((Y + 1.0) / b).astype(int)
        k3 = np.ones(3)
        cx = np.convolve(np.bincount(ix).astype(float), k3, "same")[ix]
        cy = np.convolve(np.bincount(iy).astype(float), k3, "same")[iy]
        m = self.line_min
        return (cx >= m) & (cx >= 2 * cy), (cy >= m) & (cy >= 2 * cx)

    def _kalman(self, axis, z, R):
        """측정 z (지금 보정에서 더 고칠 양), 분산 R → 반영 비율 K. 오차에 비해 너무 크면 버리되 같은 값이 계속 나오면 받아들임."""
        P = self.P[axis]
        lim = 3.0 * math.sqrt(P + R)
        if abs(z) > lim and abs(z) > (0.02 if axis < 2 else math.radians(1.5)):
            buf = self.rej[axis]
            buf.append(z)
            del buf[:-8]
            self.stat["rej"] = self.stat.get("rej", 0) + 1
            if len(buf) >= 6 and float(np.std(buf[-6:])) < (0.015 if axis < 2 else math.radians(1.0)) \
                    and all(np.sign(b) == np.sign(z) for b in buf[-6:]):
                self.P[axis] = z * z                   # 6 번 연속 같은 쪽으로 크게 어긋남 → 오돔이 정말 틀어졌다고 보고 받아들임
                buf.clear()
                P = self.P[axis]
            else:
                return 0.0
        else:
            self.rej[axis].clear()
        K = P / (P + R)
        # 연속 스캔의 벽 매칭 오차는 서로 독립이 아님(같은 벽·같은 치우침) → 오차가 너무 작아져 굳지 않게 바닥값
        self.P[axis] = max((1.0 - K) * P, 0.004 ** 2 if axis < 2 else math.radians(0.3) ** 2)
        return K


def main():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Empty, Float32, String
    from tf2_ros import Buffer, TransformBroadcaster, TransformListener

    ap = argparse.ArgumentParser(description="격자 맞춤 위치추정 (SLAM 없음)")
    ap.add_argument("--cell", type=float, default=0.40)
    ap.add_argument("--gain", type=float, default=0.5, help="스캔 한 번에 반영할 보정 비율")
    ap.add_argument("--max-r", type=float, default=1.5, help="보정에 쓰는 라이다 거리 [m]")
    ap.add_argument("--win", type=float, default=0.07, help="격자선에서 이 안의 점만 벽 점으로 씀 [m]")
    a = ap.parse_args()

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    rclpy.init()
    node = Node("grid_localizer")
    buf = Buffer()
    TransformListener(buf, node)
    st_pub = node.create_publisher(String, "/grid_loc/status", 10)
    gc = GridCorrector(cell=a.cell, gain=a.gain, max_r=a.max_r, win=a.win)

    def tf2pose(t):
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    pending = []                                   # 스캔 시각 오돔 TF 가 올 때까지 기다리는 스캔들

    def on_scan(m):
        if gc.T is not None:
            pending.append(m)

    def process():
        # 스캔 시각의 odom→laser 가 들어온 스캔만 처리. 못 찾은 채 0.3 초 지나면 버림(최신 TF 로 대신하면 시각이 어긋나 치우침).
        # 밀려 있으면 최신 3 개만 남긴다 (계산이 밀려도 늦은 스캔에 매달리지 않게)
        now = node.get_clock().now()
        if len(pending) > 3:
            gc.stat["dropped"] = gc.stat.get("dropped", 0) + len(pending) - 3
            del pending[:-3]
        while pending:
            m = pending[0]
            st = rclpy.time.Time.from_msg(m.header.stamp)
            if buf.can_transform("odom", m.header.frame_id, st):
                pending.pop(0)
                t = buf.lookup_transform("odom", m.header.frame_id, st)
                r = np.asarray(m.ranges, float)
                ang = m.angle_min + m.angle_increment * np.arange(len(r))
                gc.update(tf2pose(t), r, ang, now=st.nanoseconds * 1e-9)
            elif (now - st).nanoseconds * 1e-9 > 0.3:
                pending.pop(0)
                gc.stat["dropped"] = gc.stat.get("dropped", 0) + 1
            else:
                break

    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    node.create_timer(0.02, process)

    reset_req = {"n": 0}

    def on_reset(_m):
        # 임무 시작 때: 지금 로봇 자리를 원점((1,9) 칸 중심, 남향)으로 다시 잡는다
        try:
            gc.set_start(tf2pose(buf.lookup_transform("odom", "base_link", rclpy.time.Time())))
            gc.prev = gc.prev_pos = None
            pending.clear()
            reset_req["n"] += 1
            node.get_logger().info("원점 다시 잡음 (%d 번째): 지금 로봇 자리 = 칸 %s 중심" % (reset_req["n"], HOME))
        except Exception as e:
            node.get_logger().warn("원점 다시 잡기 실패: %s" % e)

    node.create_subscription(Empty, "/grid_loc/reset", on_reset, 10)

    def on_start_yaw(m):
        # 임무가 원점 다시 잡기 전에 출발 방향을 알려 줌 (−90 = 남향 출발, 0 = 동향 출발; 점대칭이라 (9,1) 북향 = 남향, 서향 = 동향)
        gc.psi = math.radians(m.data)
        node.get_logger().info("출발 방향: 정면 = 경기장 %.0f° (−90 남, 0 동)" % m.data)

    node.create_subscription(Float32, "/grid_loc/start_yaw", on_start_yaw, 10)

    def publish():
        if gc.T is None:
            try:
                gc.set_start(tf2pose(buf.lookup_transform("odom", "base_link", rclpy.time.Time())))
                node.get_logger().info("출발 자세 고정: map 원점 = 지금 로봇 자리 (칸 %s 중심, 남향 가정)" % (HOME,))
            except Exception:
                return
        x, y, th = gc.T
        t = TransformStamped()
        # AMCL 의 transform_tolerance 처럼 0.1 초 앞 시각으로 낸다 → 스캔 시각 TF 조회가 '미래로 외삽' 오류 없이 됨
        t.header.stamp = (node.get_clock().now() + rclpy.duration.Duration(seconds=0.1)).to_msg()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.translation.x, t.transform.translation.y = float(x), float(y)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        br.sendTransform(t)

    def status():
        s = gc.stat
        msg = String()
        msg.data = ("원점 %d · 스캔 %d 중 보정 %d (시각 못 맞춰 버림 %d, 너무 커서 버림 %d) · 마지막 보정 yaw %+.2f° X %+.1f cm Y %+.1f cm"
                    " · 벽 점 %d · 오차 X %.1f Y %.1f cm yaw %.1f°") % (
            reset_req["n"], s["n"], s["used"], s.get("dropped", 0), s.get("rej", 0), s["dyaw"], 100 * s["dX"], 100 * s["dY"],
            s["inl"], 100 * s.get("sx", 0), 100 * s.get("sy", 0), s.get("syaw", 0))
        st_pub.publish(msg)

    # TF 발행은 따로 스레드에서 30 Hz — 스캔 계산이 밀려도 map→odom 이 끊기지 않게
    pub_node = Node("grid_localizer_tf")
    br = TransformBroadcaster(pub_node)
    stop_ev = threading.Event()

    def pub_loop():
        while not stop_ev.is_set():
            try:
                publish()
            except Exception:
                pass
            time.sleep(1.0 / 30.0)

    threading.Thread(target=pub_loop, daemon=True).start()
    node.create_timer(1.0, status)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_ev.set()
        node.destroy_node()
        pub_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
