"""2임무(배틀럼블) 판단·주행. ROS 를 모른다: 로봇에선 battle_node.RosIO, 시험에선 시뮬의 SimIO 가 같은 함수를 채운다.

좌표: 경기장 중심이 원점, +x = 출발 때 정면, +y = 출발 때 왼쪽. 네 출발 칸 모두 출발 자세가 (-1.2, -1.2, 0°) 라
어느 칸에서 출발하는지 몰라도 된다 (규정 그림 3: 어디서든 중심은 왼쪽 앞 45°).

순서: 출발 → 큐브 앞 450 mm → 잡기(잡기 스테이션) → 쥠 확인(200 mm 후진) → 중앙 유지
      못 잡으면 3×3 순회 → 중앙에서 보이면 다시 잡기, 다른 곳에서 보이면 추격·밀기
모든 움직임은 cmd() 의 안전선을 마지막으로 지난다. 경기 중엔 어떤 실패에도 임무를 끝내지 않고 다음 행동으로 넘어간다.
"""
import math
from dataclasses import dataclass

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class Cfg:
    start_x: float = -1.2         # 시작 자세. 시험할 때만 바꿈 (battle_node --test)
    start_y: float = -1.2
    match_time: float = 305.0     # 경기 5 분 + 여유 [s]. 끝나면 바퀴만 멈추고 그리퍼는 쥔 채로 둠 (들어 올릴 때 큐브가 같이 들려야 함)
    start_mode: str = "front"     # front = 안 돌고 비스듬히 가서 정면 도착 / turn45 = 제자리 45° 뒤 직진
    line: float = 1.4             # 중심 → 노란 선 (규정 그림 3: 외벽 1.8 m 에서 400 mm 안쪽)
    safety: float = 0.20          # 노란 선 → 안전선. 바퀴 가운데 기준 (앞 19.1, 뒤 16.1, 옆 12.25 cm 라 몸은 선 안에 남음)
    chase_lim: float = 1.0        # 추격은 이 안에서만 (돌아 있어도 몸 모서리 22.6 cm 가 노란 선을 안 넘게)
    stop_t: float = 0.25          # 선 쪽 속도 제한: 남은 거리 / 이 시간 (멈춤 명령 뒤 굴러가는 시간 실측 0.13~0.20 s)
    grip_x: float = 0.191         # 쥘 때 바퀴 가운데 → 큐브 가운데 (cube_station --grip-x)
    stance: float = 0.45          # 잡기 시작 자리: 바퀴 가운데 → 큐브 가운데. 스테이션 한계(0.58 m) 안, 두 카메라에 다 보임(0.34 m 이상)
    ring: float = 0.48            # 3×3 순회 반경 (1 칸 + 8 cm): 두 카메라에 큐브 전체가 보이려면 0.34 m 이상, 큐브가 조금 다가와 있어도 보이게. 스테이션 한계 0.58 m 안
    look_every: float = 5.0       # 순회 중 이 간격이 지난 뒤 다음 변 중앙에서 중심을 봄 [s]
    center_r: float = 0.35        # 큐브가 중심에서 이 안이면 '제자리 큐브' (재배치는 심판이 주변에 놓을 수도 있음, 위치 오차 포함)
    back_check: float = 0.20      # 쥠 확인 후진 거리. 쥔 큐브는 바퀴 가운데 앞 19.1 cm 라 안 보이고, 놓친 큐브는 39 cm 라 두 카메라에 보임
    grab_tries: int = 2           # 헛잡기 뒤 바로 다시 잡는 횟수. 넘으면 상대가 쥔 것으로 보고 밀기로
    grab_timeout: float = 40.0
    chase_time: float = 12.0      # 추격 최대 시간 [s]
    chase_lost: float = 1.0       # 큐브가 이만큼 안 보이면 놓친 것 [s]
    push_time: float = 20.0       # 상대 밀어내기 최대 시간 [s]
    push_r: float = 0.35          # 겨냥한 자리에서 이 안의 라이다 점 = 그 상대 (로봇 지름 40 cm 안)
    chase_push: float = 4.0       # 가까이서 안 보이게 된 뒤 더 밀어붙이는 시간 [s]
    still_t: float = 1.5          # 큐브가 이만큼 그 자리에 있으면 멈춘 큐브 → 잡으러 감 [s]
    hold_dead: float = 0.10       # 중앙 유지: 이 안의 밀림엔 반응 안 함 [m]
    hold_near: float = 0.03       # 상대가 몸에 붙어 있을 때는 이만큼만 밀려도 바로 되밂 (밀리고 나면 되돌리기 어려움)
    wheel_lock: bool = False      # 중앙 유지 때 바퀴 잠금 (firmware/mecanum_grip_hold 를 구운 뒤에만)
    hold_tol: float = 0.04
    hold_seen_t: float = 2.0      # 쥔 뒤 앞쪽에 큐브가 이만큼 계속 보이면 헛잡기로 봄 [s]
    hold_yaw_dead: float = math.radians(10.0)
    max_v: float = 0.32           # 명령 상한 (펌웨어 PWM 상한 200 = 바퀴 테 0.32 m/s 명령, 실제 0.36~0.38 m/s)
    rim_turn: float = 0.224       # 옆으로 가며 회전까지 섞을 때 바퀴 테 상한 (적분 몫을 뺀 값, 1임무와 같음)
    wheel_l: float = 0.1995       # 메카넘 회전 팔 lx+ly [m]
    min_v: float = 0.10           # 앞뒤 최소 속도 (바퀴가 서는 저속 방지)
    min_v_side: float = 0.16      # 옆 최소 속도 (실기 10-30: 작은 옆 속도로는 옆으로 안 감)
    max_w: float = 1.2
    min_w: float = 0.60
    kp: float = 1.5
    kw: float = 2.0
    brake_tb: float = 0.15        # 능동 제동 켠 굴러감 = 속도 × 이 시간 (brake_learned.json 0.13~0.15)
    brake_v: float = 0.08         # 능동 제동 반대 명령 [m/s]
    brake_t: float = 0.25         # 0.3 m/s 에서 능동 제동 시간 상한 [s]
    stuck_t: float = 1.5          # 이 시간 동안 2 cm 도 못 가면 막힌 것 [s]
    block_gap: float = 0.20       # 가는 방향 몸 앞 이 거리 안에 라이다 점이 있으면 막힘 [m]
    front: float = 0.191
    back: float = 0.161
    side: float = 0.1225
    evade_zone: float = 0.50      # 안전선에서 이 안쪽까지 밀려오면 미는 상대에게 맞섬 [m]
    contact_gap: float = 0.08     # 몸에서 이 안에 점이 있으면 상대가 붙어 있음 [m]
    wall_fix: bool = True         # 벽 실측으로 위치·방향 바로잡기 (밀려서 바퀴가 미끄러지면 오돔이 못 봄)
    wall_d: float = 1.8           # 중심 → 외벽 (규정 3.6 m)
    wall_see: float = 2.2         # 이 거리 안의 벽만 씀 [m] (중앙에서 벽까지 1.8 m)
    wall_span: float = 0.50       # 이 폭 넘게 곧게 이어져야 벽. 상대 로봇은 지름 40 cm 안 (규정 2.2.1)
    wall_jump: float = 0.70       # 오돔과 이보다 많이 다르면 벽으로 안 믿음 (다른 벽·사람과 혼동 방지)


class MatchOver(Exception):
    pass


class Battle:
    def __init__(self, io, cfg=None):
        self.io, self.c = io, cfg or Cfg()
        self.x, self.y, self.yaw = self.c.start_x, self.c.start_y, 0.0
        self.ox, self.oy = self.x, self.y    # 벽 보정 없이 오돔만 쌓은 위치 (기록·밀림 시험용)
        self.lim = self.c.line - self.c.safety
        self.v_meas = 0.0
        self.last_cmd = (0.0, 0.0)
        self.t0 = None
        self.prev_o = None
        self.prev_p = None
        self.prev_g = None
        self.fix_t = -9.0
        self.fixes = 0
        self.evades = 0
        self.seen = 0
        self.guard_hits = 0
        self.errors = 0
        self.held_at = None
        self.state = "준비"
        self.turn_dir = 1              # 순회 방향: +1 반시계, -1 시계

    # ── 기본 ──
    def log(self, msg):
        t = 0.0 if self.t0 is None else self.io.now() - self.t0
        self.io.log("%6.1f s [%s] (%+.2f, %+.2f, %+4.0f°) %s" % (t, self.state, self.x, self.y, math.degrees(self.yaw), msg))

    def set_state(self, s):
        if s != self.state:
            self.state = s
            self.log("→ " + s)

    def tick(self):
        """제어 한 번마다: 경기 시간 확인, 위치 갱신. 오돔이 끊기면 바퀴를 세우고 돌아올 때까지 기다림 (경기를 끝내지 않음)."""
        waited = False
        while True:
            if self.t0 is not None and self.io.now() - self.t0 > self.c.match_time:
                raise MatchOver()
            o = self.io.odom()
            if o is not None:
                break
            if not waited:
                self.log("오돔 끊김 → 바퀴 세우고 기다림")
                waited = True
            self.io.send(0.0, 0.0, 0.0)
            self.io.sleep(0.05)
        if waited:
            self.log("오돔 돌아옴")
            self.prev_o = None
        ox, oy, oth = o
        g = self.io.gyro_yaw()
        if self.prev_p is not None:          # 이동량은 바퀴(EKF), 방향은 자이로: 충돌로 돌 때 EKF 방향은 일부만 따라감 (실기 10-30). 자이로가 끊기면 그동안만 EKF 방향
            self.yaw = wrap(self.yaw + (wrap(g - self.prev_g) if g is not None and self.prev_g is not None else wrap(oth - self.prev_p[2])))
            dx, dy = ox - self.prev_p[0], oy - self.prev_p[1]
            c, s = math.cos(self.prev_p[2]), math.sin(self.prev_p[2])
            bx, by = c * dx + s * dy, -s * dx + c * dy
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += c * bx - s * by
            self.y += s * bx + c * by
            self.ox += c * bx - s * by
            self.oy += s * bx + c * by
        self.prev_p, self.prev_g = (ox, oy, oth), g
        now = self.io.now()
        if self.prev_o is not None and now - self.prev_o[2] >= 0.08:
            self.v_meas = math.hypot(ox - self.prev_o[0], oy - self.prev_o[1]) / (now - self.prev_o[2])
            self.prev_o = (ox, oy, now)
        elif self.prev_o is None:
            self.prev_o = (ox, oy, now)
        if self.c.wall_fix and now - self.fix_t >= 0.25:
            self.fix_t = now
            self.fix_from_walls()

    def fix_from_walls(self):
        """벽이 가까우면 실측 거리로 위치·방향을 바로잡음. 격자 맞춤과 달리 벽은 네 면뿐이라 칸을 혼동해 튀지 않음.
        벽 판정: 벽 쪽 방향 점들이 곧은 줄로 wall_span 넘게 이어질 것 (상대 로봇은 그만큼 길지 못함)."""
        pts = self.io.scan()
        if pts is None or len(pts) < 20:
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]        # 경기장 방향으로 돌린 몸 기준 점
        for axis, sign, a, b, p in ((0, 1, ax, ay, self.x), (0, -1, -ax, ay, -self.x), (1, 1, ay, ax, self.y), (1, -1, -ay, ax, -self.y)):
            if self.c.wall_d - p > self.c.wall_see + 0.2:
                continue
            m = (a > 0.2) & (np.abs(b) < 1.0)
            if np.count_nonzero(m) < 20:
                continue
            aa, bb = a[m], b[m]
            best = None
            for k in np.linspace(-0.09, 0.09, 13):                                   # 방향 오차 ±5° 안에서 가장 곧은 줄
                r = aa - k * bb
                h, e = np.histogram(r, bins=np.arange(0.2, self.c.wall_see + 0.25, 0.02))
                i = int(np.argmax(h))
                if best is None or h[i] > best[0]:
                    best = (int(h[i]), k, 0.5 * (e[i] + e[i + 1]))
            n, k, d0 = best
            on = np.abs(aa - k * bb - d0) < 0.025
            if n < 15 or np.count_nonzero(on) < 15:
                continue
            bs = np.sort(bb[on])
            if bs[-1] - bs[0] < self.c.wall_span or np.max(np.diff(bs)) > 0.12:        # 끊김 없이 이어져야 벽 (상대 둘이 나란히 서면 가운데가 빔)
                continue
            d = float(np.median(aa[on] - k * bb[on]))
            if np.count_nonzero((aa - k * bb > d + 0.06) & (bb > bs[0]) & (bb < bs[-1])) > 3:   # 벽 뒤로는 점이 없어야 함 (상대 뒤로는 진짜 벽이 보임)
                continue
            meas = self.c.wall_d - d                                                  # 그 벽 쪽 좌표 (벽에서 d 만큼 안쪽)
            if abs(meas - p) > self.c.wall_jump:
                continue
            new = sign * (p + 0.5 * (meas - p))
            if axis == 0:
                self.x = new
            else:
                self.y = new
            self.yaw = wrap(self.yaw + 0.3 * math.atan(k) * sign * (1 if axis == 0 else -1))   # 줄 기울기 = 방향 오차 (벽 면마다 부호가 다름)
            self.fixes += 1

    def guard(self, vx, vy):
        """안전선: 선 쪽 속도는 남은 거리만큼만, 선 밖이면 안쪽으로 되돌림. 위치는 오돔 기준."""
        out = []
        for p, v in ((self.x, vx), (self.y, vy)):
            if abs(p) > self.lim + 0.01:
                self.guard_hits += 1
                v = -math.copysign(self.c.max_v, p)                              # 선 밖: 무슨 명령이든 최대 속도로 안쪽 (미는 상대를 이겨야 함)
            elif p * v > 0:
                room = max(0.0, self.lim - abs(p))
                vm = room / self.c.stop_t
                if abs(v) > vm:
                    self.guard_hits += 1
                    v = math.copysign(vm, v)
            out.append(v)
        return out

    def contact(self):
        """몸에 붙은 상대의 방향 (경기장 좌표 단위 벡터). 없으면 None."""
        pts = self.io.scan()
        if pts is None or len(pts) == 0:
            return None
        c = self.c
        px, py = pts[:, 0], pts[:, 1]
        body = (px > -c.back - 0.01) & (px < c.front + 0.03) & (np.abs(py) < c.side + 0.01)
        near = ~body & (px > -c.back - c.contact_gap) & (px < c.front + c.contact_gap) & (np.abs(py) < c.side + c.contact_gap)
        if np.count_nonzero(near) < 3:
            return None
        bx, by = float(np.mean(px[near])), float(np.mean(py[near]))
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        ox, oy = cs * bx - sn * by, sn * bx + cs * by
        n = math.hypot(ox, oy)
        return (ox / n, oy / n) if n > 1e-6 else None

    def evade(self, vx, vy, w):
        """선 쪽으로 밀리면: 미는 상대를 정면으로 돌아보고 맞밈. 메카넘은 앞뒤가 옆보다 세고(옆은 힘 낭비), 비키면 그만큼 선으로 더 밀림.
        상대가 바깥쪽에 있으면(우리를 안쪽으로 미는 셈) 신경 쓰지 않음. 추격·밀기 중엔 안 씀."""
        zone = 9.0 if self.state == "중앙 유지" else self.c.evade_zone         # 쥔 뒤엔 어디서든 맞섬 (선 근처까지 밀린 뒤엔 늦음)
        if self.lim < self.c.line - self.c.safety or max(abs(self.x), abs(self.y)) < self.lim - zone or math.hypot(self.x, self.y) < self.c.hold_dead:
            return vx, vy, w
        o = self.contact()
        if o is None or o[0] * -self.x + o[1] * -self.y <= 0:
            return vx, vy, w
        self.evades += 1
        e = wrap(math.atan2(o[1], o[0]) - self.yaw)
        v = self.c.max_v * max(0.0, math.cos(e))
        return v * o[0], v * o[1], clip(self.c.kw * e, -self.c.max_w, self.c.max_w)

    def cmd(self, vx, vy, w):
        """경기장 좌표 속도 → 밀리면 맞서기 → 안전선 → 몸 기준 → 바퀴 상한(방향 유지하며 줄임, 회전 먼저) → 보냄."""
        vx, vy, w = self.evade(vx, vy, w)
        vx, vy = self.guard(vx, vy)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx, by = c * vx + s * vy, -s * vx + c * vy
        rim = self.c.rim_turn if abs(by) > abs(bx) and abs(w) >= 0.1 else self.c.max_v
        room = max(0.0, rim - self.c.wheel_l * abs(w))
        mv = abs(bx) + abs(by)
        if mv > room and mv > 1e-6:
            bx, by = bx * room / mv, by * room / mv
        self.last_cmd = (bx, by)
        self.io.send(bx, by, w)

    def stop(self):
        for _ in range(5):
            self.io.send(0.0, 0.0, 0.0)
            self.io.sleep(0.02)
        self.last_cmd = (0.0, 0.0)

    def brake(self):
        """능동 제동: 가던 반대로 짧게 (1임무 halt 와 같은 방법). 가던 방향 속도가 0.05 m/s 아래면 바로 끊음."""
        cx, cy = self.last_cmd
        n = math.hypot(cx, cy)
        if n >= 0.05:
            ux, uy = cx / n, cy / n
            t_end = self.io.now() + min(0.35, self.c.brake_t * max(self.v_meas, 0.05) / 0.3)
            while self.io.now() < t_end:
                self.io.send(-ux * self.c.brake_v, -uy * self.c.brake_v, 0.0)
                self.io.sleep(0.02)
                self.tick()
                if self.v_meas < 0.05:
                    break
        self.stop()

    def wait(self, sec):
        t_end = self.io.now() + sec
        while self.io.now() < t_end:
            self.tick()
            self.cmd(0.0, 0.0, 0.0)
            self.io.sleep(0.05)

    # ── 라이다·카메라 ──
    def blocked(self, bx, by):
        """몸 기준 방향 (bx, by) 로 가는 길 앞이 막혔나. 몸에 맞은 점은 빼고 판단."""
        pts = self.io.scan()
        if pts is None or len(pts) == 0:
            return False
        n = math.hypot(bx, by)
        if n < 1e-6:
            return False
        ux, uy = bx / n, by / n
        c = self.c
        edge = abs(ux) * (c.front if ux > 0 else c.back) + abs(uy) * c.side
        width = abs(uy) * max(c.front, c.back) + abs(ux) * c.side + 0.02
        px, py = pts[:, 0], pts[:, 1]
        body = (px > -c.back - 0.01) & (px < c.front + 0.03) & (np.abs(py) < c.side + 0.01)
        a = px * ux + py * uy
        return int(np.count_nonzero(~body & (a > edge) & (a < edge + c.block_gap) & (np.abs(-px * uy + py * ux) < width))) >= 3

    def cube_arena(self):
        """지금 보이는 큐브의 경기장 좌표. 안 보이면 None. 움직이는 중 값은 cm 오차가 있어 후보로만 씀."""
        m = self.io.cube()
        if m is None:
            self.seen = 0
            return None
        self.seen += 1
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * m[0] - s * m[1], self.y + s * m[0] + c * m[1]

    def look(self, sec=0.6):
        """서서 본다: 절반 넘게 보이면 경기장 좌표 평균, 아니면 None."""
        self.stop()
        self.wait(0.3)
        xs, ys, n = [], [], 0
        t_end = self.io.now() + sec
        while self.io.now() < t_end:
            self.tick()
            self.cmd(0.0, 0.0, 0.0)
            c = self.cube_arena()
            n += 1
            if c is not None:
                xs.append(c[0]); ys.append(c[1])
            self.io.sleep(0.05)
        if len(xs) * 2 <= n:
            return None
        return sum(xs) / len(xs), sum(ys) / len(ys)

    # ── 움직이기 ──
    def go(self, X, Y, yaw=None, vmax=None, tol=0.03, timeout=12.0, until=None, watch=False, kp=None):
        """(X, Y) 로 곧게, 몸 방향은 yaw 유지. 돌려줌: ok / until / blocked / stuck / timeout."""
        c = self.c
        vmax = vmax or c.max_v
        self.tick()
        yaw_t = self.yaw if yaw is None else yaw
        t0 = self.io.now()
        best, t_best = 9.0, t0
        while True:
            self.tick()
            ex, ey = X - self.x, Y - self.y
            d = math.hypot(ex, ey)
            now = self.io.now()
            if d < max(tol, self.v_meas * c.brake_tb):
                self.brake()
                return "ok"
            if now - t0 > timeout:
                self.stop()
                return "timeout"
            if d < best - 0.02:
                best, t_best = d, now
            elif now - t_best > c.stuck_t:
                self.stop()
                return "stuck"
            if until is not None and until():
                return "until"
            cs, sn = math.cos(self.yaw), math.sin(self.yaw)
            bx, by = cs * ex + sn * ey, -sn * ex + cs * ey
            if watch and self.blocked(bx, by):
                self.brake()
                return "blocked"
            vm = c.min_v_side if abs(by) > abs(bx) else c.min_v
            v = clip((kp or c.kp) * d, vm, vmax)
            ew = wrap(yaw_t - self.yaw)
            w = clip(c.kw * ew, -c.max_w, c.max_w)
            self.cmd(ex / d * v, ey / d * v, w)
            self.io.sleep(0.05)

    def turn_to(self, yaw_t, tol=math.radians(2.0), timeout=8.0):
        """제자리 회전, 밀린 자리는 되밀며. 돌려줌: True / False (시간 초과)."""
        c = self.c
        self.tick()
        x0, y0 = self.x, self.y
        t0 = self.io.now()
        prev, w_meas = (self.yaw, t0), 0.0
        while True:
            self.tick()
            now = self.io.now()
            if now - prev[1] >= 0.05:
                w_meas = 0.5 * w_meas + 0.5 * wrap(self.yaw - prev[0]) / (now - prev[1])
                prev = (self.yaw, now)
            e = wrap(yaw_t - self.yaw)
            if abs(e) < max(tol, abs(w_meas) * 0.12):
                self.stop()
                return True
            if now - t0 > timeout:
                self.stop()
                return False
            w = clip(c.kw * e, -c.max_w, c.max_w)
            if abs(w) < c.min_w:
                w = math.copysign(c.min_w, e)
            hx, hy = x0 - self.x, y0 - self.y
            k = 3.0 if math.hypot(hx, hy) > 0.004 else 0.0
            self.cmd(clip(k * hx, -0.08, 0.08), clip(k * hy, -0.08, 0.08), w)
            self.io.sleep(0.05)

    # ── 잡기 ──
    def grab_here(self):
        """스테이션에 맡기고(그동안 바퀴 명령 안 냄), 쥠 확인까지. 돌려줌: held / again (헛잡기) / lost (못 잡음)."""
        self.set_state("잡기")
        self.stop()
        self.io.grab_start()
        t0 = self.io.now()
        while True:
            self.tick()
            r = self.io.grab_poll()
            if r is not None:
                break
            if self.io.now() - t0 > self.c.grab_timeout or abs(self.x) > self.lim + 0.05 or abs(self.y) > self.lim + 0.05:
                self.log("잡기 멈춤 (시간 초과 또는 안전선 밖)")
                self.io.grab_abort()
                self.io.sleep(0.2)
                return "lost"
            self.io.sleep(0.1)
        self.log("스테이션 결과: %s (%.1f 초)" % (r, self.io.now() - t0))
        if r != "ok":
            return "lost"
        self.set_state("쥠 확인")
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        self.go(self.x - self.c.back_check * cs, self.y - self.c.back_check * sn, yaw=self.yaw, vmax=0.16, timeout=4.0)
        c = self.look()
        if c is not None and math.hypot(c[0] - self.x, c[1] - self.y) > 0.25:      # 쥔 큐브(19 cm)가 아닌 앞쪽 큐브
            self.log("후진했는데 큐브가 앞에 보임 (%+.2f, %+.2f) → 헛잡기" % c)
            return "again"
        self.log("큐브 안 보임 → 쥠 성공")
        if self.held_at is None:
            self.held_at = self.io.now() - self.t0
        return "held"

    def grab_loop(self):
        for i in range(self.c.grab_tries):
            r = self.grab_here()
            if r != "again":
                return r == "held"
        return False

    def approach(self, cx, cy):
        """멈춘 큐브 앞 stance 에 정면으로 선다. 안전선 밖이 되는 자리면 False."""
        self.tick()
        b = math.atan2(cy - self.y, cx - self.x)
        X, Y = cx - self.c.stance * math.cos(b), cy - self.c.stance * math.sin(b)
        gx, gy = cx - self.c.grip_x * math.cos(b), cy - self.c.grip_x * math.sin(b)      # 쥘 때 바퀴 가운데 자리
        if max(abs(X), abs(Y), abs(gx), abs(gy)) > self.lim:
            self.log("큐브 (%+.2f, %+.2f) 를 잡으면 안전선 밖 → 안 감" % (cx, cy))
            return False
        self.set_state("큐브 앞으로")
        self.turn_to(b)
        return self.go(X, Y, yaw=b, timeout=8.0) == "ok"

    # ── 임무 단계 ──
    def start_run(self):
        self.set_state("출발")
        if self.c.start_mode == "turn45":
            self.turn_to(math.pi / 4)
            k = self.c.stance * math.sqrt(0.5)
            r = self.go(-k, -k, yaw=math.pi / 4)
        else:
            r = self.go(-self.c.stance, 0.0, yaw=0.0)
        self.log("큐브 앞 도착: %s" % r)

    def hold(self):
        """중앙 유지: 경기 끝까지. 작은 밀림엔 반응 안 함. 밀리면 세게 되밀어 복귀 (천천히 돌아오면 미는 상대를 못 이김)."""
        self.set_state("중앙 유지")
        yaw_h = self.yaw
        self.go(0.0, 0.0, yaw=yaw_h, tol=self.c.hold_tol)
        seen_t = None
        while True:
            self.tick()
            # 쥠 확인 때 상대에 가려 못 본 헛잡기: 큐브는 하나뿐이라 몸 앞 30 cm 넘는 곳에 계속 보이면 쥔 게 아님
            c = self.cube_arena()
            if c is not None and math.hypot(c[0] - self.x, c[1] - self.y) > 0.30:
                seen_t = seen_t if seen_t is not None else self.io.now()
                if self.io.now() - seen_t > self.c.hold_seen_t:
                    self.log("쥐고 있다고 봤는데 큐브가 (%+.2f, %+.2f) 에 보임 → 다시 잡으러" % c)
                    self.held_at = None
                    if self.c.wheel_lock:
                        self.io.wheel_lock(False)
                    return
            elif c is None and seen_t is not None and self.io.now() - seen_t > self.c.hold_seen_t + 0.5:
                seen_t = None
            if self.c.wheel_lock:
                self.io.wheel_lock(True)
            d = math.hypot(self.x, self.y)
            yaw_e = wrap(yaw_h - self.yaw)
            if d > (self.c.hold_near if self.contact() is not None else self.c.hold_dead):
                self.log("밀림 %.0f cm, 방향 %+.0f° → 복귀" % (100 * d, math.degrees(yaw_e)))
                r = self.go(0.0, 0.0, yaw=yaw_h, tol=self.c.hold_tol, timeout=8.0, kp=3.0)
                if r != "ok" and math.hypot(self.x, self.y) < 2 * self.c.hold_dead:     # 중앙 근처에서 막힘: 모터를 잠깐 쉼. 멀면 안 쉼
                    self.wait(1.0)
            elif abs(yaw_e) > self.c.hold_yaw_dead:
                self.turn_to(yaw_h, tol=math.radians(5.0), timeout=3.0)
            else:
                self.cmd(0.0, 0.0, 0.0)
                self.io.sleep(0.05)

    def ring_points(self):
        r = self.c.ring
        return [(r, -r), (r, 0.0), (r, r), (0.0, r), (-r, r), (-r, 0.0), (-r, -r), (0.0, -r)]     # 반시계. 홀수 번호 = 변 중앙

    def patrol(self):
        """3×3 순회. 큐브를 찾으면 ("center"|"away", x, y) 를 돌려줌. 앞이 막히면 방향을 반대로."""
        self.set_state("순회")
        pts = self.ring_points()
        self.tick()
        i = min(range(8), key=lambda k: math.hypot(pts[k][0] - self.x, pts[k][1] - self.y))
        last_look = self.io.now() - self.c.look_every      # 들어오자마자 한 번 봄
        fails = 0
        while True:
            X, Y = pts[i]
            self.tick()
            d = math.hypot(X - self.x, Y - self.y)
            if d > 0.05:
                b = math.atan2(Y - self.y, X - self.x)
                self.turn_to(b)
                self.seen = 0
                r = self.go(X, Y, yaw=b, tol=0.04, until=lambda: self.moving_cube_seen(), watch=True, timeout=8.0)
                if r == "until":
                    self.stop()
                    c = self.look()
                    if c is not None:
                        return self.classify(c)
                elif r in ("blocked", "stuck", "timeout"):
                    fails += 1
                    self.turn_dir = -self.turn_dir
                    self.log("순회 길 막힘(%s) → 방향 반대로" % r)
                    if fails >= 2:                       # 양쪽 다 막힘: 중심을 보고 잠깐 기다림 (상대가 움직이면 풀림)
                        fails = 0
                        self.turn_to(math.atan2(-self.y, -self.x))
                        c = self.look()
                        last_look = self.io.now()
                        if c is not None:
                            return self.classify(c)
                        self.wait(1.0)
                    i = (i + self.turn_dir) % 8
                    continue
            fails = 0
            if i % 2 == 1 and self.io.now() - last_look >= self.c.look_every:      # 변 중앙: 90° 돌아 중심을 봄
                self.turn_to(math.atan2(-self.y, -self.x))
                c = self.look()
                last_look = self.io.now()
                if c is not None:
                    return self.classify(c)
            i = (i + self.turn_dir) % 8

    def moving_cube_seen(self):
        return self.cube_arena() is not None and self.seen >= 2

    def someone_near(self, cx, cy, r=0.30):
        """큐브 둘레 r 안에 라이다 점이 있나 = 상대가 쥐고 있거나 붙어 있음. 큐브(5 cm)는 라이다 높이(16 cm)보다 낮아 안 잡힘."""
        pts = self.io.scan()
        if pts is None or len(pts) == 0:
            return False
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = self.x + cs * pts[:, 0] - sn * pts[:, 1], self.y + sn * pts[:, 0] + cs * pts[:, 1]
        return int(np.count_nonzero(np.hypot(ax - cx, ay - cy) < r)) >= 5

    def classify(self, c):
        kind = "center" if math.hypot(c[0], c[1]) < self.c.center_r else "away"
        self.log("큐브 발견 (%+.2f, %+.2f) → %s" % (c[0], c[1], kind))
        return kind, c[0], c[1]

    def chase(self, can_grab=True):
        """보이는 큐브 쪽으로 최대 속도로 밀고 들어감. 가까우면 큐브가 시야 아래로 사라지므로(34 cm 안) 안 보여도 잠깐 더 밀어붙임.
        돌려줌: ("still", x, y) 멈춘 큐브 (can_grab 일 때만) / ("lost",) / ("time",). 상대 탈락으로 큐브가 중앙에 돌아오면 순회가 찾음."""
        self.set_state("추격" if can_grab else "밀기")
        self.lim = min(self.lim, self.c.chase_lim)
        try:
            t0 = t_seen = self.io.now()
            still_at, still_t, near = None, None, False
            while True:
                self.tick()
                now = self.io.now()
                if now - t0 > self.c.chase_time:
                    return ("time",)
                c = self.cube_arena()
                if c is None:
                    if now - t_seen > (self.c.chase_push if near else self.c.chase_lost):
                        return ("lost",)
                    cs, sn = math.cos(self.yaw), math.sin(self.yaw)
                    v = self.c.max_v if near else 0.0
                    self.cmd(v * cs, v * sn, 0.0)
                    self.io.sleep(0.05)
                    continue
                t_seen = now
                near = math.hypot(c[0] - self.x, c[1] - self.y) < 0.60
                if still_at is None or math.hypot(c[0] - still_at[0], c[1] - still_at[1]) > 0.05:
                    still_at, still_t = c, now
                elif can_grab and now - still_t > self.c.still_t:
                    return ("still", c[0], c[1])
                b = math.atan2(c[1] - self.y, c[0] - self.x)
                e = wrap(b - self.yaw)
                w = clip(self.c.kw * e, -self.c.max_w, self.c.max_w)
                v = self.c.max_v * max(0.0, math.cos(e))
                self.cmd(v * math.cos(self.yaw), v * math.sin(self.yaw), w)
                self.io.sleep(0.05)
        finally:
            self.lim = self.c.line - self.c.safety
            self.brake()

    def push(self, cx, cy):
        """상대 밀어내기: 큐브 둘레의 라이다 덩어리(상대 몸)를 겨냥해 밀고 나감. 큐브는 가까우면 시야 아래로 사라지지만 몸은 라이다에 계속 보임.
        상대가 사라지거나(탈락) 놓치거나 시간이 다 되면 끝. 큐브가 중앙에 돌아오면 순회가 찾음."""
        self.set_state("밀기")
        self.lim = min(self.lim, self.c.chase_lim)
        tx, ty = cx, cy
        try:
            t0 = t_seen = self.io.now()
            while self.io.now() - t0 < self.c.push_time:
                self.tick()
                now = self.io.now()
                pts = self.io.scan()
                if pts is not None and len(pts):
                    cs, sn = math.cos(self.yaw), math.sin(self.yaw)
                    ax, ay = self.x + cs * pts[:, 0] - sn * pts[:, 1], self.y + sn * pts[:, 0] + cs * pts[:, 1]
                    m = np.hypot(ax - tx, ay - ty) < self.c.push_r
                    if np.count_nonzero(m) >= 3:
                        tx, ty, t_seen = float(np.mean(ax[m])), float(np.mean(ay[m])), now
                if now - t_seen > self.c.chase_lost:
                    self.log("밀던 상대가 안 보임")
                    return
                e = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
                v = self.c.max_v * max(0.0, math.cos(e))
                self.cmd(v * math.cos(self.yaw), v * math.sin(self.yaw), clip(self.c.kw * e, -self.c.max_w, self.c.max_w))
                self.io.sleep(0.05)
        finally:
            self.lim = self.c.line - self.c.safety
            self.brake()

    def calibrate(self, sec=1.0):
        """서 있는 동안 자이로 치우침 재기 (5 분 경기라 치우침이 쌓임)."""
        while self.io.odom() is None:
            self.io.sleep(0.05)
        self.io.gyro_calibrate(sec)
        self.t0 = self.io.now()
        self.tick()

    def search_once(self):
        kind, cx, cy = self.patrol()
        if kind == "away":                                   # 다른 곳의 큐브: 움직이면 쫓고, 멈춰 있으면 잡음
            r = self.chase()
            if r[0] != "still":
                return
            cx, cy = r[1], r[2]
        if self.approach(cx, cy) and self.grab_loop():
            self.hold()
            return
        c = self.look()
        if c is not None and self.someone_near(c[0], c[1]):
            self.push(c[0], c[1])                            # 상대가 붙어 있어 못 잡음 → 밀어냄 (3.4.6). 혼자 놓인 큐브는 밀지 않고 다시 잡음

    def play(self):
        started = False
        while True:
            try:
                if not started:
                    started = True
                    self.start_run()
                    if self.grab_loop():
                        self.hold()
                elif self.held_at is not None:               # 오류 뒤에도 쥔 상태면 중앙 유지로
                    self.hold()
                self.search_once()
            except MatchOver:
                raise
            except Exception as e:                           # 경기 중엔 어떤 오류에도 안 멈춤: 바퀴 세우고 기록하고 다음 행동으로
                import traceback
                self.lim = self.c.line - self.c.safety
                self.errors += 1
                self.log("오류 %r → 계속\n%s" % (e, traceback.format_exc()))
                self.io.send(0.0, 0.0, 0.0)
                self.io.sleep(0.2)

    def wait_signal(self, signal):
        """라운드 대기: 준비를 끝내고 출발 신호만 기다림. 대기 중엔 영속도만 내고, 신호 때 그 자리를 출발 자세로 잡음."""
        self.log("준비 끝 — 출발 신호 기다림")
        while not signal():
            self.io.send(0.0, 0.0, 0.0)
            self.io.gyro_still()
            self.io.sleep(0.02)
        self.x, self.y, self.ox, self.oy, self.yaw = self.c.start_x, self.c.start_y, self.c.start_x, self.c.start_y, 0.0
        self.prev_p = self.prev_g = self.prev_o = None
        self.fix_t = -9.0
        self.t0 = self.io.now()
        self.tick()

    def run(self, test=None, signal=None):
        """test = 현장 단계 시험: start (출발해 큐브 앞까지) / grab (이 자리에서 잡기·쥠 확인) / hold (중앙 유지) / patrol (순회·재확보).
        signal = 준 경우 이 함수가 참이 될 때까지 대기 뒤 출발."""
        self.calibrate()
        if signal is not None:
            self.wait_signal(signal)
        self.log("시작: %s, 안전선 ±%.2f m%s" % (self.c.start_mode, self.lim, ", 시험 " + test if test else ""))
        try:
            if test == "start":
                self.start_run()
            elif test == "grab":
                self.log("잡기 결과: %s" % self.grab_loop())
            elif test == "hold":
                self.hold()
            elif test == "patrol":
                self.search_once()
                self.log("순회 시험 한 번 끝")
            else:
                self.play()
        except MatchOver:
            self.log("경기 시간 끝 → 바퀴만 멈춤 (그리퍼는 그대로)")
        self.stop()
        return self.held_at
