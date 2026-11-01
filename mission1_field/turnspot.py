"""회전 자리·회전 표 길찾기 (시뮬레이션 회전자리_20261028 의 합친 판을 로봇용으로 옮김, 판단 규칙은 같음).
- TablePlanner: 성환 planner 의 plan_path_mecanum 과 같되, 칸 둘레 모양(3×3, turn_table.pkl)으로 '돌 수 있는 칸'에서만 회전 간선.
  planner.py 원본은 그대로 두고 이 프로세스 안에서만 P.plan_path_mecanum 을 바꿔 끼움.
- turn_spot: 회전 명령이면 돌 자리(지금 칸·이웃 아는 빈 칸, 1 cm 격자) 중 [지금→자리 직선, 회전, 옆으로 줄 맞춤, 곧게 다음 목표]가
  벽에서 떨어지고 시간이 가장 짧은 곳. 여유 5 cm 자리를 두 칸 안에서 먼저, 없으면 1 cm 자리를 한 칸 안에서. 없으면 표 자리.
- 몸 모양(바퀴 가운데 기준, x 앞 y 왼, m): 차체·바퀴·그리퍼·카메라 받침 (10-28 실측 + 메인 프레임.stl)."""
import math, os, pickle, heapq, time
import numpy as np
import shapely
from shapely.geometry import box, Point
from shapely.ops import unary_union
from shapely import affinity

FRONT, BACK = 0.191, 0.161
PARTS = [box(-BACK, -0.0975, 0.130, 0.0975),
         box(0.056, 0.0975, 0.136, 0.1225), box(0.056, -0.1225, 0.136, -0.0975),
         box(-0.136, 0.0975, -0.056, 0.1225), box(-0.136, -0.1225, -0.056, -0.0975),
         box(0.130, -0.040, FRONT, 0.040),
         box(FRONT - 0.0044, 0.065, FRONT, 0.1205),
         box(FRONT - 0.006 - 0.0044, -0.1205, FRONT - 0.006, -0.065)]
FOOT = unary_union(PARTS)
R_SWEEP = max(math.hypot(x, y) for p in PARTS for x, y in p.exterior.coords)
PREF = 0.05
V_FWD, V_SIDE, W_ROT, T_STOP = 0.31, 0.25, 1.0, 0.4
NB = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
TURNED = 5
TABLE = pickle.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "turn_table.pkl"), "rb"))


def place(geom, x, y, yaw): return affinity.translate(affinity.rotate(geom, yaw, origin=(0, 0), use_radians=True), x, y)
def body_at(x, y, yaw): return [place(p, x, y, yaw) for p in PARTS]


def obstacles(occ, C, N=9):
    wall = box(-0.2, -0.2, N * C + 0.2, N * C + 0.2).difference(box(0, 0, N * C, N * C))
    return unary_union([wall] + [box((c - 1) * C, (r - 1) * C, c * C, r * C) for c, r in occ])


def sweep_trans(x0, y0, x1, y1, yaw):
    a, b = body_at(x0, y0, yaw), body_at(x1, y1, yaw)
    return unary_union([unary_union([pa, pb]).convex_hull for pa, pb in zip(a, b)])


def sweep_rot(x, y, yaw0, yaw1, step=math.radians(3)):
    d = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
    n = max(2, int(abs(d) / step) + 1)
    return unary_union([place(FOOT, x, y, yaw0 + d * k / (n - 1)) for k in range(n)])


def t_move(x0, y0, x1, y1, yaw):
    dx, dy = x1 - x0, y1 - y0; L = math.hypot(dx, dy)
    if L < 1e-6: return 0.0
    ca = abs(math.cos(math.atan2(dy, dx) - yaw))                        # 앞뒤 성분 비율 (시뮬레이션 turn_sim.t_move 와 같은 식)
    v = 1 / math.hypot(ca / V_FWD, math.sqrt(max(0, 1 - ca * ca)) / V_SIDE)
    return L / v + T_STOP


def turn_spot(x, y, yaw, cell, yaw1, tx, ty, OB, ok_cells, C, yaw_move=None):
    """(시간, 자리 x, 자리 y) 또는 None. ok_cells = 자리로 쓸 수 있는 칸(아는 빈 칸 + 지금 칸).
    yaw_move = 돈 다음 갈 방향 (없으면 yaw1 = 앞으로). 돈 뒤 옆걸음이면 줄 맞춤 기준선이 다름."""
    ym = yaw1 if yaw_move is None else yaw_move
    r = _turn_spot(x, y, yaw, cell, yaw1, tx, ty, OB, ok_cells, C, PREF, 2, 0.02, ym)
    return r if r is not None else _turn_spot(x, y, yaw, cell, yaw1, tx, ty, OB, ok_cells, C, 0.01, 1, 0.01, ym)


def foot(px, py, tx, ty, yaw1):
    """돈 뒤 먼저 가서 닿는 점: 다음 목표를 지나는 '가는 방향'(yaw1) 직선 위로 (px, py) 를 내린 점 — 가는 방향에 수직인 어긋남만 먼저 없앰.
    (10-30 가상: 몸 방향 직선으로 내리면 돈 뒤 옆걸음일 때 다음 칸까지 46 cm 를 '줄 맞춤'으로 감)
    앞으로 가며 비스듬히 맞추면 옆 성분이 초당 5 cm 안팎이라 메카넘이 옆으로 거의 안 감 (필드 10-30 01:34 (6,2)→(6,3) 긁음)
    → 실제 움직임대로 '옆으로 → 곧게' 두 직선이 벽에서 떨어지는지 봄."""
    c, s_ = math.cos(yaw1), math.sin(yaw1)
    a = (px - tx) * c + (py - ty) * s_
    return tx + a * c, ty + a * s_


def _turn_spot(x, y, yaw, cell, yaw1, tx, ty, OB, ok_cells, C, margin, reach, path_margin, yaw_move):
    cells = [(cell[0] + i, cell[1] + j) for i in range(-reach, reach + 1) for j in range(-reach, reach + 1)]
    cells = [c for c in cells if c in ok_cells]
    grid = np.arange(-0.15, 0.1501, 0.01)
    pts = [((c[0] - 0.5) * C + dx, (c[1] - 0.5) * C + dy) for c in cells for dx in grid for dy in grid]
    if not pts: return None
    P_ = np.array(pts)
    d = shapely.distance(OB, shapely.points(P_))          # 한 번에 (점마다 따로 재면 로봇에서 수 초)
    cand = sorted(set(map(tuple, P_[d >= R_SWEEP + margin].tolist())), key=lambda p: math.hypot(p[0] - x, p[1] - y) + math.hypot(tx - p[0], ty - p[1]))
    best = None
    for px, py in cand[:600]:
        d1, d2 = math.hypot(px - x, py - y), math.hypot(tx - px, ty - py)
        if best is not None and (d1 + d2) / V_FWD + T_STOP * ((d1 > 1e-6) + (d2 > 1e-6)) > best[0]:
            break                                              # 이후 후보는 거리가 더 멀어 시간이 최선보다 짧을 수 없음 (같은 답)
        if not _clear(x, y, px, py, yaw, OB, path_margin): continue
        fx, fy = foot(px, py, tx, ty, yaw_move)          # 가는 방향 직선 위로 (앞으로면 yaw1, 옆걸음이면 그 옆 방향)
        if not _clear(px, py, fx, fy, yaw1, OB, path_margin): continue
        if not _clear(fx, fy, tx, ty, yaw1, OB, path_margin): continue
        t = t_move(x, y, px, py, yaw) + t_move(px, py, fx, fy, yaw1) + t_move(fx, fy, tx, ty, yaw1)
        if best is None or t < best[0]: best = (t, px, py)
    return best


_CORN = np.array([[(p.bounds[0], p.bounds[1]), (p.bounds[2], p.bounds[1]), (p.bounds[2], p.bounds[3]), (p.bounds[0], p.bounds[3])] for p in PARTS])  # (8, 4, 2)


def _sweep_dist(x0, y0, x1, y1, yaw, OB):
    """sweep_trans(...).distance(OB) 와 같은 값: 합집합의 거리 = 부품별(양 끝 볼록 껍질) 거리의 최솟값 → 합치지 않고 한 번에 계산."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s_], [s_, c]])
    q = _CORN @ R.T                                                     # (8, 4, 2)
    pts = np.concatenate([q + (x0, y0), q + (x1, y1)], axis=1)           # (8, 8, 2)
    hulls = shapely.convex_hull(shapely.multipoints(pts))
    return float(shapely.distance(OB, hulls).min())


def _clear(x0, y0, x1, y1, yaw, OB, margin):
    """몸이 (x0,y0)→(x1,y1) 직선으로 갈 때 벽에서 margin 이상인가. 중심 선이 벽에서 R_SWEEP+margin 넘으면 몸 전체도 반드시 통과
    (몸은 중심에서 R_SWEEP 안) → 몸 모양 계산 생략, 아니면 정확히 계산 (답은 sweep_trans 만 쓸 때와 같음)."""
    if shapely.distance(OB, shapely.linestrings([(x0, y0), (x1, y1)]) if math.hypot(x1 - x0, y1 - y0) > 1e-9 else shapely.points(x0, y0)) >= R_SWEEP + margin:
        return True
    return _sweep_dist(x0, y0, x1, y1, yaw, OB) >= margin


def bits_of(cell, is_wall, P):
    b = 0
    for k, d in enumerate(NB):
        n = (cell[0] + d[0], cell[1] + d[1])
        if not P.inside(n) or is_wall(n): b |= 1 << k
    return b


def table_offset(cell, f, nf, occ, P):
    """칸 둘레 모양으로 표에서 회전 자리(칸 중심 기준 dx, dy) 또는 None(이 칸에선 못 돎)."""
    return TABLE[(bits_of(cell, lambda n: n in occ, P), f, nf)]


class TablePlanner:
    """plan_table.py 의 TablePlanner 와 같음 (P = planner 모듈)."""
    def __init__(self, P):
        self.P, self.ORIG = P, P.plan_path_mecanum
        self.calls, self.tsum, self.fallback, self.forbid = 0, 0.0, 0, set()
        self.mirror_off = 0            # 미러 벽 때문에 길이 없어 미러 없이 계획한 횟수
        self.turn_known_r = -1         # 큐브 쪽으로 갈 때, 큐브에서 이 칸 수 안에서는 둘레가 확인된 곳에서만 돌 계획을 세움 (-1 = 안 씀)

    def plan(self, grid, start, facing, goal, t=0.0, moving=None, turn_cost=1.0, strafe_cost=1.25,
             reverse_cost=1.0, change_cost=0.3, unknown_cost=0.5, face_goal=False):
        t0 = time.perf_counter()
        try:
            return self._plan(grid, start, facing, goal, t, moving, turn_cost, strafe_cost, reverse_cost, change_cost, unknown_cost, face_goal)
        finally:
            self.calls += 1; self.tsum += time.perf_counter() - t0

    def _plan(self, grid, start, facing, goal, t, moving, turn_cost, strafe_cost, reverse_cost, change_cost, unknown_cost, face_goal):
        P = self.P
        if grid.state(goal) == P.OCC: return None, math.inf
        wall = lambda n: grid.state(n) == P.OCC       # 모르는 칸은 빈 칸으로 (시뮬레이션과 같은 가정)
        # 큐브 앞은 마지막에 큐브를 봐야 함. 둘레를 모르는 칸에서 '가서 돌면 되겠지' 하고 복도에 들어가면, 못 돌 때 되돌아 나와야 함
        # → 큐브 가까이에서는 모르는 칸을 벽으로 치고 회전 가능 여부를 봄 (이동 비용은 그대로). 그 결과 들어가기 전에 몸 방향을 정함
        wall_known = lambda n: grid.state(n) != P.FREE
        near = lambda c: face_goal and max(abs(c[0] - goal[0]), abs(c[1] - goal[1])) <= self.turn_known_r
        bits = {}
        hmin = min(1.0, strafe_cost, reverse_cost)
        h = lambda c: hmin * (abs(c[0] - goal[0]) + abs(c[1] - goal[1]))
        s0 = (start[0], start[1], facing, P.STOP if moving is None else moving)
        g = {s0: 0.0}; prev = {s0: None}; closed = set(); pq = [(h(start), 0, s0)]; tie = 0

        def push(ns, ng, s, c):
            nonlocal tie
            if ng < g.get(ns, math.inf):
                g[ns], prev[ns] = ng, s; tie += 1; heapq.heappush(pq, (ng + h(c), tie, ns))
        while pq:
            _, _, s = heapq.heappop(pq)
            if s in closed: continue
            closed.add(s)
            x, y, f, m = s
            if (x, y) == goal:
                cost, states = g[s], []
                while s is not None: states.append(s); s = prev[s]
                return states[::-1], cost
            if m != TURNED:
                if (x, y) not in bits: bits[(x, y)] = bits_of((x, y), wall_known if near((x, y)) else wall, P)
                for k, nf in ((1, (f + 1) % 4), (1, (f + 3) % 4), (2, (f + 2) % 4)):
                    if ((x, y), f, nf) in self.forbid: continue
                    if TABLE[(bits[(x, y)], f, nf)] is not None:
                        push((x, y, nf, TURNED), g[s] + k * turn_cost, s, (x, y))
            for d in range(4):
                c = (x + P.DXY[d][0], y + P.DXY[d][1])
                st = grid.state(c)
                if st == P.OCC or grid.blocked(c, t): continue
                rel = (d - f) % 4
                if face_goal and c == goal and rel != 0: continue
                step = 1.0 if rel == 0 else reverse_cost if rel == 2 else strafe_cost
                if m not in (P.STOP, TURNED) and m != d: step += change_cost
                if st == P.UNKNOWN: step += unknown_cost
                push((c[0], c[1], f, d), g[s] + step, s, c)
        return None, math.inf

    def plan_or_orig(self, grid, *a, **kw):
        r = self.plan(grid, *a, **kw)
        if r[0] is None and not isinstance(grid, self.P._BlockedView):
            keep, self.turn_known_r = self.turn_known_r, -1   # 확인된 곳만으로 길이 없으면 (이미 안쪽에 들어와 있는 등) 원래 가정으로
            r = self.plan(grid, *a, **kw)
            if r[0] is None and getattr(grid, "mirror_on", False):   # 미러로 아는 벽이 길을 막음 → 직접 본 것만으로 (가짜 벽이 길을 봉쇄해도 안 멈춤)
                grid.mirror_on = False
                r = self.plan(grid, *a, **kw)
                grid.mirror_on = True
                self.mirror_off += r[0] is not None
            self.turn_known_r = keep
        if r[0] is None and not isinstance(grid, self.P._BlockedView):
            self.fallback += 1; return self.ORIG(grid, *a, **kw)
        return r
