"""
1임무(자율탐색 및 요구조자 구출) 경로계획 모듈 — 파이썬 표준 라이브러리만 사용 (로봇 온보드 이식용)

좌표계
  격자 셀 (x, y): x = 1..9 (왼쪽 → 오른쪽), y = 1..9 (아래 → 위). 1칸 = 400 mm.
  경기장 좌표 (X, Y) [mm]: 원점 = 경기장 좌하단 모서리, X 오른쪽, Y 위쪽.
  셀 (x, y) 의 중심 = ((x - 0.5) * 400, (y - 0.5) * 400).

  경기장은 (5,5) 기준 점대칭이고 두 시작점((1,9) 남향, (9,1) 북향)도 서로 점대칭이다.
  따라서 어느 코너에 배치되든 "우리는 (1,9)에서 남쪽을 보고 출발한다"고 두면 완전히 같은 문제가 된다.
  이 모듈은 항상 그 좌표계를 쓴다 (실제 배치 코너를 알 필요 없음).

구성
  GridMap        : 9x9 격자 지도 (모름 / 빈 칸 / 장애물). 한 칸을 알면 점대칭 칸도 같이 안다.
  plan_path      : (x, y, 방향) 상태 A*. 모르는 칸은 갈 수 있다고 가정 + 벌점, 90도 회전마다 비용.
  MissionPlanner : 큐브까지 이동 → 집기 → 시작점 복귀 상태기계. 새 관측이 들어올 때마다 다시 계획.

센서는 무엇이든 된다. 이 모듈의 입력은 {셀: 장애물 여부} 하나뿐이다.
"""
__version__ = "1.0"

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

Cell = Tuple[int, int]

N_CELLS = 9
CELL_MM = 400.0
TARGET = (5, 5)          # 요구조자(빨간 큐브) 초기 위치
HOME = (1, 9)            # 우리 시작점 (점대칭이므로 항상 이 코너로 간주)
OPP_HOME = (9, 1)        # 상대 시작점

N, E, S, W = 0, 1, 2, 3  # 방향 번호 (시계방향)
DXY = [(0, 1), (1, 0), (0, -1), (-1, 0)]
HEADING_NAME = "NESW"

UNKNOWN, FREE, OCC = 0, 1, 2

# 규정 <그림 1> 첫 번째 경기장 예시 (실제 경기장은 당일 공개). 위에서부터 y = 9 .. 1, 왼쪽부터 x = 1 .. 9
# '#' 장애물, 'S' 시작점, 'T' 요구조자(큐브), '.' 빈 칸. 시뮬레이션·테스트용.
RULEBOOK_MAP = [
    "S.#...#..",
    "..#......",
    "..#.##...",
    "..#...#..",
    ".##.T.##.",
    "..#...#..",
    "...##.#..",
    "......#..",
    "..#...#.S",
]


def parse_map(rows) -> set:
    return {(j + 1, N_CELLS - i) for i, row in enumerate(rows) for j, ch in enumerate(row) if ch == "#"}


# ── 좌표 도우미 ─────────────────────────────────────────────────────
def inside(c: Cell) -> bool:
    return 1 <= c[0] <= N_CELLS and 1 <= c[1] <= N_CELLS


def mirror(c: Cell) -> Cell:
    """(5,5) 기준 점대칭 셀."""
    return (N_CELLS + 1 - c[0], N_CELLS + 1 - c[1])


def cell_center_mm(c: Cell) -> Tuple[float, float]:
    return ((c[0] - 0.5) * CELL_MM, (c[1] - 0.5) * CELL_MM)


def mm_to_cell(X: float, Y: float) -> Optional[Cell]:
    c = (int(math.floor(X / CELL_MM)) + 1, int(math.floor(Y / CELL_MM)) + 1)
    return c if inside(c) else None


def direction(a: Cell, b: Cell) -> int:
    return DXY.index((b[0] - a[0], b[1] - a[1]))


# ── 격자 지도 ───────────────────────────────────────────────────────
class GridMap:
    """9x9 격자 지도.

    symmetric=True 이면 한 칸을 관측할 때 점대칭 칸도 같은 값으로 채운다.
    출발점 두 곳과 (5,5)는 규정상 빈 칸이므로 처음부터 FREE.
    이미 빈 칸으로 확인된 칸에서 '장애물'이 보이면 고정 장애물이 아니라 상대 로봇으로 보고
    그 칸만 dyn_ttl 초 동안 임시 차단한다 (장애물은 움직이지 않으므로).
    """

    def __init__(self, symmetric: bool = True, dyn_ttl: float = 3.0):
        self.symmetric, self.dyn_ttl = symmetric, dyn_ttl
        self.fixed_free = {HOME, OPP_HOME, TARGET}
        self.cells: Dict[Cell, int] = {c: FREE for c in self.fixed_free}
        self.temp_block: Dict[Cell, float] = {}   # 셀 -> 차단 해제 시각 [s]
        self.direct: set = set()                  # 직접 관측한 셀 (시각화용)

    def _set(self, c: Cell, st: int) -> None:
        if c not in self.fixed_free:
            self.cells[c] = st

    def observe(self, c: Cell, occupied: bool, t: float = 0.0) -> None:
        if not inside(c):
            return
        self.direct.add(c)
        if occupied and self.state(c) == FREE:     # 빈 칸이던 곳에 물체 → 상대 로봇
            if c != TARGET:                        # (5,5)에선 큐브 자체가 잡힐 수 있음
                self.block(c, t + self.dyn_ttl)
            return
        st = OCC if occupied else FREE
        self._set(c, st)
        if self.symmetric:
            self._set(mirror(c), st)

    def state(self, c: Cell) -> int:
        return self.cells.get(c, UNKNOWN) if inside(c) else OCC

    def block(self, c: Cell, until_t: float) -> None:
        self.temp_block[c] = max(until_t, self.temp_block.get(c, -1.0))

    def blocked(self, c: Cell, t: float) -> bool:
        return self.temp_block.get(c, -1.0) > t


# ── 경로계획: 방향 포함 A* ──────────────────────────────────────────
def plan_path(grid: GridMap, start: Cell, heading: int, goal: Cell, t: float = 0.0,
              turn_cost: float = 1.0, unknown_cost: float = 0.5):
    """상태 = (x, y, 방향). 이동 1칸 = 1, 90도 회전 1회 = turn_cost, 모르는 칸 진입 = +unknown_cost.

    모르는 칸을 '갈 수 있다'고 가정하고 계획한 뒤, 새 관측이 들어올 때마다 다시 계획한다.
    9x9x4 = 324 상태라 매번 처음부터 계산해도 1 ms 미만이므로 D* Lite 같은 증분 기법은 필요 없다.
    반환: (셀 목록 [start, ..., goal], 비용) / 경로 없음이면 (None, inf)
    """
    if grid.state(goal) == OCC:
        return None, math.inf

    def h(c):
        return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

    s0 = (start[0], start[1], heading)
    g = {s0: 0.0}
    prev = {s0: None}
    closed = set()
    pq = [(h(start), 0, s0)]
    tie = 0
    while pq:
        _, _, s = heapq.heappop(pq)
        if s in closed:
            continue
        closed.add(s)
        x, y, hd = s
        if (x, y) == goal:
            cost = g[s]
            path = []
            while s is not None:
                path.append((s[0], s[1]))
                s = prev[s]
            return path[::-1], cost
        for d in range(4):
            c = (x + DXY[d][0], y + DXY[d][1])
            st = grid.state(c)
            if st == OCC or grid.blocked(c, t):
                continue
            turns = min((d - hd) % 4, (hd - d) % 4)
            ng = g[s] + 1.0 + turn_cost * turns + (unknown_cost if st == UNKNOWN else 0.0)
            ns = (c[0], c[1], d)
            if ng < g.get(ns, math.inf):
                g[ns], prev[ns] = ng, s
                tie += 1
                heapq.heappush(pq, (ng + h(c), tie, ns))
    return None, math.inf


STOP = 4   # 메카넘 상태의 '지금 이동 방향' 자리: 서 있음


def plan_path_mecanum(grid: GridMap, start: Cell, facing: int, goal: Cell, t: float = 0.0,
                      moving: Optional[int] = None, turn_cost: float = 1.0, strafe_cost: float = 1.25,
                      reverse_cost: float = 1.0, change_cost: float = 0.3, unknown_cost: float = 0.5,
                      face_goal: bool = False):
    """메카넘 휠: 몸 방향(facing)과 이동 방향이 따로다. 상태 = (x, y, 몸 방향, 지금 이동 방향 또는 STOP).

    비용 (직진 1칸 = 1 기준)
      몸 기준 앞으로 1칸 = 1, 뒤로 = reverse_cost, 옆으로 = strafe_cost
      움직이던 방향을 바꿈(서서 다시 출발) = +change_cost
      제자리 90도 회전 = turn_cost (회전하면 선 상태가 됨)
      모르는 칸 진입 = +unknown_cost
    face_goal=True 이면 goal 칸에는 정면(몸 방향 = 이동 방향)으로만 들어간다 — 집게가 앞에 있으므로
    큐브 칸은 정면으로 들어가야 하고, 옆으로 들어가면 큐브를 밀어 버린다.
    대각선 이동은 없다: 메카넘 대각선은 바퀴 2개만 써서 속도가 약 0.7배라 직선 두 칸과 시간이 같다.
    반환: (상태 목록 [(x, y, 몸 방향, 이동 방향), ...], 비용) / 경로 없음이면 (None, inf)
    """
    if grid.state(goal) == OCC:
        return None, math.inf
    hmin = min(1.0, strafe_cost, reverse_cost)

    def h(c):
        return hmin * (abs(c[0] - goal[0]) + abs(c[1] - goal[1]))

    s0 = (start[0], start[1], facing, STOP if moving is None else moving)
    g = {s0: 0.0}
    prev = {s0: None}
    closed = set()
    pq = [(h(start), 0, s0)]
    tie = 0

    def push(ns, ng, s, c):
        nonlocal tie
        if ng < g.get(ns, math.inf):
            g[ns], prev[ns] = ng, s
            tie += 1
            heapq.heappush(pq, (ng + h(c), tie, ns))

    while pq:
        _, _, s = heapq.heappop(pq)
        if s in closed:
            continue
        closed.add(s)
        x, y, f, m = s
        if (x, y) == goal:
            cost, states = g[s], []
            while s is not None:
                states.append(s)
                s = prev[s]
            return states[::-1], cost
        for nf in ((f + 1) % 4, (f + 3) % 4):                     # 제자리 회전
            push((x, y, nf, STOP), g[s] + turn_cost, s, (x, y))
        for d in range(4):                                          # 몸 방향 그대로 d 방향으로 1칸
            c = (x + DXY[d][0], y + DXY[d][1])
            st = grid.state(c)
            if st == OCC or grid.blocked(c, t):
                continue
            rel = (d - f) % 4
            if face_goal and c == goal and rel != 0:
                continue
            step = 1.0 if rel == 0 else reverse_cost if rel == 2 else strafe_cost
            if m != STOP and m != d:
                step += change_cost
            if st == UNKNOWN:
                step += unknown_cost
            push((c[0], c[1], f, d), g[s] + step, s, c)
    return None, math.inf


class _BlockedView:
    """지정한 칸들을 막힌 칸으로 보는 지도 보기 (동점 깨기에서 '이 길이 막혔다면'을 계산할 때 씀)."""

    def __init__(self, grid: GridMap, cells: set):
        self.grid, self.cells = grid, cells

    def state(self, c: Cell) -> int:
        return OCC if c in self.cells else self.grid.state(c)

    def blocked(self, c: Cell, t: float) -> bool:
        return self.grid.blocked(c, t)


def count_turns(path: List[Cell], heading: int) -> int:
    turns, hd = 0, heading
    for a, b in zip(path, path[1:]):
        d = direction(a, b)
        turns += min((d - hd) % 4, (hd - d) % 4)
        hd = d
    return turns


# ── 임무 상태기계 ───────────────────────────────────────────────────
@dataclass
class Command:
    action: str                                   # MOVE | GRAB | SEARCH | WAIT | DONE
    phase: str                                    # TO_TARGET | GRAB | SEARCH | TO_HOME | DONE
    next_cell: Optional[Cell] = None
    next_xy_mm: Optional[Tuple[float, float]] = None
    next_heading: Optional[int] = None            # 이동하는 동안 몸이 바라볼 방향
    path: List[Cell] = field(default_factory=list)
    cost: float = math.inf
    move_dir: Optional[int] = None                # 움직이는 방향 (차동 구동이면 next_heading 과 같음)


class MissionPlanner:
    """사용법 (로봇 제어 루프에서 반복 호출):

        mp = MissionPlanner()                           # 메카넘 휠 (차동 구동이면 drive="diff")
        while True:
            cell, heading = 현재 셀, 몸이 바라보는 방향(N/E/S/W)   # (1,9) 남향 출발 기준 좌표
            obs = {셀: 장애물 여부, ...}                       # 센서로 본 칸들
            cmd = mp.update(cell, heading, obs, t=현재시각, moving=지금 움직이는 방향 또는 None)
            MOVE   → 몸을 cmd.next_heading 으로 돌린 뒤(이미 그쪽이면 그대로)
                     cmd.move_dir 방향으로 cmd.next_xy_mm (다음 칸 중심)까지 평행 이동
            GRAB   → 집기 동작 후 mp.grab_result(성공 여부)
            SEARCH → 큐브를 다시 찾아 mp.set_target(셀)
            WAIT   → 정지 후 다음 주기에 다시 update (임시 차단이 풀리면 경로가 다시 나옴)
            DONE   → 정지
    """

    def __init__(self, home: Cell = HOME, target: Cell = TARGET, turn_cost: float = 1.0,
                 unknown_cost: float = 0.5, grid: Optional[GridMap] = None, drive: str = "mecanum",
                 strafe_cost: float = 1.25, reverse_cost: float = 1.0, change_cost: float = 0.3,
                 tie_break_risk: bool = True, obstacle_prob: float = 0.3):
        self.home, self.target = home, target
        self.turn_cost, self.unknown_cost = turn_cost, unknown_cost
        self.drive = drive
        self.strafe_cost, self.reverse_cost, self.change_cost = strafe_cost, reverse_cost, change_cost
        self.tie_break_risk, self.obstacle_prob = tie_break_risk, obstacle_prob
        self.grid = grid if grid is not None else GridMap()
        self.phase = "TO_TARGET"

    # ── 메카넘 계획 + 동점 깨기 ──
    def _plan_mecanum(self, grid, cell, heading, goal, t, moving):
        return plan_path_mecanum(grid, cell, heading, goal, t, moving, self.turn_cost, self.strafe_cost,
                                 self.reverse_cost, self.change_cost, self.unknown_cost,
                                 face_goal=(self.phase == "TO_TARGET"))

    def _states_cost(self, states, moving, t):
        """상태 목록을 지금 지도로 다시 값 매김 (plan_path_mecanum 과 같은 비용)."""
        cost = 0.0
        for a, b in zip(states, states[1:]):
            if a[:2] == b[:2]:
                cost, moving = cost + self.turn_cost, None
                continue
            d = direction(a[:2], b[:2])
            rel = (d - b[2]) % 4
            cost += 1.0 if rel == 0 else self.reverse_cost if rel == 2 else self.strafe_cost
            if moving is not None and moving != d:
                cost += self.change_cost
            if self.grid.state(b[:2]) == UNKNOWN:
                cost += self.unknown_cost
            moving = d
        return cost

    def _risk_cost(self, states, cost, goal, t, moving):
        """막힐 위험까지 넣은 기대 비용과 경로 위 모르는 칸.
        q = 모르는 칸이 모두 뚫렸을 확률 = (1 - obstacle_prob)^개수
        기대 비용 = q × 경로 비용 + (1 - q) × (첫 모르는 칸 바로 앞까지 비용 + 거기서 그 칸들을 막고 다시 계획한 비용)"""
        cells = [st[:2] for i, st in enumerate(states) if i == 0 or st[:2] != states[i - 1][:2]]
        unk = [c for c in cells if self.grid.state(c) == UNKNOWN and c != goal]
        if not unk:
            return cost, unk
        k = next(i for i, st in enumerate(states) if st[:2] == unk[0])
        prefix = self._states_cost(states[:k], moving, t)
        r = states[k - 1]
        sf, cf = self._plan_mecanum(_BlockedView(self.grid, set(unk)), r[:2], r[2], goal, t, None)
        if sf is None:                           # 다 막히면 목표에 못 가므로 적어도 하나는 뚫려 있음
            return cost, unk
        q = (1.0 - self.obstacle_prob) ** len(unk)
        return q * cost + (1.0 - q) * (prefix + cf), unk

    def _break_tie(self, states, cost, cell, heading, goal, t, moving):
        """비용이 똑같은 다른 길이 있으면, 막혔을 때 손해가 적은(기대 비용이 작은) 쪽을 고른다.
        비용이 다르면 아무것도 바꾸지 않는다 — 어려운 배치 1,200회 시험에서 나빠진 경우 0 (docs/PLANNER_DETAILS.md 6.3.1)."""
        e1, unk = self._risk_cost(states, cost, goal, t, moving)
        if not unk:
            return states, cost
        s2, c2 = self._plan_mecanum(_BlockedView(self.grid, set(unk)), cell, heading, goal, t, moving)
        if s2 is None or c2 > cost + 1e-9:
            return states, cost
        e2, _ = self._risk_cost(s2, c2, goal, t, moving)
        return (s2, c2) if e2 < e1 - 1e-9 else (states, cost)

    def set_target(self, cell: Cell) -> None:
        """큐브 위치가 바뀌었을 때(상대가 밀었거나 SEARCH 후 발견) 호출."""
        self.target = cell
        if self.phase in ("TO_TARGET", "SEARCH"):
            self.phase = "TO_TARGET"

    def grab_result(self, success: bool) -> None:
        self.phase = "TO_HOME" if success else "SEARCH"

    def update(self, cell: Cell, heading: int, observations: Optional[Dict[Cell, bool]] = None,
               t: float = 0.0, dynamic: Optional[Iterable[Cell]] = None,
               moving: Optional[int] = None) -> Command:
        """heading: 몸이 바라보는 방향. observations: {셀: 장애물 여부}.
        dynamic: 상대 로봇이 보이는 셀들 (임시 차단). moving: 지금 움직이는 방향 (서 있으면 None, 메카넘만 씀)."""
        for c in dynamic or ():
            if c != cell and c != self.target:
                self.grid.block(c, t + self.grid.dyn_ttl)
        for c, occ in (observations or {}).items():
            self.grid.observe(c, occ, t)
        self.grid.observe(cell, False, t)            # 지금 서 있는 칸은 비어 있다

        if self.phase == "TO_TARGET" and cell == self.target:
            self.phase = "GRAB"
        if self.phase == "TO_HOME" and cell == self.home:
            self.phase = "DONE"
        if self.phase in ("GRAB", "SEARCH", "DONE"):
            return Command(self.phase, self.phase)

        goal = self.target if self.phase == "TO_TARGET" else self.home
        if self.drive == "diff":
            path, cost = plan_path(self.grid, cell, heading, goal, t, self.turn_cost, self.unknown_cost)
            if path is None:
                return Command("WAIT", self.phase)
            nxt = path[1]
            d = direction(cell, nxt)
            return Command("MOVE", self.phase, nxt, cell_center_mm(nxt), d, path, cost, d)

        states, cost = self._plan_mecanum(self.grid, cell, heading, goal, t, moving)
        if states is None:
            return Command("WAIT", self.phase)
        if self.tie_break_risk:
            states, cost = self._break_tie(states, cost, cell, heading, goal, t, moving)
        path = [cell]
        for s in states[1:]:
            if (s[0], s[1]) != path[-1]:
                path.append((s[0], s[1]))
        k = next(i for i, s in enumerate(states) if (s[0], s[1]) != cell)   # 첫 이동 (그 전은 제자리 회전)
        nxt = (states[k][0], states[k][1])
        return Command("MOVE", self.phase, nxt, cell_center_mm(nxt), states[k][2], path, cost,
                       direction(cell, nxt))
