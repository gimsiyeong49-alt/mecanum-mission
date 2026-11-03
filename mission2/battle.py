"""2임무(배틀럼블) 판단·주행. ROS 를 모른다: 로봇에선 battle_node.RosIO, 시험에선 시뮬의 SimIO 가 같은 함수를 채운다.

좌표: 경기장 중심이 원점, +x = 출발 때 정면, +y = 출발 때 왼쪽. 네 출발 칸 모두 출발 자세가 (-1.2, -1.2, 0°) 라
어느 칸에서 출발하는지 몰라도 된다 (규정 그림 3: 어디서든 중심은 왼쪽 앞 45°).

순서: 출발 → 큐브 앞 450 mm → 잡기(잡기 스테이션) → 쥠 확인(200 mm 후진) → 중앙 유지
      못 잡으면 3×3 순회 → 중앙에서 보이면 다시 잡기, 다른 곳에서 보이면 추격·밀기
모든 움직임은 cmd() 의 안전선을 마지막으로 지난다. 경기 중엔 어떤 실패에도 임무를 끝내지 않고 다음 행동으로 넘어간다.
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np


LIDAR_X = 0.095                    # 바퀴 가운데 → 라이다 (battle_node LASER)


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
    grab_lim: float = 1.2         # 쥘 때 바퀴 가운데가 이 안일 큐브만 잡으러 감
    chase_lim: float = 1.22       # 밀기는 이 안에서만 (바퀴 가운데, 평소 안전선 1.20 보다 바깥도 됨). 11-03 사용자: 확실히 밀게 1.16 → 1.19 → 1.22. 옆면이 선 5.8 cm 안, 뒷면 1.9 cm 안, 비스듬하면 모서리가 선 약 4.6 cm 넘음 (가운데는 선 18 cm 안)
    stop_t: float = 0.25          # 선 쪽 속도 제한: 남은 거리 / 이 시간 (멈춤 명령 뒤 굴러가는 시간 실측 0.13~0.20 s)
    grip_x: float = 0.191         # 쥘 때 바퀴 가운데 → 큐브 가운데 (cube_station --grip-x)
    grab_fast: bool = False       # 첫 잡기를 스테이션 grabfast 로 (11-03 실기: 4 판 중 2 판 큐브를 밀어 놓침 → 꺼 둠)
    aim: bool = True              # 출발: 큐브가 보이기 시작하면 카메라 값으로 도착 자리를 고침 (멈춰서 옆맞춤하는 시간을 줄임)
    aim_near: float = 0.25        # 출발 때 중심에서 이 안의 큐브만 믿음 (다른 빨간 것에 끌려가지 않게)
    grip_left: float = 0.010      # 쥘 때 큐브가 몸 가운데에서 왼쪽으로 있을 자리 (cube_station --grip-left)
    stance: float = 0.45          # 잡기 시작 자리: 바퀴 가운데 → 큐브 가운데. 스테이션 한계(0.58 m) 안, 두 카메라에 다 보임(0.34 m 이상)
    ring: float = 0.48            # 3×3 순회 반경 (1 칸 + 8 cm): 두 카메라에 큐브 전체가 보이려면 0.34 m 이상, 큐브가 조금 다가와 있어도 보이게. 스테이션 한계 0.58 m 안
    patrol_mode: str = "alt"      # 순회: alt 멈춤 없이 돌며 한 바퀴씩 중심·바깥을 봄 (시뮬 11-03 끝쥠 37/50, 옛판 34) / in 늘 중심 (29) / fixed 몸 방향 고정 (26) / stop 변 가운데서 멈춰 돌아봄 (옛판)
    look_every: float = 5.0       # 순회 중 이 간격이 지난 뒤 다음 변 중앙에서 중심을 봄 [s]
    center_r: float = 0.35        # (cube_rule old) 큐브가 중심에서 이 안이면 '제자리 큐브' (재배치는 심판이 주변에 놓을 수도 있음, 위치 오차 포함)
    cube_rule: str = "prio"       # prio = 11-03 21시 사용자 우선순위 (1 주변 비면 잡기·2 번 실패면 20 초 공격, 2 주변에 점 있으면 20 초 공격, 3 큐브 안 보이면 라이다 물체 공격) / 예전: (11-03 20:10 사용자 결정. 시뮬 중앙칸규칙비교: cell 81/120·선 9, old 97/120·선 3 — 상대가 쳐서 밀려난 혼자 큐브까지 쳐냄) cell = 중앙 칸에 있는 큐브만 잡고, 칸 밖에 보이면 누가 쥔 것으로 보고 공격 (11-03 사용자) / old = 예전 (서 있으면 어디든 잡음)
    center_half: float = 0.25     # 중앙 칸 (±0.20) + 위치 오차 5 cm [m]. 라이다 벽 보정 위치 기준
    near_free_r: float = 0.0      # (0 = 안 씀, 사용자: 중앙 칸 큐브만 잡음) 칸 밖이라도 중심에서 이 안에 혼자 서 있는 큐브는 잡음: 중앙에 로봇이 있으면 심판이 주변에 놓음 (3.4.6) [m]
    fail_limit: int = 2           # prio: 잡기 이만큼 실패하면 attack_time 동안 공격
    attack_time: float = 30.0     # prio: 공격 시간 [s] (11-03 21시 사용자: 20 → 30)
    grab_first_t: float = 12.0    # prio: 출발 뒤 이 시간 동안은 공격 안 하고 잡기만 (11-03 21시 사용자 20 → 21:15 12) [s]
    free_wait: float = 20.0       # 잡기 실패 뒤 이 시간 동안은 위 예외 안 씀 (같은 큐브에 매달리지 않게) [s]
    back_check: float = 0.20      # 쥠 확인 후진 거리. 쥔 큐브는 바퀴 가운데 앞 19.1 cm 라 안 보이고, 놓친 큐브는 39 cm 라 두 카메라에 보임
    grab_tries: int = 1           # 한 번에 잡기 시도 횟수 (11-03 사용자: 실패하면 바로 순회, 중앙 칸에 있으면 그때 다시). 2 = 헛잡기면 바로 한 번 더 (사용자 안: 1 + attack_win 25 = 1 번 해 보고 안 되면 공격, 시뮬 11-03 공격모드비교 에선 나빴음 → 결정 대기)
    attack_win: float = 0.0       # (0 = 끔) 잡기 실패 뒤 이 시간 동안은 큐브가 보여도 잡지 않고 공격 (라이다로 보이면 옆밀기, 안 보이면 큐브 쪽으로 돌진) [s]
    grab_timeout: float = 40.0
    chase_time: float = 12.0      # 추격 최대 시간 [s]
    chase_lost: float = 1.0       # 큐브가 이만큼 안 보이면 놓친 것 [s]
    push_time: float = 16.0       # 상대 밀어내기 최대 시간 [s] (11-03 21시 사용자: 밀다가 맒 → 12 → 16) (11-03 실기: 20 초 밀다 복귀 → 줄임. 시뮬 밀기멈춤비교2: 3 초·12 초+안피함 77/80, 1.5 초·8 초 72/80, 예전 70/80)
    push_stop: bool = True        # 밀기·돌진은 밀기 선에 닿거나 stall_t 동안 안 밀리면 멈춤 (11-03)
    stall_t: float = 4.0          # 이 시간 동안 5 cm 도 못 나가면 더 안 밀리는 것 [s] (11-03 21시: 3 → 4) (1.5 는 쥔 상대를 밀다 너무 일찍 그만둠: 시뮬 느린셋 20 → 13)
    patrol_watch: bool = False    # 순회 중 라이다로 앞이 막혀도 피하지 않고 밀고 감. 못 나가면(stuck_t) 그때 방향 반대로 (11-03 사용자: 같은 자리 13 초 왕복)
    push_r: float = 0.35          # 겨냥한 자리에서 이 안의 라이다 점 = 그 상대 (로봇 지름 40 cm 안)
    chase_push: float = 4.0       # 가까이서 안 보이게 된 뒤 더 밀어붙이는 시간 [s]
    still_t: float = 1.5          # 큐브가 이만큼 그 자리에 있으면 멈춘 큐브 → 잡으러 감 [s]
    hold_r: float = 0.43          # 버티는 자리: 십자 칸. (5,5) 는 큐브가 돌아올 자리라 비워 둠 (3.4.6). 바퀴 가운데 기준 뒤 16.1, 쥔 큐브 앞끝 21.6 cm 라 0.416~0.439 면 몸이 칸 안
    hold_dead: float = 0.10       # 중앙 유지: 이 안의 밀림엔 반응 안 함 [m]
    dodge: bool = False           # 쥔 채 상대가 계속 붙어 밀면 다른 십자 칸으로 옮김 (시험 중)
    dodge_mode: str = "cell"      # 옮길 곳: cell 미는 쪽 반대의 십자 칸 / free 라이다로 가장 빈 쪽으로 빠져나간 뒤 비어 있는 십자 칸
    attack: bool = True           # 빈손일 때 움직이는 큐브로 밀고 들어가기·큐브 옆 상대 밀기. 끄면 멈춘 큐브만 잡으러 감
    dodge_after: float = 1.5      # 몸에 이만큼 계속 붙어 있으면 옮김 [s]
    dodge_gap: float = 3.0        # 옮긴 뒤 다시 옮기기까지 [s]
    hunt: bool = False            # 빈손 순회 중 노란 선 안의 라이다 덩어리(상대)를 밀러 감 (시험 중). 큐브를 쥔 상대가 탈락하면 큐브가 돌아옴 (3.4.6)
    hunt_r: float = 1.60          # 이 거리 안의 상대만 [m] (늘 공격: 순회 고리에서 경기장 거의 전부)
    hunt_any: bool = True         # 늘 공격: 서 있는 로봇도, 순회 한 바퀴를 안 기다리고 바로 (11-03 사용자). False = 한 바퀴 뒤 움직이는 로봇만
    edge_margin: float = 0.10     # 노란 선에서 이만큼 바깥까지 걸친 로봇·큐브도 목표로 봄 (11-03 22시: 걸친 로봇이 탈락 안 됨) [m]
    push_pwm: int = 255           # 밀 때만 펌웨어 출력 상한 (평소 경기 중 228). 막혀서 바퀴가 못 돌면 PI 가 이 상한까지 올림 → 미는 힘 약 12% 더 (11-03 22시 사용자)
    push_v: float = 0.40          # 밀 때 속도 명령 [m/s] (평소 max_v 0.32, 출발 start_v 0.45 @255 는 실기에서 씀)
    push_face: str = "line"       # 미는 면: line = 상대를 가장 가까운 노란 선 쪽으로 방향 정하고 메카넘으로 옆으로 따라가며 우리 앞면으로 (11-03 22시 사용자: 저번처럼 따라가며) / front = 자세 안 잡고 바로 상대 가운데로 앞면 돌진 (11-03 22시 사용자: 옆·뒤는 자세 잡는 데 오래 걸림) / side = 우리 옆면 → 안 밀리면 뒷면
    push_retry: int = 1           # 앞면으로 안 밀리면(stall_t) 같은 방향 그대로 차체를 180° 돌려 우리 뒷면으로 다시 밂 (11-03 사용자). 0 = 안 함
    hunt_gap: float = 3.0         # 밀기 끝나고 다시 찾기까지 [s]
    push_mode: str = "side"       # 밀기 방식 (시뮬 11-03 옆밀기비교c: 끝쥠 50/80 대 돌진 45/80, 선 넘은 판 2 대 18): side = 라이다로 상대 몸을 따라가며 옆(안 되면 뒤)에서 선 쪽으로 밂 (11-03 사용자: 앞은 상대 바퀴 힘을 정면으로 받음) / front = 큐브·상대로 곧장 돌진 (예전)
    hunt_dyn: bool = True         # (11-03 21시 사용자: 늘 공격. 예전 시뮬은 이득 없었음) 빈손 순회를 한 바퀴 돌아도 중앙 큐브가 없으면, 라이다로 움직이는 로봇을 찾아 옆·뒤에서 밂 (push_mode side 와 같이)
    dyn_v: float = 0.06           # 라이다 덩어리가 이 속도 넘게 움직이면 '움직이는 로봇' [m/s]
    seg_gap: float = 0.10         # 라이다 점 사이가 이보다 벌어지면 다른 덩어리 [m] (거리로 묶기는 붙어 있는 여러 대를 하나로 봐서 시뮬 느린셋 19 → 7, 되돌림)
    opp_max: float = 0.50         # 로봇으로 볼 덩어리 폭 상한 (지름 40 cm 안, 규정 2.2.1) [m]
    opp_out: float = 0.15         # 노란 선 밖 이만큼까지는 상대로 봄 (밀어내는 끝까지 따라감) [m]
    opp_r_in: float = 0.10        # 보이는 점 가운데 → 몸 가운데 (가정) [m]
    opp_r: float = 0.17           # 상대 반지름 (밀 자리 계산, 가정) [m]
    rear_pen: float = 0.25        # 뒤에서 밀기 벌점: 옆을 먼저 [m]
    hold_near: float = 0.03       # 상대가 몸에 붙어 있을 때는 이만큼만 밀려도 바로 되밂 (밀리고 나면 되돌리기 어려움)
    wheel_lock: bool = False      # 중앙 유지 때 바퀴 잠금 (firmware/mecanum_grip_hold 를 구운 뒤에만)
    hold_tol: float = 0.015       # 칸 안 여유가 약 1 cm 라 작게 (11-03: 0.04 면 3 cm 앞에 서서 큐브가 가운데 칸에 걸침)
    hold_seen_t: float = 2.0      # 쥔 뒤 앞쪽에 큐브가 이만큼 계속 보이면 헛잡기로 봄 [s]
    hold_yaw_dead: float = math.radians(10.0)
    start_pwm: int = 255          # 출발해 큐브 앞까지만 펌웨어 출력 상한. 짧아서 열 부담이 작음. 그 뒤는 평소 값 (--pwm 228)
    start_v: float = 0.45         # 출발해 큐브 앞까지만 (펌웨어 출력 255 와 함께, 11-03 실기 7.3 → 5.4 초). 그 밖의 움직임은 max_v
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
        self.no_evade = False
        self.rim_now = 0.0                   # 지금 go 의 속도 상한이 max_v 보다 크면 바퀴 상한도 그만큼
        self.hunt_t = -99.0
        self.hold_saw = None                 # 버티다 앞에서 본 큐브 (순회 없이 바로 잡으러)
        self.state = "준비"
        self.orbit = False             # 순회 중: 옆+회전도 바퀴 상한까지 (자리가 조금 틀어져도 됨)
        self.turn_dir = 1              # 순회 방향: +1 반시계, -1 시계
        self.tracks, self.trk_t, self.trk_n = [], -9.0, 0     # 라이다로 따라가는 상대 로봇 후보
        self.hunt_id = None            # 순회 중 밀러 갈 상대 번호
        self.lap_done = False          # 순회를 한 바퀴 돌았는데 중앙 큐브를 못 봄 (누가 쥐고 있음)
        self.fail_t = -99.0            # 마지막 잡기 실패 시각 (attack_win 동안 공격)
        self.hunt_skip = {}            # 밀 자리가 없던 상대 번호 → 시각 (10 초 건너뜀)
        self.fails, self.attack_until = 0, -99.0     # prio: 잡기 실패 수, 공격 끝 시각
        self.stall_p, self.stall_t0 = (self.x, self.y), 0.0     # 밀기 '안 밀림' 판단 기준점

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
        if self.c.push_mode == "side" or self.c.hunt_dyn:
            self.update_tracks()

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
        if self.no_evade:
            return vx, vy, w
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
        rim = self.c.rim_turn if abs(by) > abs(bx) and abs(w) >= 0.1 and not self.orbit else max(self.c.max_v, self.rim_now)
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
    def go(self, *args, **kw):
        try:
            return self._go(*args, **kw)
        finally:
            self.rim_now = 0.0

    def _go(self, X, Y, yaw=None, vmax=None, tol=0.03, timeout=12.0, until=None, watch=False, kp=None, retarget=None, face=None):
        """(X, Y) 로 곧게, 몸 방향은 yaw 유지. retarget() 이 자리를 주면 그쪽으로 고침. 돌려줌: ok / until / blocked / stuck / timeout.
        face = +1 중심을 봄, -1 바깥을 봄 (순회): 가면서 몸도 돌리고, 도착해도 안 세움."""
        c = self.c
        vmax = vmax or c.max_v
        self.rim_now = vmax
        self.tick()
        yaw_t = self.yaw if yaw is None else yaw
        t0 = self.io.now()
        best, t_best = 9.0, t0
        while True:
            self.tick()
            p = retarget() if retarget is not None else None
            if p is not None:
                X, Y = p
            ex, ey = X - self.x, Y - self.y
            d = math.hypot(ex, ey)
            now = self.io.now()
            if d < max(tol, self.v_meas * c.brake_tb):
                if face is None:
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
            if face is not None:
                ux, uy = ex / d, ey / d
                g = (self.x * uy - self.y * ux) / max(self.x ** 2 + self.y ** 2, 0.04)       # 단위 속도당 몸이 돌아야 할 양. 뒤따라 돌면 시야(가운데에서 약 ±11°)를 벗어남
                v = c.max_v / (abs(cs * ux + sn * uy) + abs(-sn * ux + cs * uy) + c.wheel_l * abs(g))
                w = clip(g * v + c.kw * wrap(math.atan2(-face * self.y, -face * self.x) - self.yaw), -c.max_w, c.max_w)
            self.cmd(ex / d * v, ey / d * v, w)
            self.io.sleep(0.05)

    def turn_to(self, yaw_t, tol=math.radians(2.0), timeout=8.0, settle=True):
        """제자리 회전, 밀린 자리는 되밀며. 돌려줌: True / False (시간 초과)."""
        c = self.c
        self.tick()
        x0, y0 = self.x, self.y
        t0 = self.io.now()
        prev, w_meas = (self.yaw, t0), 0.0
        tries = 0
        while True:
            self.tick()
            now = self.io.now()
            if now - prev[1] >= 0.05:
                w_meas = 0.5 * w_meas + 0.5 * wrap(self.yaw - prev[0]) / (now - prev[1])
                prev = (self.yaw, now)
            e = wrap(yaw_t - self.yaw)
            if abs(e) < max(tol, abs(w_meas) * 0.12):
                self.stop()
                if not settle:                       # 버티는 중: 맞추느라 멈춰 있을 틈이 없음
                    return True
                self.wait(0.2)                       # 멈춘 뒤에 다시 봄: 돌던 힘으로 6~9° 더 가거나 덜 멈춤 (11-03 순회 실기). 어긋났으면 천천히 마저 돎
                self.tick()
                e = wrap(yaw_t - self.yaw)
                tries += 1
                if abs(e) <= max(tol, math.radians(3.0)) or tries >= 3:
                    return True
                prev, w_meas = (self.yaw, self.io.now()), 0.0
                now = self.io.now()
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
    def grab_here(self, fast=False):
        """스테이션에 맡기고(그동안 바퀴 명령 안 냄), 쥠 확인까지. 돌려줌: held / again (헛잡기) / lost (못 잡음)."""
        self.set_state("잡기")
        self.stop()
        self.io.grab_start(fast)
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
        if self.io.went_in() is False:                       # 쥔 큐브는 화면 아래로 빠져 들어옴. 그 모습 없이 앞에도 안 보이면 누가 가져간 것
            self.log("큐브가 그리퍼로 들어오는 걸 못 봄 → 빈손, 순회로")
            return "lost"
        self.log("큐브 안 보임 → 쥠 성공")
        if self.held_at is None:
            self.held_at = self.io.now() - self.t0
        return "held"

    def grab_loop(self):
        for i in range(self.c.grab_tries):
            r = self.grab_here(fast=self.c.grab_fast and i == 0)
            if r != "again":
                return r == "held"
        return False

    def approach(self, cx, cy):
        """멈춘 큐브 앞 stance 에 정면으로 선다. 안전선 밖이 되는 자리면 False."""
        self.tick()
        b = math.atan2(cy - self.y, cx - self.x)
        X, Y = cx - self.c.stance * math.cos(b), cy - self.c.stance * math.sin(b)
        gx, gy = cx - self.c.grip_x * math.cos(b), cy - self.c.grip_x * math.sin(b)      # 쥘 때 바퀴 가운데 자리
        if max(abs(X), abs(Y), abs(gx), abs(gy)) > min(self.lim, self.c.grab_lim):        # 스테이션이 앞으로 들어가는 동작엔 안전선이 안 걸림
            self.log("큐브 (%+.2f, %+.2f) 를 잡으면 안전선 밖 → 안 감" % (cx, cy))
            return False
        self.set_state("큐브 앞으로")
        self.turn_to(b)
        return self.go(X, Y, yaw=b, timeout=8.0) == "ok"

    # ── 임무 단계 ──
    def start_run(self):
        self.set_state("출발")
        self.io.gripper_open()
        self.io.set_pwm(self.c.start_pwm)
        if self.c.start_mode == "turn45":
            self.turn_to(math.pi / 4)
            k = self.c.stance * math.sqrt(0.5)
            r = self.go(-k, -k, yaw=math.pi / 4)
        else:
            def aim():
                q = self.cube_arena()
                if q is None or self.seen < 3 or math.hypot(q[0], q[1]) > self.c.aim_near:
                    return None
                return q[0] - self.c.stance, q[1] - self.c.grip_left
            r = self.go(-self.c.stance, 0.0, yaw=0.0, vmax=self.c.start_v, retarget=aim if self.c.aim else None)
        self.io.set_pwm(None)
        self.log("큐브 앞 도착: %s" % r)

    def hold(self):
        """중앙 유지: 경기 끝까지. 작은 밀림엔 반응 안 함. 밀리면 세게 되밀어 복귀 (천천히 돌아오면 미는 상대를 못 이김)."""
        self.set_state("중앙 유지")
        self.tick()
        # 가까운 십자 칸에서 (5,5) 를 보고 섬. 쥐었는지 맞히지 않아도 (5,5) 에 큐브가 나타나면 내 손에 없는 것. 승패는 끝날 때 쥐었는지만 봄 (3.4.7.1)
        hx, hy = (math.copysign(self.c.hold_r, self.x), 0.0) if abs(self.x) >= abs(self.y) else (0.0, math.copysign(self.c.hold_r, self.y))
        yaw_h = math.atan2(-hy, -hx)
        self.go(hx, hy, yaw=self.yaw, tol=self.c.hold_tol)
        self.turn_to(yaw_h)
        seen_t = None
        contact_t, dodge_t = None, -99.0

        def pressed():                                       # 되미는 중에도 오래 눌리면 끊고 옮김
            nonlocal contact_t
            ct, now = self.contact(), self.io.now()
            if ct is None:
                contact_t = None
                return False
            contact_t = now if contact_t is None else contact_t
            return now - contact_t >= self.c.dodge_after and now - dodge_t >= self.c.dodge_gap

        while True:
            self.tick()
            # 쥠 확인 때 상대에 가려 못 본 헛잡기: 큐브는 하나뿐이라 몸 앞 30 cm 넘는 곳에 계속 보이면 쥔 게 아님
            c = self.cube_arena()
            if c is not None and math.hypot(c[0] - self.x, c[1] - self.y) > 0.30:
                seen_t = seen_t if seen_t is not None else self.io.now()
                if self.io.now() - seen_t > self.c.hold_seen_t:
                    self.log("쥐고 있다고 봤는데 큐브가 (%+.2f, %+.2f) 에 보임 → 다시 잡으러" % c)
                    self.held_at, self.hold_saw = None, c
                    if self.c.wheel_lock:
                        self.io.wheel_lock(False)
                    return
            elif c is None and seen_t is not None and self.io.now() - seen_t > self.c.hold_seen_t + 0.5:
                seen_t = None
            if self.c.wheel_lock:
                self.io.wheel_lock(True)
            ct, now = self.contact(), self.io.now()
            if self.c.dodge:
                if ct is None:
                    contact_t = None
                elif contact_t is None:
                    contact_t = now
                elif now - contact_t >= self.c.dodge_after and now - dodge_t >= self.c.dodge_gap:
                    hx, hy = self.dodge(ct, hx, hy)
                    yaw_h = math.atan2(-hy, -hx)
                    contact_t, dodge_t = None, self.io.now()
                    continue
            d = math.hypot(self.x - hx, self.y - hy)
            yaw_e = wrap(yaw_h - self.yaw)
            if d > (self.c.hold_near if ct is not None else self.c.hold_dead):
                self.log("밀림 %.0f cm, 방향 %+.0f° → 복귀" % (100 * d, math.degrees(yaw_e)))
                r = self.go(hx, hy, yaw=yaw_h, tol=0.04, timeout=8.0, kp=3.0, until=pressed if self.c.dodge else None)     # 밀릴 때는 빨리 되미는 게 먼저, 칸에 딱 맞추는 건 처음 설 때만
                if r != "ok" and math.hypot(self.x - hx, self.y - hy) < 2 * self.c.hold_dead:     # 자리 근처에서 막힘: 모터를 잠깐 쉼. 멀면 안 쉼
                    self.wait(1.0)
            elif abs(yaw_e) > self.c.hold_yaw_dead:
                self.turn_to(yaw_h, tol=math.radians(5.0), timeout=3.0, settle=False)
            else:
                self.cmd(0.0, 0.0, 0.0)
                self.io.sleep(0.05)

    @staticmethod
    def snap(a):
        """축에서 15° 안이면 축으로. 순회는 경기장 축을 따라가니 몸도 벽과 나란히 (자리 오차로 비스듬히 돌지 않게, 벽 보정도 ±5° 안에서만 됨)."""
        ax = round(a / (math.pi / 2)) * (math.pi / 2)
        return ax if abs(wrap(a - ax)) < math.radians(15.0) else a

    def free_way(self):
        """라이다로 몸이 지나갈 만큼 가장 빈 방향의 목표점 (경기장 좌표). 같으면 가운데 쪽을. 안전선 밖으로는 안 고름. 스캔이 없으면 None."""
        pts = self.io.scan()
        if pts is None or len(pts) == 0:
            return None
        c = self.c
        px, py = pts[:, 0], pts[:, 1]
        keep = ~((px > -c.back - 0.01) & (px < c.front + 0.03) & (np.abs(py) < c.side + 0.01))
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = cs * px[keep] - sn * py[keep], sn * px[keep] + cs * py[keep]
        best = None
        for k in range(16):
            a = k * math.pi / 8
            ux, uy = math.cos(a), math.sin(a)
            along, cross = ax * ux + ay * uy, np.abs(-ax * uy + ay * ux)
            ahead = along[(along > 0.0) & (cross < c.side + 0.08)]
            free = float(np.min(ahead)) - c.front if len(ahead) else 9.0
            room = min((self.lim - math.copysign(p, u)) / max(abs(u), 1e-6) if abs(u) > 1e-6 else 9.0 for p, u in ((self.x, ux), (self.y, uy)))
            reach = min(free, room, 0.60)
            score = reach + 0.5 * (self.lim - max(abs(self.x + ux * reach), abs(self.y + uy * reach)))      # 빈 쪽이라도 선에 가까워지면 깎음 (바깥으로 빠지면 몰려서 탈락)
            if best is None or score > best[0]:
                best = (score, reach, ux, uy)
        _, reach, ux, uy = best
        return (self.x + ux * reach, self.y + uy * reach) if reach > 0.05 else None

    def dodge(self, ct, hx, hy):
        """쥔 채 계속 밀리면 비킴. 몸 방향은 그대로, 맞서기는 잠시 끔. 돌려줌: 새 자리.
        cell: 미는 쪽 반대의 다른 십자 칸. free: 빈 쪽으로 먼저 빠져나간 뒤 아무도 없는 가장 가까운 십자 칸."""
        r = self.c.hold_r
        spots = [p for p in ((r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)) if p != (hx, hy)]
        if self.c.dodge_mode == "free":
            q = self.free_way()
            self.no_evade = True
            try:
                if q is not None:
                    self.log("계속 밀림 → 빈 쪽 (%+.2f, %+.2f) 으로" % q)
                    self.go(q[0], q[1], yaw=self.yaw, tol=0.06, timeout=3.0, kp=3.0)
            finally:
                self.no_evade = False
            self.tick()
            spots = sorted(((r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)), key=lambda p: math.hypot(p[0] - self.x, p[1] - self.y))
            return next((p for p in spots if not self.someone_near(p[0], p[1], 0.30)), spots[0])
        X, Y = max(spots, key=lambda p: -(p[0] - self.x) * ct[0] - (p[1] - self.y) * ct[1])
        self.log("계속 밀림 → 다른 십자 칸 (%+.2f, %+.2f) 으로" % (X, Y))
        self.no_evade = True
        try:
            self.go(X, Y, yaw=self.yaw, tol=0.06, timeout=4.0, kp=3.0)
        finally:
            self.no_evade = False
        return X, Y

    def nearest_robot(self):
        """노란 선 안에서 가장 가까운 라이다 덩어리의 경기장 좌표. 큐브(5 cm)는 라이다 높이 아래라 안 찍힘. 없으면 None."""
        pts = self.io.scan()
        if pts is None or len(pts) == 0:
            return None
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = self.x + cs * pts[:, 0] - sn * pts[:, 1], self.y + sn * pts[:, 0] + cs * pts[:, 1]
        dist = np.hypot(ax - self.x, ay - self.y)
        m = (np.abs(ax) < self.c.line) & (np.abs(ay) < self.c.line) & (dist > 0.25) & (dist < self.c.hunt_r)
        if np.count_nonzero(m) < 5:
            return None
        ax, ay, dist = ax[m], ay[m], dist[m]
        i = int(np.argmin(dist))
        near = np.hypot(ax - ax[i], ay - ay[i]) < 0.20
        return (float(np.mean(ax[near])), float(np.mean(ay[near]))) if np.count_nonzero(near) >= 4 else None

    # ── 상대 로봇 따라가기·옆에서 밀기 (11-03 사용자: 옆·뒤로 밀되 라이다로 감지) ──
    def scan_robots(self):
        """라이다 점을 각도 순으로 이어 덩어리로 나눔 → 노란 선 근처 안쪽의 로봇 크기 덩어리마다 몸 가운데(경기장 좌표).
        벽은 선 밖이라 빠지고 큐브(5 cm)는 라이다 높이 아래라 안 찍힘. 보이는 면 가운데에서 라이다 반대쪽으로 opp_r_in 만큼 옮김."""
        pts = self.io.scan()
        if pts is None or len(pts) < 5:
            return []
        c = self.c
        px, py = pts[:, 0], pts[:, 1]
        keep = ~((px > -c.back - 0.01) & (px < c.front + 0.03) & (np.abs(py) < c.side + 0.01))      # 몸에 맞은 점만 뺌 (붙은 상대는 남김, contact() 와 같은 몸)
        px, py = px[keep], py[keep]
        if len(px) < 4:
            return []
        o = np.argsort(np.arctan2(py, px - LIDAR_X))
        px, py = px[o], py[o]
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = self.x + cs * px - sn * py, self.y + sn * px + cs * py
        inside = (np.abs(ax) < c.line + c.opp_out) & (np.abs(ay) < c.line + c.opp_out)
        lx, ly = self.x + cs * LIDAR_X, self.y + sn * LIDAR_X
        cut = np.ones(len(ax), bool)
        cut[1:] = (np.hypot(np.diff(ax), np.diff(ay)) > c.seg_gap) | ~inside[1:] | ~inside[:-1]
        starts = np.nonzero(cut)[0]
        ends = np.append(starts[1:], len(ax))
        dets = []
        for s0, e0 in zip(starts, ends):
            if not inside[s0] or e0 - s0 < 3:
                continue
            sx, sy = ax[s0:e0], ay[s0:e0]
            span = math.hypot(sx[-1] - sx[0], sy[-1] - sy[0])
            if span > c.opp_max:                         # 작은 덩어리도 로봇 (11-03 실기: 상대가 라이다 높이에서 3~4 cm 에 점 7~11 개로만 찍힘)
                continue
            mx, my = float(np.mean(sx)), float(np.mean(sy))
            dx, dy = mx - lx, my - ly
            n = math.hypot(dx, dy)
            if n > 1e-6:
                dets.append([mx + dx / n * c.opp_r_in, my + dy / n * c.opp_r_in, e0 - s0])
        merged = []                                          # 뒤쪽(±180°)에서 둘로 잘린 덩어리 합침
        for d in sorted(dets, key=lambda q: -q[2]):
            m = next((q for q in merged if math.hypot(q[0] - d[0], q[1] - d[1]) < 0.18), None)
            if m is None:
                merged.append(d)
            else:
                w = m[2] + d[2]
                m[0], m[1], m[2] = (m[0] * m[2] + d[0] * d[2]) / w, (m[1] * m[2] + d[1] * d[2]) / w, w
        return [(q[0], q[1]) for q in merged]

    def update_tracks(self):
        """라이다 덩어리를 이전 것과 이어 붙여 상대마다 위치·속도·가는 방향을 계속 셈 (초당 약 14 번)."""
        now = self.io.now()
        if now - self.trk_t < 0.07:
            return self.tracks
        self.trk_t = now
        dets = self.scan_robots()
        free = list(range(len(dets)))
        for tr in sorted(self.tracks, key=lambda t: -t["t"]):
            if not free:
                break
            j = min(free, key=lambda k: math.hypot(dets[k][0] - tr["x"], dets[k][1] - tr["y"]))
            if math.hypot(dets[j][0] - tr["x"], dets[j][1] - tr["y"]) < 0.25 + 0.4 * (now - tr["t"]):
                free.remove(j)
                tr["x"], tr["y"], tr["t"] = dets[j][0], dets[j][1], now
                tr["hist"].append((now, tr["x"], tr["y"]))
        for j in free:
            self.tracks.append(dict(id=self.trk_n, x=dets[j][0], y=dets[j][1], t=now, hist=deque([(now, dets[j][0], dets[j][1])], maxlen=30), v=0.0, h=None))
            self.trk_n += 1
        self.tracks = [t for t in self.tracks if now - t["t"] < 1.0]
        for tr in self.tracks:
            hs = [p for p in tr["hist"] if now - p[0] <= 1.2]
            if len(hs) >= 5 and hs[-1][0] - hs[0][0] >= 0.5:
                T = np.array([p[0] for p in hs])
                T = T - T.mean()
                X, Y = np.array([p[1] for p in hs]), np.array([p[2] for p in hs])
                k = float(np.dot(T, T))
                vx, vy = float(np.dot(T, X - X.mean())) / k, float(np.dot(T, Y - Y.mean())) / k
                tr["v"] = math.hypot(vx, vy)
                if tr["v"] > self.c.dyn_v:
                    tr["h"] = math.atan2(vy, vx)             # 가는 쪽 = 상대 앞 (가정). 멈추면 마지막 값 유지
        return self.tracks

    def tracks_desc(self, x, y):
        """기록용: (x, y) 에서 가까운 상대 후보 셋 (번호, 거리, 속도, 마지막으로 본 지 몇 초)."""
        now = self.io.now()
        ts = sorted(self.tracks, key=lambda t: math.hypot(t["x"] - x, t["y"] - y))[:3]
        return ", ".join("#%d %.2f m %.2f m/s %.1f s 전" % (t["id"], math.hypot(t["x"] - x, t["y"] - y), t["v"], now - t["t"]) for t in ts) or "라이다 상대 후보 없음"

    def contested(self, cx, cy):
        """큐브를 다른 로봇이 쥐거나 가리고 있나: 큐브 35 cm 안 로봇, 또는 우리 → 큐브 길(폭 ±25 cm)에 선 로봇. 있으면 그 번호.
        11-03 실기: 터틀봇이 큐브를 가리면 들어가다 멈추고 덜 잡음."""
        tid = self.track_near(cx, cy, r=0.35)
        if tid is not None:
            return tid
        now = self.io.now()
        dx, dy = cx - self.x, cy - self.y
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return None
        ux, uy = dx / L, dy / L
        for t in self.tracks:
            if now - t["t"] > 1.0:
                continue
            ex, ey = t["x"] - self.x, t["y"] - self.y
            along = ex * ux + ey * uy
            if 0.15 < along < L + 0.10 and abs(-ex * uy + ey * ux) < 0.25:
                return t["id"]
        return None

    def track_near(self, x, y, r=0.40):
        """(x, y) 가까이 있는 상대 번호. 큐브를 쥔 로봇은 큐브 뒤 약 20 cm."""
        trs = [t for t in self.update_tracks() if self.io.now() - t["t"] < 1.0 and math.hypot(t["x"] - x, t["y"] - y) < r]       # 들락날락 찍혀도 1 초 안이면 씀 (11-03 실기 #84)
        return min(trs, key=lambda t: math.hypot(t["x"] - x, t["y"] - y))["id"] if trs else None

    def opp_heading(self, tr):
        """상대 앞 방향: 큐브가 붙어 보이면 큐브 쪽(쥔 입), 아니면 움직인 방향. 모르면 None."""
        cb = self.cube_arena()
        now = self.io.now()
        if cb is not None and math.hypot(cb[0] - tr["x"], cb[1] - tr["y"]) < 0.40:
            a = math.atan2(cb[1] - tr["y"], cb[0] - tr["x"])
            hc = tr.get("hc")
            tr["hc"] = a if hc is None else wrap(hc + 0.3 * wrap(a - hc))      # 큐브가 보일 때마다 조금씩 (몸 가운데 추정이 보는 각도 따라 흔들림)
            tr["hc_t"] = now
        if tr.get("hc") is not None and now - tr.get("hc_t", -9.0) < 3.0:
            return tr["hc"]
        return tr["h"]

    def exit_dist(self, P, dx, dy):
        """P 를 (dx, dy) 로 밀 때 노란 선까지 거리."""
        best = 9.0
        for p, d in ((P[0], dx), (P[1], dy)):
            if abs(d) > 1e-6:
                s = (math.copysign(self.c.line, d) - p) / d
                if s > 0:
                    best = min(best, s)
        return best

    def pick_push(self, P, h, prev, exclude=(), rear_first=False, use_back=False):
        """어느 쪽에서 밀지: 상대 앞을 모르면 중심 → 바깥 방향, 알면 왼옆·오른옆·뒤 가운데 선까지 가깝고 우리가 빨리 닿는 쪽. 돌려줌 (이름, 미는 방향, 붙을 자리) / None."""
        c = self.c
        if h is None:
            cands = [("바깥", math.atan2(P[1], P[0]) if math.hypot(*P) > 0.05 else self.yaw)]
        else:
            cands = [("왼옆", h + math.pi / 2), ("오른옆", h - math.pi / 2), ("뒤", h)]
        gap = c.opp_r + (c.back if use_back else c.front) + 0.02           # 우리 뒷면으로 밀 땐 바퀴 가운데 → 뒤끝 16.1 cm
        best = None
        for name, a in cands:
            if any(abs(wrap(a - e)) < math.radians(40) for e in exclude):
                continue                                     # 이미 해 봤는데 안 밀린 쪽
            dx, dy = math.cos(a), math.sin(a)
            A = (P[0] - dx * gap, P[1] - dy * gap)
            if max(abs(A[0]), abs(A[1])) > self.lim - 0.03:
                continue                                     # 붙을 자리가 밀기 선 밖
            sc = self.exit_dist(P, dx, dy) + 0.5 * math.hypot(A[0] - self.x, A[1] - self.y) + ((-0.5 if rear_first else c.rear_pen) if name == "뒤" else 0.0)
            if prev is not None and abs(wrap(a - prev[1])) < math.radians(50):
                sc -= 0.40                                   # 정한 방향 유지 (상대 방향 추정이 흔들려도 왼옆·오른옆이 뒤집히지 않게)
            if best is None or sc < best[0]:
                best = (sc, name, wrap(a), A)
        return None if best is None else best[1:]

    def pick_dir(self, P, prev, gap):
        """상대를 어느 쪽으로 밀지 (11-03 사용자: 상대 어느 면이든 상관없이 가장 가까운 노란 선 쪽으로). 돌려줌 (이름, 미는 방향, 붙을 자리) / None."""
        cands = [("+x 선 쪽", 0.0), ("+y 선 쪽", math.pi / 2), ("-x 선 쪽", math.pi), ("-y 선 쪽", -math.pi / 2), ("바깥 대각", math.atan2(P[1], P[0]))]
        best = None
        for name, a in cands:
            dx, dy = math.cos(a), math.sin(a)
            A = (P[0] - dx * gap, P[1] - dy * gap)
            if max(abs(A[0]), abs(A[1])) > self.lim - 0.03:
                continue                                     # 붙을 자리가 밀기 선 밖
            sc = self.exit_dist(P, dx, dy) + 0.5 * math.hypot(A[0] - self.x, A[1] - self.y)
            if prev is not None and abs(wrap(a - prev[1])) < math.radians(30):
                sc -= 0.4
            if best is None or sc < best[0]:
                best = (sc, name, wrap(a), A)
        return None if best is None else best[1:]

    def around(self, P, S, r=0.45):
        """상대 몸을 뚫고 가지 않게: 붙을 자리가 상대 너머면 상대 둘레 r 를 50° 씩 돌아서 감."""
        ux, uy = self.x - P[0], self.y - P[1]
        if math.hypot(ux, uy) < 0.05:
            return S
        da = wrap(math.atan2(S[1] - P[1], S[0] - P[0]) - math.atan2(uy, ux))
        if abs(da) < math.radians(50):
            return S
        a = math.atan2(uy, ux) + math.copysign(math.radians(50), da)
        return P[0] + r * math.cos(a), P[1] + r * math.sin(a)

    def push_side(self, tid, why="", cube=None):
        """상대 로봇을 우리 옆면으로(메카넘 옆걸음) 가장 가까운 노란 선 쪽으로 밀어냄. 안 밀리면 차체를 돌려 우리 뒷면으로 더 세게 (11-03 사용자: 앞면엔 카메라).
        라이다 덩어리로 상대를 계속 따라감. (예전 설명: 상대를 옆(안 되면 뒤)에서 노란 선 쪽으로 밀어냄)
        앞(큐브를 쥔 입)에서 맞밀면 상대 바퀴 힘을 정면으로 받지만, 옆에선 상대가 맞밀 힘이 없음.
        붙으면 라이다 최소 거리 안이라 상대가 안 보임 → 정한 방향으로 chase_push 초까지 그대로 밂.
        돌려줌: out (상대가 선 밖) / lost / time / limit (우리가 밀기 선에 닿음) / nowhere (붙을 자리 없음) / stuck."""
        c = self.c
        self.set_state("옆밀기")
        vt = None                                            # 라이다에 안 보이는 상대: 카메라 큐브로 짐작 (큐브가 우리 쪽을 보는 입에 있으니 상대는 큐브 뒤 20 cm)
        if tid is None:
            ux, uy = cube[0] - self.x, cube[1] - self.y
            n = max(math.hypot(ux, uy), 1e-6)
            ux, uy = ux / n, uy / n
            vt = dict(x=cube[0] + 0.2 * ux, y=cube[1] + 0.2 * uy, t=self.io.now(), v=0.0, h=math.atan2(-uy, -ux))
        self.log("밀 상대 %s %s" % ("#%d" % tid if tid is not None else "(라이다에 안 보임 → 카메라 큐브로 짐작 (%+.2f, %+.2f))" % (vt["x"], vt["y"]), why))
        self.lim = c.chase_lim                               # 밀 때 선 그대로 (평소 안전선 1.20 과 작은 쪽을 쓰면 1.22 가 안 먹힘)
        self.io.set_pwm(c.push_pwm)                          # 밀 때만 출력 상한을 올림 (끝나면 평소로)
        self.rim_now = c.push_v
        pick, engaged, t_blind, P, t_touch = None, False, None, None, None
        went_out = False                                     # 상대가 노란 선을 넘은 게 보였나 (그래도 끝까지 밂)
        tried, retries, back = [], 0, False                  # back = 우리 뒷면으로 미는 중 (아니면 옆면)
        face_off = -math.pi / 2                              # 미는 방향 → 우리 몸 방향: -90° = 왼 옆면이 앞장, +90° = 오른 옆면, 180° = 뒷면
        self.stall_reset()
        t0 = self.io.now()
        best_d, t_best = 9.0, t0
        try:
            while True:
                self.tick()
                now = self.io.now()
                if now - t0 > c.push_time:
                    self.log("옆밀기 시간 끝")
                    return "out" if went_out else "time"
                cb = self.cube_arena()                       # 미는 중에도 카메라로 큐브 확인: 중앙 칸에 혼자 있으면 1순위(잡기)가 먼저 (11-03 사용자)
                if cb is not None and self.seen >= 2 and max(abs(cb[0]), abs(cb[1])) <= c.center_half and not self.someone_near(cb[0], cb[1], r=0.30, n=3):
                    self.log("밀기 중 중앙 큐브 (%+.2f, %+.2f) 가 혼자 보임 → 밀기 멈추고 잡으러 감 (1순위)" % cb)
                    self.hold_saw, self.attack_until = cb, -99.0
                    return "cube"
                if vt is not None:
                    t2 = [t for t in self.tracks if now - t["t"] < 0.6 and math.hypot(t["x"] - vt["x"], t["y"] - vt["y"]) < 0.35]
                    if t2:
                        tid = min(t2, key=lambda t: math.hypot(t["x"] - vt["x"], t["y"] - vt["y"]))["id"]
                        vt = None
                        self.log("라이다에 상대 #%d 잡힘 → 그걸 따라감" % tid)
                    elif engaged and pick is not None:      # 붙어서 미는 중: 상대는 미는 쪽에 붙어 있다고 봄 (멈춤·선·시간으로 끝냄)
                        gap = c.opp_r + (c.back if back else (c.front if c.push_face == "line" else c.side))
                        vt["x"], vt["y"] = self.x + math.cos(pick[1]) * gap, self.y + math.sin(pick[1]) * gap
                    else:
                        cb = self.cube_arena()
                        if cb is not None:
                            nx, ny = cb[0] - 0.2 * math.cos(vt["h"]), cb[1] - 0.2 * math.sin(vt["h"])
                            if math.hypot(nx - vt["x"], ny - vt["y"]) < 0.5:
                                vt["x"], vt["y"] = nx, ny
                    if vt is not None:
                        vt["t"] = now
                tr = vt if vt is not None else next((t for t in self.tracks if t["id"] == tid), None)
                if tr is not None and now - tr["t"] < 0.6:
                    t_blind = None
                    P = (tr["x"], tr["y"])
                    if max(abs(P[0]), abs(P[1])) > c.line and not went_out:
                        went_out = True                      # 바로 멈추지 않음: 걸친 채 남지 않게 밀기 선(1.22)·안 밀림·시간까지 끝까지 (11-03 22시 사용자)
                        self.log("상대 %s 노란 선 넘음 (%+.2f, %+.2f) → 확실히 더 밂" % ("#%d" % tid if vt is None else "(짐작)", P[0], P[1]))
                    if max(abs(P[0]), abs(P[1])) > c.line + c.opp_out:
                        self.log("상대가 노란 선 밖으로 멀리 나감 → 끝")
                        return "out"
                    h = self.opp_heading(tr)
                    if not engaged and tr["v"] > c.dyn_v and tr["h"] is not None:          # 움직이면 닿을 때쯤 자리를 내다봄
                        lead = min(1.0, math.hypot(P[0] - self.x, P[1] - self.y) / c.max_v)
                        P = (P[0] + tr["v"] * math.cos(tr["h"]) * lead, P[1] + tr["v"] * math.sin(tr["h"]) * lead)
                    if engaged and c.push_face != "front" and math.hypot(P[0] - self.x, P[1] - self.y) > 0.60:
                        engaged, pick = False, None          # 상대가 빠져나감 → 방향 새로 정해 다시 붙음. 돌아서 우리를 봐도 떼지 않고 계속 밂
                        self.log("상대가 60 cm 넘게 빠져나감 → 방향 새로 정해 다시 붙음")      # (11-03 21:09 실기: '돌아섬→다시 붙음' 이 시간 안 흐르고 19 만 번 돌아 제자리에 섬)
                    if c.push_face == "front":               # 정면: 자세 안 잡고 상대 가운데를 매 순간 다시 겨눠 앞면으로 밀고 들어감
                        a = math.atan2(P[1] - self.y, P[0] - self.x)
                        if not engaged:
                            self.stall_reset()
                            self.log("상대 %s (%+.2f, %+.2f) 로 정면 밀기 (방향 %+.0f°)" % ("#%d" % tid if vt is None else "(짐작)", P[0], P[1], math.degrees(a)))
                        pick, face_off, back, engaged = ("정면", a, (self.x, self.y)), 0.0, False, True
                        self.push_dir = (math.cos(a), math.sin(a))
                    if not engaged:                          # 미는 방향은 한 번 정하면 고정 (11-03 실기: 움직이는 상대의 앞 방향이 흔들려 왼옆·오른옆이 0.5 초마다 뒤집혀 제자리 스텝)
                        gap = c.opp_r + (c.back if back else (c.front if c.push_face == "line" else c.side)) + 0.02
                        A = None if pick is None else (P[0] - math.cos(pick[1]) * gap, P[1] - math.sin(pick[1]) * gap)
                        if A is None or max(abs(A[0]), abs(A[1])) > self.lim - 0.03:      # 처음이거나 붙을 자리가 밀기 선 밖이 됐을 때만 다시 정함
                            new = self.pick_dir(P, pick, gap)
                            if new is None:
                                self.log("붙을 자리가 밀기 선 밖 → 그만")
                                return "nowhere"
                            face_off = math.pi if back else (0.0 if c.push_face == "line" else min((-math.pi / 2, math.pi / 2), key=lambda o: abs(wrap(new[1] + o - self.yaw))))   # line: 앞면, side: 덜 도는 쪽 옆면
                            self.log("상대 %s (%+.2f, %+.2f) 를 %s으로 밂 (미는 방향 %+.0f°, 우리 %s, 이 시도 동안 고정)" % ("#%d" % tid if vt is None else "(짐작)", P[0], P[1], new[0], math.degrees(new[1]),
                                     "뒷면" if back else ("앞면" if face_off == 0.0 else ("왼 옆면" if face_off < 0 else "오른 옆면"))))
                            pick = new
                        else:
                            pick = (pick[0], pick[1], A)
                elif pick is None or P is None or (not engaged and math.hypot(P[0] - self.x, P[1] - self.y) > 0.50):
                    self.log("밀 상대 %s 놓침" % ("#%d" % tid if tid is not None else "(짐작)"))
                    return "lost"
                else:                                        # 붙으면 라이다 최소 거리 안이라 덩어리로 안 잡힘 → 몸에 닿아 있는 동안 정한 방향으로 그대로 밂
                    if t_blind is None:
                        t_blind = now
                        if not engaged:
                            self.stall_reset()
                        engaged = True
                    if self.contact() is not None:
                        t_touch = now
                    elif now - max(t_blind, t_touch or t_blind) > 2.0:                # 몸에서 떨어진 뒤 2 초까지는 계속 밂 (11-03 21시: 1 → 2 초, 밀다가 맒)
                        self.log("상대가 몸에서 떨어짐 → 끝")
                        return "out" if went_out else "lost"
                da = pick[1]
                dx, dy = math.cos(da), math.sin(da)
                ew = wrap(da + face_off - self.yaw)          # 옆면이나 뒷면이 미는 쪽을 보게 (앞면 카메라는 안 댐)
                w = clip(c.kw * ew, -c.max_w, c.max_w)
                if not engaged:
                    A = pick[2]
                    S = (A[0] - dx * 0.10, A[1] - dy * 0.10)
                    dS = math.hypot(S[0] - self.x, S[1] - self.y)
                    if dS < 0.08 and abs(ew) < math.radians(25):
                        engaged = True
                        self.push_dir = (dx, dy)
                        self.stall_reset()
                        self.log("붙음 → %s으로 우리 %s으로 밂" % (pick[0], "뒷면" if back else ("앞면" if face_off == 0.0 else "옆면")))
                        self.io.sleep(0.05)                  # 판단만 바뀌고 시간이 안 흐르는 고리 방지
                        continue
                    if dS < best_d - 0.02:
                        best_d, t_best = dS, now
                    elif now - t_best > 5.0:                 # 움직이는 상대를 따라가는 시간 (3 → 5 초)
                        self.log("붙을 자리로 못 감")
                        return "stuck"
                    tx, ty = self.around(P, S)
                    ex, ey = tx - self.x, ty - self.y
                    d = max(math.hypot(ex, ey), 1e-6)
                    v = clip(c.kp * d, c.min_v, c.max_v)
                    self.cmd(ex / d * v, ey / d * v, w)
                else:
                    why = self.push_done()
                    if why and "선" not in why and retries < c.push_retry and pick is not None and not back and c.push_face == "side":
                        retries += 1                         # 옆면으론 안 밀림 → 같은 방향 그대로 차체를 돌려 우리 뒷면으로 더 세게 (11-03 사용자: 앞뒤가 옆걸음보다 셈)
                        back = True
                        face_off = math.pi
                        gap = c.opp_r + c.back + 0.02
                        A = (P[0] - dx * gap, P[1] - dy * gap) if P is not None else pick[2]
                        pick = (pick[0], pick[1], A)
                        self.log("%s으로 옆면으로 밀다 %s → 차체를 돌려 우리 뒷면으로 다시 밂" % (pick[0], why))
                        engaged, t0 = False, self.io.now()
                        best_d, t_best = 9.0, t0
                        self.stall_reset()
                        self.io.sleep(0.05)
                        continue
                    if why:
                        self.log("옆밀기 끝: %s" % why)
                        return "out" if went_out else ("limit" if "선" in why else "stall")
                    if t_blind is None and P is not None:    # 상대 가운데를 겨눔 (미는 방향으로 조금 너머): 옆으로 미끄러져 빠지지 않게
                        gx, gy = P[0] + dx * 0.10 - self.x, P[1] + dy * 0.10 - self.y
                        g = math.hypot(gx, gy)
                        if g > 1e-6 and gx * dx + gy * dy > 0:
                            self.push_dir = (gx / g, gy / g)
                    ux, uy = getattr(self, "push_dir", (dx, dy))
                    w = clip(c.kw * wrap(math.atan2(uy, ux) + face_off - self.yaw), -c.max_w, c.max_w)
                    self.cmd(ux * c.push_v, uy * c.push_v, w)
                    if (abs(dx) > 0.3 and self.x * dx > 0 and abs(self.x) > self.lim - 0.02) or (abs(dy) > 0.3 and self.y * dy > 0 and abs(self.y) > self.lim - 0.02):
                        self.log("밀기 선에 닿음 (%+.2f, %+.2f)" % (self.x, self.y))
                        return "out" if went_out else "limit"
                self.io.sleep(0.05)
        finally:
            self.lim = c.line - c.safety
            self.rim_now = 0.0
            self.io.set_pwm(None)                            # 평소 출력 상한으로
            self.brake()

    def cube_moving(self, cx, cy):
        """서서 still_t 초 동안 큐브를 봄. 돌려줌 (움직임?, 마지막 자리). 절반 넘게 안 보이면 (True, None): 시야 밖으로 움직였다고 봄."""
        self.stop()
        xs, n = [], 0
        t_end = self.io.now() + self.c.still_t
        while self.io.now() < t_end:
            self.tick()
            self.cmd(0.0, 0.0, 0.0)
            q = self.cube_arena()
            n += 1
            if q is not None:
                xs.append(q)
            self.io.sleep(0.05)
        if len(xs) * 2 <= n:
            return True, (cx, cy) if xs == [] else xs[-1]
        k = max(1, len(xs) // 4)
        a = (sum(p[0] for p in xs[:k]) / k, sum(p[1] for p in xs[:k]) / k)
        b = (sum(p[0] for p in xs[-k:]) / k, sum(p[1] for p in xs[-k:]) / k)
        return math.hypot(b[0] - a[0], b[1] - a[1]) > 0.06, b

    def attack(self, cx, cy, why):
        """잡기 대신 공격: 큐브 옆 로봇이 라이다에 보이면 옆·뒤에서 밀고, 안 보이면(라이다보다 낮은 로봇 등) 큐브 쪽으로 돌진해 쥔 로봇을 밂."""
        tid = self.track_near(cx, cy, r=9.0)                  # 거리 조건 없음 (11-03 사용자): 큐브에서 가장 가까운 라이다 로봇
        self.log("공격 (%s): 큐브 (%+.2f, %+.2f) · 라이다 로봇 %s · 가까운 후보: %s" % (why, cx, cy, "#%d" % tid if tid is not None else "없음 → 카메라로 짐작", self.tracks_desc(cx, cy)))
        self.push_side(tid, why, cube=(cx, cy))              # 늘 옆·뒤에서 밂 (정면 돌진 안 함)
        self.hunt_t = self.io.now()

    def patrol_until(self):
        """순회를 끊을 때: 큐브가 보임, 또는 (한 바퀴 돌아도 중앙 큐브가 없을 때) 가까이 움직이는 로봇."""
        if self.moving_cube_seen():
            return True
        c = self.c
        if not c.hunt_dyn or self.io.now() - self.hunt_t < c.hunt_gap or not (c.hunt_any or self.lap_done):
            return False
        if self.t0 is not None and self.io.now() - self.t0 < c.grab_first_t:
            return False                                     # 출발 뒤 20 초: 로봇 공격 안 함
        now = self.io.now()
        cand = [t for t in self.tracks if now - t["t"] < 0.5 and (c.hunt_any or t["v"] > c.dyn_v) and math.hypot(t["x"] - self.x, t["y"] - self.y) < c.hunt_r
                and max(abs(t["x"]), abs(t["y"])) < c.line + c.edge_margin and now - self.hunt_skip.get(t["id"], -99.0) > 10.0]
        if not cand:
            return False
        self.hunt_id = min(cand, key=lambda t: math.hypot(t["x"] - self.x, t["y"] - self.y))["id"]
        return True

    def ring_points(self):
        r = self.c.ring
        return [(r, -r), (r, 0.0), (r, r), (0.0, r), (-r, r), (-r, 0.0), (-r, -r), (0.0, -r)]     # 반시계. 홀수 번호 = 변 중앙

    def patrol(self):
        """사각 순회: 옆·앞으로만 움직이고 제자리 회전이나 멈춤이 없음 (메카넘). 몸 방향은 매 순간 다시 맞추니 조금 틀어져도 됨.
        큐브를 찾으면 ("center"|"away", x, y) 를 돌려줌. 앞이 막히면 방향을 반대로."""
        c = self.c
        if c.patrol_mode == "stop" or c.hunt:
            return self.patrol_stop()
        self.set_state("순회")
        r = c.ring
        pts = [(r, -r), (r, r), (-r, r), (-r, -r)]           # 반시계
        self.tick()
        i = min(range(4), key=lambda k: math.hypot(pts[k][0] - self.x, pts[k][1] - self.y))
        face = None if c.patrol_mode == "fixed" else 1
        yaw0 = self.snap(self.yaw)
        fails, corners = 0, 0
        self.orbit = face is not None
        try:
            while True:
                self.seen = 0
                self.hunt_id = None
                res = self.go(pts[i][0], pts[i][1], yaw=yaw0, tol=0.10, until=self.patrol_until, watch=c.patrol_watch, timeout=8.0, face=face)
                if res == "until" and self.hunt_id is not None:
                    tid, self.hunt_id = self.hunt_id, None
                    return ("robot", tid, 0.0)
                if res == "until":
                    self.stop()
                    b = self.look()
                    if b is not None:
                        return self.classify(b)
                    continue
                if res in ("blocked", "stuck", "timeout"):
                    fails += 1
                    self.turn_dir = -self.turn_dir
                    self.log("순회 길 막힘(%s) → 방향 반대로" % res)
                    if fails >= 2:                           # 양쪽 다 막힘: 잠깐 서서 봄 (상대가 움직이면 풀림)
                        fails = 0
                        b = self.look()
                        if b is not None:
                            return self.classify(b)
                        self.wait(1.0)
                else:
                    fails = 0
                    corners += 1
                    if corners >= 4:
                        self.lap_done = True                 # 한 바퀴 돌았는데 중앙 큐브를 못 봄 → 누가 쥐고 있음
                    if c.patrol_mode == "alt" and corners % 4 == 0:      # 다음 바퀴는 반대쪽을 봄
                        face = -face
                i = (i + self.turn_dir) % 4
        finally:
            self.orbit = False

    def patrol_stop(self):
        """옛판 3×3 순회: 가는 쪽을 보고 달리다 변 가운데서 멈춰 90° 돌아 중심을 봄. 큐브를 찾으면 ("center"|"away", x, y) 를 돌려줌. 앞이 막히면 방향을 반대로."""
        self.set_state("순회")
        pts = self.ring_points()
        self.tick()
        i = min(range(8), key=lambda k: math.hypot(pts[k][0] - self.x, pts[k][1] - self.y))
        last_look = self.io.now() - self.c.look_every      # 들어오자마자 한 번 봄
        fails = 0
        while True:
            X, Y = pts[i]
            self.tick()
            if self.c.hunt and self.io.now() - self.hunt_t >= self.c.hunt_gap:
                q = self.nearest_robot()
                if q is not None:
                    self.log("순회 중 상대 (%+.2f, %+.2f) → 밀러 감" % q)
                    self.push(q[0], q[1])
                    self.hunt_t = self.io.now()
                    self.set_state("순회")
                    i = min(range(8), key=lambda k: math.hypot(pts[k][0] - self.x, pts[k][1] - self.y))
                    continue
            d = math.hypot(X - self.x, Y - self.y)
            if d > 0.05:
                b = self.snap(math.atan2(Y - self.y, X - self.x))
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
                self.turn_to(self.snap(math.atan2(-self.y, -self.x)))
                c = self.look()
                last_look = self.io.now()
                if c is not None:
                    return self.classify(c)
            i = (i + self.turn_dir) % 8

    def moving_cube_seen(self):
        c = self.cube_arena()
        return c is not None and self.seen >= 2 and max(abs(c[0]), abs(c[1])) < self.c.line + self.c.edge_margin     # 선 밖 멀리 큐브(탈락한 로봇이 쥔 것)는 무시, 걸친 건 봄

    def someone_near(self, cx, cy, r=0.30, n=5):
        """큐브 둘레 r 안에 라이다 점이 있나 = 상대가 쥐고 있거나 붙어 있음. 큐브(5 cm)는 라이다 높이(16 cm)보다 낮아 안 잡힘."""
        pts = self.io.scan()
        if pts is None or len(pts) == 0:
            return False
        c = self.c
        px, py = pts[:, 0], pts[:, 1]
        keep = ~((px > -c.back - 0.01) & (px < c.front + 0.03) & (np.abs(py) < c.side + 0.01))     # 우리 몸에 맞은 점은 뺌 (순회 자리에서 큐브까지 약 30 cm)
        px, py = px[keep], py[keep]
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = self.x + cs * px - sn * py, self.y + sn * px + cs * py
        return int(np.count_nonzero(np.hypot(ax - cx, ay - cy) < r)) >= n

    def classify(self, c):
        if max(abs(c[0]), abs(c[1])) > self.c.line + self.c.edge_margin:
            kind = "out"                                     # 노란 선 밖: 탈락한 쪽 → 더 안 밀고 무시
        elif self.c.cube_rule == "cell":
            kind = "center" if max(abs(c[0]), abs(c[1])) <= self.c.center_half else "away"
        else:
            kind = "center" if math.hypot(c[0], c[1]) < self.c.center_r else "away"
        self.log("큐브 발견 (%+.2f, %+.2f) → %s" % (c[0], c[1], kind))
        if kind == "center":
            self.lap_done = False
        return kind, c[0], c[1]

    def stall_reset(self):
        self.stall_p, self.stall_t0 = (self.x, self.y), self.io.now()

    def push_done(self):
        """밀기 그만둘 때: 밀기 선에 닿음 / stall_t 동안 5 cm 도 못 나감 (상대가 안 밀림). push_stop 이 꺼져 있으면 None."""
        if not self.c.push_stop:
            return None
        cx, cy = self.last_cmd
        cs, sn = math.cos(self.yaw), math.sin(self.yaw)
        vx, vy = cs * cx - sn * cy, sn * cx + cs * cy                      # 마지막으로 보낸 속도 (경기장 좌표)
        for p, v in ((self.x, vx), (self.y, vy)):
            if abs(p) > self.lim - 0.03 and p * v > 0:
                return "밀기 선에 닿음 (%+.2f, %+.2f)" % (self.x, self.y)
        now = self.io.now()
        if math.hypot(self.x - self.stall_p[0], self.y - self.stall_p[1]) > 0.05:
            self.stall_reset()
        elif now - self.stall_t0 > self.c.stall_t:
            return "%.1f 초 동안 안 밀림" % self.c.stall_t
        return None

    def chase(self, can_grab=True):
        """보이는 큐브 쪽으로 최대 속도로 밀고 들어감. 가까우면 큐브가 시야 아래로 사라지므로(34 cm 안) 안 보여도 잠깐 더 밀어붙임.
        돌려줌: ("still", x, y) 멈춘 큐브 (can_grab 일 때만) / ("lost",) / ("time",). 상대 탈락으로 큐브가 중앙에 돌아오면 순회가 찾음."""
        self.set_state("추격" if can_grab else "밀기")
        self.lim = min(self.lim, self.c.chase_lim)
        try:
            t0 = t_seen = self.io.now()
            still_at, still_t, near = None, None, False
            self.stall_reset()
            while True:
                self.tick()
                now = self.io.now()
                if now - t0 > self.c.chase_time:
                    return ("time",)
                why = self.push_done() if near else None
                if why:
                    self.log("돌진 끝: %s" % why)
                    return ("lost",)
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
                if max(abs(c[0]), abs(c[1])) > self.c.line + 0.02:
                    self.log("큐브가 노란 선 밖 (%+.2f, %+.2f) → 그만" % c)
                    return ("out",)
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
            self.stall_reset()
            while self.io.now() - t0 < self.c.push_time:
                self.tick()
                now = self.io.now()
                why = self.push_done()
                if why:
                    self.log("밀기 끝: %s" % why)
                    return
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
        c, self.hold_saw = self.hold_saw, None               # 버티다 앞에 큐브를 봤으면 순회 없이 바로
        kind, cx, cy = self.classify(c) if c is not None else self.patrol()
        if kind == "robot":                                  # 순회 중 라이다로 본 로봇 (늘 공격)
            r = self.push_side(int(cx), "순회 중 본 로봇")
            if r == "nowhere":
                self.hunt_skip[int(cx)] = self.io.now()
            self.end_attack_if_out(r)
            self.hunt_t = self.io.now()
            return
        if kind == "out":
            return
        if self.c.cube_rule == "prio" and self.c.attack:
            return self.search_prio(cx, cy)
        if self.c.cube_rule == "cell" and self.c.attack:
            return self.search_cell(kind, cx, cy)
        if self.c.attack and self.c.push_mode == "side":
            tid = self.track_near(cx, cy, r=0.35)
            if tid is not None:                              # 큐브 바로 옆에 로봇 = 쥐고 있음 → 잡지 말고 옆·뒤에서 밂 (11-03 실기: 선 끝 로봇 옆 큐브를 잡으러 감)
                self.attack(cx, cy, "큐브 바로 옆 로봇 #%d" % tid)
                return
        if self.c.attack and self.io.now() - self.fail_t < self.c.attack_win:     # 막 잡기에 실패함 → 또 잡지 말고 공격
            free = False
            if kind == "center" and self.track_near(cx, cy) is None:              # 중앙에 혼자 서 있는 큐브 = 상대 탈락으로 돌아온 큐브일 수 있음 → 잠깐 봄
                mv, q = self.cube_moving(cx, cy)
                free = not mv
                if free:
                    self.log("중앙 큐브 (%+.2f, %+.2f) 가 서 있고 옆에 로봇 없음 → 공격 대신 잡기" % q)
                    cx, cy = q
            if not free:
                self.attack(cx, cy, "잡기 실패 뒤 %.0f 초 안" % (self.io.now() - self.fail_t))
                return
        if kind == "away":                                   # 다른 곳의 큐브: 움직이면 쫓고, 멈춰 있으면 잡음
            if self.c.attack and self.c.push_mode == "side":
                mv, q = self.cube_moving(cx, cy)             # 잠깐 서서 봄: 큐브가 움직이면 쥔 로봇이 있음 → 옆에서 밀어냄
                self.log("큐브 %s (%+.2f, %+.2f)" % ("움직임 → 공격" if mv else "서 있음 → 잡기", q[0], q[1]))
                if mv:
                    self.attack(q[0], q[1], "움직이는 큐브")
                    return
                cx, cy = q                                   # 서 있는 큐브: 먼저 잡아 보고 못 잡으면(아래) 옆 로봇을 밂
            elif self.c.attack:
                r = self.chase()
                if r[0] != "still":
                    return
                cx, cy = r[1], r[2]
            else:                                            # 공격 끔: 멈춰 있는 큐브만 잡으러 감
                self.wait(self.c.still_t)
                b = self.look()
                if b is None or math.hypot(b[0] - cx, b[1] - cy) > 0.05 or self.someone_near(b[0], b[1]):
                    return
                cx, cy = b
        if self.approach(cx, cy) and self.grab_loop():
            self.hold()
            return
        self.fail_t = self.io.now()
        c = self.look()
        if self.c.attack and c is not None and self.c.attack_win > 0:          # 1 번 해 보고 안 되면 바로 공격
            self.attack(c[0], c[1], "잡기 실패")
            return
        if self.c.attack and c is not None and self.someone_near(c[0], c[1]):
            if self.c.push_mode == "side":
                self.attack(c[0], c[1], "잡기 실패, 큐브 옆에 무언가 있음")
                return
            self.push(c[0], c[1])                            # 상대가 붙어 있어 못 잡음 → 밀어냄 (3.4.6). 혼자 놓인 큐브는 밀지 않고 다시 잡음

    def search_prio(self, cx, cy):
        """큐브가 보일 때 (11-03 21시 사용자 우선순위). 1: 큐브 주변(30 cm)에 라이다 점이 없으면 잡기, fail_limit 번 실패하면 attack_time 동안 공격.
        2: 주변에 점이 있으면(쥐거나 붙은 로봇) attack_time 동안 공격. 미는 건 옆 먼저, 안 밀리면 상대 뒤로 돌아가서."""
        now = self.io.now()
        early = self.t0 is not None and now - self.t0 < self.c.grab_first_t       # 출발 뒤 20 초: 잡기만
        tid = None if early else self.contested(cx, cy)
        near = False if early else (tid is not None or self.someone_near(cx, cy, r=0.30, n=3))
        if not early and (near or now < self.attack_until):
            if near and now >= self.attack_until:
                self.attack_until = now + self.c.attack_time
                self.log("2순위: 큐브 (%+.2f, %+.2f) 주변에 라이다 점 → %.0f 초 공격 · %s" % (cx, cy, self.c.attack_time, self.tracks_desc(cx, cy)))
            target = tid if tid is not None else self.track_near(cx, cy, r=9.0)
            if target is None and not near:
                self.log("공격 중인데 밀 로봇이 안 보임 (큐브 주변 비어 있음) → 순회")
                return
            r = self.push_side(target, "공격 (%.0f 초 남음)" % max(0.0, self.attack_until - self.io.now()), cube=(cx, cy))
            if r == "nowhere" and target is not None:
                self.hunt_skip[target] = self.io.now()
            self.end_attack_if_out(r)
            self.hunt_t = self.io.now()
            return
        self.log("%s: 큐브 (%+.2f, %+.2f) → 잡기 (실패 %d/%d)" % ("출발 뒤 %.0f 초 (잡기만)" % (now - self.t0) if early else "1순위 (주변 비어 있음)", cx, cy, self.fails, self.c.fail_limit))
        if not self.approach(cx, cy):
            return                                           # 안전선 밖이라 안 감 / 못 감: 실패로 안 셈
        if self.grab_loop():
            self.fails = 0
            self.hold()
            return
        self.fail_count()

    def end_attack_if_out(self, r):
        """밀던 로봇을 노란 선 밖으로 내보냄(탈락) → 30 초 공격은 제한시간일 뿐이라 바로 끝내고 우선순위로 (11-03 사용자)."""
        if r == "out":
            if self.io.now() < self.attack_until:
                self.log("밀던 로봇 탈락 → 공격 끝, 우선순위로 돌아감")
            self.attack_until, self.fails = -99.0, 0

    def fail_count(self):
        self.fails += 1
        if self.t0 is not None and self.io.now() - self.t0 < self.c.grab_first_t:
            return                                           # 출발 뒤 20 초 안: 실패를 세기만 하고 공격으로 안 넘어감
        if self.fails >= self.c.fail_limit:
            self.fails = 0
            self.attack_until = self.io.now() + self.c.attack_time
            self.log("잡기 %d 번 실패 → %.0f 초 공격" % (self.c.fail_limit, self.c.attack_time))

    def search_cell(self, kind, cx, cy):
        """중앙 칸 규칙 (11-03 사용자): 중앙 칸 큐브만 잡음. 칸 밖에 보이면 누가 쥐고 있다고 보고 공격 (라이다로 보이면 옆밀기, 안 보이면 큐브로 돌진).
        예외: 중심 near_free_r 안에 혼자 서 있는 큐브는 심판이 놓은 것일 수 있어 잡음 (최근 잡기 실패가 없을 때만)."""
        tid = self.contested(cx, cy)
        if tid is not None:                                  # 큐브 옆에 로봇(쥐고 있음, 움직이는 큐브도 이 경우) 또는 가는 길을 가림 → 중앙 칸이어도 잡지 않고 밂
            self.attack(cx, cy, "큐브를 쥐거나 가린 로봇 #%d" % tid)
            return
        if kind == "away":
            free = math.hypot(cx, cy) < self.c.near_free_r and self.io.now() - self.fail_t > self.c.free_wait and self.track_near(cx, cy) is None
            if free:
                mv, q = self.cube_moving(cx, cy)
                free = not mv and math.hypot(*q) < self.c.near_free_r
                if free:
                    self.log("중앙 칸 밖이지만 가까이 혼자 서 있는 큐브 (%+.2f, %+.2f) → 잡기 (심판이 놓은 것일 수 있음)" % q)
                    cx, cy = q
            if not free:
                self.attack(cx, cy, "중앙 칸 밖 큐브")
                return
        if self.approach(cx, cy) and self.grab_loop():
            self.hold()
            return
        self.fail_t = self.io.now()
        c = self.look()
        if c is None:
            return
        if self.track_near(*c) is not None or self.someone_near(c[0], c[1]):
            self.attack(c[0], c[1], "잡기 실패, 큐브 옆에 무언가 있음")
        # 아무도 안 보이면: 큐브가 중앙 칸에 그대로면 다음 순회에서 다시 잡고, 칸 밖이면 다음에 볼 때 공격

    def play(self):
        started = False
        while True:
            try:
                if not started:
                    started = True
                    self.start_run()                         # 첫 잡기는 로봇이 있어도 비집고 들어가 잡음 (11-03 사용자: 4 대 동시 출발이라 밀기보다 먼저 잡기)
                    if self.grab_loop():
                        self.hold()
                    else:
                        self.fail_t = self.io.now()          # 첫 잡기 실패 → 다음에 보이면 공격
                        self.fail_count()                    # prio: 첫 잡기 실패도 1 번으로 셈
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
                if self.hold_saw is not None:                # 앞에 큐브를 봤으면 잡으러 가는 데까지 시험
                    self.search_once()
            elif test == "patrol":
                self.search_once()
                self.log("순회 시험 한 번 끝")
            else:
                self.play()
        except MatchOver:
            self.log("경기 시간 끝 → 바퀴만 멈춤 (그리퍼는 그대로)")
        self.stop()
        return self.held_at
