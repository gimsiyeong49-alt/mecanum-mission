#!/usr/bin/env python3
"""
1임무 로봇 제어 루프 예시 — planner.py 를 로봇 제어 코드에 붙이는 방법 (표준 라이브러리만 사용)

  python3 robot_loop_example.py --fake     # 로봇 없이: 가짜 로봇이 규정 그림 1 경기장을 끝까지 돈다
  python3 robot_loop_example.py            # 실제 로봇: 아래 RealRobot 의 TODO 를 채운 뒤

로봇 쪽에서 채울 것은 RealRobot 클래스의 메서드들뿐이다. 제어 루프(run)는 그대로 쓴다.
좌표는 모두 '우리는 (1,9)에서 남쪽을 보고 출발' 기준의 경기장 좌표다 (docs/ROBOT_SETUP.md 2절).
"""
import argparse
import math
import time

from planner import (HEADING_NAME, HOME, RULEBOOK_MAP, S, TARGET, MissionPlanner, __version__, direction,
                     mm_to_cell, parse_map)


# ── 실제 로봇 연결부: 로봇 담당자가 채운다 ────────────────────────────
class RealRobot:
    """모든 위치는 경기장 좌표 [mm] (원점 = 경기장 좌하단, X 오른쪽, Y 위쪽), 방향은 planner 의 N/E/S/W (0~3)."""

    def current_cell(self):
        """지금 있는 칸 (x, y). 위치 추정값 (X, Y) mm 를 planner.mm_to_cell(X, Y) 로 바꾸면 된다."""
        raise NotImplementedError("TODO: 위치 추정 → 칸")

    def current_heading(self):
        """몸(카메라·집게)이 바라보는 방향. yaw 를 가장 가까운 N/E/S/W 로 반올림 (N=0, E=1, S=2, W=3)."""
        raise NotImplementedError("TODO: yaw → N/E/S/W")

    def observed_cells(self):
        """센서로 이번에 본 칸들 {(x, y): 장애물이면 True, 비었으면 False}. 못 본 칸은 넣지 않는다.
        잡음이 있으면 여러 번 같은 결과가 나온 칸만 넣는다."""
        raise NotImplementedError("TODO: 센서 → {칸: 장애물 여부}")

    def opponent_cells(self):
        """(선택) 상대 로봇이 보이는 칸들. 구분하지 못하면 빈 집합 — planner 가 스스로도 판단한다."""
        return set()

    def now(self):
        """초 단위 시각 (임시 차단 시간 계산용)."""
        return time.monotonic()

    def wait(self, seconds):
        time.sleep(seconds)

    def stop(self):
        raise NotImplementedError("TODO: 정지")

    def turn_to(self, heading):
        """제자리에서 몸을 heading(N/E/S/W)으로 돌린다."""
        raise NotImplementedError("TODO: 제자리 회전")

    def translate_to(self, x_mm, y_mm, keep_heading):
        """몸 방향(keep_heading)을 유지한 채 (x_mm, y_mm) 까지 평행 이동 (메카넘: 앞·뒤·옆).
        다음 칸 중심에 도착하면 반환한다. 같은 방향이 이어지면 서지 않고 계속 가도 된다."""
        raise NotImplementedError("TODO: 평행 이동")

    def grab_cube(self):
        """큐브를 집고 성공 여부를 반환한다."""
        raise NotImplementedError("TODO: 집기")

    def find_cube_cell(self):
        """(큐브를 놓쳤을 때) 카메라로 큐브가 보이는 칸을 찾아 반환, 못 찾으면 None."""
        return None


# ── 가짜 로봇: 로봇 없이 루프 전체를 확인 ─────────────────────────────
class FakeRobot:
    """규정 그림 1 경기장. 장애물에 가리지 않은 칸을 모두 보고(사거리 12칸), 명령은 즉시 수행한다.
    시간은 시뮬레이션과 같은 가정: 앞·뒤 1칸 1.6 s, 옆 1칸 2.0 s, 90도 회전 1.5 s, 방향 전환 0.5 s, 집기 3 s."""

    def __init__(self, obstacles, sensor_range=12.0):
        self.obst, self.range = obstacles, sensor_range
        self.cell, self.heading, self.moving = HOME, S, None
        self.t, self.moves, self.turns, self.holding = 0.0, 0, 0, False

    def current_cell(self):
        return self.cell

    def current_heading(self):
        return self.heading

    def observed_cells(self):
        cx, cy = self.cell[0] - 0.5, self.cell[1] - 0.5
        out = {}
        for x in range(1, 10):
            for y in range(1, 10):
                tx, ty = x - 0.5, y - 0.5
                d = math.hypot(tx - cx, ty - cy)
                if d > self.range:
                    continue
                seen = True
                for k in range(1, int(d / 0.05)):
                    px, py = cx + (tx - cx) * k * 0.05 / d, cy + (ty - cy) * k * 0.05 / d
                    c = (math.floor(px) + 1, math.floor(py) + 1)
                    if c != (x, y) and c in self.obst:
                        seen = False
                        break
                if seen:
                    out[(x, y)] = (x, y) in self.obst
        return out

    def opponent_cells(self):
        return set()

    def now(self):
        return self.t

    def wait(self, seconds):
        self.t += seconds

    def stop(self):
        self.moving = None

    def turn_to(self, heading):
        q = min((heading - self.heading) % 4, (self.heading - heading) % 4)
        self.t += 1.5 * q
        self.turns += q
        self.heading, self.moving = heading, None

    def translate_to(self, x_mm, y_mm, keep_heading):
        c = mm_to_cell(x_mm, y_mm)
        assert c is not None and abs(c[0] - self.cell[0]) + abs(c[1] - self.cell[1]) == 1, f"한 칸 이동이 아님: {c}"
        assert c not in self.obst, f"장애물 {c} 로 이동 명령"
        d = direction(self.cell, c)
        rel = (d - keep_heading) % 4
        self.t += (1.6 if rel in (0, 2) else 2.0) + (0.5 if self.moving not in (None, d) else 0.0)
        self.cell, self.moving = c, d
        self.moves += 1

    def grab_cube(self):
        self.t += 3.0
        self.holding = self.cell == TARGET
        return self.holding

    def find_cube_cell(self):
        return TARGET


# ── 제어 루프: 그대로 쓴다 ────────────────────────────────────────────
def run(robot, mp, max_steps=500, log=print):
    """planner 명령을 받아 로봇에 전달하는 루프. 임무를 마치면 True."""
    moving = None
    for _ in range(max_steps):
        cell, heading = robot.current_cell(), robot.current_heading()
        cmd = mp.update(cell, heading, robot.observed_cells(), t=robot.now(), dynamic=robot.opponent_cells(),
                        moving=moving)
        if cmd.action == "MOVE":
            turn = "" if cmd.next_heading == heading else f"회전 {HEADING_NAME[heading]}→{HEADING_NAME[cmd.next_heading]}, "
            log(f"[{cmd.phase:9s}] {cell} → {cmd.next_cell}  {turn}몸 {HEADING_NAME[cmd.next_heading]} 유지, "
                f"{HEADING_NAME[cmd.move_dir]} 쪽으로 이동")
            if cmd.next_heading != heading:
                robot.stop()
                robot.turn_to(cmd.next_heading)
            robot.translate_to(*cmd.next_xy_mm, keep_heading=cmd.next_heading)
            moving = cmd.move_dir
            continue
        moving = None
        robot.stop()
        if cmd.action == "GRAB":
            ok = robot.grab_cube()
            log(f"[GRAB     ] {cell} 큐브 집기 → {'성공' if ok else '실패'}")
            mp.grab_result(ok)
        elif cmd.action == "SEARCH":
            c = robot.find_cube_cell()
            log(f"[SEARCH   ] 큐브 찾기 → {c}")
            if c is not None:
                mp.set_target(c)
            else:
                robot.wait(0.2)
        elif cmd.action == "WAIT":
            log(f"[WAIT     ] {cell} 갈 길이 막힘 (상대 로봇 등) → 잠깐 대기")
            robot.wait(0.2)
        elif cmd.action == "DONE":
            log(f"[DONE     ] {cell} 출발점 도착, 임무 완료")
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description="1임무 로봇 제어 루프 예시")
    ap.add_argument("--fake", action="store_true", help="로봇 없이 가짜 로봇으로 (규정 그림 1 경기장)")
    args = ap.parse_args()
    print(f"planner {__version__}")
    if args.fake:
        robot = FakeRobot(parse_map(RULEBOOK_MAP))
        ok = run(robot, MissionPlanner())
        print(f"{'임무 완료' if ok else '임무 실패'}: 왕복 {robot.moves}칸, 회전 {robot.turns}회, 약 {robot.t:.0f}초 (가짜 로봇 기준)")
        raise SystemExit(0 if ok else 1)
    run(RealRobot(), MissionPlanner())


if __name__ == "__main__":
    main()
