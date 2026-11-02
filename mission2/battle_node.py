#!/usr/bin/env python3
"""2임무 실주행 노드: battle.py 의 판단을 로봇에 붙인다.

입력: /odometry/filtered (위치), /imu/data (몸 방향), /scan (길 막힘), 잡기 스테이션 /status (큐브)
출력: /cmd_vel, 잡기 스테이션 /do?a=grab
격자 위치추정(map)은 안 쓴다: 2임무 경기장엔 장애물이 없어 격자 맞춤이 칸을 혼동하면 40 cm 튄다.

  python3 battle_node.py            # dry-run: 바퀴·그리퍼 안 움직임, 센서·큐브만 찍음
  python3 battle_node.py --execute  # 실제 주행
"""
import argparse
import collections
import json
import math
import os
import queue
import sys
import threading
import time
import urllib.request

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from battle import Battle, Cfg           # noqa: E402

GO_FILE = "/tmp/battle_go"               # --wait-go: 이 파일이 생기면 출발 (field_arm.sh go)
LASER = (0.095, 0.0, 1.641)              # base_link → laser (robot.launch 정적 TF)


class RosIO(Node):
    def __init__(self, a):
        super().__init__("battle")
        self.a = a
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom, self._odom_t = None, 0.0
        self._gyro = None                # (시각, 회전 속도, 적분 방향)
        self.gyro_bias = 0.0
        self._gyro_raw, self._still_t = collections.deque(maxlen=800), 0.0
        self._pts, self._pts_t = None, 0.0
        self._status, self._status_t = {}, 0.0
        self._lock_t = 0.0
        self.estop = False
        self.trail = []
        self.log_q = queue.Queue()
        self.log_f = open(a.log, "a") if a.log else None
        threading.Thread(target=self._printer, daemon=True).start()
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, "/mission1/estop", self._on_estop, 10)
        threading.Thread(target=self._poll_status, daemon=True).start()

    # ── 받기 ──
    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self._odom = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self._odom_t = time.monotonic()

    def _on_imu(self, m):
        now, wz = time.monotonic(), m.angular_velocity.z - self.gyro_bias
        self._gyro_raw.append((now, m.angular_velocity.z))
        if self._gyro is None or now - self._gyro[0] > 0.2:
            self._gyro = (now, wz, 0.0 if self._gyro is None else self._gyro[2])
        else:
            t0, w0, yaw0 = self._gyro
            self._gyro = (now, wz, yaw0 + 0.5 * (w0 + wz) * (now - t0))

    def _on_scan(self, m):
        r = np.asarray(m.ranges, float)
        ang = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER[2]
        ok = np.isfinite(r) & (r > 0.15) & (r < 2.4)
        self._pts = np.column_stack([LASER[0] + r[ok] * np.cos(ang[ok]), LASER[1] + r[ok] * np.sin(ang[ok])])
        self._pts_t = time.monotonic()

    def _on_estop(self, m):
        self.estop = bool(m.data)
        self.log("[비상정지] %s" % ("정지" if self.estop else "해제"))

    def _http(self, path, timeout=2.0):
        with urllib.request.urlopen(self.a.station_url.rstrip("/") + path, timeout=timeout) as r:
            return json.loads(r.read().decode())

    def _poll_status(self):
        """스테이션 상태는 따로 받아 둠 (응답이 늦어도 제어 반복이 안 멈추게)."""
        while True:
            try:
                self._status = self._http("/status")
                self._status_t = time.monotonic()
            except Exception:
                pass
            time.sleep(0.1)

    # ── battle.py 가 부르는 것 ──
    def now(self):
        return time.monotonic()

    def sleep(self, sec):
        time.sleep(sec)

    def log(self, msg):
        self.log_q.put(time.strftime("%H:%M:%S ") + msg)

    def _printer(self):
        """터미널이 막혀도(와이파이 지연) 제어가 안 멈추게 기록은 뒤에서 (11-01 18:18 판 후보 원인)."""
        while True:
            line = self.log_q.get()
            if self.log_f:
                self.log_f.write(line + "\n")
                self.log_f.flush()
            print(line, flush=True)

    def odom(self):
        if self._odom is None or time.monotonic() - self._odom_t > 0.5:
            return None
        return self._odom

    def gyro_yaw(self):
        if self._gyro is None or time.monotonic() - self._gyro[0] > 0.5:
            return None
        return self._gyro[2]

    def gyro_calibrate(self, sec):
        vals, t_end = [], time.monotonic() + sec
        while time.monotonic() < t_end:
            if self._gyro is not None:
                vals.append(self._gyro[1])
            time.sleep(0.01)
        if vals:
            self.gyro_bias += float(np.median(vals))
            self.log("자이로 치우침 %+.2f °/s 뺌 (%d 번)" % (math.degrees(self.gyro_bias), len(vals)))
        else:
            self.log("주의: /imu/data 없음 → 오돔 방향 사용")

    def gyro_still(self, sec=1.0, still=math.radians(0.5)):
        """출발 대기 중: 최근 sec 초 동안 가만있었으면 그 구간으로 치우침을 다시 잼. 준비 직후 로봇을 들고 있었어도 내려놓고 1 초면 맞음."""
        now = time.monotonic()
        if now - self._still_t < 0.5:
            return
        self._still_t = now
        raw = tuple(self._gyro_raw)
        vals = [w for t, w in raw if now - t <= sec]
        if len(vals) < 20 or raw[0][0] > now - sec or float(np.std(vals)) > still:
            return
        bias = float(np.median(vals))
        if abs(bias - self.gyro_bias) > math.radians(0.05):
            self.log("자이로 치우침 다시 잼: %+.2f → %+.2f °/s" % (math.degrees(self.gyro_bias), math.degrees(bias)))
        self.gyro_bias = bias

    def send(self, vx, vy, wz):
        if not self.a.execute:
            return
        t = Twist()
        if not self.estop:
            t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(t)

    def scan(self):
        return self._pts if time.monotonic() - self._pts_t < 0.5 else None

    def cube(self):
        lv = self._status.get("live") if time.monotonic() - self._status_t < 0.5 else None
        return (lv["x"], lv["y"], lv.get("yaw", 0.0)) if lv and lv.get("ok") else None

    def wheel_lock(self, on):
        """바퀴 잠금 명령을 구동 포트에 한 줄 씀 (cube_station 그리퍼 명령과 같은 방법). 켜는 동안 0.5 초마다, 1 초 안 오면 펌웨어가 품."""
        now = time.monotonic()
        if not self.a.execute or (on and now - self._lock_t < 0.5):
            return
        self._lock_t = now if on else 0.0
        try:
            fd = os.open(self.a.port, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                os.write(fd, b"H,1\n" if on else b"H,0\n")
            finally:
                os.close(fd)
        except OSError as e:
            self.log("바퀴 잠금 명령 실패: %s" % e)

    def grab_start(self):
        self._grab_t = time.monotonic()
        self._busy_seen = False
        if not self.a.execute:
            return
        try:
            self._http("/do?a=grab&owner=mission")
        except Exception as e:
            self.log("스테이션 연결 안 됨: %s" % e)

    def grab_poll(self):
        """None = 진행 중, 그 밖 = 결과 글 ("ok" 면 쥠). 응답이 없으면 결과 없음으로 침."""
        if not self.a.execute:
            return "dry-run"
        try:
            st = self._http("/status")
        except Exception as e:
            return "스테이션 응답 없음: %s" % e
        if st.get("busy"):
            self._busy_seen = True
            return None
        if self._busy_seen or time.monotonic() - self._grab_t > 1.0:
            for line in (st.get("log") or [])[-6:]:
                self.log("    [스테이션] " + line)
            return st.get("result") or "결과 없음"
        return None

    def grab_abort(self):
        try:
            self._http("/do?a=abort")
        except Exception:
            pass


def record(b, path):
    """5 Hz 기록 (그림·분석용, draw_battle.py). 제어와 따로 돌림."""
    with open(path, "w") as f:
        f.write("t,x,y,yaw_deg,odom_x,odom_y,state,wall_fixes,guard,evades,errors\n")
        while True:
            if b.t0 is not None:
                f.write("%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%s,%d,%d,%d,%d\n" % (time.monotonic() - b.t0, b.x, b.y, math.degrees(b.yaw), b.ox, b.oy, b.state, b.fixes, b.guard_hits, b.evades, b.errors))
                f.flush()
            time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser(description="2임무 배틀럼블")
    ap.add_argument("--execute", action="store_true", help="실제 주행 (없으면 dry-run)")
    ap.add_argument("--start-mode", choices=["front", "turn45"], default="front")
    ap.add_argument("--match-time", type=float, default=Cfg.match_time)
    ap.add_argument("--max-v", type=float, default=Cfg.max_v)
    ap.add_argument("--wait-go", action="store_true", help="준비만 끝내고 출발 신호(/tmp/battle_go)를 기다림")
    ap.add_argument("--wheel-lock", action="store_true", help="중앙 유지 때 바퀴 잠금 (firmware/mecanum_grip_hold 를 구운 뒤에만)")
    ap.add_argument("--port", default="/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0")
    ap.add_argument("--no-wall-fix", dest="wall_fix", action="store_false", help="벽 실측 보정 끔 (오돔만)")
    ap.add_argument("--test", choices=["start", "grab", "hold", "patrol"], default=None, help="단계 시험 (없으면 경기 전체)")
    ap.add_argument("--at", type=float, nargs=2, default=None, metavar=("X", "Y"), help="시작 자리 [m, 중심 원점, 출발 때 정면 +x]. 시험할 때만")
    ap.add_argument("--station-url", default="http://127.0.0.1:8090")
    ap.add_argument("--log", default="")
    a = ap.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    io = RosIO(a)
    threading.Thread(target=rclpy.spin, args=(io,), daemon=True).start()
    if not a.execute:
        io.log("dry-run: 바퀴·그리퍼 안 움직임. Ctrl+C 로 끝")
    at = a.at or {"grab": (-Cfg.stance, 0.0), "hold": (0.0, 0.0), "patrol": (-Cfg.ring, 0.0)}.get(a.test, (Cfg.start_x, Cfg.start_y))
    cfg = Cfg(start_mode=a.start_mode, match_time=a.match_time, max_v=a.max_v, wall_fix=a.wall_fix, wheel_lock=a.wheel_lock, start_x=at[0], start_y=at[1])
    b = Battle(io, cfg)
    if a.log:
        threading.Thread(target=record, args=(b, a.log[:-4] + ".csv"), daemon=True).start()
    try:
        if a.execute:
            if a.wait_go:
                if os.path.exists(GO_FILE):
                    os.remove(GO_FILE)
            b.run(a.test, (lambda: os.path.exists(GO_FILE) or io.estop) if a.wait_go else None)
            while True:                  # 경기가 끝나도 영속도만 냄 (그리퍼 그대로)
                io.send(0.0, 0.0, 0.0)
                time.sleep(0.2)
        else:
            b.calibrate()
            while True:
                b.tick()
                io.log("위치 (%+.3f, %+.3f, %+.1f°) · 오돔만 (%+.3f, %+.3f) · 벽 보정 %d 번 · 라이다 점 %s · 큐브 %s · 앞 막힘 %s" % (
                    b.x, b.y, math.degrees(b.yaw), b.ox, b.oy, b.fixes, None if io.scan() is None else len(io.scan()), io.cube(), b.blocked(1.0, 0.0)))
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            io.send(0.0, 0.0, 0.0)
            time.sleep(0.02)
        time.sleep(0.2)                  # 기록 마저 쓰기
        io.log("끝. 잡기 스테이션·그리퍼는 그대로 둠")
        time.sleep(0.1)
        os._exit(0)


if __name__ == "__main__":
    main()
