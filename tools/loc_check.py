#!/usr/bin/env python3
"""화면 없이 위치추정 확인: 지금 위치·방향과, 라이다로 직접 잰 사방 거리를 글자로 찍음. 바퀴 안 움직임, 아무것도 안 바꿈.
  python3 tools/loc_check.py          # 1 초마다 (Ctrl+C 로 끝)
  python3 tools/loc_check.py --once   # 한 번만
출발 칸에 놓고 임무를 준비한 상태에서 보면 '출발 칸 중심에서 몇 cm' 가 나옴. 로컬 모드에서도 SSH 로 됨 (RViz 만 안 됨).
좌표 map 원점 = 출발 칸 (1,9) 가운데, side=right 면 정면이 남쪽. 사방 거리는 몸 가운데(바퀴 축) 기준."""
import argparse, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LASER = (0.095, 0.0, 1.641)
ap = argparse.ArgumentParser()
ap.add_argument("--once", action="store_true")
ap.add_argument("--side", default="right", choices=["right", "left"])
ap.add_argument("--cell", type=float, default=0.40)
a = ap.parse_args()
rclpy.init()
n = Node("loc_check")
buf = Buffer()
TransformListener(buf, n)
scan = {}


def on_scan(m):
    r = np.asarray(m.ranges, float)
    ang = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER[2]
    ok = np.isfinite(r) & (r > 0.12) & (r < 6.0)
    scan.update(x=LASER[0] + r[ok] * np.cos(ang[ok]), y=LASER[1] + r[ok] * np.sin(ang[ok]), t=time.monotonic())


n.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)


def wall_dists():
    """몸 기준 앞·뒤·왼·오 방향 (폭 ±10 cm) 가장 가까운 점까지 거리의 중앙값 근처 [cm]."""
    if "x" not in scan or time.monotonic() - scan["t"] > 1.0:
        return "라이다 없음"
    x, y = scan["x"], scan["y"]
    out = []
    for name, m, d in (("앞", (x > 0) & (np.abs(y) < 0.10), x), ("뒤", (x < 0) & (np.abs(y) < 0.10), -x), ("왼", (y > 0) & (np.abs(x) < 0.10), y), ("오", (y < 0) & (np.abs(x) < 0.10), -y)):
        v = d[m]
        out.append("%s %s" % (name, "%.0f cm" % (100 * np.percentile(v, 20)) if v.size >= 3 else "-"))
    return " · ".join(out)


def once():
    try:
        t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
    except Exception:
        return "위치추정 없음 (기반이 켜져 있는지 확인) · 사방 %s" % wall_dists(), False
    age = (n.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
    x, y = t.transform.translation.x, t.transform.translation.y
    q = t.transform.rotation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    d = math.hypot(x, y)
    good = age < 0.5 and d < 0.06 and abs(yaw) < 5.0
    return "%s 출발 칸 중심에서 %.1f cm (앞 %+.1f, 왼 %+.1f), 방향 %+.1f°, %.2f 초 전 값 · 사방 %s" % (
        "정상" if good else "확인 필요", 100 * d, 100 * x, 100 * y, yaw, age, wall_dists()), good


t_end = time.monotonic() + 2.0
while time.monotonic() < t_end:                 # TF·스캔을 받을 시간
    rclpy.spin_once(n, timeout_sec=0.05)
try:
    while True:
        msg, good = once()
        print(time.strftime("%H:%M:%S ") + msg, flush=True)
        if a.once:
            raise SystemExit(0 if good else 1)
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            rclpy.spin_once(n, timeout_sec=0.05)
except KeyboardInterrupt:
    pass
