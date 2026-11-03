#!/usr/bin/env python3
"""밀린 양을 라이다로 잼 (벽·지도 없이). 바퀴 안 움직임. 시작 때 스캔을 기억하고 지금 스캔과 맞춰 몸이 얼마나 옮겨졌는지 계산.
  source ~/mecanum_slam_ws/env.sh; python3 ~/mecanum_slam_ws/tools/push_watch.py [초]
칸: 시각, 라이다로 잰 앞(+)·왼(+) 이동 [cm], 회전 [도], 맞은 점 비율, 오돔이 본 이동 [cm]. 움직이는 사람·로봇 점은 안 맞는 점으로 빠짐."""
import math, sys, time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LASER_X, LASER_YAW = 0.095, 1.641
R_MIN, R_MAX = 0.35, 10.0        # 몸·그리퍼 점과 너무 먼 점은 뺌 [m]


def body_points(m):
    r = np.array(m.ranges, float)
    a = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER_YAW
    ok = np.isfinite(r) & (r > R_MIN) & (r < R_MAX)
    return np.column_stack((LASER_X + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))


def align(ref, tree, pts, x, y, th, gates=(0.30, 0.15, 0.08, 0.04)):
    """pts(지금 몸 기준) 를 ref(처음 몸 기준) 에 맞추는 이동 (x, y, th) = 처음 자세에서 본 지금 자세. 먼 점이 몇 cm 흔들려도 괜찮게 평균."""
    frac = 0.0
    for gate in gates:
        for _ in range(4):
            c, s = math.cos(th), math.sin(th)
            q = np.column_stack((c * pts[:, 0] - s * pts[:, 1] + x, s * pts[:, 0] + c * pts[:, 1] + y))
            d, i = tree.query(q)
            ok = d < gate
            frac = float(np.mean(ok))
            if np.count_nonzero(ok) < 30:
                return x, y, th, frac
            a, b = pts[ok], ref[i[ok]]
            ma, mb = a.mean(axis=0), b.mean(axis=0)
            a0, b0 = a - ma, b - mb
            th = math.atan2(np.sum(a0[:, 0] * b0[:, 1] - a0[:, 1] * b0[:, 0]), np.sum(a0 * b0))
            c, s = math.cos(th), math.sin(th)
            x, y = mb[0] - (c * ma[0] - s * ma[1]), mb[1] - (s * ma[0] + c * ma[1])
    return x, y, th, frac


class W(Node):
    def __init__(self):
        super().__init__("push_watch")
        self.scan, self.odom = None, None
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odometry/filtered", lambda m: setattr(self, "odom", m), 10)


rclpy.init()
n = W()
t0 = time.time()
while n.scan is None and time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.1)
if n.scan is None:
    sys.exit("/scan 안 들어옴")
refs = []
last = None
while len(refs) < 8:                      # 처음 몇 장을 합쳐 기억 (한 장은 점이 듬성듬성)
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.scan is not last:
        last = n.scan
        refs.append(body_points(last))
ref = np.vstack(refs)
tree = cKDTree(ref)
o0 = None if n.odom is None else (n.odom.pose.pose.position.x, n.odom.pose.pose.position.y)
print("기억한 점 %d 개. 지금부터 밀어 보세요" % len(ref), flush=True)
x, y, th = 0.0, 0.0, 0.0
t_print = 0.0
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    if n.scan is last:
        continue
    last = n.scan
    t1 = time.time()
    x, y, th, frac = align(ref, tree, body_points(last), x, y, th)
    dt = time.time() - t1
    if time.time() - t_print >= 0.5:
        t_print = time.time()
        od = -1.0
        if n.odom is not None and o0 is not None:
            od = 100 * math.hypot(n.odom.pose.pose.position.x - o0[0], n.odom.pose.pose.position.y - o0[1])
        print("%5.1f s 앞 %+6.1f 왼 %+6.1f cm (거리 %5.1f), 회전 %+5.1f°, 맞은 점 %3.0f%%, 오돔 %5.1f cm, 계산 %2.0f ms" % (
            time.time() - t0, 100 * x, 100 * y, 100 * math.hypot(x, y), math.degrees(th), 100 * frac, od, 1000 * dt), flush=True)
