#!/usr/bin/env python3
"""정면 벽이 가까울 때 라이다에 어디까지 찍히는지 잼. 바퀴 안 움직임. 로봇 앞에 벽을 두고 실행.
  source ~/mecanum_slam_ws/env.sh; python3 ~/mecanum_slam_ws/tools/front_scan.py [스캔 수]
몸 기준 x = 앞, y = 왼쪽 [m]. 지금 앞벽 계산 띠(|y| < 8.25 cm)와 넓힌 띠로 앞면 위치를 각각 계산."""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = int(sys.argv[1]) if len(sys.argv) > 1 else 24
LASER_X, LASER_YAW, FRONT = 0.095, 1.641, 0.191
BANDS = (0.0825, 0.12, 0.16, 0.19)
S = []
rclpy.init()
n = Node("front_scan")
n.create_subscription(LaserScan, "/scan", lambda m: S.append(m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 6 and len(S) < N:
    rclpy.spin_once(n, timeout_sec=0.1)
if not S:
    sys.exit("/scan 안 들어옴")
xs, ys, angs, rs = [], [], [], []
for m in S:
    r = np.array(m.ranges, float)
    a = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER_YAW
    ok = np.isfinite(r) & (r > 0.01)
    xs.append(LASER_X + r[ok] * np.cos(a[ok]))
    ys.append(r[ok] * np.sin(a[ok]))
    angs.append(np.degrees(np.arctan2(np.sin(a[ok]), np.cos(a[ok]))))
    rs.append(r[ok])
x, y, ang, rr = map(np.concatenate, (xs, ys, angs, rs))
print("스캔 %d 장, 드라이버 range_min %.3f, 찍힌 최소 거리 %.3f m" % (len(S), S[0].range_min, rr.min()))
print("앞쪽 각도별 (스캔당 점 수, 라이다 거리, 몸 기준 x, y)")
for lo in range(-90, 90, 10):
    m = (ang >= lo) & (ang < lo + 10)
    if m.any():
        print("  %4d~%4d도: %5.1f 점, 거리 %.3f, x %.3f, y %+.3f" % (lo, lo + 10, m.sum() / len(S), np.median(rr[m]), np.median(x[m]), np.median(y[m])))
    else:
        print("  %4d~%4d도:   0 점" % (lo, lo + 10))
print("앞면까지 (몸 앞끝 기준, 아래 20%)")
for b in BANDS:
    m = (np.abs(y) < b) & (x > FRONT - 0.05) & (x < FRONT + 0.5)
    k = m.sum() / len(S)
    if m.sum() >= 5:
        print("  띠 |y|<%4.1f cm: 스캔당 %5.1f 점, 앞끝에서 %5.1f cm, 흩어짐 %.1f cm, 점 폭 y %+.3f~%+.3f" % (100 * b, k, 100 * (np.percentile(x[m], 20) - FRONT), 100 * np.std(x[m]), y[m].min(), y[m].max()))
    else:
        print("  띠 |y|<%4.1f cm: 점 부족 (스캔당 %.1f)" % (100 * b, k))
