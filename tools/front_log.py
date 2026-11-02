#!/usr/bin/env python3
"""달리는 동안 정면 앞쪽 라이다 점을 스캔마다 기록. 읽기만 함. 주행에 영향 없음.
  python3 front_log.py 파일.csv [초]
칸: 시각, 띠별 점 수(|y|<8.25, <12, <16 cm), 앞끝 기준 앞면 거리(|y|<16 cm, 점 5개 이상일 때), 점 폭 y 최소·최대"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

OUT = sys.argv[1]
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 130.0
LASER_X, LASER_YAW, FRONT = 0.095, 1.641, 0.191
f = open(OUT, "w", buffering=1)
f.write("t,n8,n12,n16,gap16,ymin,ymax\n")


def on_scan(m):
    r = np.array(m.ranges, float)
    a = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER_YAW
    ok = np.isfinite(r) & (r > 0.01)
    x, y = LASER_X + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    near = (x > FRONT - 0.05) & (x < FRONT + 0.30)
    n8, n12 = int((near & (np.abs(y) < 0.0825)).sum()), int((near & (np.abs(y) < 0.12)).sum())
    b = near & (np.abs(y) < 0.16)
    n16 = int(b.sum())
    if n16 >= 5:
        f.write("%.3f,%d,%d,%d,%.4f,%.3f,%.3f\n" % (time.time(), n8, n12, n16, np.percentile(x[b], 20) - FRONT, y[b].min(), y[b].max()))
    else:
        f.write("%.3f,%d,%d,%d,,,\n" % (time.time(), n8, n12, n16))


rclpy.init()
n = Node("front_log")
n.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
