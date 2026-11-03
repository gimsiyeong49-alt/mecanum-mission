#!/usr/bin/env python3
"""지금 라이다 스캔을 몸 기준 점으로 저장. 바퀴 안 움직임. RViz 없이 로컬 모드에서 벽을 확인할 때.
  source ~/mecanum_slam_ws/env.sh; python3 ~/mecanum_slam_ws/tools/scan_dump.py 파일.npy [장 수]"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

OUT = sys.argv[1]
N = int(sys.argv[2]) if len(sys.argv) > 2 else 12
LASER_X, LASER_YAW = 0.095, 1.641
S = []
rclpy.init()
n = Node("scan_dump")
n.create_subscription(LaserScan, "/scan", lambda m: S.append(m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 6 and len(S) < N:
    rclpy.spin_once(n, timeout_sec=0.1)
if not S:
    sys.exit("/scan 안 들어옴")
P = []
for m in S:
    r = np.array(m.ranges, float)
    a = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER_YAW
    ok = np.isfinite(r) & (r > 0.01)
    P.append(np.column_stack((LASER_X + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]))))
np.save(OUT, np.vstack(P))
print("저장 %d 장, 점 %d 개" % (len(S), sum(len(p) for p in P)))
