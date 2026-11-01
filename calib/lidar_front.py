"""라이다로 몸 기준 정면(±3°)·뒤(±3°) 거리 중앙값 [m]. 라이다는 몸 기준 yaw 1.641 rad 로 달림 → 몸 각도 0 = 라이다 각도 −1.641."""
import math, sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
LYAW = 1.641
rclpy.init(); n = Node("lidar_front"); got = []
n.create_subscription(LaserScan, "/scan", lambda m: got.append(m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < 10 and time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.2)
def med(body_deg):
    vals = []
    for m in got:
        r = np.array(m.ranges); a = m.angle_min + m.angle_increment * np.arange(len(r)) + LYAW
        d = np.degrees(np.arctan2(np.sin(a), np.cos(a))) - body_deg
        d = (d + 180) % 360 - 180
        sel = (np.abs(d) < 3) & np.isfinite(r) & (r > 0.05)
        if sel.any(): vals.append(np.median(r[sel]))
    return float(np.median(vals)) if vals else float("nan")
print("앞 %.4f 뒤 %.4f 왼 %.4f 오른 %.4f m (%d 스캔)" % (med(0), med(180), med(90), med(-90), len(got)))
