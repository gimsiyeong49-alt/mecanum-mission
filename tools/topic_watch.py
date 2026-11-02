#!/usr/bin/env python3
"""통신 끊김 시험용: 로봇 안 토픽이 끊기는지 잰다. 바퀴 안 움직임.
  python3 tools/topic_watch.py 90     # 90 초 동안 재고 토픽별 초당 횟수·가장 긴 끊김을 찍음
가장 긴 끊김이 1 초를 넘으면 그 토픽이 와이파이에 묶여 있다고 본다 (임무 pose() 대기 3 초, 2임무 오돔 신선도 0.5 초)."""
import sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
rec = {}


def mark(name):
    now = time.monotonic()
    r = rec.setdefault(name, dict(n=0, last=now, gap=0.0, gap_at=0.0))
    if now - r["last"] > r["gap"]:
        r["gap"], r["gap_at"] = now - r["last"], now - T0
    r["last"] = now
    r["n"] += 1


def on_tf(m):
    for t in m.transforms:
        if t.child_frame_id == "odom":
            mark("격자 위치추정 map→odom")
        elif t.child_frame_id == "base_link":
            mark("EKF odom→base_link")


rclpy.init()
n = Node("topic_watch")
T0 = time.monotonic()
n.create_subscription(LaserScan, "/scan", lambda m: mark("/scan"), qos_profile_sensor_data)
n.create_subscription(Imu, "/imu/data", lambda m: mark("/imu/data"), qos_profile_sensor_data)
n.create_subscription(Odometry, "/odometry/filtered", lambda m: mark("/odometry/filtered"), 10)
n.create_subscription(TFMessage, "/tf", on_tf, 50)
t_print = T0
while time.monotonic() - T0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    now = time.monotonic()
    if now - t_print > 5.0:
        t_print = now
        print("%5.0f 초: " % (now - T0) + " · ".join("%s %.1f 초 전" % (k, now - r["last"]) for k, r in sorted(rec.items())), flush=True)
now = time.monotonic()
print("\n토픽                        초당    가장 긴 끊김   그때")
bad = 0
for k in ("/scan", "/imu/data", "/odometry/filtered", "EKF odom→base_link", "격자 위치추정 map→odom"):
    r = rec.get(k)
    if r is None:
        print("%-26s  한 번도 안 들어옴" % k); bad += 1; continue
    gap = max(r["gap"], now - r["last"])
    bad += gap > 1.0
    print("%-26s %5.1f   %6.2f 초     %5.0f 초" % (k, r["n"] / DUR, gap, r["gap_at"]))
print("판정: 끊김 없음 (전부 1 초 안)" if not bad else "판정: 끊김 있음 (%d 개) → field_dds.sh local 뒤 다시" % bad)
