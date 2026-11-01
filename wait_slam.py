#!/usr/bin/env python3
"""SLAM 준비 확인: /scan 이 들어오고 map→base_link TF 가 잡힐 때까지 기다린다 (최대 --timeout 초)."""
import argparse, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
ap = argparse.ArgumentParser(); ap.add_argument("--timeout", type=float, default=40.0); a = ap.parse_args()
rclpy.init(); n = Node("wait_slam"); buf = Buffer(); TransformListener(buf, n); got = {"scan": 0}
n.create_subscription(LaserScan, "/scan", lambda m: got.__setitem__("scan", got["scan"] + 1), qos_profile_sensor_data)
t0 = time.monotonic(); ok = False; last = -1
while time.monotonic() - t0 < a.timeout:
    rclpy.spin_once(n, timeout_sec=0.2)
    tf_ok = buf.can_transform("map", "base_link", rclpy.time.Time())
    if got["scan"] > 10 and tf_ok:
        ok = True; break
    s = int(time.monotonic() - t0)
    if s != last and s % 5 == 0:
        last = s; print("  위치추정 기다리는 중 %2d초 (라이다 %s, 위치 %s)" % (s, "OK" if got["scan"] > 10 else "아직", "OK" if tf_ok else "아직"), flush=True)
print("위치추정 준비 완료" if ok else "위치추정 준비 실패 (%.0f초): 라이다 %d 스캔, 위치 TF %s" % (a.timeout, got["scan"], "있음" if buf.can_transform("map", "base_link", rclpy.time.Time()) else "없음"))
n.destroy_node(); rclpy.try_shutdown(); sys.exit(0 if ok else 1)
