"""제자리 회전 (오돔 되먹임, 자리 유지): python3 turn_rel.py 각도[°, 왼쪽 +] [--force].
돌기 전 라이다로 몸이 쓸고 갈 자리(turnspot 몸 모양)가 벽에서 1 cm 남는지 봄 — 배터리 쪽(몸 기준 120~163°, 193~260°)은 라이다가 못 봄."""
import math, os, sys, time, threading
import numpy as np, shapely
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
sys.path.insert(0, os.path.expanduser("~/mecanum_slam_ws/mission1_field"))
import turnspot as TS

FORCE = "--force" in sys.argv; sys.argv = [x for x in sys.argv if x != "--force"]   # --force: 잡기 스테이션이 쉬고 있을 때
deg = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
if abs(deg) > 180: sys.exit("180° 넘음")
LASER = (0.11, 0.0, 1.641); CLEAR = 0.01; FOOTBUF = TS.FOOT.buffer(0.005)
rclpy.init(); node = Node("turn_rel"); pub = node.create_publisher(Twist, "/cmd_vel", 10); pose = {}; scan = {}


def cb(m):
    q = m.pose.pose.orientation
    pose["p"] = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))); pose["t"] = time.time()


def scb(m):
    r = np.asarray(m.ranges, float); ang = m.angle_min + m.angle_increment * np.arange(len(r)) + LASER[2]
    ok = np.isfinite(r) & (r > 0.15) & (r < 1.5)
    P = np.column_stack([LASER[0] + r[ok] * np.cos(ang[ok]), LASER[1] + r[ok] * np.sin(ang[ok])])
    if len(P): P = P[~shapely.contains_xy(FOOTBUF, P[:, 0], P[:, 1])]
    scan.update(pts=P, t=time.time())


node.create_subscription(Odometry, "/odometry/filtered", cb, 10)
node.create_subscription(LaserScan, "/scan", scb, qos_profile_sensor_data)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
for _ in range(50):
    if "p" in pose and "pts" in scan: break
    time.sleep(0.1)
if "p" not in pose: sys.exit("오돔 없음")
if "pts" not in scan: sys.exit("라이다 없음")
if node.count_publishers("/cmd_vel") > (2 if FORCE else 1): sys.exit("/cmd_vel 다른 발행자 있음")
dyaw = math.radians(deg); P = scan["pts"]
c_ = 9.0 if len(P) == 0 else float(shapely.distance(TS.sweep_rot(0, 0, 0.0, dyaw), shapely.multipoints(P)))
print("돌 자리 벽까지 %.1f cm" % (100 * c_), flush=True)
if c_ < CLEAR: sys.exit("벽이 %.0f cm 안 → 돌지 않음" % (100 * CLEAR))
x0, y0, h0 = pose["p"]; ht = h0 + dyaw; t0 = time.time(); ew = dyaw
try:
    while True:
        x, y, h = pose["p"]; ew = math.atan2(math.sin(ht - h), math.cos(ht - h))
        if abs(ew) < math.radians(2.0) or time.time() - t0 > 12 or time.time() - pose["t"] > 0.5: break
        w = max(-1.2, min(1.2, 2.0 * ew)); w = math.copysign(max(abs(w), 0.75), ew)
        ex, ey = x0 - x, y0 - y; bx = ex * math.cos(h) + ey * math.sin(h); by = -ex * math.sin(h) + ey * math.cos(h)
        tw = Twist(); tw.linear.x = max(-0.08, min(0.08, 3.0 * bx)); tw.linear.y = max(-0.08, min(0.08, 3.0 * by)); tw.angular.z = w
        pub.publish(tw); time.sleep(0.05)
finally:
    for _ in range(5): pub.publish(Twist()); time.sleep(0.02)
    time.sleep(0.5); x, y, h = pose["p"]
    print("끝: 남은 각도 %.1f°, 자리 밀림 %.1f cm, %.1f 초" % (math.degrees(math.atan2(math.sin(ht - h), math.cos(ht - h))), 100 * math.hypot(x - x0, y - y0), time.time() - t0), flush=True)
    node.destroy_node(); rclpy.try_shutdown(); os._exit(0)
