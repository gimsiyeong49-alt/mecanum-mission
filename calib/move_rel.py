"""짧은 이동 (오돔 되먹임, 방향 유지): python3 move_rel.py 앞[m] [왼쪽 m]. 예: -0.20 = 뒤로 20 cm. 최대 0.5 m."""
import math, sys, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

FORCE = "--force" in sys.argv; sys.argv = [x for x in sys.argv if x != "--force"]   # --force: 잡기 스테이션이 쉬고 있을 때 (그쪽도 /cmd_vel 발행자라 수가 2)
dx = float(sys.argv[1]); dy = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
if math.hypot(dx, dy) > 0.5: sys.exit("0.5 m 넘음")
rclpy.init(); node = Node("move_rel"); pub = node.create_publisher(Twist, "/cmd_vel", 10); pose = {}


def cb(m):
    q = m.pose.pose.orientation
    pose["p"] = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))); pose["t"] = time.time()


node.create_subscription(Odometry, "/odometry/filtered", cb, 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
for _ in range(50):
    if "p" in pose: break
    time.sleep(0.1)
if "p" not in pose: sys.exit("오돔 없음")
if node.count_publishers("/cmd_vel") > (2 if FORCE else 1): sys.exit("/cmd_vel 다른 발행자 있음")
x0, y0, h0 = pose["p"]; t0 = time.time(); minv = 0.16 if abs(dy) > abs(dx) else 0.10
try:
    while True:
        x, y, h = pose["p"]; ex, ey = x - x0, y - y0
        px = ex * math.cos(h0) + ey * math.sin(h0); py = -ex * math.sin(h0) + ey * math.cos(h0)
        rx, ry = dx - px, dy - py; d = math.hypot(rx, ry)
        if d < 0.004 or time.time() - t0 > 15 or time.time() - pose["t"] > 0.5: break
        sp = max(minv, min(0.16, 1.5 * d)); ch, sh = math.cos(h - h0), math.sin(h - h0)
        tw = Twist(); tw.linear.x = sp * (rx * ch + ry * sh) / d; tw.linear.y = sp * (-rx * sh + ry * ch) / d
        tw.angular.z = 1.5 * math.atan2(math.sin(h0 - h), math.cos(h0 - h)); pub.publish(tw); time.sleep(0.05)
finally:
    for _ in range(5): pub.publish(Twist()); time.sleep(0.02)
    print("끝 (남은 %.1f mm)" % (1000 * d), flush=True); node.destroy_node(); rclpy.try_shutdown()
    import os; os._exit(0)
