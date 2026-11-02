#!/usr/bin/env python3
"""준비된 1임무를 버튼과 같은 순서로 출발시켜 시간을 잼. 바퀴가 도니 바퀴를 띄우고 할 것. 잴 초가 지나면 임무를 끔.
  python3 start_latency.py [돌릴 초]   # 먼저 field_arm.sh 1
누름(가상) → 0.5 초에 r → 1.0 초에 g. 재는 값: r·g 를 보낸 시각, 첫 바퀴 명령, 첫 바퀴 움직임(엔코더), 끈 뒤 바퀴가 멈춘 시각"""
import os, subprocess, sys, threading, time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM = os.path.expanduser("~/mecanum_slam_ws/field_arm.sh")
RUN = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
S = dict(cmd=None, wheel=None, base=None, pos=None, moved_t=None, cmds=0)


def on_cmd(m):
    if abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z) > 1e-3:
        S["cmds"] += 1
        if S["cmd"] is None:
            S["cmd"] = time.monotonic()


def on_js(m):
    S["pos"] = list(m.position)
    if S["base"] is None:
        S["base"] = S["pos"]
        return
    if max(abs(a - b) for a, b in zip(S["pos"], S["base"])) > 0.02:       # 0.02 rad 넘게 돔
        if S["wheel"] is None:
            S["wheel"] = time.monotonic()
        S["base"] = S["pos"]
        S["moved_t"] = time.monotonic()


if not os.path.exists("/tmp/mission_armed"):
    sys.exit("준비된 임무 없음: 먼저 field_arm.sh 1")
rclpy.init()
n = Node("start_latency")
n.create_subscription(Twist, "/cmd_vel", on_cmd, 50)
n.create_subscription(JointState, "/joint_states", on_js, 50)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
time.sleep(2.0)
if S["pos"] is None:
    print("주의: /joint_states 안 들어옴 → 바퀴 움직임은 못 잼", flush=True)
t0 = time.monotonic()
try:
    time.sleep(0.5)
    subprocess.call([ARM, "recheck"])
    t_r = time.monotonic() - t0
    time.sleep(max(0.0, 1.0 - (time.monotonic() - t0)))
    subprocess.call([ARM, "go"])
    t_g = time.monotonic() - t0
    time.sleep(RUN)
finally:
    t_stop = time.monotonic()
    subprocess.call([ARM, "stop"])
time.sleep(1.5)
rel = lambda t: "-" if t is None else "%.2f 초" % (t - t0)
print("r 보냄 %.2f 초, g 보냄 %.2f 초 (누름 기준)" % (t_r, t_g))
print("첫 바퀴 명령 %s, 첫 바퀴 움직임 %s" % (rel(S["cmd"]), rel(S["wheel"])))
print("바퀴 명령 %d 번, 끄기 시작 %s, 바퀴 마지막 움직임 %s" % (S["cmds"], rel(t_stop), rel(S["moved_t"])))
