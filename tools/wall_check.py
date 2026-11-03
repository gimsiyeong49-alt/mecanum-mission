#!/usr/bin/env python3
"""라이다 수평 실시간 확인: 네 방향 벽이 다 찍히는지, 레이저가 벽 위로 넘어가는지. 바퀴·임무는 안 건드림. Ctrl+C 로 끝.
  ssh -t uoeee@172.20.137.29 'source ~/mecanum_slam_ws/env.sh; python3 ~/mecanum_slam_ws/tools/wall_check.py'
로봇을 경기장 가운데에 벽과 나란히 두고 봄 (네 벽 모두 약 1.8 m). 라이다가 안 켜져 있으면 라이다만 켜고, 끝낼 때 끔.
라이다 높이 17~18 cm, 벽 20 cm 라 여유가 2~3 cm 뿐: 1° 기울면 약 1.4 m 넘는 벽은 레이저가 위로 넘어감 (11-03 왼쪽 벽 사라짐).
'넘음' = 벽 줄 너머에서 찍힌 점 (경기장 밖) + 아무것도 안 찍힌 빔. 수평이면 네 방향 모두 거의 0 %."""
import argparse, math, os, re, shutil, signal, subprocess, sys, time
from collections import deque
import numpy as np

LASER_X, LASER_YAW = 0.095, 1.641            # base_link → laser (robot.launch 정적 TF)
LIDAR_H, WALL_H = 0.175, 0.20                # 라이다 높이 (17~18 cm), 규정 벽 높이
SIDES = (("앞", 0.0), ("왼", math.pi / 2), ("뒤", math.pi), ("오", -math.pi / 2))
W = os.path.expanduser("~/mecanum_slam_ws")


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def side_stats(r, a, half, max_d):
    """r, a = 빔 거리·몸 기준 각 (여러 장 이어 붙임). 방향마다 벽 거리, 벽 점 비율, 넘음 비율, 벽 점 가장 먼 거리."""
    ok = np.isfinite(r) & (r > 0.05)
    x, y = LASER_X + r * np.cos(a), r * np.sin(a)
    out = []
    for name, th in SIDES:
        sec = np.abs(wrap(a - th)) < half
        nb = int(np.count_nonzero(sec))
        m = sec & ok
        c, s = math.cos(th), math.sin(th)
        al, ac = c * x[m] + s * y[m], -s * x[m] + c * y[m]          # 그 방향으로 얼마나 (along), 옆으로 얼마나 (across)
        rr = r[m]
        cand = (al > 0.25) & (al < max_d)
        best = None
        if np.count_nonzero(cand) >= 15:
            for k in np.linspace(-0.09, 0.09, 13):                  # 벽과 몸이 ±5° 안에서 비뚤어도 곧은 줄을 찾음
                p = al[cand] - k * ac[cand]
                h, e = np.histogram(p, bins=np.arange(0.25, max_d + 0.02, 0.02))
                i = int(np.argmax(h))
                if best is None or h[i] > best[0]:
                    best = (int(h[i]), k, 0.5 * (e[i] + e[i + 1]))
        if best is not None and best[0] >= 10:
            _, k, d0 = best
            on = np.abs(al - k * ac - d0) < 0.03
            d = float(np.median(al[on] - k * ac[on]))
            over = int(np.count_nonzero(al - k * ac > d + 0.10)) + int(np.count_nonzero(sec & ~ok))
            out.append((name, d, np.count_nonzero(on) / max(nb, 1), over / max(nb, 1), float(rr[on].max())))
        else:
            over = int(np.count_nonzero(al > 0.25)) + int(np.count_nonzero(sec & ~ok))
            out.append((name, None, 0.0, over / max(nb, 1), None))
    return out


def lines(stats, over_lim):
    L = ["  방향  벽까지   벽 점   넘음   벽 점 가장 먼 곳"]
    up = []
    for name, d, f_on, f_over, far in stats:
        if d is None:
            L.append("  %s    벽 못 찾음        %3.0f %%  (넘었거나 너무 멂)" % (name, 100 * f_over))
            up.append(name)
            continue
        if f_over > over_lim:                                       # 각도는 안 냄: 기운 방향이 그 방향과 다르면 틀린 값이 나옴
            up.append(name)
        L.append("  %s    %.2f m   %3.0f %%  %3.0f %%   %.2f m" % (name, d, 100 * f_on, 100 * f_over, far))
    if up:
        L.append("  넘는 쪽: %s → 그쪽이 올라감. 그쪽을 낮추거나 반대쪽 밑을 받침" % ", ".join(up))
    else:
        L.append("  네 방향 모두 넘음 %.0f %% 이하 → 수평 괜찮음" % (100 * over_lim))
    return L


def lidar_pid():
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            c = open("/proc/%s/cmdline" % d, "rb").read().split(b"\0")[0]
        except OSError:
            continue
        if c.endswith(b"/sllidar_node"):
            return int(d)
    return None


def start_lidar(mode):
    src = open(W + "/src/mecanum_slam/launch/robot.launch.py", encoding="utf-8").read()
    port = re.search(r'^LIDAR_PORT *= *"([^"]+)"', src, re.M).group(1)
    log = open("/tmp/wall_check_lidar.log", "w")
    return subprocess.Popen(["ros2", "run", "sllidar_ros2", "sllidar_node", "--ros-args", "-r", "__node:=sllidar_node",
                             "-p", "channel_type:=serial", "-p", "serial_port:=" + port, "-p", "serial_baudrate:=256000",
                             "-p", "frame_id:=laser", "-p", "inverted:=false", "-p", "angle_compensate:=true", "-p", "scan_mode:=" + mode],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)    # ros2 run 밑의 실제 노드까지 한 묶음으로 끄려고


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="Sensitivity", help="라이다를 이 도구가 켤 때 모드 (2임무 = Sensitivity, 1임무 = Boost)")
    ap.add_argument("--half", type=float, default=30.0, help="방향마다 볼 부채꼴 반각 [°]. 가운데서 ±30° 면 벽 2.08 m 까지 봄")
    ap.add_argument("--max-d", type=float, default=2.6, help="이 거리 안에서만 벽을 찾음 [m]")
    ap.add_argument("--scans", type=int, default=6, help="몇 장 모아서 볼지 (12 Hz 라 6 장 = 0.5 초)")
    ap.add_argument("--over", type=float, default=0.05, help="넘음이 이 비율을 넘으면 그쪽이 올라간 것으로 봄")
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    child = None
    if lidar_pid() is None:
        print("라이다가 안 켜져 있음 → 라이다만 켬 (%s, 끝낼 때 끔). 기록 /tmp/wall_check_lidar.log" % a.mode, flush=True)
        child = start_lidar(a.mode)
    else:
        print("켜져 있는 라이다 (PID %d) 를 그대로 씀" % lidar_pid(), flush=True)
    signal.signal(signal.SIGHUP, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))     # 접속이 끊겨도 켠 라이다를 끄고 나감
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    buf = deque(maxlen=a.scans)
    rclpy.init()
    n = Node("wall_check")
    n.create_subscription(LaserScan, "/scan", lambda m: buf.append(m), qos_profile_sensor_data)
    print("경기장 가운데에 벽과 나란히 두고 보세요 (네 벽 모두 약 1.8 m). 부채꼴 ±%.0f°, Ctrl+C 로 끝" % a.half, flush=True)
    drawn, t_draw, t0 = 0, 0.0, time.monotonic()
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.monotonic() - t_draw < 0.2:
                continue
            t_draw = time.monotonic()
            if not buf:
                if time.monotonic() - t0 > 15 and drawn == 0:
                    print("15 초째 /scan 안 들어옴 (라이다 선·전원 확인, /tmp/wall_check_lidar.log)", flush=True)
                    t0 = time.monotonic()
                continue
            R, A = [], []
            for m in list(buf):
                r = np.array(m.ranges, float)
                R.append(r)
                A.append(m.angle_min + m.angle_increment * np.arange(len(r)) + LASER_YAW)
            L = lines(side_stats(np.concatenate(R), np.concatenate(A), math.radians(a.half), a.max_d), a.over)
            L.insert(0, time.strftime("  %H:%M:%S") + "  (스캔 %d 장)" % len(buf))
            w = shutil.get_terminal_size((80, 20)).columns - 1
            if drawn:
                sys.stdout.write("\033[%dF" % drawn)                 # 그린 줄 수만큼 올라가서 덮어씀
            sys.stdout.write("".join("\033[K" + s[:w] + "\n" for s in L))
            sys.stdout.flush()
            drawn = len(L)
    except KeyboardInterrupt:
        pass
    finally:
        if child is not None:                                       # ros2 run 만 끄면 밑의 sllidar_node 가 남음 (11-03) → 묶음째 끄고 없어질 때까지 기다림
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                child.wait(6)                                       # ros2 run 은 밑의 노드가 끝나야 끝남
            except subprocess.TimeoutExpired:
                pass
            t_end = time.monotonic() + 3.0
            while lidar_pid() is not None and time.monotonic() < t_end:
                time.sleep(0.2)
            if lidar_pid() is not None:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            print("이 도구가 켠 라이다 끔" if lidar_pid() is None else "라이다가 안 꺼짐: PID %s" % lidar_pid(), flush=True)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
