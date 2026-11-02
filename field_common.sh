# field_keyboard.sh / field_auto.sh 공용 (직접 실행하지 않음)
W=~/mecanum_slam_ws
export MISSION_DIR=${MISSION_DIR:-mission1_field}   # 지정 안 하면 대회판
source $W/env.sh
LOC=${LOC:-grid}        # grid = 오돔+격자 벽 보정 (기본, SLAM 없음) / carto = Cartographer SLAM (비교용)
mkdir -p $W/run_logs

slam_running() { screen -ls 2>/dev/null | grep -q "\.slam[[:space:]]"; }

wait_devices() {    # 전원 직후 USB 장치가 다 잡힐 때까지 (최대 60 초). 다 있으면 바로 끝남
  for i in $(seq 1 60); do
    [ -e /dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0 ] && [ -e /dev/serial/by-path/platform-3610000.usb-usb-0:2.3:1.0-port0 ] && ls /dev/serial/by-id/usb-Silicon_Labs_CP2102* >/dev/null 2>&1 && [ "$(ls /dev/v4l/by-path/ 2>/dev/null | grep -c video-index0)" -ge 2 ] && break
    sleep 1
  done
  echo "$(date '+%H:%M:%S') 장치 확인 ($i 초)"
}

# 위치추정기: 임무 폴더(MISSION_DIR)에 사본이 있으면 그것 (실험판 전용 개선판), 없으면 안정판 것
LOCALIZER=$W/${MISSION_DIR:-mission1}/grid_localizer.py
[ -f "$LOCALIZER" ] || LOCALIZER=$W/mission1/grid_localizer.py
export LOCALIZER
running_localizer() {   # 지금 켜진 위치추정기 파일 경로 (없으면 빈칸)
  for d in /proc/[0-9]*; do
    a1=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 2p)
    case "$a1" in */grid_localizer.py) echo "$a1"; return;; esac
  done
}

start_slam() {      # 기반(구동·센서·위치추정)을 새로 켠다
  L=$W/run_logs/slam_$(date +%m%d_%H%M%S).log
  if [ "$LOC" = carto ]; then
    echo "[기반] Cartographer SLAM 켜는 중 → 기록 $L"
    screen -dmS slam bash -c "$W/start_mapping.sh > $L 2>&1"
  else
    echo "[기반] 구동·IMU·EKF·라이다 + 격자 위치추정(SLAM 없음) 켜는 중 → 기록 $L"
    screen -dmS slam bash -c "$W/start_grid.sh > $L 2>&1"
  fi
  sleep 3
  if ! python3 $W/wait_slam.py --timeout 40; then
    echo "기반이 안 켜졌습니다. 기록 끝부분:"; tail -15 $L
    stop_slam; exit 1
  fi
}

ensure_base() {     # 기반이 켜져 있으면 그대로 쓰고(빠름), 없으면 켠다
  if [ "$LOC" != carto ] && slam_running && [ "$(running_localizer)" != "$LOCALIZER" ]; then
    echo "[기반] 켜진 위치추정기($(running_localizer))가 이번 임무용($LOCALIZER)과 달라 기반을 다시 켭니다"
    stop_slam
  fi
  if slam_running && python3 $W/wait_slam.py --timeout 5 > /dev/null 2>&1; then
    echo "[기반] 이미 켜져 있음 → 그대로 사용 (원점은 임무 시작 때 지금 자리로 다시 잡음)"
  else
    slam_running && stop_slam
    start_slam
  fi
}

stop_slam() {
  slam_running || return 0
  screen -S slam -X stuff $'\003'            # launch 에 Ctrl+C
  for i in $(seq 1 20); do slam_running || break; sleep 0.5; done
  slam_running && screen -S slam -X quit
  sleep 1
  echo "  위치추정(구동 포함) 꺼짐"
}

check_no_keyboard() {
  for d in /proc/[0-9]*; do
    a1=$(tr '\0' '\n' < $d/cmdline 2>/dev/null | sed -n 2p)
    if [ "${a1##*/}" = "mecanum_keyboard" ]; then
      echo "키보드 조종(keyboard.sh)이 켜져 있습니다 (PID ${d#/proc/}). 먼저 끄세요."; exit 1
    fi
  done
}

ask_stop_slam() {
  read -r -p "기반(구동·센서·위치추정)도 끌까요? 다시 할 거면 켜 두는 게 빠름 [y/N] " ans
  case "$ans" in y|Y) stop_slam;; *) echo "기반은 켜 둡니다. 다음 임무는 바로 시작됩니다 (전부 끄기: ~/mecanum_slam_ws/field_stop.sh)";; esac
}
