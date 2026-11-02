# 메카넘 로봇 1임무

제29회 창원대 자율로봇경진대회 1임무(자율탐색 및 요구조자 구출)용 코드다. 9×9 격자(칸 40 cm)에서 (5,5)의 빨간 큐브를 잡아 출발 칸으로 가져온다.

2026-11-01 저녁 로봇(Jetson)의 `~/mecanum_slam_ws`에 올라가 있던 판을 그대로 옮겼고, 로그와 녹화, 백업은 뺐다. 옮긴 파일의 md5는 `MANIFEST_robot_md5.txt`에 있다. `README.md`, `.gitignore`, `start_station.sh`, `MANIFEST_robot_md5.txt`는 이 저장소를 만들면서 추가한 파일이다.

대회 주행에 안 쓰는 지도 작성 파일과 카메라 캘리브레이션 도구 같은 보조 도구는 뺐다. 필요하면 첫 커밋(8f68d61)에서 꺼내 쓴다. 로봇에서 옮긴 파일 중 고친 것은 지도 작성 파일을 빼느라 손댄 `src/mecanum_slam`의 `CMakeLists.txt`와 `package.xml`, 기본 임무 폴더를 `mission1_field`로 정한 `field_common.sh`, 잡기 스테이션을 켜 주게 한 `field_compete.sh`, 로봇 안 통신 방식을 고르게 한 `env.sh`, 출발 직전 위치를 확인하게 한 `mission1_field/mission1_node.py`뿐이다.

## 2임무 (배틀럼블)

`mission2/`에 있다. 판단과 주행은 `battle.py`, ROS와 잡기 스테이션 연결은 `battle_node.py`가 맡는다. 1임무 파일은 건드리지 않는다.

출발 칸은 지정하지 않는다. 네 칸 모두 경기장 중심이 왼쪽 앞 45°라 같은 좌표로 계산된다. 규정 그림 3의 방향으로 놓기만 하면 된다.

```bash
~/mecanum_slam_ws/field_battle.sh ready   # 준비 시간에: 기반과 잡기 스테이션을 켜고 확인
~/mecanum_slam_ws/field_battle.sh dry     # 바퀴가 안 움직임: 위치, 라이다, 큐브 확인
~/mecanum_slam_ws/field_battle.sh go      # 심판 신호에: 로봇 안에서 출발
~/mecanum_slam_ws/mission2/stop.sh        # 비상정지. 그리퍼는 그대로 둔다
```

위치는 격자 위치추정을 쓰지 않고 오돔, 자이로, 외벽까지의 라이다 거리로 구한다. 벽 보정을 끄려면 `go --no-wall-fix`, 제자리 45° 출발은 `go --start-mode turn45`를 쓴다. 기록은 `mission2/logs/`에 남는다.

## 통신

대회에서는 임무 시작 뒤 통신이 끊긴다. `field_dds.sh local`로 두면 로봇 안 ROS가 와이파이와 상관없이 돈다. 이때 노트북 RViz에는 안 보이고, 평소로 돌리려면 `field_dds.sh wifi`를 쓴다. 끊김 측정은 `python3 tools/topic_watch.py 90`이다.

대회장 라우터에는 USB-C로 접속해 `field_wifi.sh "이름" "비밀번호"`로 붙인다. 재부팅해도 다시 붙고, `field_wifi.sh remove`로 지운다. 대회 전 점검은 `field_check.sh`로 하며 아무것도 바꾸지 않는다. 화면 없이 위치를 보려면 `python3 tools/loc_check.py`를 쓴다.

라운드마다 `field_arm.sh 1` 또는 `field_arm.sh 2`로 준비해 두면 신호 때 `field_arm.sh go` 한 번으로 출발한다. 로봇 안 screen에서 돌므로 접속이 끊겨도 된다.

`field_autostart.sh on`을 해 두면 로봇 전원을 켤 때 기반과 잡기 스테이션이 저절로 켜진다. 임무를 시작할 때 원점을 다시 잡으므로 미리 켜 둔 뒤 로봇을 옮겨도 된다. 끄려면 `field_autostart.sh off`다.

## 버튼

`field_button.py`가 버튼 네 개로 두 임무를 띄운다. 1번은 1임무 준비(약 15초), 2번은 1초 뒤 1임무 출발, 3번은 1초 눌러 1임무를 끄고 2임무 준비, 4번은 2임무 출발이다. 출발 방향은 라이다로 판단하므로 1번을 누른 뒤 로봇을 옮겨 놓아도 되고, 2번을 누르면 출발 전에 자리를 다시 잡는다.

입력은 USB 키보드의 1~4 키, 40핀(`--gpio`), 시험용 파이프(`--fifo /tmp/field_btn`) 중 하나다. 로봇 실기 시험은 파이프로만 했다. 40핀은 아직 쓸 수 없다. 핀 → 버튼 → GND 배선에서 7·29번은 버튼을 떼도 HIGH로 돌아오지 않았고, 31번은 처음부터 LOW였으며, 32번은 칩 입력이 꺼져 있다. `field_autostart.sh on button`은 이 40핀 설정으로 띄우므로 배선을 마친 뒤에 켠다.

## 필요한 것

Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble 기준이다. Python 패키지는 numpy 2.2.6, opencv 4.13.0, shapely 2.1.1을 쓰고 있고, `screen`과 `rmw_cyclonedds_cpp`가 있어야 한다.

아래 두 작업공간은 이 저장소에 없으며 로봇에 먼저 빌드되어 있어야 한다. `env.sh`가 둘 다 불러온다.

| 경로 | 쓰는 것 | 커밋 |
|---|---|---|
| `~/ros2_ws` | `sllidar_ros2` | 3430009 |
| `~/mecanum_ros2_ws` | `mecanum_bringup`, `mecanum_serial_hardware` | e957eac (로봇에서 고친 내용이 더 있을 수 있음) |

## 설치

스크립트에 경로가 `~/mecanum_slam_ws`로 고정되어 있어서 반드시 그 자리에 받아야 한다. 이미 폴더가 있으면 옮겨 두고 받는다.

```bash
git clone <저장소 주소> ~/mecanum_slam_ws
cd ~/mecanum_slam_ws
source /opt/ros/humble/setup.bash
colcon build
```

`robot.launch.py`를 고치면 `colcon build`를 다시 해야 반영된다. 펌웨어는 그리퍼와 적분 제어가 들어간 `firmware/mecanum_grip_pi`를 쓴다. 보드에 이미 구워져 있으면 다시 구울 필요가 없고, 구울 때는 `firmware/flash_grip_pi.sh`의 포트와 경로를 먼저 확인한다.

## 대회 실행

기반과 잡기 스테이션을 먼저 켜 둔다. `field_compete.sh`는 잡기 스테이션이 꺼져 있으면 켜 주지만 약 15초 더 걸린다.

```bash
~/mecanum_slam_ws/field_base.sh
~/mecanum_slam_ws/start_station.sh
```

로봇을 출발 칸 가운데에 놓고 아래 명령을 실행하면 키 입력 없이 바로 출발한다. 인자는 출발 칸 옆 바깥벽이 로봇의 어느 쪽인지를 뜻한다.

```bash
~/mecanum_slam_ws/field_compete.sh right   # (1,9) 남쪽 보기 또는 (9,1) 북쪽 보기
~/mecanum_slam_ws/field_compete.sh left    # (1,9) 동쪽 보기 또는 (9,1) 서쪽 보기
~/mecanum_slam_ws/field_compete.sh auto    # 라이다로 판단
```

임무는 로봇 안 screen에서 돌기 때문에 노트북 연결이 끊겨도 계속된다. 진행은 `tail -f /tmp/mission_screen.log`로 볼 수 있고, 다시 시작하려면 먼저 `screen -S mission -X quit`을 한다.

## 시험 실행

```bash
~/mecanum_slam_ws/field_auto.sh --dry   # 바퀴 안 움직임
~/mecanum_slam_ws/field_auto.sh         # g 출발, 스페이스 비상정지, x 종료
~/mecanum_slam_ws/field_keyboard.sh     # 키보드로 한 칸씩 이동
~/mecanum_slam_ws/field_stop.sh         # 전부 끄기
```

기록은 `mission1_field/logs/`, 잡기 사진은 `calib/grab_runs/`에 쌓인다. 둘 다 저장소에는 올리지 않는다.

## 폴더

| 경로 | 내용 |
|---|---|
| `mission1_field/` | 주행 노드, 위치추정, 경로계획, 회전 표, 제동 학습값 |
| `calib/` | 잡기 스테이션, 큐브 인식, 카메라 캘리브레이션 값 |
| `src/mecanum_slam/` | 구동, 센서, EKF를 켜는 launch |
| `firmware/` | 아두이노 펌웨어 소스와 빌드본 |
| `mission1/` | `field_stop.sh`가 쓰는 `estop.sh`만 있음 |

## 알아둘 점

로봇 안 ROS 통신은 `env.sh`가 시작할 때 고른다. 와이파이가 살아 있으면 와이파이 장치를 쓰고, 없으면 로봇 내부(localhost)만 쓴다. 프로세스마다 시작 시점에 정해지므로 와이파이 상태가 바뀐 뒤에 일부만 다시 켜면 서로 통신하지 못할 수 있다. 그때는 `field_stop.sh`로 전부 끄고 다시 켠다.

`field_common.sh`에 Cartographer를 켜는 `LOC=carto` 분기가 남아 있지만, 파일을 뺐으므로 이 저장소에서는 쓸 수 없다.

아직 실기로 확인하지 못한 것은 `field_compete.sh`로 시작한 주행, 돌던 중 와이파이가 끊기는 경우, 빈 Jetson에 이 저장소를 새로 받아 돌리는 경우다.
