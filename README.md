# 메카넘 로봇 1임무

제29회 창원대 자율로봇경진대회 1임무(자율탐색 및 요구조자 구출)용 코드다. 9×9 격자(칸 40 cm)에서 (5,5)의 빨간 큐브를 잡아 출발 칸으로 가져온다.

2026-11-01 저녁 로봇(Jetson)의 `~/mecanum_slam_ws`에 올라가 있던 판을 그대로 옮겼고, 로그와 녹화, 백업은 뺐다. 옮긴 파일의 md5는 `MANIFEST_robot_md5.txt`에 있다. `README.md`, `.gitignore`, `start_station.sh`, `MANIFEST_robot_md5.txt`는 이 저장소를 만들면서 추가한 파일이다.

대회 주행에 안 쓰는 지도 작성 파일과 카메라 캘리브레이션 도구 같은 보조 도구는 뺐다. 필요하면 첫 커밋(8f68d61)에서 꺼내 쓴다. 지도 작성 파일을 빼면서 `src/mecanum_slam`의 `CMakeLists.txt`와 `package.xml`만 고쳤고, 이 둘을 뺀 나머지는 로봇 파일과 같다.

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

기반과 잡기 스테이션을 먼저 켜 둔다. `field_compete.sh`는 잡기 스테이션을 켜 주지 않으므로, 이걸 빠뜨리면 큐브 앞까지 간 뒤 잡기에 실패하고 임무가 멈춘다.

```bash
MISSION_DIR=mission1_field ~/mecanum_slam_ws/field_base.sh
~/mecanum_slam_ws/start_station.sh
```

`MISSION_DIR`을 빼면 없는 옛 위치추정 경로를 찾아서 기반이 켜지지 않는다. 시험 실행 명령도 마찬가지다.

로봇을 출발 칸 가운데에 놓고 아래 명령을 실행하면 키 입력 없이 바로 출발한다. 인자는 출발 칸 옆 바깥벽이 로봇의 어느 쪽인지를 뜻한다.

```bash
~/mecanum_slam_ws/field_compete.sh right   # (1,9) 남쪽 보기 또는 (9,1) 북쪽 보기
~/mecanum_slam_ws/field_compete.sh left    # (1,9) 동쪽 보기 또는 (9,1) 서쪽 보기
```

임무는 로봇 안 screen에서 돌기 때문에 노트북 연결이 끊겨도 계속된다. 진행은 `tail -f /tmp/mission_screen.log`로 볼 수 있고, 다시 시작하려면 먼저 `screen -S mission -X quit`을 한다.

## 시험 실행

```bash
MISSION_DIR=mission1_field ~/mecanum_slam_ws/field_auto.sh --dry   # 바퀴 안 움직임
MISSION_DIR=mission1_field ~/mecanum_slam_ws/field_auto.sh         # g 출발, 스페이스 비상정지, x 종료
MISSION_DIR=mission1_field ~/mecanum_slam_ws/field_keyboard.sh     # 키보드로 한 칸씩 이동
~/mecanum_slam_ws/field_stop.sh                                    # 전부 끄기
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
