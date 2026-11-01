# 메카넘 SLAM 실행 환경: source ~/mecanum_slam_ws/env.sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash          # sllidar_ros2
source ~/mecanum_ros2_ws/install/setup.bash  # mecanum_bringup (수정 안 함)
source ~/mecanum_slam_ws/install/setup.bash  # mecanum_slam
export ROS_DOMAIN_ID=78
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS 통신 인터페이스: 지금 살아 있는(UP + IPv4) 것만 골라 쓴다. 없으면 Jetson 내부(localhost)만.
# (고정으로 적어 두면 USB-C 를 뺐을 때 l4tbr0 가 DOWN 이라 노드가 아예 안 뜸 — 2026-10-21)
_mk_ifs=""
for _i in wlP1p1s0; do        # USB-C(l4tbr0)는 SSH 전용. ROS 에 넣으면 선을 뺄 때 통신이 깨짐 (10-21)
  if [ "$(cat /sys/class/net/$_i/operstate 2>/dev/null)" = up ] && ip -4 addr show $_i 2>/dev/null | grep -q inet; then _mk_ifs="$_mk_ifs $_i"; fi
done
if [ -n "$_mk_ifs" ]; then
  _mk_x=$HOME/mecanum_slam_ws/cyclonedds_auto.xml
  { echo "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"; echo "<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain id=\"any\"><General><Interfaces>";
    for _i in $_mk_ifs; do echo "  <NetworkInterface name=\"$_i\"/>"; done; echo "</Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>"; } > $_mk_x.tmp && mv $_mk_x.tmp $_mk_x
  export ROS_LOCALHOST_ONLY=0 CYCLONEDDS_URI=file://$_mk_x
else
  export ROS_LOCALHOST_ONLY=1; unset CYCLONEDDS_URI
fi
export MECANUM_DDS_IFS="${_mk_ifs:- (내부 전용 localhost)}"
