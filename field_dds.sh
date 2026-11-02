#!/bin/bash
# 로봇 안 ROS 통신 방식 바꾸기. 대회에선 임무 시작 뒤 통신이 끊기므로(규정 1.3, 3.3.3) 와이파이에 묶이지 않게 local 로 둔다.
#   field_dds.sh local   # 로봇 안에서만 (와이파이가 끊겨도 영향 없음, 노트북 RViz 로는 안 보임, 웹 :8090 은 됨)
#   field_dds.sh wifi    # 노트북 RViz 와 통신 (평소)
#   field_dds.sh         # 지금 방식 확인
# 방식이 다른 노드끼리는 안 보이므로 기반·스테이션을 끄고 바꾼다. 다음 ready 때 새 방식으로 켜짐.
source ~/mecanum_slam_ws/field_common.sh
F=$W/.dds_local
case "${1:-}" in
  local|wifi)
    stop_slam
    screen -S station -X quit 2>/dev/null
    if [ "$1" = local ]; then touch $F; else rm -f $F; fi
    echo "방식: $1 (기반·스테이션 꺼짐. 다음에 켜면 적용)";;
  "") [ -f $F ] && echo "방식: local (로봇 안에서만)" || echo "방식: wifi";;
  *) echo "사용: field_dds.sh local | wifi"; exit 1;;
esac
