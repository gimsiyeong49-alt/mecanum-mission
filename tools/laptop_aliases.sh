# 노트북용 로봇 단축 명령. 노트북 ~/.bashrc 에 ". 이파일경로" 한 줄을 넣어 불러 씀. rhelp 가 사용법을 보여 줌.
#   rhot                  노트북을 폰 핫스팟 esy 에 붙이고 로봇 주소를 핫스팟용으로 (평소)
#   rusb                  USB-C 선으로 연결 (연결만, 이후 rb1~4·rstat 이 선으로)
#   rssh                  핫스팟(esy)으로 로봇 접속 (172.20.137.29)
#   rwifi 이름 [비번]     USB-C 로 로봇을 대회장 와이파이에 붙임 → 마지막 줄 ROBOT_IP 가 새 주소
#   rwifist               로봇 와이파이 상태 (USB-C)
#   rwifiback             대회장 와이파이 지우고 핫스팟 esy 로 되돌림 (USB-C)
#   rip 주소              로봇 주소 바꾸기 (대회장 와이파이 IP). 이 터미널에서만, rssh·rstop·rbtn·rwall 이 그 주소로
#   rstop                 준비 취소·주행 멈춤 (바퀴 멈춤)
#   rbtn                  버튼 확인 도구 (Ctrl+C 로 끝)
#   rwall                 라이다 수평 확인 (경기장 가운데에 두고, Ctrl+C 로 끝)
#   rgrip                 그리퍼 닫기→열기 시험 (바퀴 안 움직임)
#   rstat                 로봇 상태: 통신 방식·자동 실행·준비 상태
#   rb1 rb2 rb3 rb4       버튼이 안 먹을 때 ssh 로 버튼 1~4 와 같은 일 (rb2·rb4 는 바퀴가 움직임!)
#     rb1 = 1임무 준비 (끝나면 그리퍼 1번)   rb2 = 1임무 출발 (1 초 뒤, 출발 칸 다시 잡음)
#     rb3 = 켜진 임무 끄고 2임무 준비 (끝나면 그리퍼 2번)   rb4 = 2임무 출발 (녹화도 켬)
ROBOT_USB=192.168.55.1
ROBOT_IP=${ROBOT_IP:-192.168.55.1}    # 기본 = USB-C 선 (11-03 사용자: 대회 때 노트북은 대회장 와이파이, 로봇은 USB-C 선). 핫스팟으로 하려면 rhot
rusb()      { ROBOT_IP=$ROBOT_USB; if ssh -o ConnectTimeout=5 uoeee@$ROBOT_IP true 2>/dev/null; then echo "USB-C 선으로 연결됨 (로봇 $ROBOT_IP). 이제 rb1~4 · rstop · rstat"; else echo "선으로 연결 안 됨: 선 다시 꽂고 10~20초 뒤 rusb"; fi; }
rhot()      { nmcli con up esy >/dev/null 2>&1; ROBOT_IP=172.20.137.29; if ssh -o ConnectTimeout=5 uoeee@$ROBOT_IP true 2>/dev/null; then echo "폰 핫스팟으로 연결됨 (로봇 $ROBOT_IP). 이제 rb1~4 · rstop · rstat"; else echo "핫스팟으로 연결 안 됨: 폰 핫스팟 켜졌나 확인, 안 되면 USB-C 선 꽂고 rusb"; fi; }
rssh()      { ssh uoeee@$ROBOT_IP "$@"; }
rip()       { ROBOT_IP=$1; echo "로봇 주소: $ROBOT_IP"; }
rwifi()     { [ -z "$1" ] && { echo "사용: rwifi 와이파이이름 [비밀번호]   (노트북↔로봇 USB-C 선 꽂은 채)"; return 1; }; local out=/tmp/rwifi_$$.txt; ssh -t uoeee@$ROBOT_USB "~/mecanum_slam_ws/field_wifi.sh \"$1\" \"${2:-}\"" | tee $out; local ip=$(tr -d "\r" < $out | sed -n "s/^ROBOT_IP=//p" | tail -1); rm -f $out; if [ -n "$ip" ]; then ROBOT_IP=$ip; echo; echo "로봇이 와이파이에 붙음 → 로봇 주소 $ROBOT_IP 기억함. 이제 rb1~4 · rstat · rssh 가 와이파이로 감 (선 빼도 됨)"; else echo; echo "와이파이 연결 실패: 이름·비밀번호 확인. 그동안은 선(rusb)으로 쓰면 됨"; fi; }
rwifist()   { ssh uoeee@$ROBOT_USB '~/mecanum_slam_ws/field_wifi.sh status'; }
rwifiback() { ssh -t uoeee@$ROBOT_USB '~/mecanum_slam_ws/field_wifi.sh remove'; }
rstop()     { ssh uoeee@$ROBOT_IP '~/mecanum_slam_ws/field_arm.sh stop'; }
rbtn()      { ssh -t uoeee@$ROBOT_IP 'python3 ~/mecanum_slam_ws/tools/pin_monitor.py'; }
rwall()     { ssh -t uoeee@$ROBOT_IP 'source ~/mecanum_slam_ws/env.sh; python3 ~/mecanum_slam_ws/tools/wall_check.py'; }
_rsign='for i in $(seq 1 $N); do for a in close open; do curl -s -m 2 -o /dev/null "http://127.0.0.1:8090/do?a=$a"; sleep 0.7; done; done'
rb1()       { ssh uoeee@$ROBOT_IP "set -o pipefail; W=~/mecanum_slam_ws; N=1; \$W/field_arm.sh 1 auto 2>&1 | grep -v cmdline && $_rsign"; }
rb2()       { ssh uoeee@$ROBOT_IP 'W=~/mecanum_slam_ws; sleep 0.5; $W/field_arm.sh recheck; sleep 0.5; $W/field_arm.sh go 2>&1 | grep -v cmdline'; }
rb3()       { ssh uoeee@$ROBOT_IP "set -o pipefail; W=~/mecanum_slam_ws; N=2; \$W/field_arm.sh stop 2>&1 | grep -v cmdline; \$W/field_arm.sh 2 2>&1 | grep -v cmdline && $_rsign"; }
rb4()       { ssh uoeee@$ROBOT_IP '~/mecanum_slam_ws/field_arm.sh go 2>&1 | grep -v cmdline'; }
rgrip()     { ssh uoeee@$ROBOT_IP 'curl -s -m 3 -o /dev/null "http://127.0.0.1:8090/do?a=close"; sleep 1.5; curl -s -m 3 -o /dev/null "http://127.0.0.1:8090/do?a=open"; echo "닫기→열기 보냄"'; }
rstat()     { ssh uoeee@$ROBOT_IP 'W=~/mecanum_slam_ws; $W/field_dds.sh; $W/field_autostart.sh status | head -2; $W/field_arm.sh status 2>&1 | grep -v cmdline | head -3'; }
rhelp() { cat <<'HELP'
==================== 메카넘 대회 비상 카드 ====================
[평소] 노트북 필요 없음. 버튼만
  전원 켜고 1분 뒤 · 0.05~3초 눌렀다 떼기
  1 빨강  미션1 준비            → 그리퍼 1번
  2 흰색  미션1 출발             ※ 바퀴 움직임
  3 노랑  미션2 준비 (미션1 끔)  → 그리퍼 2번
  4 파랑  미션2 출발             ※ 바퀴 움직임
  그리퍼 3번 = 실패 → 같은 버튼 다시

[노트북 쓸 때] 경기 중에도 노트북으로 명령하려면 로봇을 대회장 와이파이에 (처음 한 번)
  왜: 로봇은 선 없이 달려야 함 → 로봇도 노트북과 같은 와이파이에. 화면이 없어 선으로 한 번 알려 줌
  1) 노트북을 대회장 와이파이에 연결 (평소 와이파이 메뉴)
  2) 로봇 켜고 USB-C 선으로 노트북에 꽂기 (10~20초)
  3) rwifi "대회장와이파이이름" "비밀번호"
  4) '로봇 주소 … 기억함' 나오면 선 뽑기 → 아래 명령이 와이파이로 감
  ※ 실패하면 이름·비번 확인 후 다시 (그동안은 선 꽂은 채로도 됨) · 대회 끝나고 되돌리기: rwifiback (선 꽂고)

 버튼 대신
  rb1   rb2   rb3   rb4     # = 버튼 1~4  (rb2·rb4 바퀴 움직임)
  rstop                     # 멈춤 · 준비 취소

 점검
  rstat       # 상태: '방식: local' · '켜짐' 이면 정상
  rgrip       # 그리퍼 닫기 → 열기
  rbtn        # 버튼 핀 (누르면 0) · Ctrl+C 로 끝
  rwall       # 라이다 수평 (경기장 가운데에서)
  rssh        # 로봇 안으로 들어가기 (나올 땐 exit)

이름 규칙: r(로봇) + 할 일  usb 선 · b 버튼 · stop · stat 상태 · grip 그리퍼 · btn 버튼핀 · wall 벽
===============================================================
HELP
}
