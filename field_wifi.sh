#!/bin/bash
# 대회장 라우터에 로봇 붙이기. USB-C 로 접속해서 실행 (와이파이로 접속한 채 바꾸면 접속이 끊김).
#   field_wifi.sh "이름" "비밀번호"   # 비밀번호 없는 망은 비밀번호 생략
#   field_wifi.sh status              # 지금 연결·IP·주변 망
#   field_wifi.sh remove              # 대회장 프로필 지우기 (평소 핫스팟으로 돌아감)
# 프로필 이름 venue_router, 우선순위 100 (핫스팟 esy 10 보다 높음), 와이파이 절전 끔, 자동 연결 → 재부팅해도 다시 붙음.
IF=wlP1p1s0; CON=venue_router
nm() { if nmcli "$@" 2>/tmp/nm_err; then return 0; fi; if grep -qi "not authorized\|권한\|insufficient" /tmp/nm_err; then sudo nmcli "$@"; else cat /tmp/nm_err >&2; return 1; fi; }
show() {
  echo "연결: $(nmcli -t -f GENERAL.CONNECTION device show $IF 2>/dev/null | cut -d: -f2-)"
  echo "IP  : $(ip -4 -o addr show $IF 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
  echo "기본 경로: $(ip route show default dev $IF 2>/dev/null | awk '{print $3}' | head -1)"
  echo "절전: $(iw dev $IF get power_save 2>/dev/null)"
}
case "${1:-}" in
  ""|-h) sed -n 2,6p "$0"; exit 1;;
  status) show; nmcli -f SSID,CHAN,SIGNAL,SECURITY device wifi list ifname $IF | head -15; exit 0;;
  remove) nm connection delete $CON; nmcli connection up esy 2>/dev/null; show; exit 0;;
esac
SSID=$1; PW=${2:-}
case "${SSH_CONNECTION:-}" in *192.168.55.*|"") ;; *) echo "주의: USB-C 가 아닌 망으로 접속 중. 연결을 바꾸면 이 접속이 끊길 수 있음 (5 초 뒤 계속)"; sleep 5;; esac
nmcli device wifi rescan ifname $IF 2>/dev/null; sleep 3
L=$(nmcli -t -f SSID,CHAN,SIGNAL,SECURITY device wifi list ifname $IF | grep -F "$SSID:" | head -1)
if [ -n "$L" ]; then echo "찾음: $L"; else echo "주의: 스캔에 '$SSID' 없음 (숨김 망이거나 대역 미지원). 숨김으로 시도"; fi
nm connection delete $CON >/dev/null 2>&1
if [ -n "$PW" ]; then
  nm connection add type wifi ifname $IF con-name $CON ssid "$SSID" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PW" >/dev/null || exit 1
else
  nm connection add type wifi ifname $IF con-name $CON ssid "$SSID" >/dev/null || exit 1
fi
[ -z "$L" ] && nm connection modify $CON 802-11-wireless.hidden yes
nm connection modify $CON connection.autoconnect yes connection.autoconnect-priority 100 802-11-wireless.powersave 2 ipv4.method auto ipv6.method ignore || exit 1
if ! nm connection up $CON; then
  echo "연결 실패. 비밀번호·이름 확인. 주변 망:"; nmcli -f SSID,CHAN,SIGNAL,SECURITY device wifi list ifname $IF | head -12; exit 1
fi
sleep 1; show
GW=$(ip route show default dev $IF | awk '{print $3}' | head -1)
[ -n "$GW" ] && ping -c 3 -W 1 "$GW" | tail -1
echo "ROBOT_IP=$(ip -4 -o addr show $IF | awk '{print $4}' | cut -d/ -f1)"
