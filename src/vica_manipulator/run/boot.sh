#!/usr/bin/env bash
# 부팅 때 systemd(run/systemd/vica-arm.service)가 부른다(11-25 사용자: NX 명령에 바로 움직이게 나노 부팅 때 자동 실행).
# 랜선·팔 보드·카메라가 잡힐 때까지 기다렸다가 all.sh camera:=true 를 켠다 → 팔이 홈으로 간 뒤 힘을 빼고 NX 명령을 기다린다.
#   랜선 10.77.0.2 : 없이 켜면 와이파이로만 말해 NX 와 못 만난다(11-25) → 올 때까지 기다린다
#   팔 보드        : 올 때까지 기다린다
#   카메라         : 최대 60 초 기다리고, 없으면 그대로 켠다(손 인식이 없으면 /arm/demo 가 '손 인식 답 없음'으로 안 움직인다)
set -u
HERE=$(dirname "$(readlink -f "$0")")
ARM_PORT=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_83315A9250323952582E3120FF05251B-if00
log() { echo "[boot $(date +%T)] $*"; }

n=0
until ip -4 -o addr show 2>/dev/null | grep -q ' 10\.77\.0\.2/'; do
  [ $((n % 30)) -eq 0 ] && log "랜선 주소 10.77.0.2 를 기다립니다(NX 랜선 확인)"
  n=$((n + 1)); sleep 1
done
log "랜선 10.77.0.2 있음"

n=0
until [ -e "$ARM_PORT" ]; do
  [ $((n % 30)) -eq 0 ] && log "팔 보드(OpenRB-150)를 기다립니다"
  n=$((n + 1)); sleep 1
done
log "팔 보드 있음"

n=0
until lsusb 2>/dev/null | grep -q '2bc5:0404' || [ $n -ge 60 ]; do n=$((n + 1)); sleep 1; done
if lsusb 2>/dev/null | grep -q '2bc5:0404'; then log "카메라 있음"; else log "⚠ 카메라가 60 초 안에 안 잡힘 — 그대로 켭니다(손 인식 없음)"; fi

sleep 3   # 장치가 막 잡힌 직후 바로 열면 카메라가 'Resource busy' 로 꼬인 적이 있다(11-25)
log "all.sh camera:=true 를 켭니다"
exec bash "$HERE/all.sh" camera:=true "$@"
