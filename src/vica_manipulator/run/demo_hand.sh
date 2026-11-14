#!/usr/bin/env bash
# 시연용(11-02): 카메라 켜고 한 번에 띄우되, 길목 신호등(깊이 칸)은 끄고 손 인식(hand_guard)으로만 멈춘다.
# 길목 신호등은 앞에 벽이 없으면(실습실 등) 깊이 빈칸을 '렌즈 가림'으로 봐 관찰 자세에서 계속 빨간불이 된다.
#   bash run/demo_hand.sh               down 버튼, 저장한 자리로(--blind)
#   bash run/demo_hand.sh press:=up     다른 버튼
set -euo pipefail
if pgrep -f astra_camera_node >/dev/null; then
  echo "다른 카메라(astra_camera_node)가 이미 떠 있습니다 → 이번 실행의 카메라가 안 열려 손 인식이 안 됩니다."
  echo "끄고 다시 실행하세요:  pkill -INT -f astra_camera_node"
  exit 1
fi
# 11-14: all.sh camera:=true 는 NX 서비스 대기가 기본이 됐다 → 여기서는 서비스·버튼 찾기를 끄고 한 번 누르기로
exec bash "$(dirname "$0")/all.sh" camera:=true service:=false buttons:=false press:=down \
  press_args:="--observe --yes --blind --no-path-guard" "$@"
