#!/usr/bin/env bash
# 한 번에 띄운다(11-14). 자세한 것은 launch/all.launch.py 머리말
#   bash run/all.sh                 시연 데모: 팔 + MoveIt → 홈·장애물 뒤 press down --observe --yes --blind 한 번(카메라 없음)
#   bash run/all.sh camera:=true    NX 명령 대기: 팔 + MoveIt + 카메라 + YOLO 버튼 찾기 + 손 인식 + NX 서비스
#   다른 버튼: press:=up   띄우기만: press:=none   팔 없이: use_mock_hardware:=true   RViz: RVIZ=true bash run/all.sh
set -euo pipefail
source "$(dirname "$0")/common.sh"
if [[ " $* " != *"use_mock_hardware:=true"* ]] && [ ! -e "$ARM_PORT" ]; then
  echo "팔 보드가 안 보입니다: $ARM_PORT"; exit 1
fi
exec ros2 launch vica_manipulator all.launch.py port_name:="$ARM_PORT" start_rviz:="$RVIZ" "$@"
