#!/usr/bin/env bash
# 팔 제어(ros2_control) + 홈 자세로 이동. 화면에 RViz 를 띄우려면 RVIZ=true bash run/arm.sh
# 팔 없이 시험하려면 bash run/arm.sh use_mock_hardware:=true
set -euo pipefail
source "$(dirname "$0")/common.sh"
if [[ " $* " != *"use_mock_hardware:=true"* ]] && [ ! -e "$ARM_PORT" ]; then
  echo "팔 보드가 안 보입니다: $ARM_PORT"; exit 1
fi
exec ros2 launch vica_manipulator arm.launch.py port_name:="$ARM_PORT" start_rviz:="$RVIZ" "$@"
