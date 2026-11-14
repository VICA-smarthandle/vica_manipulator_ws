#!/usr/bin/env bash
# MoveIt (경로 계획). run/arm.sh 를 먼저 띄운 뒤에 실행한다.
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec ros2 launch vica_manipulator moveit.launch.py start_rviz:="$RVIZ" "$@"
