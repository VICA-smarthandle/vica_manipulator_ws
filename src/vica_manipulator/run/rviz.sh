#!/usr/bin/env bash
# 보기 전용 RViz. 로봇·장애물·MoveIt 이 세운 경로(보라색 반투명 팔)·버튼 공(빨강 버튼, 노랑 접근점)을 보여 준다.
# 그냥 rviz2 를 켜면 도메인이 달라(77 이 아님) 로봇이 안 뜬다. run/arm.sh·run/moveit.sh 가 떠 있어야 한다.
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec ros2 launch vica_manipulator rviz.launch.py
