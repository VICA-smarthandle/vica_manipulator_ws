#!/usr/bin/env bash
# 설치·연결 상태 점검. 팔을 움직이지 않는다.
set -uo pipefail
source "$(dirname "$0")/common.sh"
echo "작업 폴더 : $WS"
echo "보드      : $(uname -m), $(head -1 /etc/nv_tegra_release 2>/dev/null | cut -c1-40)"
echo "도메인    : $ROS_DOMAIN_ID"
for p in vica_manipulator dynamixel_hardware_interface astra_camera moveit_ros_move_group; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    echo "패키지 OK : $p ($(ros2 pkg prefix "$p"))"
  else
    echo "패키지 없음: $p"
  fi
done
if [ -e "$ARM_PORT" ]; then echo "팔 보드   : 연결됨 → $(readlink -f "$ARM_PORT")"; else echo "팔 보드   : 안 보임 (OpenRB USB 확인)"; fi
if [ -e /dev/astra_mini ]; then echo "Astra     : 연결됨"; else echo "Astra     : 안 보임"; fi
if id -nG | grep -qw dialout; then echo "권한      : dialout OK"; else echo "권한      : dialout 없음 → 로그아웃 후 다시 로그인"; fi
