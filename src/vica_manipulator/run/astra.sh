#!/usr/bin/env bash
# 손목 Astra Mini S. publish_tf 를 끄지 않으면 드라이버가 URDF 카메라 위치를 덮어쓴다.
# 정합을 켜면 깊이가 색 카메라 시점(camera_color_optical_frame)으로 나온다. 최소 거리 약 0.36 m.
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec ros2 launch astra_camera astra.launch.xml publish_tf:=false depth_registration:=true \
  color_depth_synchronization:=true enable_ir:=false enable_point_cloud:=false "$@"
