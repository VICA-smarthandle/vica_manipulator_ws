#!/usr/bin/env bash
# 카메라 영상 보기(rqt_image_view). 팔 도메인에서 연다 — 그냥 rqt 를 켜면 도메인 0 이라 안 보인다.
#   bash run/view.sh                            색 영상
#   bash run/view.sh /camera/depth/image_raw    깊이 영상
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec ros2 run rqt_image_view rqt_image_view "${1:-/camera/color/image_raw}"
