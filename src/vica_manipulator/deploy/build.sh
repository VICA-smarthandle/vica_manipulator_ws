#!/usr/bin/env bash
# 팔 보드 작업 폴더(~/vica_manipulator_ws) 만들기와 빌드. 관리자 권한이 필요 없다.
# install_system.sh 를 먼저 한 번 실행해 둔다.
#
#   bash ~/vica_arm_kit/build.sh
#
# 이미 있는 소스는 덮어쓰지 않는다. ~/ros2_ws 는 불러오지도 건드리지도 않는다.
set -euo pipefail
K="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/vica_manipulator_ws"

echo "[1/3] 소스 풀기 → $WS/src"
mkdir -p "$WS/src" "$WS/models"
for t in "$K/vica_manipulator.tar.gz" "$K"/deps/{dynamixel_hardware_interface,ros2_astra_camera,image_publisher}.tar.gz; do
  name="$(basename "$t" .tar.gz)"
  if [ -d "$WS/src/$name" ]; then
    echo "  $name 이미 있음 — 건너뜀"
  else
    tar -xzf "$t" -C "$WS/src"
    echo "  $name 풀었음"
  fi
done
cp -n "$K"/models/* "$WS/models/"

echo "[2/3] 빌드 (다른 작업 폴더를 불러오지 않은 깨끗한 환경에서)"
# 오린 나노는 메모리가 작아 병렬 작업을 2 개로 줄인다.
env -i HOME="$HOME" USER="${USER:-}" LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c "
    set -e
    source /opt/ros/humble/setup.bash
    cd '$WS'
    MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 --packages-select \
      dynamixel_hardware_interface vica_manipulator image_publisher astra_camera_msgs astra_camera
  "

echo "[3/3] 점검"
bash "$WS/src/vica_manipulator/run/check.sh"
