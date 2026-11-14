#!/usr/bin/env bash
# 팔 보드(Jetson Orin Nano) 시스템 준비 — 관리자 권한(sudo)이 필요한 부분만 모았다.
# 한 번만 실행한다. 여러 번 돌려도 된다.
#
#   bash ~/vica_arm_kit/install_system.sh
#
# 다른 프로젝트(~/ros2_ws)는 건드리지 않는다. apt 패키지와 libuvc, udev 규칙만 더한다.
set -euo pipefail
K="$(cd "$(dirname "$0")" && pwd)"

[ "$(uname -m)" = aarch64 ] || { echo "aarch64 가 아닙니다. 젯슨에서 실행하세요"; exit 1; }
[ -f /opt/ros/humble/setup.bash ] || { echo "ROS 2 Humble 이 없습니다 (/opt/ros/humble)"; exit 1; }

echo "[1/3] apt 패키지 (이미 있는 것은 건너뛴다)"
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-dynamixel-sdk ros-humble-dynamixel-interfaces \
  ros-humble-moveit ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-image-geometry ros-humble-camera-info-manager ros-humble-image-transport \
  libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev nlohmann-json3-dev \
  python3-colcon-common-extensions cmake build-essential

echo "[2/3] libuvc (Astra 드라이버가 쓴다)"
if ldconfig -p | grep -q libuvc; then
  echo "  이미 있음"
else
  # 노트북과 같은 소스(d07de4f)를 묶음에서 푼다. 인터넷이 없어도 된다.
  rm -rf /tmp/libuvc
  tar -xzf "$K/deps/libuvc.tar.gz" -C /tmp
  cmake -S /tmp/libuvc -B /tmp/libuvc/build
  make -C /tmp/libuvc/build -j4
  sudo make -C /tmp/libuvc/build install
  sudo ldconfig
fi

echo "[3/3] 장치 권한 (팔 보드 직렬포트, Astra USB)"
sudo usermod -aG dialout,video "$USER"
sudo cp "$K/56-orbbec-usb.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo
echo "시스템 준비 끝. 다음은 bash ~/vica_arm_kit/build.sh"
