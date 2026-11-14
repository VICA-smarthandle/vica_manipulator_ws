# run/*.sh 가 불러 쓰는 공통 설정. 직접 실행하지 않는다.
#
# 나노의 .bashrc 는 다른 프로젝트(~/ros2_ws)를 자동으로 불러온다. 거기에도
# open_manipulator_* 패키지가 있어 섞이면 어느 쪽이 쓰이는지 알 수 없다.
# 그래서 처음 한 번 환경을 비우고(env -i) 이 스크립트를 다시 실행한다.
if [ -z "${VICA_ARM_ENV:-}" ]; then
  exec env -i VICA_ARM_ENV=1 HOME="$HOME" USER="${USER:-}" LANG="${LANG:-C.UTF-8}" \
    TERM="${TERM:-xterm}" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" RVIZ="${RVIZ:-false}" VICA_ARM_DDS="${VICA_ARM_DDS:-wired}" VICA_ESTOP_DOMAIN="${VICA_ESTOP_DOMAIN:-7}" VICA_ESTOP_REQUIRED="${VICA_ESTOP_REQUIRED:-1}" \
    VICA_ARM_WS="${VICA_ARM_WS:-$HOME/vica_manipulator_ws}" VICA_ARM_DOMAIN="${VICA_ARM_DOMAIN:-77}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash "$0" "$@"
fi

WS="$VICA_ARM_WS"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

# 팔 보드는 주행 보드(z-jet, 7)·다른 프로젝트와 섞이지 않게 도메인을 따로 쓴다.
# 노트북에서 가짜 팔로 시험할 때는 VICA_ARM_DOMAIN=78 처럼 바꾼다. 같은 공유기에 붙은
# 나노의 실제 팔이 노트북 명령을 받지 않게 하기 위해서다(2026-10-16).
export ROS_DOMAIN_ID="$VICA_ARM_DOMAIN"
# 보드 간 랜선 주소(10.77.0.2)가 있는 기계(나노)에서는 ROS 통신을 랜선과 나노 안으로만 한다(2026-10-18).
# 엘리베이터 안에서 와이파이가 끊겨도 NX 와의 연결이 흔들리지 않고, 와이파이의 다른 컴퓨터는 팔을 못 부른다.
# 노트북처럼 그 주소가 없으면 그대로 둔다. 잠깐 끄려면 VICA_ARM_DDS=all 을 앞에 붙인다.
if [ "${VICA_ARM_DDS:-wired}" = wired ] && ip -4 -o addr show 2>/dev/null | grep -q ' 10\.77\.0\.2/'; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/vica_manipulator/config/dds_wired.xml"
fi
# 팔 OpenRB-150 고유 이름 (2026-10-15 확인). ttyACM 번호는 꽂는 순서로 바뀐다.
ARM_PORT=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_83315A9250323952582E3120FF05251B-if00
TOOLS="$WS/src/vica_manipulator/tools"
