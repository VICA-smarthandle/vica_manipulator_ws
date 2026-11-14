#!/usr/bin/env bash
# NX 가 부르는 팔 서비스(/arm/status·home·press_up·press_down). 도메인 77.
# run/arm.sh·run/moveit.sh·run/buttons.sh 가 먼저 떠 있어야 한다.
# NX 에서: ROS_DOMAIN_ID=77 ros2 service call /arm/status std_srvs/srv/Trigger
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/arm_service.py" "$@"
