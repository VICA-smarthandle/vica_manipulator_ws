#!/usr/bin/env bash
# 정해 둔 자세로 보내기. 예) bash run/pose.sh front --dry   (계획만, 팔 안 움직임)
# run/arm.sh 와 run/moveit.sh 가 떠 있어야 한다.
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/pose.py" "$@"
