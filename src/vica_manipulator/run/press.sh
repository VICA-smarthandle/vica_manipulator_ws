#!/usr/bin/env bash
# 버튼 누르기. 예) bash run/press.sh down --dry   (계획만, 팔 안 움직임)
# run/arm.sh 와 run/moveit.sh 가 떠 있어야 한다.
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/press.py" "$@"
