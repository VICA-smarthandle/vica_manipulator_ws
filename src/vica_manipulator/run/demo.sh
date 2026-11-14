#!/usr/bin/env bash
# 시연: 기록한 길로 홈 → 관찰 → 버튼 누르기 → 홈 (카메라 없이). 예) bash run/demo.sh down --loop 3
# run/arm.sh 와 run/moveit.sh 가 떠 있어야 한다. 길은 python3 tools/record_path.py down --button down 으로 먼저 기록.
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/demo.py" "$@"
