#!/usr/bin/env bash
# 손 인식 신호등 혼자 시험. run/astra.sh 가 떠 있어야 한다. 확인 화면 bash run/view.sh /hand_guard/image (바로 선 방향)
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/hand_guard.py" "$@"
