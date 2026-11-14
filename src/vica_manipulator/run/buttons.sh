#!/usr/bin/env bash
# 버튼 찾기(YOLO + 깊이). run/astra.sh 가 떠 있어야 한다. 팔이 켜져 있으면 world 위치까지 나온다.
# 확인 화면은 bash run/view.sh /buttons/image (원본 방향이라 rqt 회전 버튼을 두 번 누른다).
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/buttons.py" "$@"
