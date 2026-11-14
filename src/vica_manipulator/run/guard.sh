#!/usr/bin/env bash
# 길목 신호등 시험. 팔은 꺼 둔 채 카메라 앞 칸(기본 0.6 m 안)에 손을 넣었다 뺐다 한다.
# 확인 화면은 bash run/view.sh /path_guard/image (원본 방향이라 rqt 회전 버튼을 두 번 누른다).
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/path_guard.py" "$@"
