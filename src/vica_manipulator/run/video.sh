#!/usr/bin/env bash
# 시연 영상: YOLO 상자가 그려진 손목 카메라 화면을 나노에서 바로 mp4 로 저장(~/vica_arm_logs/video). Ctrl+C = 끝
# ③ astra.sh 와 ④ buttons.sh 가 떠 있어야 한다. 예) bash run/video.sh --sec 60
set -euo pipefail
source "$(dirname "$0")/common.sh"
exec python3 -u "$TOOLS/record_video.py" "$@"
