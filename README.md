# vica_manipulator_ws

VICA 매니퓰레이터(엘리베이터 버튼 누르기 팔) ROS 2 워크스페이스.
패키지 상세 사용법은 [src/vica_manipulator/README.md](src/vica_manipulator/README.md) 참고.

## 구성

| 경로 | 내용 |
|---|---|
| `src/vica_manipulator` | URDF, MoveIt 설정, launch, 실행 스크립트(`run/`), 도구(`tools/`) |
| `src/image_publisher` | image_pipeline의 image_publisher 패키지 (vendored) |
| `models/` | 버튼 검출 YOLO 모델 (`elev_v3_aug_best.pt`) |
| `vica.repos` | 외부 의존성 (ros2_astra_camera, dynamixel_hardware_interface) |

## 설치

```bash
git clone git@github.com:VICA-smarthandle/vica_manipulator_ws.git ~/vica_manipulator_ws
cd ~/vica_manipulator_ws
vcs import src < vica.repos
# 시스템 의존성/빌드는 src/vica_manipulator/deploy/ 참고
src/vica_manipulator/deploy/install_system.sh
src/vica_manipulator/deploy/build.sh
```
