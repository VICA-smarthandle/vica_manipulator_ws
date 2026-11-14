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
vcs import src < vica.repos                      # 외부 의존성 받기 (python3-vcstool)
bash src/vica_manipulator/deploy/install_system.sh   # 시스템 패키지/udev (최초 1회, sudo)

source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
bash src/vica_manipulator/run/check.sh           # 점검
```

> `deploy/build.sh`는 tarball 배포 키트(`~/vica_arm_kit`)용 스크립트입니다. git으로 받았다면 위 명령을 쓰세요.
