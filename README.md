# vica_manipulator_ws

VICA 다층 이동을 위한 엘리베이터 버튼 누르기 매니퓰레이터 ROS 2 워크스페이스.

- **하드웨어**: OpenMANIPULATOR-X 기반 4축 팔(상완 357 · 전완 365 mm, 축↔축 — 2026-11-25 링크·마운트 일체형 재출력), 누름 막대 FR12_H104, 손목 카메라 Astra Mini S, 제어보드 OpenRB-150
- **플랫폼**: Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble
- **기능**: YOLO로 버튼 인식 → MoveIt으로 접근 → 누름 → 불빛으로 눌림 확인 → 홈 복귀. NX(주행 보드)가 ROS 서비스로 호출

## 구성

```
vica_manipulator_ws/
├── vica.repos                    # 외부 의존성 (ros2_astra_camera, dynamixel_hardware_interface)
├── models/                       # 이전 버튼 모델(v3), 참고용
└── src/
    ├── image_publisher/          # image_pipeline에서 가져온 패키지
    └── vica_manipulator/
        ├── urdf/ meshes/         # 팔 모델, ros2_control
        ├── config/               # MoveIt, 컨트롤러, 누르기 설정(press.yaml), AMR 장애물(scene.yaml)
        ├── launch/               # arm, moveit, rviz, all
        ├── run/                  # 실행 스크립트
        ├── tools/                # 버튼 인식, 누르기, 손 감시, NX 서비스
        └── deploy/               # 시스템 패키지 설치, udev 규칙
```

ROBOTIS `open_manipulator_*` 패키지에 의존하지 않습니다. 나노의 다른 워크스페이스(`~/ros2_ws`)와 이름이 겹치지 않도록, 필요한 것만 `vica_manipulator` 하나로 묶었습니다.

## 설치

```bash
git clone git@github.com:VICA-smarthandle/vica_manipulator_ws.git ~/vica_manipulator_ws
cd ~/vica_manipulator_ws
vcs import src < vica.repos
bash src/vica_manipulator/deploy/install_system.sh    # apt, libuvc, udev (sudo, 최초 1회)

source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
bash src/vica_manipulator/run/check.sh                # 설치·연결 점검 (팔 안 움직임)
```

**모델 파일**은 리포에 없고 `~/vica_arm_kit/models/`에서 읽습니다.

| 파일 | 용도 |
|---|---|
| `elev_v4_rf_best.engine` (없으면 `.pt`) | 버튼 인식 YOLO. engine은 나노에서 TensorRT로 변환 |
| `hand/hand_landmarker.task` | 손 인식 (MediaPipe) |

MediaPipe는 의존성 충돌을 피하려고 이렇게 설치합니다:
`pip3 install --user --no-deps mediapipe==1.2.0 absl-py sounddevice cffi pycparser`

## 실행

모든 스크립트는 `src/vica_manipulator/run/`에 있습니다. 실행할 때 환경을 비우고 이 워크스페이스만 불러오므로 `~/ros2_ws`와 섞이지 않습니다.

### 한 번에 띄우기

```bash
bash run/all.sh camera:=true     # 운용: 팔 + MoveIt + 카메라 + 버튼 인식 + 손 감시 + NX 서비스
bash run/all.sh                  # 시연: 카메라 없이 저장된 경로로 down 버튼 한 번 누르기
```

| 인자 | 설명 |
|---|---|
| `use_mock_hardware:=true` | 팔 없이 가짜 하드웨어로 실행 |
| `service:=false` | NX 서비스 없이 실행 |
| `press:=up` / `press:=none` | 시연에서 누를 버튼 / 누르지 않음 |

### 개별 실행

| 명령 | 내용 |
|---|---|
| `run/arm.sh` | 팔 제어 + 홈 자세로 이동 (`RVIZ=true` 앞에 붙이면 RViz) |
| `run/moveit.sh` | MoveIt (arm 다음에) |
| `run/astra.sh` | 손목 카메라 |
| `run/buttons.sh` | 버튼 인식 (astra 다음에) |
| `run/service.sh` | NX용 서비스 (arm, moveit, buttons 다음에) |
| `run/press.sh <버튼>` | 수동 누르기. `--observe` 관찰부터, `--yes` 확인 없이, `--dry` 계획만 |
| `run/pose.sh front\|right\|home` | 관찰 자세로 이동 |
| `run/demo.sh <버튼>` | 기록한 경로로 시연 (카메라 없이) |
| `run/rviz.sh` / `run/view.sh` | RViz 보기 / 카메라 영상 보기 |

버튼: `up` `down` (승강장 호출), `f1`~`f4` (엘리베이터 안 층), `open` `close`

## NX 연동

| 항목 | 값 |
|---|---|
| ROS 도메인 | **77** (노트북 시험은 `VICA_ARM_DOMAIN=78`) |
| 통신 | 나노-NX 랜선(10.77.0.x)으로만 (`config/dds_wired.xml`). 해제하려면 `VICA_ARM_DDS=all` |
| 비상정지 | 도메인 7 `/estop_state` 수신 시 정지 후 홈 |

서비스 (`std_srvs/srv/Trigger`):

| 서비스 | 동작 |
|---|---|
| `/arm/status` | 상태 확인 |
| `/arm/home` | 홈 복귀 |
| `/arm/press_up`, `/arm/press_down` | 호출 버튼 누르기 |
| `/arm/press_f1` ~ `/arm/press_f4` | 층 버튼 누르기 후 해당 층 도착까지 불빛 감시 |
| `/arm/my_floor` | 문 열렸을 때 목표 층인지 확인 |
| `/arm/demo` | 시연 한 번: 관찰 → 저장한 버튼 자리(`demo.service_target`, 기본 down)로 뻗기 → 홈. 손 인식으로만 멈춤(`run/demo_hand.sh` 와 같은 동작). 버튼판이 없어도 끝까지 뻗고 홈에 오면 성공 |

```bash
ROS_DOMAIN_ID=77 ros2 service call /arm/press_down std_srvs/srv/Trigger
ROS_DOMAIN_ID=77 ros2 service call /arm/demo std_srvs/srv/Trigger     # NX 앱 시연 버튼
```

## 안전

- **손 감시**: 카메라 0.8 m 안에 손이 보이면 어느 단계든 정지, 손을 빼면 다시 계획해 이어감. 30초 넘으면 실패
- **경로 감시**: 버튼으로 가는 길에 물체가 들어오면 정지
- **장애물**: `config/scene.yaml`의 AMR 차체·짐칸이 MoveIt에 자동으로 들어감. 없으면 누르기 거부
- **전류 제한**: 모터는 전류 제한 위치 제어(모드 5)로, 막혀도 설정 이상 밀지 않음

## 설정

| 파일 | 내용 |
|---|---|
| `config/press.yaml` | 누르기 전체 설정: 관찰 자세, 높이 보정, 누름 깊이, 속도, 손·경로 감시 |
| `config/scene.yaml` | AMR 장애물. 짐칸이 바뀌면 같이 수정 |
| `config/home.yaml` | 홈 자세 (AMR 위에 납작하게 접은 자세) |
| `run/common.sh` | 도메인, 팔 시리얼 포트 |

누를 때마다 손목 카메라 사진과 기록이 `~/vica_arm_logs/press/`에 저장됩니다.
