# vica_manipulator

VICA 엘리베이터 버튼 팔. ROBOTIS OpenMANIPULATOR-X 를 바탕으로 장링크(상완 355 · 전완 375 mm),
FR12_H104 누름 막대(손목축 → 끝 156 mm), 손목 Astra Mini S 를 실측 반영했다.
**팔 보드(Jetson Orin Nano, Ubuntu 22.04 + ROS 2 Humble) 전용**이다.

## 왜 따로 만들었나

나노에는 다른 프로젝트(`~/ros2_ws`)가 있고, 거기에 `open_manipulator_bringup`·`open_manipulator_description`
이 이미 설치돼 있다. 같은 이름을 쓰면 어느 쪽이 불리는지 알 수 없으므로, 이 팔에 실제로 쓰는 것만
골라 이름을 `vica_manipulator` 하나로 묶었다. ROBOTIS 의 `open_manipulator_*` 패키지에 의존하지 않는다.

| 원본 (ROBOTIS) | 여기 |
|---|---|
| `open_manipulator_description` URDF·메시 | `urdf/`, `meshes/` (메시는 쓰는 3 개만) |
| `open_manipulator_bringup` launch·설정·`joint_trajectory_executor` | `launch/arm.launch.py`, `config/controllers.yaml`·`home.yaml`, `scripts/go_home.py` |
| `open_manipulator_moveit_config` | `launch/moveit.launch.py`, `config/*.srdf`·`kinematics`·`joint_limits`·`moveit_controllers` |
| 컨테이너의 `docker/workspace/button/` (git 밖) | `tools/` |

- 계획기(ompl·pilz·chomp) 설정은 두지 않는다. Humble 전용이라 `moveit_configs_utils` 기본값을 쓴다
- Gazebo 분기·prefix 인자는 뺐다
- 홈 자세는 AMR 부착용 **납작 접기** `[-0.023, -1.535, 1.540, 0.724]`(11-09 재조립 뒤 밑동이 틀어져 joint1 -1.3도, 손목은 위를 보게 41.5도). 위팔 뒤로 수평, 아래팔 위에 겹쳐 수평,
  손목 64도. 손으로 놓은 손목 79도는 막대가 link3·link2 와 최대 4 mm 겹쳐 MoveIt 이 거부해서 64도로 뒀다.
  10-18 joint2·3 을 닿는 자리보다 1도 앞으로 옮겼다(옛 `-1.565, 1.562` 는 닿는 자리보다 안쪽이라 13번 과부하)
- `go_home.py` 는 팔이 이미 홈이면 `IndexError` 로 죽던 상류 버그를 고쳤다

## AMR 장애물 (`config/scene.yaml`)

차체·짐칸·마스트를 base_link 기준 상자로 적는다. `moveit.launch.py` 가 move_group 을 띄우면
`load_scene.py` 가 mount(base_link → 팔 world)를 빼서 자동으로 넣는다. `press.py` 는 `amr_body`·`cargo_bin` 이
없으면 누르기를 거부한다(`--no-scene` 은 책상 위 시험용). 확인은 `ros2 run vica_manipulator load_scene.py --list`.
새 홈에서 위팔은 짐칸 윗면보다 약 2~3 cm 위에 눕는다 — 짐칸이 바뀌면 이 파일을 같이 고칠 것.

## 실행 (`run/`)

모든 스크립트는 처음에 환경을 비우고(`env -i`) 이 작업 폴더만 불러온다. `~/ros2_ws` 와 섞이지 않는다.
도메인은 **77**, 팔 포트는 OpenRB-150 고유 이름(`/dev/serial/by-id/...`)이다.
나노(보드 간 랜선 10.77.0.2 가 있는 기계)에서는 ROS 통신을 **랜선과 나노 안으로만** 한다(`config/dds_wired.xml`).
NX 는 랜선으로만 팔을 부를 수 있고, 와이파이의 다른 컴퓨터는 못 부른다. 잠깐 끄려면 `VICA_ARM_DDS=all bash run/...`.

| 명령 | 하는 일 |
|---|---|
| `bash run/check.sh` | 설치·연결 점검. 팔 안 움직임 |
| `bash run/all.sh` | **한 번에**(`launch/all.launch.py`). `camera` 하나로 두 가지 — **`camera:=false`(기본) 시연 데모**: 팔 + MoveIt, 홈 이동·장애물 넣기가 둘 다 끝나면 `press.py down --observe --yes --blind` 한 번(카메라 없이 저장한 자리로, 다른 버튼 `press:=up`, 띄우기만 `press:=none`). **`camera:=true` NX 명령 대기**: 팔 + MoveIt + 손목 카메라 + YOLO 버튼 찾기 + 손 인식 + NX 서비스(`run/service.sh` 와 같음, 홈·장애물 뒤 뜸) — NX 가 부르면 카메라로 찾아 누르고, 손이 보이면 어느 단계든 멈춤·손 빼면 이어감. 스스로 누르지 않는다. 서비스 빼기 `service:=false`, 팔 없이 `use_mock_hardware:=true`. 홈 이동이 실패하면 서비스·누르기를 안 띄운다. Ctrl+C 로 모두 끔(11-14) |
| `bash run/arm.sh` | 팔 제어 + **홈 자세로 이동**. RViz 는 `RVIZ=true` 를 앞에 붙인다 |
| `bash run/arm.sh use_mock_hardware:=true` | 팔 없이 가짜 하드웨어로 |
| `bash run/moveit.sh` | 경로 계획(MoveIt). arm 다음에 |
| `bash run/astra.sh` | 손목 카메라. 정합 켬, TF 덮어쓰기 끔 |
| `bash run/buttons.sh` | 버튼 찾기(YOLO B 엔진, 640, 10 Hz). 깊이가 없으면(거울 스테인리스) 버튼 크기 34.5 mm 로 거리. 최근 10 장 중앙값을 `tools/latest.json` 에 계속 쓴다(`since` = 그 중 가장 오래된 장 시각). astra 다음에 |
| `bash run/pose.sh front` | 관찰 자세(정면, 카메라 바닥 1.05 m, 밑동은 config `hall.yaw_deg` 만큼 더 돌림 — 기본 0 = 정면, `--yaw -25` 면 오른쪽 25°). `right` = 엘베 안(오른쪽 −90°, 0.95 m), `home`, `--cam` 으로 높이, `--dry` 는 계획만 |
| `bash run/press.sh down` | 누르기(단계마다 Enter, `--yes` 면 없이). `up`·`down`·`f1~f4`·`open`·`close`. `--observe` 는 관찰 자세 → 찾기부터(NX 서비스와 같은 흐름). `--watch` 는 엘베 안: 누른 뒤 접근점에서 그 불이 꺼질 때까지 보고 홈. `--dry` 계획만, `--depth -0.02` 는 2 cm 앞에서 멈춤(연습). 호출 버튼은 접근점에서 화살표로 가운데를 맞춰 접근점을 다시 잡은 뒤(`aim`, 최대 2 번, `--no-aim` 이면 안 함) 누른다. 누름점까지 밀다 막대 끝 칸에 빨간불이 보이면 그 자리에서 멈추고, 누름점에 닿은 채 불이 아직이면 2 초까지 기다린다. 전류는 닿았는지만 본다. 호출 버튼은 바로 홈, 엘베 안은 3 cm 빼고 홈(난간을 장애물로 넣었다 뺀다). 마지막에 단계별 `시간:` 줄 |
| `bash run/service.sh` | NX 가 부르는 서비스 `/arm/status·home·press_up·press_down·press_f1~f4·my_floor` (`docs/nx_handoff_2026-10-18.md`). 누르기는 `press.py --observe --yes` 한 번. 엘베 안 지켜보기(`floor.watch: true`)면 `press_f*` 는 불을 보고 바로 "지켜보는 중" 으로 답하고, 팔은 그 불이 꺼질 때(= 그 층 도착)까지 접근점에서 본 뒤 홈. 문이 열리면 NX 가 `/arm/my_floor` 로 묻는다(내 층이면 홈 도착 뒤 true). `/arm/home` 은 지켜보기를 그만두고 홈, 상태는 `/arm/ride`(transient local). 닿음이 없거나 호출 버튼에 불이 안 켜지면 실패로 답한다. 도메인 7 `/estop_state` 를 듣고 비상정지면 멈추고 홈. 시험은 `VICA_ESTOP_DOMAIN=79`, 신호 없이 팔만 시험은 `VICA_ESTOP_REQUIRED=0` |
| `bash run/hand.sh` | 손 인식 혼자 시험(MediaPipe, `run/astra.sh` 다음). 0.8 m 안 손이면 빨간불. 화면 `bash run/view.sh /hand_guard/image`(바로 선 방향) |
| `bash run/guard.sh` | 길목 신호등 혼자 시험(칸 안에 들어오면 빨간불). 누르기(`press.py`)에는 11-02 에 붙였다 — 아래 '손 감시' |
| `bash run/rviz.sh` | 보기 전용 RViz — 로봇·장애물·계획 경로·버튼 공 |
| `bash run/view.sh` | 카메라 영상 보기 (rqt). 깊이는 `bash run/view.sh /camera/depth/image_raw`, 인식은 `/buttons/image` |

### 손 인식(11-02)

`bash run/all.sh camera:=true` 면 `tools/hand_guard.py` 가 손목 색 화면에서 사람 손을 찾는다(MediaPipe Hands, 모델
`~/vica_arm_kit/models/hand/hand_landmarker.task`, 약 75~120 ms/장). 손 관절 자리 깊이로 거리를 재 `hand.max_dist`(0.8 m) 안이거나
못 재면(0.36 m 안) 빨간불 → `press.py` 가 **어느 단계든**(관찰·중간·접근·누르기·홈) 그 자리에서 멈춤 → 초록불 1 초면 다시 계획해 이어감,
30 초 넘으면 실패·팔 그 자리. 손이 보인 뒤 멈추기까지 약 0.3 s. mediapipe 는 `pip3 install --user --no-deps mediapipe==1.2.0 absl-py
sounddevice cffi pycparser` 로 넣었다(그냥 설치하면 numpy 2·OpenCV 를 하나 더 깔아 ROS·YOLO 가 깨질 수 있음).
아래 길목 신호등(깊이 칸)은 그대로 같이 돈다.

### 손 감시(길목 신호등, 11-02)

`press.py` 가 버튼 자리를 안 뒤 버튼 쪽으로 가는 동안 손목 깊이 화면 가운데 네모 × (카메라 앞 ~ 버튼 벽 − 5 cm) 칸을 본다
(`tools/path_guard.py` 의 판정 그대로, 칸 끝은 지금 관절값으로 장마다 다시 잼). 칸에 뭔가 들어오면 **빨간불**(`/path_guard/blocked`,
확인 화면 `bash run/view.sh /path_guard/image`, 터미널 `● 빨간불`) → 그 자리에서 MoveIt stop → 초록불이 1 초 이어지면 지금 자세에서
다시 계획해 이어간다. 30 초 넘게 빨간불이면 실패로 끝내고 팔은 그 자리(홈으로 안 감). 빼기·홈 동안은 안 본다.
Astra 는 0.36 m 안을 못 재므로 카메라 → 벽이 0.45 m 보다 멀 때만 본다 — 호출 버튼은 관찰 자세(약 0.56 m)와 중간 확인점으로 가는
길의 앞쪽 절반쯤. 그 안쪽은 전류 감시만 남는다. 손목 카메라(`run/astra.sh`)가 없으면 경고만 하고 감시 없이 누른다(`guard.required: true`
면 안 누름). 엘베 안은 난간(벽에서 8 cm) 때문에 칸 끝을 벽 − 12 cm 로 둔다 `[가정]`. 설정은 `config/press.yaml` `guard`.

누르기 설정은 `config/press.yaml` 한 곳에 있다 — `hall`(up·down)·`floor`(엘베 안)마다 관찰 자세·카메라 높이·
높이 보정(`z_up`)·누름 깊이·닿음 전류·누른 뒤 빼기 거리, 난간 크기, 불빛 칸(`lit.region`)·문턱, 단계별 속도.
`press.sh` 와 NX 서비스가 같은 값을 쓰고, `press.sh` 옵션을 주면 그 값이 이긴다. `[가정]` 표시는 실물로 아직 못 맞춘 값이다.
누를 때마다 손목 카메라 사진(접근점 기준·불 켜짐·닿음 등)과 불빛 칸 빨강 점 수를 `~/vica_arm_logs/press/<시각>_<버튼>/`
에 남긴다(`sheet.jpg` = 바로 세운 한 장 보기, 초록 네모 = 불빛 칸, `meta.json` = 빨강 점·단계별 시간). 홈으로 가는 동안 쓴다.
모터는 전류 제한 위치 제어(모드 5)라 막혀도 관절별 상한(`urdf/ros2_control.xacro` Goal Current) 넘게 안 민다.
멈추기는 MoveIt "stop"(`/trajectory_execution_event`)으로 한다. ExecuteTrajectory 취소는 Humble 에서 안 멈춘다(10-21 확인).

### 가짜 팔 리허설 (노트북)

실물 없이 관찰 → 찾기 → 누르기 → 홈을 돌려 본다. `tools/photo_cam.py` 가 찍어 둔 Astra 사진 한 장을 손목 카메라처럼 내보내고
(깊이 0, 깃발 파일이 있으면 막대 옆에 빨간불), 진짜 `buttons.py`(노트북은 `.pt` 모델)가 버튼을 찾는다. 명령은 그 파일 머리말.
10-21: 엘베 안 3층(사진 `20261019_test_0523`, f3 신뢰도 0.93·거리 43.5 cm) → 3 cm 빼기 → 홈 12.2 s, 서비스 `/arm/press_f3` 답 13 s.

### 버튼 데이터셋 촬영 (노트북)

손목 Astra 를 노트북에 꽂고 찍는다. 같은 공유기의 나노 팔과 섞이지 않게 도메인 78 을 쓴다.
노트북은 카메라 드라이버가 `~/astra_ws` 에 있어 `run/astra.sh` 가 알아서 불러온다.

1. `VICA_ARM_DOMAIN=78 bash run/astra.sh`
2. `VICA_ARM_DOMAIN=78 bash run/capture.sh --place car` (장소 이름은 폴더 이름이 된다. `hall-3F`·`car` 등)

창 아래 버튼(또는 키): **SAVE**(Space) 수동 저장 · **AUTO**(A) 자동 저장 켜기/끄기 · LIT(L) 불 켜짐 표시 ·
NO-BTN(N) 버튼 없는 화면 · **FLIP**(F) 180° — 로봇처럼 거꾸로 들었으면 켠다 · QUIT(Q).
저장은 `~/Downloads/elev_astra/<날짜>_<장소>/images/*.jpg`(640×480, 글자 없음)와 `meta.jsonl`. 깊이 사진은 저장하지 않는다.
상태 줄의 `center` 가 초록(35~70 cm)이면 관찰 자세 거리와 같다.

찍은 뒤(노트북, ROS 필요 없음)
1. `python3 tools/autolabel.py <촬영 폴더>` — 지금 모델로 초안 라벨(`labels/`, `data.yaml`). 640 + 1280 + 판 잘라 보기를 합친다.
   모델이 헷갈리는 구간은 `--rename up=down:0-195`, `--drop up,panel:196-553`, `--one-panel` 로 다듬는다
2. `python3 tools/upload_roboflow.py <촬영 폴더> --project <id> --valid 170-195 --tag hall:0-195` — 상자째 올린다.
   valid 는 번호 구간(옆 사진끼리 거의 같아서). `--list` 로 프로젝트 id 확인. 끊기면 같은 명령으로 이어 올린다
3. Roboflow 에서 검수한 뒤 `python3 tools/build_dataset.py --project <id> --out ~/Downloads/elev_v4 --local <촬영 폴더>/images --car-panel-from-buttons`
   — 원본 사진 + 최신 라벨로 YOLO 네모 학습 폴더. 버전 내려받기는 늘림(Stretch)·증강 복사본이 섞여 쓰지 않는다
4. 학습(PC 4070 Ti): `yolo detect train data=.../data.yaml model=yolo11s.pt epochs=100 imgsz=640 batch=16 patience=30 fliplr=0.0`
   — **좌우 뒤집기 끔**: 열림(◀▶)을 뒤집으면 닫힘(▶◀)처럼 보인다

## 설치 (`deploy/`)

묶음을 나노 `~/vica_arm_kit` 에 풀고
1. `bash ~/vica_arm_kit/install_system.sh` — sudo 비밀번호 필요. apt·libuvc·udev
2. `bash ~/vica_arm_kit/build.sh` — `~/vica_manipulator_ws` 에 풀고 빌드

## 검증 기록

- 2026-10-16 노트북(x86, Humble) — 기존 URDF 와 관절 11·링크 12·형상·하드웨어 설정이 같음(포트 기본값만 다름).
  가짜 하드웨어로 홈 이동 → MoveIt → `press.py down --depth 0.01` 4 단계를 두 위치에서 계획·실행.
  `~/ros2_ws` 같은 다른 작업 폴더가 불러와진 터미널에서도 스크립트 안에서는 보이지 않음을 확인
- 2026-10-16 나노(ARM, JetPack 6 R36.5) — 5 패키지 빌드 3분 44초. 가짜 하드웨어로 같은 4 단계 통과
- `go_home.py` 가 일을 마치고도 남던 상류 버그를 고쳤다(콜백 안 `rclpy.shutdown()`·`sys.exit()` → 종료 표시 +
  main 반복문). 노트북·나노 모두 "process has finished cleanly" 확인
- 원격에서 띄울 때는 `setsid -f` 로. 비대화형 셸에서 `&` 로 띄우면 SIGINT 가 무시돼 Ctrl+C·kill -INT 로 안 꺼진다
- 2026-10-16 나노 실물 — 새 홈 이동 성공(오차 0.11° 이내, 온도 +4~5 °C). 그 외 실물 시험은 사용자 지시로 보류
- 2026-10-16 노트북(도메인 78) — 장애물을 넣은 채 누르기 3회·관찰 순서(홈→정면→오른쪽 −90°→정면→홈) 계획·실행.
  옛 홈은 cargo_bin 과 충돌로 잡혀 장애물이 작동함을 확인
- 2026-10-18 나노 실물(출력한 버튼판) — 관찰 자세 → 버튼 찾기 → 접근 → 닿기 → 빼기 → 홈 끝까지 성공.
  누름이 1~3 cm 아래로 치우침, down 은 깊이가 자주 0. NX → 나노 `/arm/status` 호출 확인.
  자세한 결과·남은 일은 `docs/nx_handoff_2026-10-18.md`
- 2026-10-21 나노 실물(호출 버튼, 정면) — z_up 0.065·닿음 200 mA 로 10 번 중 7 번 불 켜짐(사진). NX → 나노 서비스 성공 답
  23~25 s, 닿음 → 답 6.1~6.6 s. 같은 날 노트북 가짜 팔에서 ExecuteTrajectory 취소가 안 멈춤을 확인(취소 뒤 3.7~4.2 s 끝까지 가고
  SUCCESS, MoveIt "stop" 은 0.05 s). → 멈추기를 "stop" 으로 바꾸고, 관찰부터 한 프로세스·불빛 고정 칸·호출 버튼 바로 홈으로
  가짜 팔 서비스 답 11 s
- 2026-11-02 나노 가짜 팔(도메인 79) + 가짜 깊이(배경 2 m, 손 0.3 m) — `run/all.sh use_mock_hardware:=true camera:=false` 로
  홈·장애물 → `press down --observe --yes --blind` 자동 시작 확인. 관찰 자세에서 손 → 출발 안 하고 기다렸다 손 뺀 뒤 4.3 s 에 이어감,
  중간 확인점으로 가던 중 손(칸 끝 0.43 m) → 멈춤 → 남은 길(1.4 s)을 다시 계획해 이어감, 손을 안 빼면 30 s 뒤 실패·팔 그 자리.
  `camera:=true` 는 카메라가 안 꽂혀 장치 기다림까지만 확인. **실물(진짜 팔·Astra) 시험은 아직**
- 2026-11-02 나노 가짜 팔(도메인 79) + 저장 사진 가짜 카메라 + `hand_guard.py` — 관찰 자세로 가는 중 손(0.5 m) → 멈춤 → 손 빼고
  4.6 s 에 이어감, 홈으로 가는 중 손 → 멈춤 → 4.3 s 에 이어감. 1.5 m 손은 찾지만 초록불. **실물 시험은 아직**
- 2026-10-21 노트북 가짜 팔 + `photo_cam.py`(엘베 안 사진) — 엘베 안 지켜보기: `/arm/press_f3` 답 9 s "지켜보는 중", 불 켜진 채
  `/arm/my_floor` → false(2 s 더 보고), 불 꺼짐 → true "홈 도착"(2.8 s), 지켜보는 중 `/arm/home` → 홈 3.4 s, 비상정지 → 저속 홈
