#!/usr/bin/env python3
"""손목 Astra 로 buttons.py 가 찾은 버튼을 팔로 누른다.

  bash run/press.sh down --dry       계획만 세워 RViz 에 보여 주고 끝 (팔 안 움직임)
  bash run/press.sh down             단계마다 Enter 를 눌러야 움직인다(설정은 config/press.yaml)
  bash run/press.sh down --yes       Enter 없이 끝까지
  bash run/press.sh down --observe --yes   관찰 자세 → 찾기 → 누르기 → 홈 한 번에(NX 서비스와 같은 흐름)
  bash run/press.sh down --depth -0.02   버튼 2 cm 앞에서 멈춤(틈·높이 확인)
  bash run/press.sh f3               엘베 안 층 버튼(관찰 자세 right 에서). 누르는 동안 난간을 장애물로 넣는다
  (run/arm.sh 와 run/moveit.sh 가 먼저 떠 있어야 한다)

기본값(관찰 높이·높이 보정·누름 깊이·닿음 기준·속도)은 config/press.yaml 의 hall(up·down)/floor(그 밖)에서
읽는다. arm_service(NX 서비스)도 같은 값을 쓴다. 옵션을 주면 그 값이 이긴다.
--depth 가 0 이상인데 누르는 동안 닿음 전류가 안 오르면 '닿음 감지 없음' 으로 실패를 알린다
(빼기·홈은 다 한 뒤). 층 버튼은 두 번 누르면 취소되니 다시 누를지는 부르는 쪽이 정한다.

흐름: (--observe 면 관찰 자세 → 멈춘 뒤 찍은 화면으로 buttons.py 가 버튼을 낼 때까지) → 버튼 위치(latest.json)
      → 접근점(버튼 앞 standoff)·누름점 계산 → 관절값(geom.press_ik) → MoveIt 에 관절 목표로 계획 → 실행
순서: 지금 자세 → 접근점 → 누름점(불빛·전류로 멈춤, 전류면 불을 잠깐 기다림) → (retract 만큼 곧게 빼기) → home
호출 버튼은 retract 0 = 누른 자리에서 바로 홈. 그 길에서 막대 끝은 처음 2 cm 동안 뒤로 2.1·아래로 0.4 cm 라
판을 긁지 않는다(10-21 계산, 누른 관절 → 홈 관절 직선 보간). 엘베 안은 난간·이웃 층 버튼이 있어 3 cm 뺀다.
단계마다 걸린 시간을 '시간:' 줄에 찍는다.

경로·궤적은 MoveIt 이 만든다. 기본은 Pilz PTP(관절을 곧게 보간 + 충돌 검사),
안 되면 OMPL 로 다시 시도한다. 단 누르기(접근점 → 누름점 → 더 누르기)는 11-16 부터 손끝 곧은 선(2 mm 간격 IK,
line_traj)이다. 찌르는 방향은 config push_yaw_deg(벽 법선, 호출 버튼 0 = AMR 정면 벽에 수직), 없으면 방사 방향.
누름점까지 안 닿으면 같은 선으로 seek 만큼 더 밀며 닿음을 찾는다. 단계별 손끝·화살표 자리는 meta.json "push".
"""
import argparse, json, math, os, shutil, signal, sys, threading, time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, DisplayTrajectory, MoveItErrorCodes
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents, RobotState, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402
from path_guard import Guard, debug_image, depth_array  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = ["joint1", "joint2", "joint3", "joint4"]
HOME = [0.016, -1.535, 1.540, 0.724]      # SRDF home 과 같은 값 (납작 접기. 10-18 닿기 1도 앞으로, 11-25 재조립 뒤 밑동 +0.9도, 손목 41.5도)
# 밑동을 크게 돌 때 거치는 '세운 자세'. 10-22 사용자: 짐칸에 큰 박스가 실려 있어 접힌 채 돌면 위팔(홈에서 뒤로 수평)이
# 짐칸·박스 쪽을 쓸고 지나간다 → 세우고 → 돌고 → 내린다. 손으로 기록한 길(config/paths)이 있으면 그 길이 먼저다(route).
UPRIGHT = [-0.068, 0.068, 0.0]            # joint2~4: 위팔 수직·아래팔과 막대 수평(ㄱ자, 사용자 사진). joint1 은 그 자리 값
TURN_RAD = math.radians(20)               # 밑동이 이보다 크게 바뀌면 세운 자세(또는 기록한 길)를 거친다
PATHS = os.path.join(HERE, "..", "config", "paths")   # record_path.py 가 쓴 손으로 옮긴 길
SAME_RAD = 0.02                           # 관절이 모두 이 안이면 같은 자세로 본다
NEAR_HOME_RAD = math.radians(5)           # 기록한 길에서 이 안의 앞·뒤 점은 버리고 홈 자세로 잇는다(off_home)


def via_upright(cur, target):
    """cur → target 사이에 거칠 관절값 목록(마지막 = target). 밑동 변화가 TURN_RAD 를 넘으면 세우기 → 돌기 → 내리기."""
    if cur is None or abs(target[0] - cur[0]) <= TURN_RAD:
        return [list(target)]
    return [[cur[0]] + UPRIGHT, [target[0]] + UPRIGHT, list(target)]


def load_path(name):
    """손으로 기록한 길(config/paths/<name>.yaml)의 거른 점 관절값 목록. 없으면 None."""
    if not name:
        return None
    try:
        with open(os.path.join(PATHS, f"{name}.yaml")) as f:
            pts = [[float(v) for v in p[1:]] for p in yaml.safe_load(f)["keypoints"]]
        return pts if len(pts) >= 2 else None
    except (OSError, KeyError, TypeError, ValueError):
        return None


def same(a, b, tol=SAME_RAD):
    return a is not None and max(abs(x - y) for x, y in zip(a, b)) <= tol


def off_home(path):
    """기록한 길 앞쪽에서 홈 근처(joint1~3 이 NEAR_HOME_RAD 안)에 머문 점을 버린다. 손목(joint4)은 안 본다.

    10-22 실물: 토크를 푼 팔은 기계적으로 닿는 자리에 얹혀 있어 기록 첫 점이 팔꿈치 88.8°(홈 88.2° 는 닿는 자리보다
    1° 앞, 10-18 13번 과부하). 그 점으로 보내자 팔꿈치가 벽을 밀어 전류가 튀고 충돌로 멈췄다 → 홈에서 바로 길로 잇는다.
    """
    k = 0
    while k < len(path) - 1 and max(abs(path[k][j] - HOME[j]) for j in range(3)) < NEAR_HOME_RAD:
        k += 1
    return path[k:]


def route(cur, target, path):
    """cur → target 단계 목록 (종류, 값, 이름): 'ptp' = MoveIt 계획(이름 None = 부르는 쪽 이름), 'path' = 기록한 길 그대로.

    10-22 사용자: 짐칸 박스를 피해 사람이 손으로 옮긴 길(홈 → 엘베 안 관찰 자세)을 그대로 재현한다("직접 했을 때 안
    부딪혔다"). 홈 쪽에서 길 끝(관찰 자세)으로 갈 땐 순서대로, 관찰 쪽에서 홈으로 올 땐 끝 자세로 돌아간 뒤 거꾸로.
    길이 없거나 다른 자세면 via_upright.
    """
    if path and cur is not None:
        if same(target, path[-1], 1e-6) and abs(cur[0] - path[0][0]) <= TURN_RAD:
            return (([] if same(cur, HOME, 0.1) else [("ptp", list(HOME), "홈(기록한 길 출발점)으로")])
                    + [("path", off_home(path), "기록한 길 따라 관찰 자세로")])
        if same(target, HOME, 1e-6) and abs(cur[0] - path[-1][0]) <= TURN_RAD:
            return (([] if same(cur, path[-1]) else [("ptp", path[-1], "기록한 길 끝(관찰 자세)으로")])
                    + [("path", off_home(path)[::-1] + [list(HOME)], "기록한 길 거꾸로 → 홈")])
    steps = via_upright(cur, target)
    names = ["세우기", "세운 채 돌기"] if len(steps) > 1 else []
    return [("ptp", q, names[k] if k < len(steps) - 1 else None) for k, q in enumerate(steps)]


def path_traj(cur, pts, speed, acc=1.0):
    """cur 에서 시작해 pts 를 차례로 지나는 RobotTrajectory 와 걸리는 초. 가장 많이 도는 관절 기준 최대 speed rad/s,
    사다리꼴 가감속(acc rad/s²). 사람 손 속도(기록 시각)는 쓰지 않는다 — 멈칫한 곳까지 따라 하지 않게."""
    q = [list(cur)]
    for p in pts:
        if max(abs(a - b) for a, b in zip(p, q[-1])) > 1e-4:
            q.append(list(p))
    s = [0.0]
    for a, b in zip(q, q[1:]):
        s.append(s[-1] + max(abs(x - y) for x, y in zip(a, b)))
    length = s[-1]
    ta = speed / acc
    if acc * ta * ta > length:                 # 최고 속도까지 못 올라가는 짧은 길
        ta = math.sqrt(length / acc)
    vpk, sa = acc * ta, 0.5 * acc * ta * ta
    total = 2 * ta + (length - 2 * sa) / vpk if vpk > 0 else 0.0

    def t_of(x):
        if x <= sa:
            return math.sqrt(2 * x / acc)
        if x <= length - sa:
            return ta + (x - sa) / vpk
        return total - math.sqrt(max(0.0, 2 * (length - x) / acc))
    ts = [t_of(x) for x in s]
    jt = JointTrajectory(joint_names=list(JOINTS))
    for i, (t, qq) in enumerate(zip(ts, q)):
        if 0 < i < len(q) - 1 and ts[i + 1] > ts[i - 1]:
            vel = [(b - a) / (ts[i + 1] - ts[i - 1]) for a, b in zip(q[i - 1], q[i + 1])]
        else:
            vel = [0.0] * len(JOINTS)
        jt.points.append(JointTrajectoryPoint(positions=qq, velocities=vel,
                                              time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
    return RobotTrajectory(joint_trajectory=jt), total


# AMR 위에서 이 장애물이 MoveIt 에 없으면 짐칸·차체를 뚫는 경로도 '성공'으로 나온다(10-16).
# moveit.launch.py 가 config/scene.yaml 로 자동으로 넣는다. --no-scene 은 책상 위 시험용이다.
REQUIRED_SCENE = {"amr_body", "cargo_bin"}
# 닿은 뒤(빼기·홈)는 엘리베이터 문 시간을 깎는다. 10-18 가짜 팔로 닿기 끝 → 홈 끝 8.1 s(빼기 1.8 + 홈 4.5
# + 대기·계획 0.9 × 2) → NX 조율 E "5 s 이하". 빼기·홈 속도는 config/press.yaml speed, 자리 잡기 대기는 짧게 한다.
QUICK_SETTLE = dict(timeout=0.3, window=0.2)   # 빼기·홈 앞. 접근·누르기 앞은 정확도 때문에 기본(최대 3 s)
ERR = {v: k for k in dir(MoveItErrorCodes) if k.isupper() and isinstance(v := getattr(MoveItErrorCodes, k), int)}
# 전류 감시. 10-20 실물 엘베: 막대가 버튼 4 cm 아래 테두리를 밀어 팔꿈치(ID 13) 과부하로 꺼졌다.
# 값은 /joint_states effort(XM430 Present Current, 한 칸 2.69 mA). 닿음 기준(contact_ma)은 config/press.yaml,
# 충돌 기준은 아래 [가정] — 단계마다 찍는 '기준 대비 최대 변화' 로 실물에서 정한다.
MA_PER_UNIT = 2.69
CONTACT_N = 5             # 누르기: 접근점보다 contact_ma 넘게 100 Hz 로 0.05 s 이어지면 닿음(기록만, 멈추지 않음)
CRASH_MA = 1500.0         # 다른 단계: 어느 관절이든 이만큼 넘게
CRASH_N = 20              # 0.2 s 이어지면 충돌로 보고 멈춤
WATCH = ["joint2", "joint3", "joint4"]     # 닿음을 보는 관절(밑동 joint1 은 찌르는 방향 힘을 거의 안 받는다)
# 엘베 안 버튼. 층 버튼은 토글식이라 두 번 누르면 취소된다(사용자 확인) — 이 파일은 다시 누르지 않는다.
# 크기는 호출 버튼과 같은 3.5 × 3.5 cm(10-21 사용자 실측, 3층 높이 0.91 m)라 BUTTON_MM 34.5 를 그대로 쓴다.
# 다만 오른쪽(팔 축 → 벽 0.45 m)으로 찌를 때 처짐은 아직 안 쟀다(정면 0.53 m 에서 4 cm).
FLOOR_BUTTONS = ["f1", "f2", "f3", "f4", "open", "close"]
CONFIG = os.path.join(HERE, "..", "config", "press.yaml")
FLOOR_Z = 0.415           # world 원점(기둥 바닥)의 바닥 높이 (pose.py 와 같은 값)
HANDRAIL_ID = "handrail"
CAM_FWD, CAM_UP = 0.109, 0.033    # 손끝에서 본 카메라: 뒤로 0.109, 위로 0.033 (arm.xacro)
ASTRA_FX = 510.7          # Astra Mini S 색 화면 초점거리 px (camera_info, 640×480)
PHOTO_KEEP = 100          # 누르기 사진 폴더를 최근 이만큼만 남긴다
SHOT_NAME = {"불 켜짐": "stop_lit", "충돌": "stop_crash", "닿음": "touch", "너무 셈": "too_hard",
             "손 감지": "stop_hand"}   # 멈춘 순간 사진 이름
HOLD_MIN_WALL = 0.38      # 빨간불로 멈춰 기다릴 때 카메라 → 벽 최소(Astra 최소 0.36 m 바로 위)
HAND = "손 감지"          # execute 가 돌려주는 값: 빨간불이 wait_sec 넘게 안 꺼져 그만둠(그 뒤 홈으로 — home_after_hand)
GUARD_DEFAULTS = dict(enabled=True, margin=0.05, min_wall=0.45, roi=None, clear_sec=1.0, wait_sec=20.0,
                      required=False, stale_sec=0.5, face_deg=15.0)
LATEST = os.path.join(HERE, "latest.json")
STOP_WATCH = threading.Event()   # SIGUSR1(서비스의 /arm/home) — 엘베 안 지켜보기를 그만두고 홈


def load_profile(target):
    """config/press.yaml 에서 이 버튼의 묶음(hall = up·down, floor = 엘베 안)과 속도를 읽는다."""
    with open(CONFIG) as f:
        c = yaml.safe_load(f)
    group = "floor" if target in FLOOR_BUTTONS else "hall"
    return group, c[group], c["speed"]


def load_lit():
    with open(CONFIG) as f:
        return yaml.safe_load(f).get("lit") or {"enabled": False}


def load_hand():
    with open(CONFIG) as f:
        return {"stale_sec": 0.6, **(yaml.safe_load(f).get("hand") or {})}


def load_guard():
    with open(CONFIG) as f:
        g = yaml.safe_load(f).get("guard") or {}
    return {**GUARD_DEFAULTS, **g}


def why_moved(label, ok):
    """execute 결과(True 가 아닌 것)를 실패 문장으로."""
    if ok == "충돌":
        return f"{label} 가다 전류가 커서 멈춤(충돌 의심)"
    if ok == HAND:
        return f"{label} 가는 길에 손(물체)이 있어 멈춤 — 빨간불이 안 꺼져 그만두고 홈으로"
    return f"{label} 실행 실패"


def load_aim():
    with open(CONFIG) as f:
        return yaml.safe_load(f).get("aim") or {}


def load_mid():
    with open(CONFIG) as f:
        return yaml.safe_load(f).get("mid") or {}


REAL_DOMAIN = "77"        # 나노 실물 팔(run/common.sh 기본). 가짜 팔·노트북 시험은 78·79


def test_suffix():
    """도메인이 실물 팔(77)이 아니면 '_test<도메인>'. 사진 폴더·성공한 버튼 자리를 실물 것과 따로 둔다 —
    11-16 가짜 팔(79) 시험 3 회가 사진 폴더 최근 100 개 정리에 걸려 10-22 실물 기록 3 건을 지웠다."""
    dom = os.environ.get("ROS_DOMAIN_ID", "")
    return "" if dom == REAL_DOMAIN else f"_test{dom or 0}"


def saved_path(target, suffix=None):
    """누르기에 성공한 버튼 자리(시연 --fallback·--blind 가 쓴다). 저장소 밖(로그 폴더)에 둔다.
    시험 도메인이면 last_press_<버튼>_test<도메인>.json 에 쓴다(실물 자리를 덮어쓰지 않게)."""
    s = test_suffix() if suffix is None else suffix
    return os.path.expanduser(f"~/vica_arm_logs/last_press_{target}{s}.json")


def load_saved(target):
    """저장한 버튼 자리. 시험 도메인은 시험용 파일이 없으면 실물 자리를 읽는다(읽기만 — 가짜 팔 --blind 시험용)."""
    for p in dict.fromkeys((saved_path(target), saved_path(target, ""))):
        try:
            with open(p) as f:
                d = json.load(f)
            if len(d.get("button") or []) == 3:
                return d
        except (OSError, ValueError):
            pass
    return None


def load_photo_dir():
    """누르기 사진을 남길 폴더(config/press.yaml photos). 비어 있으면 None = 안 남김.
    시험 도메인이면 그 옆 <폴더>_test<도메인>(예: ~/vica_arm_logs/press_test79) — 최근 100 개 정리도 그 안에서만."""
    with open(CONFIG) as f:
        d = yaml.safe_load(f).get("photos")
    return os.path.expanduser(d).rstrip("/") + test_suffix() if d else None


def look_pose(pose, cam_h, yaw_deg=0.0):
    """관찰 자세 관절값과 못 풀 때 이유. 카메라를 바닥에서 cam_h m, joint1 축 바로 위에 두고 수평으로 본다.

    손끝(카메라 앞 0.109, 아래 0.033)을 geom.press_ik 로 푼다. right 는 joint1 −90°(엘베 안, 로봇 오른쪽).
    yaw_deg 만큼 밑동(joint1)을 더 돌린다(음수 = 오른쪽). 카메라는 joint1 축 위라 돌려도 자리는 그대로다.
    """
    j, why = geom.press_ik(geom.J1_X + CAM_FWD, 0.0, cam_h - FLOOR_Z - CAM_UP, 0.0)
    if j is None:
        return None, why
    j = list(j)
    j[0] = (-math.pi / 2 if pose == "right" else 0.0) + math.radians(yaw_deg)
    return j, ""


def raw_region(shape, region):
    """lit.region(바로 세운 화면 x1 y1 x2 y2)을 원본(거꾸로 된) 화면 칸으로. 카메라가 거꾸로 달려 있다."""
    h, w = shape[:2]
    x1, y1, x2, y2 = region
    return w - x2, h - y2, w - x1, h - y1


def red_mask(bgr, lit):
    """밝은 빨강 칸(HSV 빨강 색상 + 채도·밝기 문턱)."""
    hsv = cv2.cvtColor(np.ascontiguousarray(bgr), cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return ((hue <= 10) | (hue >= 170)) & (sat >= lit["sat_min"]) & (val >= lit["val_min"])


def save_photos(d, shots, lit):
    """누르기 사진을 d 폴더에 쓰고 사진별 수치를 돌려준다. 홈으로 가는 동안 따로 돈다(답을 늦추지 않게).

    사진(png)은 원본(거꾸로 된) 방향 그대로, sheet.jpg 는 바로 세워 불빛 칸(lit.region)을 네모로 그린다.
    red_px = 칸 안 밝은 빨강 점 개수, red_hue_sv_median = 칸 안 빨강 색상 점(채도 40 이상)의 채도·밝기 중앙값
    → lit 문턱을 다시 맞출 때 본다.
    """
    os.makedirs(d, exist_ok=True)
    out, tiles = {}, []
    for name, (img, t) in shots.items():
        cv2.imwrite(os.path.join(d, name + ".png"), img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        s = {"t": round(t, 3)}
        tile = img.copy()
        if lit.get("region") and "sat_min" in lit:
            x1, y1, x2, y2 = raw_region(img.shape, lit["region"])
            hsv = cv2.cvtColor(np.ascontiguousarray(img[y1:y2, x1:x2]), cv2.COLOR_BGR2HSV).reshape(-1, 3)
            red_hue = ((hsv[:, 0] <= 10) | (hsv[:, 0] >= 170)) & (hsv[:, 1] >= 40)   # 회색(채도 0)은 색상이 0 이라 뺀다
            s["red_px"] = int(red_mask(img[y1:y2, x1:x2], lit).sum())
            if red_hue.any():
                s["red_hue_sv_median"] = [int(v) for v in np.median(hsv[red_hue][:, 1:], 0)]
            cv2.rectangle(tile, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 0), 2)
        out[name] = s
        tile = cv2.rotate(tile, cv2.ROTATE_180)
        cv2.putText(tile, name + (f"  red {s['red_px']}" if "red_px" in s else ""), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        tiles.append(tile)
    if tiles:
        sheet = np.hstack(tiles)
        if sheet.shape[1] > 1920:
            sheet = cv2.resize(sheet, (1920, int(sheet.shape[0] * 1920 / sheet.shape[1])))
        cv2.imwrite(os.path.join(d, "sheet.jpg"), sheet)
    return out


def finish_photos(root, d, shot_meta, lit, trace, info):
    """meta.json·lit_trace.csv(누르는 동안 장마다 불빛 칸 빨강 점, 빈칸 = 화면 없음)를 쓰고 오래된 폴더를 지운다."""
    with open(os.path.join(d, "lit_trace.csv"), "w") as f:
        f.write("t,red_px\n" + "".join(f"{t},{'' if c is None else c}\n" for t, c in trace))
    meta = dict(info, shots=shot_meta,
                lit={k: lit.get(k) for k in ("region", "min_px", "frames", "confirm_sec", "sat_min", "val_min")})
    with open(os.path.join(d, "meta.json"), "w") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    old = sorted(e for e in os.listdir(root) if os.path.isdir(os.path.join(root, e)))
    for e in old[:-PHOTO_KEEP]:                       # 오래된 것부터 지운다(한 번에 약 2~3 MB)
        shutil.rmtree(os.path.join(root, e), ignore_errors=True)


def retract_target(cur, dist):
    """지금 손끝을 같은 높이·같은 숙임 그대로 밑동 쪽으로 dist 만큼 당긴 관절값. 못 풀면 None.

    버튼을 누르다 멈춘 팔을 바로 접으면 막대가 버튼판을 긁을 수 있다. 그래서 먼저 찌른 방향 반대로 뺀다.
    """
    (x, y, z), pitch = geom.fk(cur)
    a = math.atan2(y, x - geom.J1_X)
    j, _ = geom.press_ik(x - dist * math.cos(a), y - dist * math.sin(a), z, pitch)
    return list(j) if j else None


def wait_button(name, since, timeout):
    """관찰 자세에서 팔이 멈춘 뒤(since) 찍은 화면만으로 buttons.py 가 버튼 위치를 낼 때까지 기다린다.

    buttons.py 는 최근 10 장 중앙값을 latest.json 에 쓰고, 창 안 가장 오래된 장의 시각을 since 로 적는다.
    since 가 없는 옛 buttons.py 면 멈춘 뒤 2 초(5 Hz × 10 장)를 기다린다. 찾으면 True.
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with open(LATEST) as f:
                d = json.load(f)
        except (OSError, ValueError):
            d = {}
        b = (d.get("buttons") or {}).get(name)
        if b and b.get("world"):
            first = b.get("since")
            if (first is not None and first >= since) or (first is None and d.get("stamp", 0) >= since + 2.0):
                return True
        time.sleep(0.05)
    return False


def forward_push(q, dist):
    """지금 관절 q 에서 손끝 높이·막대 기울기는 그대로 두고 밑동에서 바깥쪽(찌르는 방향)으로만 dist m 민 관절값.

    joint1 은 그대로, joint2~4 를 수치 야코비안 한 번으로 푼다(몇 mm 라 충분하다). 닿은 뒤 미는 2 단계에 쓴다 —
    목표를 '처지는 만큼 높게(z_up)' 잡은 채 계속 밀면 매끈한 판을 타고 위로 미끄러져 버튼 위쪽을 눌렀다(10-21 밤 실물).
    """
    def f(qq):
        (x, y, z), pitch = geom.fk(qq)
        return np.array([math.hypot(x - geom.J1_X, y), z, pitch])
    q = [float(v) for v in q]
    f0, jac, h = f(q), np.zeros((3, 3)), 1e-4
    for k in range(3):
        qq = list(q)
        qq[k + 1] += h
        jac[:, k] = (f(qq) - f0) / h
    dq = np.linalg.solve(jac, np.array([dist, 0.0, 0.0]))
    return [q[0]] + [q[k + 1] + float(dq[k]) for k in range(3)]


# 곧은 선 궤적의 속도 기준 = config/joint_limits.yaml 관절 최대(속도 4.8 rad/s·가속 5.0 rad/s²). 배율(speed.press 등)을
# 곱해 쓴다 — MoveIt PTP 와 같은 배율이면 걸리는 시간도 거의 같다(10-22 누르기 6.5 cm, PTP 1.9 s)
LINE_VMAX, LINE_AMAX = 4.8, 5.0


def line_traj(cur, end_xyz, vel):
    """지금 관절 cur 의 손끝에서 end_xyz 까지 곧은 선으로 가는 궤적. 막대 숙임(pitch)은 지금 그대로.

    11-16: 누르기를 MoveIt PTP(관절 보간)에서 이 선으로 바꿨다. PTP 도 방사 방향이면 직선에서 2 mm 안이지만,
    벽 법선으로 찌르려면(push_yaw) 손끝을 정한 선 위로 옮겨야 한다. 2 mm 간격 IK 점 → path_traj(사다리꼴 속도).
    (궤적, 초, 경유점) 또는 (None, 이유, None).
    """
    start, pitch = geom.fk(cur)
    pts, why = geom.line_joints(start, end_xyz, pitch)
    if pts is None:
        return None, why, None
    traj, sec = path_traj(cur, pts, vel * LINE_VMAX, vel * LINE_AMAX)
    return traj, sec, pts


def push_frame(button, yaw):
    """찌르는 방향 단위 벡터 u 와, 그 방향에 직각인 '오른쪽'(world 수평) r. 기록·출력용."""
    ux, uy = geom.push_dir(button, yaw)
    return (ux, uy, 0.0), (uy, -ux, 0.0)


def tip_vs_button(q, button, yaw):
    """관절 q 의 손끝이 버튼 면(찌르는 선의 버튼 점) 기준 어디인지 m: (앞으로 + = 면 너머, 오른쪽, 위)."""
    (x, y, z), _ = geom.fk(q)
    u, r = push_frame(button, yaw)
    d = (x - button[0], y - button[1], z - button[2])
    return (sum(a * b for a, b in zip(d, u)), sum(a * b for a, b in zip(d, r)), d[2])


def find_marker(raw, aim):
    """접근점 화면(원본 방향 BGR)에서 버튼 표시(빨간 화살표) 덩어리 가운데를 바로 선 화면 (u, v) 로. 못 찾으면 None.

    10-21 저녁 실물 접근점 사진: 눌린 3 회 화살표 y 232·259·259, 아래를 누른 2 회 213·216 px(1 px ≈ 0.33 mm).
    """
    up = cv2.rotate(raw, cv2.ROTATE_180)
    hsv = cv2.cvtColor(up, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = (((h <= 10) | (h >= 170)) & (s >= aim["sat_min"]) & (v >= aim["val_min"])).astype(np.uint8)
    x1, y1, x2, y2 = aim["window"]
    win = np.zeros_like(m)
    win[y1:y2, x1:x2] = 1
    cnt, _, st, cen = cv2.connectedComponentsWithStats(m & win)
    ks = []
    for k in range(1, cnt):
        x, y, w, h, area = st[k]
        if not (aim["min_px"] <= area <= aim["max_px"] and h >= 15):
            continue
        # 둘레가 밝아야 버튼 인쇄 화살표다. 버튼 바로 위 층 표시기의 빨간 숫자(LED)는 둘레가 검다 —
        # 10-22 실물 영상: 표시기 '3' 을 화살표로 보고 막대를 위로 올렸다
        p = max(6, h // 3)
        ring = v[max(0, y - p):y + h + p, max(0, x - p):x + w + p].astype(float)
        inner = v[y:y + h, x:x + w].astype(float)
        ring_mean = (ring.sum() - inner.sum()) / max(1, ring.size - inner.size)
        if ring_mean >= aim.get("ring_v_min", 120):
            ks.append(k)
    if not ks:
        return None
    tu, tv = aim["target"]                  # 여럿이면 목표 자리에 가장 가까운 것
    k = min(ks, key=lambda k: (cen[k][0] - tu) ** 2 + (cen[k][1] - tv) ** 2)
    return float(cen[k][0]), float(cen[k][1])


def aim_at_center(n, args, aim, button, timing):
    """접근점에서 버튼 표시가 aim.target 에 오도록 손끝을 옮겨 접근점을 다시 잡는다(가운데 맞추기).

    팔 처짐·덜컥임은 모터 각도에 안 잡혀 회마다 막대 높이가 약 1.5 cm 오르내린다(10-21 실물). 카메라와 막대가 한 몸이라
    접근점 화면에서 화살표가 늘 같은 자리에 오면 막대도 버튼 같은 자리를 민다. 보고 → 옮기고 → 다시 보고, 오차가 tol_m 안이면
    끝(최대 tries 번 옮김). 표시를 못 찾거나 한 번에 max_m 넘게 고쳐야 하면 잘못 본 것으로 보고 그대로 둔다.
    aim.require 면 화살표를 못 찾거나, 한 번에 max_m 넘게 고쳐야 하거나(= 버튼이 접근점 화면 가운데 근처에 없음),
    다시 봤을 때 오차가 안 줄면 실패 이유를 돌려준다 — 누르지 않는다(10-21 밤 실물: 접근점 화면에 버튼이 없는데
    포스터의 빨강을 화살표로 보고 2 cm 옮겨 버튼 위 표시기를 눌렀다). (버튼, 접근 관절, 누름 관절, 누름점, 기록, 실패 이유).
    """
    t0, d, log, fail, prev = time.time(), args.standoff + CAM_FWD, [], None, None
    need = bool(aim.get("require", True))
    approach, press = geom.approach_and_press(button, args.standoff, args.depth, args.push_yaw)
    j_app, _ = geom.press_ik(*approach)
    j_prs, _ = geom.press_ik(*press)
    for i in range(int(aim.get("tries", 2)) + 1):
        if not n.camera_live(0.5):
            print("  가운데 맞추기: 카메라 화면이 멈춤 → 고치지 않음")
            break
        got, seen, t1 = [], n.img_n, time.time()
        while len(got) < int(aim.get("frames", 5)) and time.time() - t1 < 1.0:
            rclpy.spin_once(n, timeout_sec=0.02)
            if n.img_n != seen:
                seen = n.img_n
                mk = find_marker(n.img, aim)
                if mk:
                    got.append(mk)
        if len(got) < 3:
            if need:
                fail = "접근점 화면에서 버튼 화살표를 못 찾음 — 누르지 않음"
            print("  가운데 맞추기: 버튼 표시(빨간 화살표)를 못 찾음" + (" → 누르지 않음" if need else " → 고치지 않고 누름"))
            break
        u, v = float(np.median([g[0] for g in got])), float(np.median([g[1] for g in got]))
        up_m = (aim["target"][1] - v) * d / ASTRA_FX        # + = 막대를 위로
        right_m = (u - aim["target"][0]) * d / ASTRA_FX     # + = 막대를 (카메라 기준) 오른쪽으로
        log.append({"u": round(u, 1), "v": round(v, 1), "up_cm": round(up_m * 100, 2), "right_cm": round(right_m * 100, 2)})
        n.shot(f"aim{i}")
        print(f"  가운데 맞추기 {i + 1}: 화살표 ({u:.0f}, {v:.0f}) → 목표 ({aim['target'][0]}, {aim['target'][1]}) — "
              f"위로 {up_m * 100:+.1f} cm, 오른쪽 {right_m * 100:+.1f} cm")
        err = max(abs(up_m), abs(right_m))
        if err <= aim.get("tol_m", 0.003):
            print("  가운데 맞음")
            break
        if err > aim.get("max_m", 0.015):
            print(f"  한 번에 {aim.get('max_m', 0.015) * 100:.1f} cm 넘게 고쳐야 함 — 버튼이 접근점 화면 가운데 근처에 없거나 "
                  "다른 빨강을 본 것" + (" → 누르지 않음" if need else " → 고치지 않음"))
            if need:
                fail = f"접근점에서 버튼이 가운데 근처에 없음(고칠 양 {err * 100:.1f} cm) — 누르지 않음"
            break
        if prev is not None and err > 0.7 * prev:
            print(f"  다시 봤는데 오차가 안 줄었음({prev * 100:.1f} → {err * 100:.1f} cm) — 화살표를 잘못 본 것"
                  + (" → 누르지 않음" if need else " → 더 고치지 않음"))
            if need:
                fail = "가운데 맞추기가 안 좁혀짐(화살표를 잘못 본 것) — 누르지 않음"
            break
        prev = err
        if i == int(aim.get("tries", 2)):
            print("  더 옮기지 않고 누름")
            break
        a = math.atan2(button[1], button[0] - geom.J1_X)       # 밑동 → 버튼(찌르는 방향)
        nb = (button[0] + right_m * math.sin(a), button[1] - right_m * math.cos(a), button[2] + up_m)
        ap2, pr2 = geom.approach_and_press(nb, args.standoff, args.depth, args.push_yaw)
        ja2, why = geom.press_ik(*ap2)
        jp2, why2 = geom.press_ik(*pr2)
        if ja2 is None or jp2 is None:
            print(f"  고친 자리에 못 감({why or why2}) → 고치지 않고 누름")
            break
        traj = plan_step(n, args, f"접근점 다시 잡기 {i + 1}", ja2, args.press_vel, None)
        ok = None if isinstance(traj, str) else n.execute(traj)
        if ok == HAND:
            fail = why_moved(f"접근점 다시 잡기 {i + 1}", ok)
            break
        if ok is not True:
            print("  접근점 다시 잡기 실패 → 지금 자리에서 누름")
            break
        n.settle()
        button, approach, press, j_app, j_prs = nb, ap2, pr2, ja2, jp2
    timing["가운데"] = time.time() - t0
    return button, j_app, j_prs, press, log, fail


def mid_check(n, args, mid, button):
    """버튼 앞 mid.standoff 에 막대 끝을 두고(아래로 mid.pitch_deg 숙여) 버튼을 다시 찾아 목표를 고친다(중간 확인점).

    10-22 실물: 같은 높이를 시켜도 접근점에서 막대가 버튼 위 0.5~6 cm 로 회마다 달랐다. 관찰 자세(62 cm)에서 한 번 본
    값만 믿고 가면 빗나가고, 접근점(6 cm)은 너무 가까워 위아래 2~3 cm 밖은 안 보인다. 그래서 팔을 거의 누르는 자세로
    뻗은 채 30 cm 에서 YOLO(buttons.py)로 다시 찾는다 — 처지는 양이 그 자세에 이미 들어 있다. 막대가 화면 아래를
    가려서 아래로 숙여 본다(막대가 버튼보다 위 약 8 cm·아래 약 6 cm 까지 버튼이 보인다).
    고칠 양이 mid.tol_m 안이면 확인 끝, 아니면 고친 자리로 옮겨 다시 본다(최대 mid.tries 번).
    (고친 버튼 목표, 기록, 실패 이유). 실패면 누르지 않는다. 팔은 그 자리 — 홈은 부르는 쪽이 보낸다.
    """
    standoff, pitch = float(mid.get("standoff", 0.20)), math.radians(float(mid.get("pitch_deg", 15)))
    z_up, tol, max_m = float(mid.get("z_up", 0.0)), float(mid.get("tol_m", 0.01)), float(mid.get("max_m", 0.10))
    tries, log = int(mid.get("tries", 3)), []
    for i in range(tries):
        spot, _ = geom.approach_and_press(button, standoff, 0.0)
        j_mid, why = geom.press_ik(*spot, pitch)
        if j_mid is None:
            return button, log, f"중간 확인점에 못 감({why}) — 누르지 않음"
        traj = plan_step(n, args, f"중간 확인점 {i + 1} (버튼 앞 {standoff * 100:.0f} cm, 아래로 {math.degrees(pitch):.0f}°)",
                         j_mid, args.vel, "quick")
        if isinstance(traj, str):
            return button, log, traj
        ok = n.execute(traj)
        if ok is not True:
            return button, log, why_moved("중간 확인점으로", ok)
        n.settle(timeout=2.0, window=0.3)
        if n.blind:                              # 시연: 저장한 자리(이미 이 단계 보정까지 들어 있음) — 보이기만 하고 안 고친다
            n.spin_for(1.0)
            n.shot(f"mid{i + 1}")
            log.append({"blind": True})
            print("  중간 확인: 저장한 버튼 자리로 시연 — 보지 않고 그대로 누름")
            break
        found = wait_button(args.target, time.time(), float(mid.get("wait_sec", 3.0)))
        n.spin_for(0.15)                    # wait_button 은 파일만 본다 — 사진용 화면을 새로 받는다
        n.shot(f"mid{i + 1}")
        try:
            with open(LATEST) as f:
                b = (json.load(f).get("buttons") or {}).get(args.target) or {}
        except (OSError, ValueError):
            b = {}
        if not found:
            why = ("손목 카메라 화면이 멈춤" if not n.camera_live(0.5) else
                   "버튼은 보이지만 거리를 못 잼(화면 끝이거나 가운데에서 15° 넘게 비낌)" if b and b.get("z_m") is None
                   else "버튼이 안 보임")
            log.append({"seen": False, "why": why})
            if args.fallback:
                print(f"  중간 확인 {i + 1}: {why} → [--fallback] 고치지 않고 누름")
                return button, log, None
            print(f"  중간 확인 {i + 1}: {why} → 누르지 않음")
            return button, log, f"중간 확인점에서 버튼을 다시 못 찾음({why}) — 누르지 않음"
        w = b["world"]
        new = (w[0], w[1], w[2] + z_up)
        a = math.atan2(button[1], button[0] - geom.J1_X)       # 밑동 → 버튼(찌르는 방향)
        dx, dy = new[0] - button[0], new[1] - button[1]
        fwd, right, up = dx * math.cos(a) + dy * math.sin(a), dx * math.sin(a) - dy * math.cos(a), new[2] - button[2]
        err = max(abs(up), abs(right))
        log.append({"u": b.get("u"), "v": b.get("v"), "size_px": b.get("size_px"), "z_m": b.get("z_m"),
                    "conf": b.get("conf"), "world": w, "up_cm": round(up * 100, 2), "right_cm": round(right * 100, 2),
                    "fwd_cm": round(fwd * 100, 2)})
        print(f"  중간 확인 {i + 1}: 카메라 거리 {b['z_m'] * 100:.1f} cm(버튼 크기 {b.get('size_px')} px) → 목표를 "
              f"위로 {up * 100:+.1f} cm, 오른쪽 {right * 100:+.1f} cm, 앞으로 {fwd * 100:+.1f} cm 고침")
        if max(err, abs(fwd)) > max_m:
            print(f"  한 번에 {max_m * 100:.0f} cm 넘게 다름 — 다른 것을 버튼으로 본 것일 수 있음 → 누르지 않음")
            return button, log, f"중간 확인점에서 본 버튼이 {max(err, abs(fwd)) * 100:.1f} cm 달라 누르지 않음"
        button = new
        if err <= tol:
            print("  버튼 위치 확인됨")
            break
        if i == tries - 1:
            print("  더 옮기지 않고 마지막에 본 자리로 누름")
    return button, log, None


def watch_floor(n, lit, sec, timing):
    """접근점에서 방금 누른 버튼 불이 꺼질 때까지 본다. 엘베는 그 층에 서야 그 버튼 불을 끈다. True 또는 멈춘 이유.

    누르기 전 접근점 사진(base)과 지금(불 켜짐)을 견줘 새로 빨개진 점 덩어리(= 불빛)를 찾고, 그 칸의 빨강 점이
    켜짐과 꺼짐 사이 30 % 아래로 lit.off_frames 장 이어 내려가면 꺼짐. 진행은 stdout 줄로 알린다 — arm_service 가
    '지켜보기 시작'·'내 층 불 꺼짐' 줄을 읽어 NX 에 답한다. SIGUSR1(서비스의 /arm/home)이면 그만 보고 홈.
    사람이 버튼판 앞을 가려도 불이 꺼진 것처럼 보인다 — 로봇 혼자 타는 범위에서만 쓴다.
    """
    t0 = time.time()
    n.spin_for(0.2)                                  # 멈춘 뒤 새 화면
    base = n.shots.get("base")
    if base is None or n.img is None:
        return "카메라 화면이 없어 내 층을 지켜볼 수 없음"
    m0 = cv2.dilate(red_mask(base[0], lit).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0   # 조금 흔들려도 같은 점
    new = (red_mask(n.img, lit) & ~m0).astype(np.uint8)
    cnt, _, stats, _ = cv2.connectedComponentsWithStats(new)
    if cnt < 2 or stats[1:, cv2.CC_STAT_AREA].max() < lit["min_px"] / 2:
        return "접근점에서 버튼 불빛이 안 보여 내 층을 지켜볼 수 없음"
    x, y, w, h, _ = stats[1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))]
    pad, (H, W) = max(w, h) // 2, n.img.shape[:2]
    x1, y1, x2, y2 = max(0, x - pad), max(0, y - pad), min(W, x + w + pad), min(H, y + h + pad)
    count = lambda img: int(red_mask(img[y1:y2, x1:x2], lit).sum())   # noqa: E731
    on, off = count(n.img), count(base[0])
    th, need = off + 0.3 * (on - off), int(lit.get("off_frames", 6))
    n.shot("watch")
    print(f"지켜보기 시작: 내 층 버튼 불 {on}점(누르기 전 {off}점) — {th:.0f}점 아래로 {need}장 이어지면 도착", flush=True)
    below, seen = 0, n.img_n
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.02)
        if STOP_WATCH.is_set():
            timing["지켜보기"] = time.time() - t0
            return "지켜보기 그만(홈 요청)"
        if n.img_n == seen:
            continue
        seen = n.img_n
        c = count(n.img)
        below = below + 1 if c < th else 0
        if below >= need:
            n.shot("light_off")
            timing["지켜보기"] = time.time() - t0
            print(f"내 층 불 꺼짐: {c}점 → 홈", flush=True)
            return True
    timing["지켜보기"] = time.time() - t0
    return f"지켜보기 {sec:.0f}초 넘김"


def plan_step(n, args, label, target, vel, settle):
    """자리 잡기 → (Enter 확인) → 계획. 궤적 또는 실패 이유(문자열). settle = 'full'·'quick'·None(안 기다림)."""
    print(f"\n[{label}]")
    n.stage = label
    n.say()
    if settle:
        print(f"  팔이 자리 잡기를 기다림 {n.settle(**({} if settle == 'full' else QUICK_SETTLE)):.1f}초")
    if not args.yes and not args.dry:
        if n.plan_any(target, vel) is None:
            return f"{label} 계획 실패"
        if input("  RViz 에서 경로를 확인하고 Enter = 실행 / q = 중단 : ").strip().lower() == "q":
            return f"{label} 앞에서 사용자가 중단"
        n.settle()                      # Enter 를 기다리는 사이 팔이 조금 움직였을 수 있다 → 지금 자세에서 다시 계획
    return n.plan_any(target, vel) or f"{label} 계획 실패"


class Presser(Node):
    def __init__(self, vel):
        super().__init__("button_presser")
        self.vel = vel
        self.cur = None
        self.amp = None                 # 관절별 전류 mA (JOINTS 순서)
        self.amp_hist = []              # 실행 중 받은 전류(감시용). execute 가 비운다
        self.img, self.img_n, self.img_t = None, 0, 0.0   # 불빛 확인용 손목 카메라 화면(BGR)·장 번호·받은 시각
        self.shots = {}                 # 이름 → (화면, 시각). 홈으로 가는 동안 save_photos 가 쓴다
        self.lit_trace = []             # 누르는 동안 (경과 초, 불빛 칸 빨강 점)
        self.lit_img_n, self.lit_over, self.lit_t0 = 0, 0, time.time()   # 불빛: 마지막으로 본 장 번호·연속 장 수·시작 시각
        self.trip_t = None              # 마지막 실행이 전류·불빛으로 멈춘 시각
        self.touched = False            # 마지막 실행 중 닿음 전류가 올랐음
        self.photos_saved = False       # run_steps 가 사진을 다 썼음(아니면 main 이 끝에 쓴다)
        self.mid_log = None             # 중간 확인점에서 다시 본 기록(meta.json)
        self.blind = False              # 시연: 카메라로 찾은 자리 대신 저장한 버튼 자리로 누름(--blind·--fallback)
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.create_subscription(Image, "/camera/color/image_raw", self._img, qos_profile_sensor_data)
        self.mk_pub = self.create_publisher(MarkerArray, "/button_target", 10)
        self.disp_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.stop_pub = self.create_publisher(String, "/trajectory_execution_event", 10)   # MoveIt "stop"
        self.state_pub = self.create_publisher(String, "/arm/press_state", 10)   # 단계·불빛 칸 빨강 점(record_video.py 가 그림)
        self.stage = ""
        self.plan_cli = ActionClient(self, MoveGroup, "/move_action")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.markers = MarkerArray()
        self.create_timer(0.5, lambda: self.mk_pub.publish(self.markers))   # RViz 가 늦게 붙어도 보이게
        # 길목 신호등(path_guard). 버튼 쪽으로 가는 동안 손목 깊이 화면 칸(카메라 앞 ~ 버튼 벽 − margin)에 뭔가 들어오면
        # 빨간불 → 멈춤. 칸 끝은 지금 관절값으로 장마다 다시 잰다(팔이 다가가면 칸이 줄어든다)
        self.gcfg = load_guard()
        self.guard = Guard()
        self.guard_on = False           # body 가 버튼 자리를 안 뒤 켜고, 빼기·홈(move) 앞에서 끈다
        self.guard_wall = None          # 버튼 world 위치(벽)
        self.guard_margin = float(self.gcfg["margin"])
        self.guard_min = float(self.gcfg["min_wall"])   # 멈춰 기다리는 동안은 HOLD_MIN_WALL 까지 본다
        self.depth_n, self.depth_t, self.guard_n = 0, 0.0, 0
        self.guard_pub = self.create_publisher(Bool, "/path_guard/blocked", 10)
        self.guard_img_pub = self.create_publisher(Image, "/path_guard/image", 2)
        if self.gcfg["enabled"]:
            self.create_subscription(Image, "/camera/depth/image_raw", self._depth, qos_profile_sensor_data)
        # 손 인식(hand_guard.py, camera:=true 일 때만 뜬다). 답이 오는 동안은 어느 단계든 손이 보이면 멈춘다
        self.hand_stale = float(load_hand()["stale_sec"])
        self.hand_seen, self.hand_t, self.hand_used = False, 0.0, False
        # --no-path-guard(시연, run/demo_hand.sh): 손 인식만이 유일한 감시라 답이 끊기면 빨간불로 본다(11-02: 카메라가
        # 안 열려 손 인식이 조용했는데 팔이 손을 무시하고 감)
        self.hand_required = False
        # 11-02 사용자: 빨간불이 wait_sec(20 s) 넘게 안 꺼지면 그만두고 홈으로. 그 홈 길은 손 감시를 끈다
        # (먼 손을 잘못 본 빨간불이면 영영 못 돌아온다). 전류(충돌) 감시는 그대로. move_to 만 켠다
        self.hand_ignore = False
        self.create_subscription(Bool, "/hand_guard/detected", self._hand, 10)

    def _hand(self, m):
        if not self.hand_used:
            self.hand_used = True
            print("  손 인식(hand_guard) 연결됨 — 손이 보이면 어느 단계든 멈춥니다")
        if m.data != self.hand_seen:
            self.say(hand=m.data, by="hand")
        self.hand_seen, self.hand_t = m.data, time.time()

    def hand_live(self):
        return time.time() - self.hand_t <= self.hand_stale

    def guard_far(self):
        """지금 자세에서 칸 끝(카메라 광축 방향 m). 감시할 수 없으면 None.

        안 보는 때: 꺼짐, 버튼 자리 모름, 밑동이 버튼 쪽을 face_deg 넘게 비껴 있음(카메라가 벽을 안 봄),
        카메라 → 벽이 min_wall 보다 가까움(Astra 는 0.36 m 안을 못 재 벽까지 빈칸 = 늘 '렌즈 가림'이 된다).
        """
        if not self.guard_on or self.guard_wall is None or self.cur is None:
            return None
        bx, by, _ = self.guard_wall
        a = math.atan2(by, bx - geom.J1_X)
        if abs(math.remainder(self.cur[0] - a, 2 * math.pi)) > math.radians(self.gcfg["face_deg"]):
            return None
        (tx, ty, _), pitch = geom.fk(self.cur)
        r_cam = math.hypot(tx - geom.J1_X, ty) - CAM_FWD * math.cos(pitch)
        along = (math.hypot(bx - geom.J1_X, by) - r_cam) / max(math.cos(pitch), 0.5)
        if along < self.guard_min:
            return None
        return along - self.guard_margin

    def _depth(self, m):
        self.depth_n += 1
        self.depth_t = time.time()
        far = self.guard_far()
        was = self.guard.red
        depth = depth_array(m)
        roi = self.gcfg["roi"]
        if roi is None:
            cu, cv = m.width // 2, m.height // 2
            roi = (cu - 80, cv - 80, cu + 80, cv + 80)
        if far is None:                 # 안 보는 동안은 초록불로 되돌린다(다시 켜면 처음부터 ON_N 장)
            self.guard.red, self.guard._run = False, 0
            self.guard_n += 1           # 확인 화면은 계속 낸다(회색 네모 = 안 봄). 11-02: 안 내면 rqt 가 멈춘 것처럼 보였다
            if self.guard_n % 3 == 0 and self.guard_img_pub.get_subscription_count():
                img = debug_image(depth, roi, 0.0, False, active=False)
                self.guard_img_pub.publish(Image(header=m.header, height=img.shape[0], width=img.shape[1],
                                                 encoding="rgb8", step=img.shape[1] * 3, data=img.tobytes()))
            return
        u0, v0, u1, v1 = roi
        blocked, n, blank = self.guard.frame_blocked(depth[v0:v1, u0:u1], far * 1000.0)
        red = self.guard.update(blocked)
        self.guard_pub.publish(Bool(data=red))
        if red != was:
            size = (u1 - u0) * (v1 - v0)
            print(f"  {'● 빨간불 — 길목에 손(물체)' if red else '○ 초록불 — 길목 비었음'}: 칸 끝 {far:.2f} m, "
                  f"칸 안 점 {n}/{size} ({100 * n / size:.1f}%), 빈칸 {100 * blank:.0f}%")
            self.say(hand=red, far=round(far, 3))
        self.guard_n += 1
        if self.guard_n % 3 == 0 and self.guard_img_pub.get_subscription_count():
            img = debug_image(depth, roi, far * 1000.0, red)
            self.guard_img_pub.publish(Image(header=m.header, height=img.shape[0], width=img.shape[1],
                                             encoding="rgb8", step=img.shape[1] * 3, data=img.tobytes()))

    def hand_red(self):
        """지금 빨간불인가. 손 인식이 손을 봤거나, 길목 칸(깊이)이 막혔으면 True.
        길목 칸은 감시 중인데 깊이 화면이 stale_sec 넘게 안 오면 required 일 때만 빨간불로 본다.
        hand_ignore(손 때문에 그만두고 홈으로 가는 중)면 늘 False."""
        if self.hand_ignore:
            return False
        if self.hand_live() and self.hand_seen:
            return True
        if self.hand_required and not self.hand_live():
            return True
        if self.guard_far() is None:
            return False
        if time.time() - self.depth_t > self.gcfg["stale_sec"]:
            return bool(self.gcfg["required"])
        return self.guard.red

    def all_clear(self):
        """멈춘 채 '비었다' 고 말할 수 있나. 볼 수 있는 쪽이 하나도 없으면 비었다고 하지 않는다."""
        if self.hand_live() and self.hand_seen:
            return False
        if self.hand_required:
            return self.hand_live()
        if self.guard_far() is not None:                 # 길목 칸을 볼 수 있는 자리
            return time.time() - self.depth_t <= self.gcfg["stale_sec"] and not self.guard.red
        return self.hand_live()                          # 칸을 못 보는 자리(가까움·홈 쪽) → 손 인식이 봐 줘야

    def wait_clear(self):
        """멈춘 채 초록불이 clear_sec 이어질 때까지 기다린다(최대 wait_sec). 비었으면 True."""
        clear, sec = float(self.gcfg["clear_sec"]), float(self.gcfg["wait_sec"])
        print(f"  ■ 빨간불: 멈춘 채 손이 빠지길 기다림(초록불 {clear:.1f}초 이어지면 이어감, 최대 {sec:.0f}초)")
        stage, self.stage = self.stage, f"{self.stage} — 빨간불(손 감지) 멈춤"
        self.say(hand=True)
        # 멈추는 사이 min_wall 안으로 조금 들어왔어도 계속 본다. 못 보면(화면 없음·너무 가까움) 비었다고 하지 않는다
        self.guard_min = min(self.guard_min, HOLD_MIN_WALL)
        t0, green = time.time(), None
        try:
            while time.time() - t0 < sec:
                rclpy.spin_once(self, timeout_sec=0.02)
                if not self.all_clear():
                    green = None
                elif green is None:
                    green = time.time()
                elif time.time() - green >= clear:
                    print(f"  ○ 초록불 {clear:.1f}초 — {time.time() - t0:.1f}초 멈췄다가 이어갑니다")
                    return True
            print(f"  ■ {sec:.0f}초 동안 빨간불이 안 꺼짐 → 그만두고 홈으로 돌아갑니다")
            self._hand_stuck = True
            return False
        finally:
            self.guard_min = float(self.gcfg["min_wall"])
            self.stage = stage
            self.say(hand=False)

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.cur = [d[j] for j in JOINTS]
        e = dict(zip(m.name, m.effort))
        if all(j in e for j in JOINTS):
            self.amp = [abs(e[j]) * MA_PER_UNIT for j in JOINTS]
            self.amp_hist.append(self.amp)

    def _img(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        self.img = a if m.encoding == "bgr8" else a[:, :, ::-1]
        self.img_n += 1
        self.img_t = time.time()

    def red_count(self, lit):
        """막대 끝 둘레 칸(lit.region)의 밝은 빨강 점 개수. 화면이 없으면 None.

        카메라와 막대가 한 몸이라 버튼에 닿은 순간 버튼은 화면에서 늘 막대 끝 바로 옆에 있다. 그래서 위치를
        계산하지 않고 고정 칸만 본다(10-21 실물: 켜지면 막대 양옆으로 225~485 점, 꺼져 있으면 0~41 점).
        """
        if self.img is None:
            return None
        x1, y1, x2, y2 = raw_region(self.img.shape, lit["region"])
        return int(red_mask(self.img[y1:y2, x1:x2], lit).sum())

    def shot(self, name):
        """지금 화면을 사진 기록에 담는다(디스크에는 홈으로 가는 동안 쓴다). 화면이 없으면 건너뜀."""
        if self.img is not None:
            self.shots[name] = (self.img.copy(), time.time())

    def camera_live(self, sec=0.5):
        """sec 초 안에 새 화면이 왔는지. 10-21 저녁 실물: USB EMI 뒤 화면이 멈춘 채 옛 화면만 남았다."""
        t0 = time.time()
        while time.time() - self.img_t > sec and time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)
        return time.time() - self.img_t <= sec

    def red_baseline(self, lit, sec=0.3):
        """멈춰 있는 동안 불빛 칸 빨강 점 개수 중앙값(새 화면 3 장 이상, 최대 1 s 기다림). 못 모으면 None."""
        vals, seen = [], self.img_n
        t0 = time.time()
        while time.time() - t0 < sec or (len(vals) < 3 and time.time() - t0 < 1.0):
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.img_n != seen:
                seen = self.img_n
                c = self.red_count(lit)
                if c is not None:
                    vals.append(c)
        return int(np.median(vals)) if len(vals) >= 3 else None

    def say(self, **kw):
        """지금 단계와 덧붙일 값(빨강 점 등)을 /arm/press_state 에 JSON 으로 낸다. 시연 영상(record_video.py)이 화면에 쓴다."""
        self.state_pub.publish(String(data=json.dumps(dict(stage=self.stage, t=round(time.time(), 3), **kw),
                                                      ensure_ascii=False)))

    def light_on(self, light):
        """새 화면이면 불빛 칸 빨강 점을 세어 기록한다. 기준보다 min_px 넘게 frames 장 이어졌으면 True."""
        if light is None or self.img_n == self.lit_img_n:
            return False
        self.lit_img_n = self.img_n
        base, lit = light
        c = self.red_count(lit)
        if c is None:
            return False
        self.lit_trace.append((round(time.time() - self.lit_t0, 3), c))
        self.lit_over = self.lit_over + 1 if c - base >= lit["min_px"] else 0
        on = self.lit_over >= lit["frames"]
        self.say(red=c, base=base, need=base + lit["min_px"], lit=on)
        return on

    def wait_light(self, light, sec, base=None, touch_ma=None):
        """누름점에 누른 채 sec 초까지 불을 기다린다. 켜지면 True. light 가 None 이면 sec 초 누르고만 있는다.

        서 있는 동안에도 모터는 목표(면 + depth)를 향해 계속 민다 — 10-21 실물에서 버튼은 이 사이에 눌렸다.
        그 사이 전류가 base 보다 touch_ma 넘게 오르면 self.touched 를 세운다.
        """
        t0, seen, n_over = time.time(), len(self.amp_hist), 0
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.01)
            if base is not None and touch_ma is not None and not self.touched:
                rows, seen = self.amp_hist[seen:], len(self.amp_hist)
                for r in rows:
                    top = max(r[JOINTS.index(j)] - base[JOINTS.index(j)] for j in WATCH)
                    n_over = n_over + 1 if top > touch_ma else 0
                    if n_over >= CONTACT_N:
                        self.touched = True
                        print(f"  ● 닿음(누른 채): 전류가 기준보다 {top:.0f} mA")
                        break
            if self.light_on(light):
                return True
        return False

    def amp_baseline(self, sec=0.3):
        """멈춰 있는 동안 sec 초 전류 평균(관절별 mA)."""
        self.amp_hist = []
        self.spin_for(sec)
        rows = self.amp_hist or [self.amp or [0.0] * len(JOINTS)]
        return [sum(r[i] for r in rows) / len(rows) for i in range(len(JOINTS))]

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def settle(self, timeout=3.0, tol=0.002, window=0.5):
        """팔이 자리를 잡을 때까지 기다린다(window 초 동안 관절 변화 < tol rad). 걸린 초를 돌려준다.

        뻗은 자세는 도착한 뒤에도 I 게인이 몇 초에 걸쳐 관절을 끌어당긴다. 그 사이에 세운 계획은
        실행할 때 출발점이 0.01 rad 넘게 달라 MoveIt 이 거부한다(10-18 실기 2회, CONTROL_FAILED).
        """
        t0 = time.time()
        hist = []
        while time.time() - t0 < timeout:
            self.spin_for(0.1)
            if self.cur is None:
                continue
            hist.append(list(self.cur))
            n = int(window / 0.1)
            if len(hist) >= n and max(max(h[i] for h in hist[-n:]) - min(h[i] for h in hist[-n:])
                                      for i in range(len(JOINTS))) < tol:
                break
        return time.time() - t0

    def wait_future(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def path_blocked(self, pts):
        """기록한 길의 점마다 MoveIt 충돌 검사(차체·짐칸·마스트·자기 몸). 걸린 첫 점의 설명, 다 통과면 None."""
        if not self.valid_cli.wait_for_service(timeout_sec=3.0):
            return "MoveIt 충돌 검사(/check_state_validity)가 없음"
        for k, q in enumerate(pts):
            r = GetStateValidity.Request(group_name="arm")
            r.robot_state = RobotState(joint_state=JointState(name=list(JOINTS), position=[float(v) for v in q]))
            res = self.wait_future(self.valid_cli.call_async(r), 3.0)
            if res is None:
                return "충돌 검사 답 없음"
            if not res.valid:
                hit = sorted({f"{c.contact_body_1}-{c.contact_body_2}" for c in res.contacts})
                return f"{k + 1}번째 점이 {', '.join(hit) or '충돌'}"
        return None

    def go_path(self, pts, speed):
        """손으로 기록한 길을 지금 자세에서 이어 따라간다(충돌 검사 → 한 궤적으로 실행). True, HAND 또는 실패 이유."""
        why = self.path_blocked(pts)
        if why:
            return f"기록한 길이 MoveIt 장애물에 걸림({why}) — 움직이지 않음"
        traj, sec = path_traj(self.cur, pts, speed)
        print(f"  기록한 길 {len(pts)}점, 약 {sec:.1f}초(관절 최대 {speed} rad/s)")
        self.disp_pub.publish(DisplayTrajectory(trajectory=[traj]))
        ok = self.execute(traj)
        if ok is True or ok == HAND:                 # HAND: 부르는 쪽이 홈으로(home_after_hand·move_to)
            return ok
        return "기록한 길을 가다 " + ("전류가 커서 멈춤(충돌 의심)" if ok == "충돌" else "실행 실패")

    # ---------- RViz 표시 ----------
    def set_markers(self, button, approach, press):
        arr = MarkerArray()
        for i, (p, rgb, size, ns) in enumerate([(button, (1.0, 0.1, 0.1), 0.02, "button"),
                                                (approach, (1.0, 0.8, 0.0), 0.015, "approach"),
                                                (press, (0.1, 0.6, 1.0), 0.012, "press")]):
            m = Marker()
            m.header.frame_id = "world"
            m.ns, m.id, m.type, m.action = ns, i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 0.9
            arr.markers.append(m)
        self.markers = arr
        self.mk_pub.publish(arr)

    # ---------- MoveIt ----------
    def scene_box(self, oid, center=None, yaw=0.0, dims=None):
        """world 에 상자 장애물을 넣는다(center 가 None 이면 뺀다). 성공 여부."""
        o = CollisionObject()
        o.header.frame_id = "world"
        o.id = oid
        if center is None:
            o.operation = CollisionObject.REMOVE
        else:
            o.operation = CollisionObject.ADD
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dims))
            p = Pose()
            p.position.x, p.position.y, p.position.z = center
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            o.primitives, o.primitive_poses = [box], [p]
        cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not cli.wait_for_service(timeout_sec=5.0):
            return False
        req = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        req.scene.world.collision_objects = [o]
        res = self.wait_future(cli.call_async(req), 5.0)
        return bool(res and res.success)

    def set_handrail(self, button, rail):
        """찾은 층 버튼 면을 벽으로 보고, 그 벽에서 로봇 쪽으로 튀어나온 난간을 버튼 앞뒤로 넣는다."""
        x, y, _ = button
        a = math.atan2(y, x - geom.J1_X)                 # 밑동에서 버튼 쪽 방향(찌르는 방향)
        r = math.hypot(x - geom.J1_X, y) - rail["protrusion"] / 2
        center = (geom.J1_X + r * math.cos(a), r * math.sin(a),
                  (rail["z_bottom"] + rail["z_top"]) / 2 - FLOOR_Z)
        dims = (rail["protrusion"], rail["length"], rail["z_top"] - rail["z_bottom"])
        return self.scene_box(HANDRAIL_ID, center, a, dims)

    def scene_names(self):
        cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        if not cli.wait_for_service(timeout_sec=5.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = self.wait_future(cli.call_async(req), 5.0)
        return {o.id for o in res.scene.world.collision_objects} if res else None

    def plan(self, target, vel, pipeline, planner):
        self.plan_vel = vel                              # 손 감지로 멈춘 뒤 같은 목표를 다시 계획할 때 쓴다
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = "arm"
        r.pipeline_id, r.planner_id = pipeline, planner
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = vel
        r.max_acceleration_scaling_factor = vel
        r.start_state.is_diff = True                     # 지금 실제 자세에서 출발
        c = Constraints()
        for name, val in zip(JOINTS, target):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=float(val),
                                                       tolerance_above=0.001, tolerance_below=0.001,
                                                       weight=1.0))
        r.goal_constraints.append(c)
        g.planning_options.plan_only = True
        if not self.plan_cli.wait_for_server(timeout_sec=5.0):
            print("  move_group 이 없습니다. MoveIt 을 먼저 띄우세요")
            return None
        h = self.wait_future(self.plan_cli.send_goal_async(g), 10.0)
        if h is None or not h.accepted:
            print("  계획 요청이 거절됐습니다")
            return None
        res = self.wait_future(h.get_result_async(), 30.0)
        if res is None:
            print("  계획 응답이 없습니다")
            return None
        code = res.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            print(f"  계획 실패 ({pipeline}/{planner}): {ERR.get(code, code)}")
            return None
        traj = res.result.planned_trajectory
        d = DisplayTrajectory()
        d.trajectory_start = res.result.trajectory_start
        d.trajectory.append(traj)
        self.disp_pub.publish(d)                        # RViz 에 계획 재생
        pts = traj.joint_trajectory.points
        dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 if pts else 0
        print(f"  계획 성공 ({pipeline}/{planner}) — 점 {len(pts)}개, {dur:.1f}초")
        return traj

    def plan_any(self, target, vel):
        t = self.plan(target, vel, "pilz_industrial_motion_planner", "PTP")
        if t is None:
            print("  → OMPL 로 다시 시도")
            t = self.plan(target, vel, "ompl", "RRTConnectkConfigDefault")
        return t

    def execute(self, traj, replan=None, **kw):
        """궤적 실행. 길목 신호등이 빨간불이면 멈추고(출발 전이면 출발하지 않고) 손이 빠지길 기다린 뒤,
        지금 자세에서 같은 끝 자세로 다시 계획해 이어간다. 빨간불이 wait_sec 넘게 안 꺼지면 HAND.
        replan(지금 관절 → 궤적 또는 None)을 주면 다시 계획할 때 MoveIt PTP 대신 그것을 쓴다(곧은 선 누르기).
        나머지 결과는 _execute_once 와 같다."""
        names = list(traj.joint_trajectory.joint_names)
        last = traj.joint_trajectory.points[-1].positions if traj.joint_trajectory.points else None
        target = [float(last[names.index(j)]) for j in JOINTS] if last is not None and set(JOINTS) <= set(names) \
            else None
        vel, touched, stopped = getattr(self, "plan_vel", 0.1), False, False
        while True:
            if self.hand_red() and not self.wait_clear():   # 출발 전이면 그 자리에서(처음 궤적 그대로 쓸 수 있다)
                return HAND
            if stopped:                                  # 가다 멈췄다 → 처음 궤적은 출발점이 달라 못 쓴다
                # 멈추며 생긴 흔들림이 가라앉은 자세에서(11-02 실물: 1 s 로는 모자라 '출발점이 0.01 넘게 다름' 거부)
                self.settle()
                kw.pop("shots_at", None)                 # 찍을 시각은 처음 궤적 기준이라 이어가기에선 안 맞는다
                traj = (replan(self.cur) if replan else self.plan_any(target, vel)) if target else None
                if traj is None:
                    return False
            ok = self._execute_once(traj, **kw)
            if ok is False and stopped and target:
                print("  다시 이어가기 실행 거부 — 자리 잡고 한 번 더 계획")
                self.settle()
                traj = replan(self.cur) if replan else self.plan_any(target, vel)
                ok = False if traj is None else self._execute_once(traj, **kw)
            touched = touched or self.touched
            if ok != HAND:
                self.touched = touched
                return ok
            stopped = True

    @property
    def hand_stuck(self):
        """마지막 wait_clear 가 시간을 넘겼다(빨간불이 안 꺼짐) — 그 뒤 홈 길(move_to)은 손 감시를 끈다."""
        return getattr(self, "_hand_stuck", False)

    def _execute_once(self, traj, base=None, stop_ma=CRASH_MA, n_over=CRASH_N, why="충돌", touch_ma=None, light=None,
                      shots_at=None):
        """궤적 실행 + 전류(·불빛·길목 신호등) 감시. True = 끝까지 감, False = 실패, 문자열 = 그 이유로 멈춤.

        shots_at = [(실행 시작부터 초, 사진 이름), …] 이면 그 시각에 손목 카메라 사진을 담는다(누르기 1 cm 앞 등).

        base(관절별 mA)가 있으면 base 대비 오름을, 없으면 전류 자체를 WATCH 관절마다 본다.
        stop_ma 를 n_over 장(100 Hz) 이어 넘으면 멈춘다(why). touch_ma 를 CONTACT_N 장 넘으면 멈추지 않고
        self.touched 만 세운다(누르기: 닿았는지 기록). light = (빨강 점 기준, lit 설정) 이면 새 화면마다 불빛 칸
        빨강 점이 기준보다 min_px 넘게 frames 장 이어질 때 '불 켜짐' 으로 멈춘다.

        멈추기는 MoveIt "stop"(/trajectory_execution_event)으로 한다. ExecuteTrajectory 취소(cancel)는 Humble 에서
        멈추지 않는다 — 10-21 가짜 팔: cancel 뒤 3.7~4.2 s 동안 궤적을 끝까지 가고 SUCCESS, stop 은 0.05 s 에 PREEMPTED.
        그래서 10-21 까지 '닿음 → 실행 취소' 는 실제로는 멈추지 않고 누름점(면 + depth)까지 밀었다.
        """
        self.trip_t, self.touched = None, False
        if light:
            self.lit_over, self.lit_img_n, self.lit_t0 = 0, self.img_n, time.time()
        if not self.exec_cli.wait_for_server(timeout_sec=5.0):
            print("  /execute_trajectory 가 없습니다")
            return False
        g = ExecuteTrajectory.Goal()
        g.trajectory = traj
        self.amp_hist = []
        h = self.wait_future(self.exec_cli.send_goal_async(g), 10.0)
        if h is None or not h.accepted:
            print("  실행 요청이 거절됐습니다")
            return False
        fut = h.get_result_async()
        idx = [JOINTS.index(j) for j in WATCH]
        over, touch_n, peak, seen, tripped = 0, 0, [0.0] * len(JOINTS), 0, None
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 60.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            while shots_at and time.time() - t0 >= shots_at[0][0]:
                self.shot(shots_at.pop(0)[1])
            rows, seen = self.amp_hist[seen:], len(self.amp_hist)
            for r in rows:
                rise = [r[i] - (base[i] if base else 0.0) for i in range(len(JOINTS))]
                peak = [max(p, v) for p, v in zip(peak, rise)]
                top = max(rise[i] for i in idx)
                j = JOINTS[max(idx, key=lambda i: rise[i])]
                if touch_ma is not None and not self.touched:
                    touch_n = touch_n + 1 if top > touch_ma else 0
                    if touch_n >= CONTACT_N:
                        self.touched = True
                        self.shot("touch")
                        print(f"  ● 닿음: {j} 전류가 기준보다 {top:.0f} mA ({touch_ma:.0f} 넘게 {CONTACT_N / 100:.2f}초) "
                              f"— 멈추지 않고 누름점까지 민다(불이 켜지면 그때 멈춤)")
                if stop_ma is not None:
                    over = over + 1 if top > stop_ma else 0
                    if over >= n_over and tripped is None:
                        tripped = why
                        self.shot(SHOT_NAME.get(why, "stop"))
                        print(f"  ■ {why}: {j} 전류가 {'기준보다 ' if base else ''}{top:.0f} mA "
                              f"({stop_ma:.0f} 넘게 {n_over / 100:.2f}초) → 멈춤")
            if tripped is None and self.hand_red():
                tripped = HAND
                self.shot(SHOT_NAME[HAND])
                print("  ■ 빨간불: " + ("손 인식 — 손이 보임" if self.hand_live() and self.hand_seen
                                     else "손 인식 답이 끊김(카메라 확인)" if self.hand_required
                                     else "길목 칸 안에 손(물체)") + " → 멈춤")
            if tripped is None and self.light_on(light):
                tripped = "불 켜짐"
                self.shot(SHOT_NAME[tripped])
                print(f"  ■ 불 켜짐: 막대 끝 칸 빨강 {self.lit_trace[-1][1]}점(기준 {light[0]}, "
                      f"+{light[1]['min_px']} 넘게 {light[1]['frames']}장) → 멈춤")
            if tripped:
                break
        if tripped:
            self.trip_t = time.time()
            self.stop_pub.publish(String(data="stop"))
            while not fut.done() and time.time() - self.trip_t < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)
            print(f"  멈춤 처리 {time.time() - self.trip_t:.2f}초")
        self.spin_for(0.05)                 # 끝난 자세를 /joint_states 로 한 번 받기(100 Hz)
        print("  전류 " + ("기준 대비 최대 오름" if base else "최대") + ": "
              + ", ".join(f"{j} {p:.0f}" for j, p in zip(JOINTS, peak)) + " mA")
        if light:
            top = max((c for _, c in self.lit_trace if c is not None), default=None)
            print(f"  막대 끝 칸 빨강: 기준 {light[0]}점, 최대 " + (f"{top}점" if top is not None else "못 봄(화면 없음)"))
        if tripped:
            return tripped
        res = fut.result() if fut.done() else None
        code = res.result.error_code.val if res else None
        if code != MoveItErrorCodes.SUCCESS:
            print(f"  실행 실패: {ERR.get(code, code)}")
            return False
        return True


def load_button(name, max_age):
    path = LATEST
    if not os.path.exists(path):
        sys.exit("버튼 찾기 결과(latest.json)가 없습니다. run/buttons.sh 가 켜져 있는지 보세요")
    with open(path) as f:
        d = json.load(f)
    age = time.time() - d["stamp"]
    if age > max_age:
        sys.exit(f"latest.json 이 {age:.1f}초 전 것입니다. buttons.py(run/buttons.sh)가 켜져 있고 버튼이 보이는지 확인하세요")
    if not d.get("calibrated"):
        sys.exit("detect.py 가 아직 보정 전입니다 (--calib)")
    b = d["buttons"].get(name)
    if not b:
        sys.exit(f"'{name}' 버튼이 지금 안 보입니다. 보이는 것: {list(d['buttons'])}")
    if b.get("z_m") is None:
        sys.exit(f"'{name}' 버튼은 보이지만 거리를 모릅니다(깊이 0, 크기로도 못 잼). "
                 "버튼이 화면 끝에 걸렸거나 정면에서 15° 넘게 비껴 있으면 크기로 재지 않습니다 — 버튼을 화면 가운데로")
    if not b.get("world"):
        sys.exit(f"'{name}' 버튼은 보이지만 팔 기준 위치가 없습니다. 팔(run/arm.sh)이 켜져 TF 가 나오는지 보세요")
    return b


def wait_joints(n):
    for _ in range(50):                 # 새 노드가 /joint_states 와 이어지는 데 1초 넘게 걸릴 때가 있다(10-18)
        n.spin_for(0.1)
        if n.cur is not None:
            return True
    return False


def check_scene(n):
    """AMR 장애물이 MoveIt 에 있는지. 목록 읽기가 가끔 비어 오므로(10-21 실물 1회) 세 번까지 다시 읽는다."""
    names = None
    for _ in range(3):
        names = n.scene_names()
        if names is not None and not REQUIRED_SCENE - names:
            return names, ""
        n.spin_for(0.5)
    if names is None:
        return None, "MoveIt 장애물 목록을 못 읽었습니다. run/moveit.sh 가 떠 있는지 보세요"
    return None, (f"MoveIt 에 AMR 장애물이 없습니다: {sorted(REQUIRED_SCENE - names)}. "
                  "run/moveit.sh 를 다시 띄우면 config/scene.yaml 로 자동으로 넣습니다")


def main():
    T0 = time.time()
    signal.signal(signal.SIGUSR1, lambda *_: STOP_WATCH.set())
    ap = argparse.ArgumentParser()
    ap.add_argument("target", choices=["up", "down"] + FLOOR_BUTTONS,
                    help="up·down = 승강장 호출 버튼(관찰 자세 front), 나머지 = 엘베 안 버튼(관찰 자세 right)")
    ap.add_argument("--observe", action="store_true",
                    help="관찰 자세로 먼저 가서 버튼을 새로 찾은 뒤 누른다(arm_service.py 가 쓴다)")
    ap.add_argument("--dry", action="store_true", help="계획만 보여 주고 움직이지 않는다")
    ap.add_argument("--standoff", type=float, default=0.06, help="버튼 앞 멈출 거리 m (기본 6 cm)")
    ap.add_argument("--depth", type=float, default=None,
                    help="버튼 면보다 더 들어갈 거리 m. 음수면 그만큼 앞에서 멈춤 (기본 config/press.yaml)")
    ap.add_argument("--vel", type=float, default=None, help="접근 속도 배율 (기본 config/press.yaml speed.approach)")
    ap.add_argument("--max-age", type=float, default=2.0, help="latest.json 허용 나이 초")
    ap.add_argument("--no-scene", action="store_true", help="AMR 장애물 확인을 건너뛴다 (책상 위 시험용)")
    ap.add_argument("--yes", action="store_true", help="단계마다 Enter 확인 없이 진행 (arm_service.py 가 쓴다)")
    ap.add_argument("--retract", type=float, default=None,
                    help="누른 뒤 홈 전에 곧게 뺄 거리 m, 0 = 바로 홈 (기본 config/press.yaml)")
    ap.add_argument("--retract-vel", type=float, default=None, help="뒤로 빼기 속도 배율 (기본 config/press.yaml)")
    ap.add_argument("--home-vel", type=float, default=None, help="홈 속도 배율 (기본 config/press.yaml)")
    ap.add_argument("--z-up", type=float, default=None,
                    help="목표 높이를 이만큼 올림 m. 뻗은 팔이 처지는 만큼 (기본 config/press.yaml)")
    ap.add_argument("--yaw", type=float, default=None,
                    help="관찰 자세를 밑동에서 이만큼 더 돌림 도(음수 = 오른쪽, 기본 config/press.yaml yaw_deg)")
    ap.add_argument("--no-aim", action="store_true", help="접근점에서 가운데 맞추기를 하지 않는다")
    ap.add_argument("--no-path-guard", action="store_true",
                    help="길목 신호등(깊이 칸)을 끈다 — 손 인식(hand_guard)으로만 멈춤. 11-02 시연용(run/demo_hand.sh)")
    ap.add_argument("--no-mid", action="store_true", help="중간 확인점(버튼 앞 20 cm 에서 다시 찾기)을 건너뛴다")
    ap.add_argument("--right", type=float, default=0.0,
                    help="누르는 자리를 로봇 기준 오른쪽으로 m (음수 = 왼쪽). 찾은 자리·저장한 자리 모두에 마지막으로 더한다")
    ap.add_argument("--fallback", action="store_true",
                    help="시연: 카메라로 버튼을 못 찾으면 멈추지 않고 마지막으로 성공한 버튼 자리로 누른다")
    ap.add_argument("--blind", action="store_true",
                    help="시연: 카메라로 찾지 않고 마지막으로 성공한 버튼 자리로 같은 동작(관찰·중간 확인·누르기·홈)")
    ap.add_argument("--mid-z-up", type=float, default=None,
                    help="중간 확인점에서 다시 찾은 높이에 더할 m (기본 config/press.yaml mid.z_up). 호출 버튼 최종 높이는 이 값")
    ap.add_argument("--push", "--reach", dest="push", type=float, default=None,
                    help="닿은 뒤 지금 자세에서 앞으로만 더 밀 거리 m (기본 config/press.yaml push)")
    ap.add_argument("--watch", action="store_true",
                    help="엘베 안: 누르고 불을 확인하면 접근점에서 그 불이 꺼질 때(= 그 층 도착)까지 지켜본 뒤 홈")
    ap.add_argument("--contact-ma", type=float, default=None,
                    help="누르기 중 접근점보다 이만큼 전류가 오르면 닿음으로 보고 멈춤 mA (기본 config/press.yaml)")
    ap.add_argument("--push-yaw", type=float, default=None,
                    help="찌르는 방향 = 벽 법선(world, 도). 0 = AMR 정면 벽에 수직 (기본 config/press.yaml push_yaw_deg)")
    ap.add_argument("--radial", action="store_true",
                    help="11-16 전 방식: 밑동에서 버튼 쪽(방사 방향)으로 찌름 — 버튼이 옆에 있으면 벽을 비스듬히 민다")
    ap.add_argument("--seek", type=float, default=None,
                    help="누름점까지 안 닿으면 같은 선으로 이만큼 더 밀며 닿음을 찾는다 m (기본 config/press.yaml seek)")
    args = ap.parse_args()

    group, prof, speed = load_profile(args.target)
    for key, val in (("depth", prof["depth"]), ("z_up", prof["z_up"]), ("contact_ma", prof["contact_ma"]),
                     ("retract", prof.get("retract", 0.06)), ("vel", speed.get("approach", 0.1)),
                     ("retract_vel", speed["retract"]), ("home_vel", speed["home"])):
        if getattr(args, key) is None:
            setattr(args, key, float(val))
    args.press_vel = float(speed.get("press", 0.05))
    if args.push is None:
        args.push = float(prof.get("push", 0.0))
    args.push_ma = float(prof.get("push_ma", 300))
    args.need_light = bool(prof.get("need_light", False))
    if args.radial:
        args.push_yaw = None
    elif args.push_yaw is not None:
        args.push_yaw = math.radians(args.push_yaw)
    elif prof.get("push_yaw_deg") is not None:
        args.push_yaw = math.radians(float(prof["push_yaw_deg"]))
    if args.seek is None:
        args.seek = float(prof.get("seek", 0.0))
    if test_suffix():
        print(f"[시험 도메인 {os.environ.get('ROS_DOMAIN_ID') or '0'}] 사진은 {load_photo_dir()}, 성공한 버튼 자리는 "
              f"{saved_path(args.target)} 에 — 실물(도메인 {REAL_DOMAIN}) 기록과 따로")
    print("[설정] 찌르는 방향: " + ("밑동에서 버튼 쪽(방사 방향)" if args.push_yaw is None else
                               f"벽 법선 {math.degrees(args.push_yaw):+.0f}°(world) — 손끝이 벽에 수직으로 곧게 들어감")
          + (f", 안 닿으면 {args.seek * 100:.1f} cm 더 밀며 찾음" if args.seek > 0 else ""))
    print(f"[설정] {group}({'승강장 호출' if group == 'hall' else '엘베 안'}): 높이 보정 +{args.z_up * 100:.1f} cm, "
          f"깊이 {args.depth * 100:+.1f} cm, 닿음 {args.contact_ma:.0f} mA, "
          f"빼기 {'없음(바로 홈)' if args.retract <= 0 else f'{args.retract * 100:.0f} cm'}, "
          f"속도 접근 {args.vel} · 누르기 {args.press_vel} · 빼기 {args.retract_vel} · 홈 {args.home_vel}")

    args.watch_sec = float(prof.get("watch_sec", 300))
    # rclpy 가 SIGINT 로 통신을 먼저 끊지 않게 한다 → Ctrl+C·서비스 비상정지에도 finally 에서 난간 장애물을 뺀다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Presser(args.vel)
    try:
        body(n, args, group, prof, speed, T0)
    except SystemExit as e:
        # 누르기 전에 끝나도(버튼 못 찾음·못 감·가운데 아님) 관찰·접근 사진은 남긴다 — 10-22 실패 4 회가 사진 없이 끝남
        if not n.photos_saved and n.shots and load_photo_dir():
            try:
                d = os.path.join(load_photo_dir(), time.strftime("%Y%m%d_%H%M%S") + "_" + args.target)
                lit = load_lit()
                finish_photos(load_photo_dir(), d, save_photos(d, dict(n.shots), lit), lit, n.lit_trace,
                              {"target": args.target, "result": str(e.code), "z_up": args.z_up,
                               "button": getattr(n, "button_seen", None), "mid": n.mid_log})
                print(f"사진: {d}")
            except OSError:
                pass
        raise


def body(n, args, group, prof, speed, T0):
    """main 의 나머지(관찰 → 버튼 → 누르기). sys.exit 로 끝나면 main 이 사진을 남긴다."""
    if not wait_joints(n):
        sys.exit("/joint_states 가 5초 동안 없습니다. run/arm.sh 가 떠 있는지 보세요")
    if args.no_path_guard and not args.dry:
        n.hand_required = True
        t = time.time()
        while not n.hand_live() and time.time() - t < 15.0:
            n.spin_for(0.2)
        if not n.hand_live():
            sys.exit("--no-path-guard 인데 손 인식(/hand_guard/detected) 답이 15초 동안 없음 → 움직이지 않음"
                     "(카메라가 열렸는지 확인: 다른 astra_camera_node 가 떠 있으면 'Resource busy')")
        print("  손 인식만으로 감시(--no-path-guard): 답이 끊기면 그 자리에서 멈춤")
    if not args.no_scene:
        names, why = check_scene(n)
        if names is None:
            sys.exit(why)
        print("  장애물 확인:", ", ".join(sorted(names)))
    timing = {}
    if args.observe:
        t = time.time()
        yaw = args.yaw if args.yaw is not None else float(prof.get("yaw_deg", 0.0))
        path = load_path(prof.get("path"))
        if path:
            look = path[-1]
            name = f"관찰 자세 {prof['pose']} (손으로 기록한 길 끝)"
        elif prof.get("look_joints"):
            look = [float(v) for v in prof["look_joints"]]
            name = f"관찰 자세 {prof['pose']} (손으로 잡아 둔 관절값)"
        else:
            look, why = look_pose(prof["pose"], float(prof["cam_h"]), yaw)
            if look is None:
                sys.exit(f"관찰 자세를 못 풉니다: {why}")
            name = f"관찰 자세 {prof['pose']} (카메라 {prof['cam_h']} m" + (f", 밑동 {yaw:+.0f}°)" if yaw else ")")
        for k, (kind, step, label) in enumerate(route(n.cur, look, path)):
            label = label or name
            if kind == "path":
                print(f"\n[{label}]")
                n.stage = label
                n.say()
                if args.dry:
                    why = n.path_blocked(step)
                    rclpy.shutdown()
                    print(f"[--dry] {label}: " + (f"충돌 검사 걸림 — {why}" if why else "충돌 검사 통과")
                          + ". 팔은 움직이지 않았습니다")
                    return
                n.settle(**QUICK_SETTLE)
                ok = n.go_path(step, float(prof.get("path_speed", 0.5)))
                if ok == HAND:
                    sys.exit(home_after_hand(n, args, prof, timing, why_moved(label, ok)))
                if ok is not True:
                    sys.exit(ok)
                continue
            traj = plan_step(n, args, label, step, float(speed.get("observe", 0.1)), None if k == 0 else "quick")
            if isinstance(traj, str):
                sys.exit(traj)
            if args.dry:
                n.spin_for(1.0)
                rclpy.shutdown()
                print(f"[--dry] {label} 계획만 보여 주고 끝냅니다. 팔은 움직이지 않았습니다")
                return
            ok = n.execute(traj)
            if ok == HAND:
                sys.exit(home_after_hand(n, args, prof, timing, why_moved(label, ok)))
            if ok is not True:
                sys.exit(why_moved(label, ok))
        n.settle(timeout=2.0, window=0.3)
        timing["관찰"] = time.time() - t
        t = time.time()
        if args.blind:
            print("  [--blind] 카메라로 찾지 않고 저장한 버튼 자리로 누릅니다")
            n.spin_for(1.0)
            n.blind = True
        elif not wait_button(args.target, time.time(), 5.0):
            live = n.camera_live(0.5)
            if args.fallback:
                print(f"  관찰 자세에서 '{args.target}' 를 못 찾음" + ("" if live else "(손목 카메라 화면 멈춤)")
                      + " → [--fallback] 저장한 버튼 자리로 누릅니다")
                n.blind = True
            elif not live:
                sys.exit(f"관찰 자세에서 '{args.target}' 버튼을 못 찾았습니다 — 손목 카메라 화면이 멈춤"
                         "(Astra USB 확인, run/astra.sh 다시 켜기)")
            else:
                sys.exit(f"관찰 자세에서 5초 동안 '{args.target}' 버튼 위치를 못 얻었습니다. "
                         "run/buttons.sh 가 켜져 있고 버튼이 화면에 보이는지 확인하세요")
        timing["찾기"] = time.time() - t
        n.shot("observe")                   # 관찰 자세에서 버튼을 찾은 화면(버튼을 제대로 봤는지 나중에 본다)
        if not n.blind:
            print(f"  버튼 찾음 {timing['찾기']:.1f}초")
            if not n.camera_live(1.0):
                if not args.fallback:
                    sys.exit("손목 카메라 화면이 멈췄습니다 — 누르지 않습니다(Astra USB 확인, run/astra.sh 다시 켜기)")
                print("  손목 카메라 화면이 멈춤 → [--fallback] 저장한 버튼 자리로 누릅니다")
                n.blind = True
    elif args.blind or args.fallback:
        n.blind = args.blind

    if n.blind:
        saved = load_saved(args.target)
        if saved is None:
            sys.exit(f"저장한 '{args.target}' 버튼 자리가 없습니다({saved_path(args.target)}). "
                     "카메라로 한 번 누르기에 성공하면 저절로 저장됩니다")
        button = tuple(saved["button"])           # 성공했던 최종 목표(높이 보정까지 들어 있음)
        n.button_seen = {"saved": saved}
        approach, press = geom.approach_and_press(button, args.standoff, args.depth, args.push_yaw)
        print(f"[{args.target}] 저장한 버튼 자리({saved.get('time')}, {saved.get('result')})로 누릅니다")
    else:
        b = load_button(args.target, args.max_age)
        button = tuple(b["world"])
        n.button_seen = {"world": list(button), "z_m": b.get("z_m"), "z_src": b.get("z_src"), "conf": b.get("conf"),
                         "u": b.get("u"), "v": b.get("v"), "size_px": b.get("size_px")}
        if args.z_up:
            print(f"  높이 보정 +{args.z_up * 100:.1f} cm: 버튼 높이 {button[2]:.3f} → {button[2] + args.z_up:.3f} (world z)")
            button = (button[0], button[1], button[2] + args.z_up)
        approach, press = geom.approach_and_press(button, args.standoff, args.depth, args.push_yaw)
        src = {"button": "깊이", "size": "버튼 크기", "closer": "깊이·크기 중 가까운 쪽"}.get(b.get("z_src"), "?")
        print(f"[{args.target}] 신뢰도 {b['conf']}, 카메라 거리 {b['z_m'] * 100:.1f} cm ({src})")
    if group == "floor" and args.depth >= 0:
        print(f"  ⚠ 엘베 안(오른쪽) 버튼은 높이 보정(z_up {args.z_up})이 [가정]입니다. "
              "실물로 맞추기 전이면 먼저 --depth -0.02 로 틈과 높이부터 확인하세요")
    print(f"  버튼   world = ({button[0]:.3f}, {button[1]:.3f}, {button[2]:.3f})")
    print(f"  접근점 world = ({approach[0]:.3f}, {approach[1]:.3f}, {approach[2]:.3f})  (앞 {args.standoff * 100:.0f} cm)")
    print(f"  누름점 world = ({press[0]:.3f}, {press[1]:.3f}, {press[2]:.3f})  (면 + {args.depth * 100:.1f} cm)")
    j_app, why = geom.press_ik(*approach)
    if j_app is None:
        sys.exit(f"접근점에 못 갑니다: {why}")
    j_prs, why = geom.press_ik(*press)
    if j_prs is None:
        sys.exit(f"누름점에 못 갑니다: {why}")
    print("  접근 관절 =", tuple(round(v, 3) for v in j_app))
    print("  누름 관절 =", tuple(round(v, 3) for v in j_prs))
    print("  지금 관절 =", tuple(round(v, 3) for v in n.cur))
    n.set_markers(button, approach, press)
    start_guard(n, args, prof, button)

    rail = prof.get("handrail") if group == "floor" else None
    if rail:
        if not n.set_handrail(button, rail):
            sys.exit("난간 장애물을 MoveIt 에 못 넣었습니다. run/moveit.sh 가 떠 있는지 보세요")
        print(f"  난간 장애물: 버튼 면에서 로봇 쪽 {rail['protrusion'] * 100:.0f} cm, "
              f"바닥 {rail['z_bottom']}~{rail['z_top']} m (누르기가 끝나면 뺀다)")
    print("  RViz: Add → By topic → /button_target (MarkerArray) 로 빨간 공 = 버튼, 노란 공 = 접근점")
    try:
        failed = run_steps(n, args, j_app, j_prs, press, timing, T0, button, prof)
    except KeyboardInterrupt:
        failed = "중단(신호) — 팔은 그 자리"
    finally:
        if rail:
            n.scene_box(HANDRAIL_ID)
        rclpy.shutdown()
    if failed:
        sys.exit(f"여기서 멈춥니다: {failed}")      # arm_service.py 가 종료 코드와 이 마지막 줄로 실패를 안다


def start_guard(n, args, prof, button):
    """버튼 자리를 알았으니 길목 신호등을 켠다(버튼 쪽으로 가는 동안만 — 빼기·홈 앞에서 move 가 끈다)."""
    g = n.gcfg
    if getattr(args, "no_path_guard", False):
        print("  손 감시(길목 신호등): 꺼짐(--no-path-guard) — 손 인식(hand_guard)으로만 멈춥니다")
        return
    if not g["enabled"] or not prof.get("guard", True):
        print("  손 감시(길목 신호등): 꺼짐(config/press.yaml guard)")
        return
    n.guard_margin = float(prof.get("guard_margin", g["margin"]))
    n.guard_wall, n.guard_on = button, True
    if not args.dry:
        n.spin_for(0.3)
    if time.time() - n.depth_t > 1.0:
        if g["required"]:
            sys.exit("손목 깊이 화면(/camera/depth/image_raw)이 없어 손 감시를 못 합니다 — 누르지 않음"
                     "(run/astra.sh 확인. 없이 하려면 config/press.yaml guard.required: false)")
        print("  ⚠ 손 감시(길목 신호등): 손목 깊이 화면이 없음 → 손이 와도 못 멈춥니다(run/astra.sh 를 켜 주세요)")
        return
    far = n.guard_far()
    print(f"  손 감시(길목 신호등): 켬 — 버튼 쪽으로 가는 동안 카메라 → 벽이 {n.guard_min:.2f} m 보다 멀면 "
          f"칸(벽 − {n.guard_margin * 100:.0f} cm) 안에 손이 오면 빨간불·멈춤"
          + (f". 지금 칸 끝 {far:.2f} m" + (" — 빨간불" if n.guard.red else "") if far is not None else
             ". 지금 자세는 안 봄(벽을 안 보거나 너무 가까움)"))


def move_to(n, args, prof, timing, label, target, vel, settle, key):
    """빼기·홈. 출발점이 어긋나 거부되면 자리 잡고 한 번 더. 밑동을 크게 돌면 세우고 돈다. 실패 이유(없으면 None).
    벽에서 물러나는 길이라 길목 신호등은 끈다(손 앞에서 물러나는 것까지 막지 않게).

    빨간불이 wait_sec 넘게 안 꺼져 그만둔 뒤(n.hand_stuck)에는 손 인식도 끄고 간다 — 먼 손을 잘못 본 빨간불이면
    영영 못 돌아온다(11-02). 손 인식으로 이 길에서 wait_sec 을 넘겨도 그 자리에서 손 감시를 끄고 한 번 더 간다."""
    n.guard_on = False
    if n.hand_stuck and not n.hand_ignore:
        n.hand_ignore = True
        print("  손 감시 끔(빨간불이 안 꺼져 그만둠) — 전류(충돌) 감시만 켠 채 돌아갑니다")
    t = time.time()
    for k, (kind, step, name) in enumerate(route(n.cur, target, load_path(prof.get("path")))):
        name = name or label
        if kind == "path":
            print(f"\n[{name}]")
            n.stage = name
            n.say()
            n.settle(**QUICK_SETTLE)
            ok = n.go_path(step, float(prof.get("path_speed", 0.5)))
            if ok == HAND and not n.hand_ignore:
                timing[key] = time.time() - t
                return move_to(n, args, prof, timing, label, target, vel, "quick", key)
            if ok is not True:
                timing[key] = time.time() - t
                return ok
            continue
        traj = plan_step(n, args, name, step, vel, settle if k == 0 else "quick")
        if isinstance(traj, str):
            return traj
        ok = n.execute(traj)
        if ok is False:
            print("  실행 실패 — 자리 잡고 한 번 더 계획")
            n.settle(timeout=1.0)
            traj = n.plan_any(step, vel)
            ok = traj is not None and n.execute(traj)
        if ok == "충돌":
            timing[key] = time.time() - t
            return f"{name} 중 전류가 {CRASH_MA:.0f} mA 를 넘어 멈춤(충돌 의심)"
        if ok == HAND and not n.hand_ignore:
            timing[key] = time.time() - t
            return move_to(n, args, prof, timing, label, target, vel, "quick", key)
        if ok == HAND:
            timing[key] = time.time() - t
            return why_moved(name, ok)
        if not ok:
            timing[key] = time.time() - t
            return f"{name} 실행 실패"
    timing[key] = time.time() - t
    print("  완료. 지금 관절 =", tuple(round(v, 3) for v in n.cur))
    return None


def home_after_hand(n, args, prof, timing, why):
    """빨간불이 wait_sec 넘게 안 꺼져 그만뒀다 → 손 감시를 끄고 홈으로. 실패 문장(홈까지 갔으면 why 그대로)."""
    print(f"\n  {why}")
    failed = move_to(n, args, prof, timing, "home 으로", HOME, args.home_vel, "quick", "홈")
    return f"{why} — 홈 복귀도 실패: {failed}" if failed else why


def run_steps(n, args, j_app, j_prs, press, timing, T0, button, prof):
    """지금 자세 → 접근점 → 누름점 → (곧게 빼기) → 홈. 실패 이유(없으면 None)를 돌려준다.

    누름점(버튼 면 + depth)까지 밀다가 불이 켜지면 그 자리에서 멈춘다. 전류(contact_ma)는 닿았는지 기록만 하고
    멈추지 않는다 — 10-21 실물에서 버튼을 누른 것은 누름점까지 민 힘이었다(그때는 '실행 취소' 가 안 먹어서 늘 끝까지 감).
    누름점에 닿은 채 불이 아직이면 lit.confirm_sec 까지 기다린다. 불을 보면 바로 홈(retract > 0 이면 그만큼 곧게 뺀 뒤).
    모터 전류 상한(전류 제한 위치 제어)이 미는 힘을 막는다. 닿았는데 불이 끝내 안 켜졌고 need_light 이면 '안 눌림' 실패.
    """
    lit = load_lit()
    photo_dir = load_photo_dir()
    if args.dry:
        traj = plan_step(n, args, "접근점으로 (--dry: 계획만)", j_app, args.vel, "full")
        u, _ = push_frame(button, args.push_yaw)
        end = tuple(p + (args.seek if args.depth >= 0 else 0.0) * c for p, c in zip(press, u))
        app_xyz, pitch = geom.fk(j_app)
        pts, why = geom.line_joints(app_xyz, end, pitch)
        print(f"  누르기 곧은 선 {math.dist(app_xyz, end) * 100:.1f} cm: "
              + (f"풀림({len(pts)}점, 막대가 찌르는 방향과 "
                 f"{math.degrees(math.remainder(pts[-1][0] - math.atan2(u[1], u[0]), 2 * math.pi)):+.1f}° 기욺)"
                 if pts else f"못 풂 — {why}"))
        n.spin_for(3.0)
        print("팔은 움직이지 않았습니다.")
        return None if not isinstance(traj, str) else traj

    def move(label, target, vel, settle, key):
        return move_to(n, args, prof, timing, label, target, vel, settle, key)

    # 중간 확인점(호출 버튼: 버튼 앞 20 cm 에서 다시 찾아 목표를 고친 뒤 접근점·누름점을 새로 푼다)
    if prof.get("mid") and not args.no_mid:
        t = time.time()
        mid = load_mid()
        if args.mid_z_up is not None:
            mid["z_up"] = args.mid_z_up
        print(f"  중간 확인점 높이 보정 {float(mid.get('z_up', 0.0)) * 100:+.1f} cm")
        button, n.mid_log, mid_fail = mid_check(n, args, mid, button)
        timing["중간"] = time.time() - t
        n.guard_wall = button
        if mid_fail is None:
            approach, press = geom.approach_and_press(button, args.standoff, args.depth, args.push_yaw)
            j_app, why = geom.press_ik(*approach)
            j_prs, why2 = geom.press_ik(*press)
            if j_app is None or j_prs is None:
                mid_fail = f"고친 자리에 못 감({why or why2}) — 누르지 않음"
            else:
                n.set_markers(button, approach, press)
                print(f"  고친 버튼 world = ({button[0]:.3f}, {button[1]:.3f}, {button[2]:.3f}) → "
                      f"접근점 ({approach[0]:.3f}, {approach[1]:.3f}, {approach[2]:.3f})")
        if mid_fail:
            return move("home 으로", HOME, args.home_vel, "quick", "홈") or mid_fail

    # 손 명령으로 좌우 옮기기(--right, + = 로봇 오른쪽): 찌르는 방향에 직각으로(벽 법선이면 벽을 따라)
    if args.right:
        _, (rx, ry, _) = push_frame(button, args.push_yaw)
        button = (button[0] + args.right * rx, button[1] + args.right * ry, button[2])
        n.guard_wall = button
        approach, press = geom.approach_and_press(button, args.standoff, args.depth, args.push_yaw)
        j_app, why = geom.press_ik(*approach)
        j_prs, why2 = geom.press_ik(*press)
        if j_app is None or j_prs is None:
            return move("home 으로", HOME, args.home_vel, "quick", "홈") or f"오른쪽으로 옮긴 자리에 못 감({why or why2})"
        n.set_markers(button, approach, press)
        print(f"  누르는 자리를 {'오른쪽' if args.right > 0 else '왼쪽'}으로 {abs(args.right) * 100:.1f} cm 옮김 → "
              f"버튼 world ({button[0]:.3f}, {button[1]:.3f}, {button[2]:.3f})")

    # 접근점으로
    t = time.time()
    traj = plan_step(n, args, "접근점으로", j_app, args.vel, "full")
    if isinstance(traj, str):
        return traj
    ok = n.execute(traj)
    if ok == HAND:
        return home_after_hand(n, args, prof, timing, why_moved("접근점으로", ok))
    if ok is not True:
        return why_moved("접근점으로", ok)
    timing["접근"] = time.time() - t

    # 가운데 맞추기(호출 버튼: 접근점 화면의 화살표로 높이·좌우를 고쳐 접근점을 다시 잡는다)
    aim_log = []
    if prof.get("aim") and not args.no_aim:
        n.settle()
        button, j_app, j_prs, press, aim_log, aim_fail = aim_at_center(n, args, load_aim(), button, timing)
        if aim_fail:
            n.shot("aim_fail")
            return move("home 으로", HOME, args.home_vel, "quick", "홈") or aim_fail

    # 누르기. 자리 잡기 → 전류·불빛 기준 → 계획 → 바로 실행. 11-02 까지는 계획한 뒤 기준을 재느라 0.6~1.3 s 쉬었고,
    # 그사이 뻗은 팔이 처져 MoveIt 이 '출발점이 0.01 넘게 다름'으로 거부했다(10-18 3 회, 11-02 2 회 → 누르기 실행 실패·홈)
    print(f"  팔이 자리 잡기를 기다림 {n.settle():.1f}초")
    base = n.amp_baseline()
    print("  접근점 전류 기준: " + ", ".join(f"{j} {a:.0f}" for j, a in zip(JOINTS, base)) + " mA")
    n.shot("base")
    light, red0 = None, None
    if lit.get("enabled") and lit.get("region"):
        red0 = n.red_baseline(lit)
        if red0 is None and n.img is not None and (args.fallback or args.blind):
            print("  불빛: 손목 카메라 화면이 멈춤 → [시연] 전류로만 판정하고 누름")
        elif red0 is None and n.img is not None:
            # 화면이 오다가 멈췄다 — 버튼 자리도 옛 화면 것일 수 있다. 누르지 않고 홈
            print("  불빛: 손목 카메라 화면이 멈춤(1초 동안 새 화면 3장이 안 옴) → 누르지 않고 홈")
            return move("home 으로", HOME, args.home_vel, "quick", "홈") or \
                "손목 카메라 화면이 멈춰 누르지 않음(Astra USB 확인, run/astra.sh 다시 켜기)"
        if red0 is None:
            print("  불빛: 카메라 화면이 없음 → 전류로만 판정")
        elif red0 >= lit["min_px"]:
            print(f"  불빛: 접근점에서 이미 막대 끝 칸이 빨강 {red0}점 — 불빛으로 판정 못 함 → 전류로만")
        else:
            light = (red0, lit)
            print(f"  불빛: 막대 끝 칸 빨강 기준 {red0}점 — +{lit['min_px']} 넘게 {lit['frames']}장이면 불 켜짐")
    # 누르기 = 접근점 → 누름점(면 + depth) → (안 닿으면 seek 만큼 더) 한 줄의 곧은 선. 11-16: MoveIt PTP 대신.
    # 찌르는 방향(push_yaw)이 벽 법선이면 버튼이 옆에 있어도 손끝이 벽에 수직으로 들어간다 — 10-22 실물: 버튼이
    # 팔 축에서 10~12 cm 옆(밑동 10.7~11.8°)일 때 방사 방향으로 찔러 9 번 중 1 번만 눌림(정면은 7 번 중 3 번)
    u, _ = push_frame(button, args.push_yaw)
    seek = args.seek if args.depth >= 0 else 0.0           # 연습(--depth 음수)은 더 밀지 않는다
    end = tuple(p + seek * c for p, c in zip(press, u))
    q_app = list(n.cur)

    def replan_to(xyz):
        return lambda cur: line_traj(cur, xyz, args.press_vel)[0]

    print("\n[버튼 누르기]")
    n.stage = "버튼 누르기"
    n.say()
    traj, sec, pts = line_traj(n.cur, end, args.press_vel)
    if traj is None:
        return f"버튼 누르기 선을 못 풂({sec})"
    why = n.path_blocked(pts[::5] + [pts[-1]])
    if why:
        return f"버튼 누르기 선이 MoveIt 장애물에 걸림({why})"
    incid = math.degrees(math.remainder(pts[-1][0] - math.atan2(u[1], u[0]), 2 * math.pi))   # 막대 방향 = joint1
    line_cm = math.dist(geom.fk(n.cur)[0], end) * 100
    print(f"  곧은 선 {line_cm:.1f} cm(누름점 면 {args.depth * 100:+.1f} cm" + (f" + 찾기 {seek * 100:.1f} cm" if seek else "")
          + f"), 약 {sec:.1f}초 — 막대가 찌르는 방향과 {incid:+.1f}° 기욺")
    n.disp_pub.publish(DisplayTrajectory(trajectory=[traj]))
    if not args.yes and input("  RViz 에서 경로를 확인하고 Enter = 실행 / q = 중단 : ").strip().lower() == "q":
        return "버튼 누르기 앞에서 사용자가 중단"
    shots_at = []                                          # 막대 끝이 계산한 면 1 cm 앞을 지나는 시각에 한 장
    for p in traj.joint_trajectory.points:
        if tip_vs_button(p.positions, button, args.push_yaw)[0] >= -0.01:
            shots_at.append((p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, "pre1cm"))
            break
    # 1 단계: 선을 따라 밀다 닿으면(전류) 그 자리에서 멈춘다. 멈추면 모터 목표가 지금 실제 자세가 되어
    # '처지는 만큼 높게' 잡은 목표로 위로 끌려 올라가지 않는다. 누름점까지 안 닿으면 seek 만큼 더 가며 찾는다
    t = time.time()
    ok = n.execute(traj, replan=replan_to(end), base=base, stop_ma=args.contact_ma, n_over=CONTACT_N, why="닿음",
                   light=light, shots_at=shots_at)
    if ok is False:
        print("  실행 거부 — 자리 잡고 한 번 더")
        n.settle(timeout=1.0)
        traj = line_traj(n.cur, end, args.press_vel)[0]
        if traj is not None:
            t = time.time()
            ok = n.execute(traj, replan=replan_to(end), base=base, stop_ma=args.contact_ma, n_over=CONTACT_N,
                           why="닿음", light=light)
    timing["누르기"] = (n.trip_t or time.time()) - t
    if ok not in (True, "불 켜짐", "닿음"):
        return home_after_hand(n, args, prof, timing, why_moved("버튼 누르기", ok)) if ok == HAND \
            else "버튼 누르기 실행 실패"
    q_stop = list(n.cur)
    stop_at = tip_vs_button(q_stop, button, args.push_yaw)
    if ok in ("닿음", "불 켜짐"):
        print(f"  {ok} 자리: 막대 끝이 계산한 버튼 면보다 {stop_at[0] * 100:+.1f} cm "
              "(− = 면보다 앞에서 닿음 = 버튼이 카메라로 잰 것보다 가까움)")
    touched, lit_seen, post_fail = ok == "닿음", ok == "불 켜짐", None
    if touched and args.push > 0:
        # 2 단계: 지금 실제 자리에서 높이·막대 기울기는 그대로, 찌르는 방향으로만 push 만큼 더 민다
        t = time.time()
        (tx, ty, tz), _ = geom.fk(n.cur)
        p_end = (tx + args.push * u[0], ty + args.push * u[1], tz)
        print(f"  [앞으로 {args.push * 1000:.0f} mm 더 누르기] 찌르는 방향으로 곧게, 높이는 그대로")
        traj2, why2, _ = line_traj(n.cur, p_end, args.press_vel)
        if traj2 is not None:
            ok2 = n.execute(traj2, replan=replan_to(p_end), base=base, stop_ma=args.contact_ma + args.push_ma,
                            n_over=CONTACT_N, why="너무 셈", light=light)
            lit_seen = lit_seen or ok2 == "불 켜짐"
        else:
            print(f"  더 누르기 선을 못 풂({why2}) — 건너뜀")
        timing["더 누르기"] = time.time() - t
    end_at = tip_vs_button(n.cur, button, args.push_yaw)
    where = (f"막대 끝이 계산한 버튼 면보다 {end_at[0] * 100:+.1f} cm "
             f"(누름점 = 면 {args.depth * 100:+.1f} cm" + (f", 찾기 끝 = 면 {(args.depth + seek) * 100:+.1f} cm)" if seek else ")"))
    print(f"  {'불이 켜져 멈춘' if lit_seen else '누르기를 마친'} 자리: {where}")
    cm = lambda v: [round(x * 100, 2) for x in v]           # noqa: E731
    rnd = lambda q: [round(float(x), 4) for x in q]          # noqa: E731
    push_log = {"mode": "radial" if args.push_yaw is None else "wall",
                "yaw_deg": None if args.push_yaw is None else round(math.degrees(args.push_yaw), 1),
                "rod_vs_push_deg": round(incid, 1), "seek_cm": round(seek * 100, 1),
                "button": rnd(button), "press": rnd(press), "end": rnd(end),
                "j_app_cmd": rnd(j_app), "j_app": rnd(q_app), "j_stop": rnd(q_stop), "j_end": rnd(n.cur),
                "stop": str(ok),
                # 손끝(관절 각도로 계산) − 계산한 버튼 면, cm [앞으로, 오른쪽, 위]. 처짐은 관절 각도에 안 잡혀 여기 안 보인다
                "stop_vs_button_cm": cm(stop_at), "end_vs_button_cm": cm(end_at)}
    if not lit_seen and args.depth >= 0:
        # 누름점(면 + depth)에 누른 채 서 있으면 모터가 계속 민다. 10-21 실물: 버튼은 그 사이 1~2 s 안에 눌렸다.
        # 닿음 전류가 안 올랐어도 기다린다(10-21 저녁: 안 기다리고 바로 홈 → 못 누르고 돌아옴)
        t, sec = time.time(), lit.get("confirm_sec", 2.0)
        print(f"  누른 채 {'불을 기다림' if light else '기다림'}(최대 {sec:.1f}초)")
        lit_seen = n.wait_light(light, sec, base, args.contact_ma)
        touched = touched or n.touched
        n.shot("lit" if lit_seen else "nolight")
        timing["불 확인"] = time.time() - t
    if light and (touched or lit_seen):
        print("  불 켜짐 확인" if lit_seen else "  불 안 켜짐")
        if not lit_seen and args.need_light:
            post_fail = "닿았지만 불이 안 켜짐 — 안 눌렸을 수 있음"
    if not touched and not lit_seen:
        n.shot("end")

    # (지켜보기 또는 곧게 빼기) → 홈. 이 사이에 사진을 쓴다
    failed, watched = None, None
    settle = None                           # 방금까지 닿은 채 서 있었다 → 자리 잡기 없이 바로 계획
    if args.watch and not lit_seen:
        watched = "불을 못 봐서 내 층을 지켜볼 수 없음"
        print(f"  {watched} → 홈")
    elif args.watch:
        failed = move("지켜보기 자리(접근점)로 빼기", j_app, args.retract_vel, None, "빼기")
        settle = "quick"
        if failed is None:
            watched = watch_floor(n, lit, args.watch_sec, timing)
    elif args.retract > 0:
        back = retract_target(n.cur, args.retract)
        if back is None:
            print(f"  {args.retract * 100:.0f} cm 빼기는 손끝이 닿지 않는 자리라 건너뜀")
        else:
            failed = move(f"뒤로 {args.retract * 100:.0f} cm 빼기", back, args.retract_vel, None, "빼기")
            settle = "quick"
    # 사진마다 버튼 빨간 화살표 자리(바로 세운 화면 px). 접근점 → 1 cm 앞 → 닿음 사이에 얼마나 움직였는지 = 누르러 가며
    # 생긴 높이·좌우 어긋남(카메라와 막대가 한 몸). 색으로 찾는 것이라 틀릴 수 있다 — sheet.jpg 로 확인
    aim_full = dict(load_aim())
    push_log["arrow_uv"] = {}
    for k in ("base", "pre1cm", "touch", "stop_lit"):
        if k in n.shots:
            img = n.shots[k][0]
            mk = find_marker(img, dict(aim_full, window=[0, 0, img.shape[1], img.shape[0]]))
            push_log["arrow_uv"][k] = None if mk is None else [round(mk[0], 1), round(mk[1], 1)]
    saver, saved = None, {}
    d = os.path.join(photo_dir, time.strftime("%Y%m%d_%H%M%S") + "_" + args.target) if photo_dir else None
    if d and n.shots:
        shots = dict(n.shots)
        saver = threading.Thread(target=lambda: saved.update(meta=save_photos(d, shots, lit)), daemon=True)
        saver.start()
    failed = move("home 으로", HOME, args.home_vel, settle, "홈") or failed

    if failed is None and post_fail:
        failed = post_fail
    if failed is None and args.depth >= 0 and not touched and not lit_seen:
        # 누름점(면 + depth)까지 갔는데 전류가 안 올랐다 = 버튼에 못 닿았거나 덜 눌렸을 수 있다(홈은 다 함)
        failed = "닿음 감지 없음 — 누름점까지 갔지만 전류가 안 올라 버튼이 안 눌렸을 수 있음"
    how = None
    if failed is None and watched is True:
        how = "내 층 도착(불 꺼짐)"
    elif failed is None and watched and lit_seen:
        failed = watched                    # 눌렀지만 도착을 못 봄(그만·시간 넘김·불빛 안 보임)
    elif failed is None:
        how = "불 켜짐 확인 → 누름 완료" if lit_seen else "전류로 닿음 → 누름 완료(불 확인 못 함)" if touched \
            else "앞에서 멈춤(연습, --depth 음수)"
    timing["전체"] = time.time() - T0
    print("시간: " + " · ".join(f"{k} {v:.1f}" for k, v in timing.items()) + " 초")
    if saver:
        # 결과 줄보다 먼저 찍는다 — arm_service 는 마지막 줄을 답으로 쓴다
        saver.join(timeout=5.0)
        n.photos_saved = True
        try:
            finish_photos(photo_dir, d, saved.get("meta", {}), lit, n.lit_trace,
                          {"target": args.target, "result": how or failed, "red_base": red0,
                           "depth": args.depth, "contact_ma": args.contact_ma, "z_up": args.z_up,
                           "retract": args.retract, "watch": args.watch, "mid": n.mid_log, "aim": aim_log,
                           "push": push_log, "timing": {k: round(v, 2) for k, v in timing.items()}})
            print(f"사진: {d}")
        except OSError as e:
            print(f"사진을 못 남김: {e}")
    if how and n.blind:
        how += "(저장한 버튼 자리로)"
    if how:
        print(f"결과: {how} → 홈")
    if how and not n.blind and ("불 켜짐" in how or "전류로 닿음" in how):
        try:                                     # 시연 --fallback·--blind 가 쓸 '성공한 버튼 자리'
            with open(saved_path(args.target), "w") as f:
                json.dump({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "target": args.target, "result": how,
                           "button": [round(v, 4) for v in button]}, f, ensure_ascii=False)
            print(f"  이 버튼 자리를 저장: {saved_path(args.target)}")
        except OSError as e:
            print(f"  버튼 자리를 저장 못 함: {e}")
    n.stage = f"결과: {how or failed}"
    n.say(done=True)
    return failed


if __name__ == "__main__":
    main()
