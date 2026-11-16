#!/usr/bin/env python3
"""버튼 누르기에 쓰는 순수 계산 함수 모음 (ROS 없음, math 만 씀).

호스트(detect.py), 컨테이너(press.py), 나중의 젯슨 노드가 같은 파일을 그대로 가져다 쓴다.
카메라가 웹캠에서 D455/Astra 로 바뀌어도 이 파일은 그대로다. 바뀌는 건 깊이 Z 를 얻는 방법뿐.

좌표계 (모두 m, rad)
  world  : 팔 기둥 바닥 중심. x = 팔이 뻗는 앞, y = 왼쪽, z = 위   (URDF 의 world 와 같음)
  광학   : 카메라 렌즈. x = 화면 오른쪽, y = 화면 아래, z = 카메라가 보는 앞
"""
import math

# 팔 치수 — URDF·TF 실측값 (2026-10-05)
J1_X = 0.012                   # world 원점에서 joint1(밑동 회전축)까지 앞쪽 거리
SHOULDER_Z = 0.1653 + 0.0595    # joint2(어깨축) 높이 = 기둥 + joint2 오프셋
UA = (0.024, 0.35419)          # 상완: 어깨축 → 팔꿈치축 (팔이 ㄴ자일 때 앞, 위)
FA = 0.375                     # 전완: 팔꿈치축 → 손목축
EE = 0.156                     # 손목축 → TCP (FR12_H104 실측 2026-10-15, 옛 그리퍼 0.126)

LIMITS = {                     # URDF 관절 한계 (rad)
    "joint1": (-math.pi, math.pi),
    "joint2": (-2.15, 1.5),
    "joint3": (-1.5, 1.57),
    "joint4": (-1.7, 1.97),
}

_L1 = math.hypot(*UA)
_A1 = math.atan2(UA[1], UA[0])   # 상완이 수평에서 들린 각 (ㄴ자일 때 약 86도)


def depth_from_size(width_px, fx, diameter_m):
    """버튼이 화면에서 width_px 픽셀로 보이면 카메라에서 얼마나 먼가 (m).

    멀수록 작게 보인다는 원리 하나만 쓴다:  Z = fx × 실제지름 / 픽셀지름
    """
    if width_px <= 0:
        return None
    return fx * diameter_m / width_px


def fx_from_calibration(width_px, distance_m, diameter_m):
    """줄자로 잰 거리에서 버튼이 width_px 로 보였다 → 초점거리 fx (픽셀)."""
    return width_px * distance_m / diameter_m


def pixel_to_world(u, v, z, cam):
    """화면 픽셀 (u, v) 와 깊이 z 를 world 좌표로.

    cam: fx, fy, cx, cy, x, y, z, pitch_deg(아래로 숙이면 +), yaw_deg(왼쪽으로 돌리면 +)
    """
    # 1) 광학 좌표
    xo = (u - cam["cx"]) * z / cam["fx"]
    yo = (v - cam["cy"]) * z / cam["fy"]
    # 2) 카메라 몸체 좌표 (앞, 왼, 위)
    f, l, up = z, -xo, -yo
    # 3) 숙임(pitch): 앞을 아래로
    p = math.radians(cam.get("pitch_deg", 0.0))
    f, up = f * math.cos(p) + up * math.sin(p), -f * math.sin(p) + up * math.cos(p)
    # 4) 돌림(yaw): 앞을 왼쪽으로
    w = math.radians(cam.get("yaw_deg", 0.0))
    f, l = f * math.cos(w) - l * math.sin(w), f * math.sin(w) + l * math.cos(w)
    return (cam["x"] + f, cam["y"] + l, cam["z"] + up)


def fk(j):
    """관절값 (j1..j4) → end_effector_link 위치 (world) 와 손끝 숙임각."""
    j1, j2, j3, j4 = j
    t1 = _A1 - j2                      # 상완 방향 (수평 기준, 위로 +)
    t2 = -(j2 + j3)                    # 전완 방향
    t3 = -(j2 + j3 + j4)               # 손끝 방향
    h = _L1 * math.cos(t1) + FA * math.cos(t2) + EE * math.cos(t3)
    v = _L1 * math.sin(t1) + FA * math.sin(t2) + EE * math.sin(t3)
    x = J1_X + h * math.cos(j1)
    y = h * math.sin(j1)
    return (x, y, SHOULDER_Z + v), j2 + j3 + j4


def press_ik(x, y, z, pitch=0.0):
    """손끝(end_effector_link)을 world 점 (x, y, z) 에 두는 관절값.

    pitch = 손끝이 수평에서 아래로 숙인 각. 0 이면 앞을 똑바로 찌른다.
    팔꿈치가 위로 가는 해를 고른다. 못 가면 (None, 이유).
    """
    j1 = math.atan2(y, x - J1_X)
    r = math.hypot(x - J1_X, y)
    # 손목축 위치 = 손끝에서 그리퍼 길이만큼 뒤로
    wh = r - EE * math.cos(pitch)
    wv = (z - SHOULDER_Z) + EE * math.sin(pitch)
    d2 = wh * wh + wv * wv
    c = (d2 - _L1 * _L1 - FA * FA) / (2 * _L1 * FA)
    if c > 1.0:
        return None, f"너무 멀다 (어깨~손목 {math.sqrt(d2):.3f} m > 최대 {_L1 + FA:.3f} m)"
    if c < -1.0:
        return None, "너무 가깝다"
    delta = math.acos(c)               # 상완과 전완 사이 꺾임 (+ = 팔꿈치 위)
    t1 = math.atan2(wv, wh) + math.atan2(FA * math.sin(delta), _L1 + FA * math.cos(delta))
    t2 = t1 - delta
    j2 = _A1 - t1
    j3 = -t2 - j2
    j4 = pitch - j2 - j3
    j = (j1, j2, j3, j4)
    for name, val in zip(LIMITS, j):
        lo, hi = LIMITS[name]
        if not lo <= val <= hi:
            return None, f"{name} = {val:+.3f} rad 가 한계 [{lo}, {hi}] 밖"
    return j, ""


def push_dir(button_xyz, yaw=None):
    """찌르는 방향(world 수평 단위 벡터 ux, uy). yaw(rad) = 벽 법선 방향(벽 쪽으로). None = 밑동에서 버튼 쪽(방사 방향)."""
    x, y, _ = button_xyz
    a = math.atan2(y, x - J1_X) if yaw is None else yaw
    return math.cos(a), math.sin(a)


def approach_and_press(button_xyz, standoff=0.06, depth=0.005, yaw=None):
    """버튼 앞 standoff 지점과, 표면보다 depth 만큼 들어간 지점. 둘 다 찌르는 방향(push_dir) 위에 있다.

    4자유도 팔은 막대 방향(밑동 → 손끝, 수평)을 고를 수 없지만 손끝 자리는 3자유도라 어느 방향으로든 곧게 옮길 수 있다.
    yaw = None 이면 밑동에서 바깥쪽(방사 방향, 11-16 전 방식) — 버튼이 옆에 있으면 벽을 비스듬히 민다.
    yaw = 벽 법선이면 손끝이 벽에 수직으로 들어간다(막대만 밑동 쪽으로 기운 채).
    """
    x, y, z = button_xyz
    ux, uy = push_dir(button_xyz, yaw)
    approach = (x - standoff * ux, y - standoff * uy, z)
    press = (x + depth * ux, y + depth * uy, z)
    return approach, press


def line_joints(start_xyz, end_xyz, pitch=0.0, step=0.002):
    """start → end 곧은 선을 step(m) 간격으로 나눠 IK 로 푼 관절값 목록(start 는 빼고 end 는 넣는다).
    못 푸는 점이 있으면 (None, 이유)."""
    d = math.dist(start_xyz, end_xyz)
    n = max(1, int(math.ceil(d / step)))
    out = []
    for k in range(1, n + 1):
        s = k / n
        p = tuple(a + (b - a) * s for a, b in zip(start_xyz, end_xyz))
        j, why = press_ik(*p, pitch)
        if j is None:
            return None, f"선 위 {s * d * 100:.1f} cm 지점: {why}"
        out.append(list(j))
    return out, ""


if __name__ == "__main__":
    # 검산: IK 로 구한 관절을 FK 로 되돌려 같은 점이 나오는지
    for p in [(0.45, 0.0, 0.40), (0.50, 0.10, 0.30), (0.35, -0.05, 0.55), (0.60, 0.0, 0.25)]:
        j, why = press_ik(*p)
        if j is None:
            print(p, "→ 불가:", why)
            continue
        q, pitch = fk(j)
        err = math.dist(p, q) * 1000
        print(p, "→", tuple(round(v, 3) for v in j), f"| FK 오차 {err:.2f} mm, 숙임 {math.degrees(pitch):+.1f}°")
