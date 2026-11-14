#!/usr/bin/env python3
"""정해 둔 자세로 팔을 보낸다. MoveIt 이 장애물(차체·짐칸·마스트)을 피하는 경로를 만든다.

  bash run/pose.sh front --dry     계획만 (팔 안 움직임)
  bash run/pose.sh front           관찰 자세(정면). Enter 를 눌러야 움직인다
  bash run/pose.sh front --cam 0.85   카메라를 바닥에서 0.85 m 에 둔다 (버튼 높이에 맞춘다)
  bash run/pose.sh right           관찰 자세(엘베 안, 로봇 오른쪽). config/press.yaml floor.look_joints(손으로 잡아 둔 값)가 있으면 그 자세
  bash run/pose.sh home
  bash run/pose.sh home --retract  홈 전에 손끝을 같은 높이로 6 cm 뒤로 뺀다 (버튼 앞에서 멈췄을 때)
  (run/arm.sh 와 run/moveit.sh 가 먼저 떠 있어야 한다)

관찰 자세는 막대(=카메라)가 수평이라 카메라가 정면을 본다. 카메라 높이는 config/press.yaml(hall·floor cam_h).
카메라를 joint1 축 바로 위에 두고 손끝(카메라 앞 0.109, 아래 0.033)을 푼다(press.look_pose).
joint1 축은 앞 범퍼에서 약 0.113 m 뒤다(10-18 기둥 위치 정정). NX 서비스는 press.py --observe 로 관찰부터 한다.
홈 ↔ 오른쪽은 손으로 기록한 길(config/paths, floor.path)을 그대로 따라간다. 길이 없으면 밑동을 20° 넘게 돌 때
세우기 → 세운 채 돌기 → 내리기(press.route, 짐칸·박스를 안 치게).
"""
import argparse
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from press import HOME, JOINTS, Presser, QUICK_SETTLE, check_scene, load_path, look_pose, retract_target, route  # noqa: E402

# 관찰 자세 카메라 바닥 높이 기본값. 정면 = 호출 버튼(10-20 실물 1.05 m, 그 높이에서 버튼이 화면 위아래 가운데),
# 오른쪽 = 엘베 안 층 버튼(0.84~0.92 m, 3층 0.91 m 사용자 실측). 0.88 m 은 어깨 −95° 라 위팔이 짐칸에 4.8 mm
# 겹치고(10-21 가짜 팔 MoveIt), 0.95 m 은 어깨 −84.7° 로 여유가 있다. 3층은 화면 정면에서 약 5° 아래
CAM_H = {'front': 1.05, 'right': 0.95}
try:                                         # 값의 정본은 config/press.yaml(hall·floor 의 cam_h). 위는 파일이 없을 때 쓰는 값
    import yaml
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'press.yaml')) as _f:
        _c = yaml.safe_load(_f)
    CAM_H = {_c[g]['pose']: float(_c[g]['cam_h']) for g in ('hall', 'floor')}
    YAW = {_c[g]['pose']: float(_c[g].get('yaw_deg', 0.0)) for g in ('hall', 'floor')}
    TAUGHT = {_c[g]['pose']: [float(v) for v in _c[g]['look_joints']] for g in ('hall', 'floor') if _c[g].get('look_joints')}
    PATH = load_path(_c['floor'].get('path'))           # 손으로 기록한 길(홈 → 오른쪽 관찰 자세)
    PATH_SPEED = float(_c['floor'].get('path_speed', 0.5))
    if PATH:
        TAUGHT[_c['floor']['pose']] = PATH[-1]
except (OSError, KeyError, ValueError):
    YAW, TAUGHT, PATH, PATH_SPEED = {}, {}, None, 0.5
RETRACT_M = 0.06     # 홈 전에 빼는 거리 = 누르기의 접근점 거리
NEAR_HOME = 0.1      # rad. 모든 관절이 홈과 이만큼 안이면 빼기를 건너뛴다


def obs_pose(pose, cam_h, yaw):
    """관찰 자세(front·right) 관절값(press.look_pose). yaw = 밑동을 더 돌릴 도(음수 = 오른쪽)."""
    j, why = look_pose(pose, cam_h, yaw)
    if j is None:
        sys.exit(f'카메라 {cam_h} m 관찰 자세를 못 풉니다: {why}')
    return j


def go(n, target, vel, label, ask):
    """기록한 길 또는 세우고 돌기로 target 까지(press.route). (True, '') / (False, 이유) / (None, '중단')."""
    steps = route(n.cur, target, PATH)
    for kind, step, name in steps:
        name = name or label
        if len(steps) > 1:
            print(f'  [{name}]')
        if kind == 'path':
            if ask and input('  Enter = 실행 / q = 중단 : ').strip().lower() == 'q':
                return None, '중단'
            n.settle(**QUICK_SETTLE)
            ok = n.go_path(step, PATH_SPEED)
            if ok is not True:
                return False, ok
            continue
        ok, why = go_one(n, step, vel, name, ask)
        if not ok:
            return ok, why
    return True, ''


def go_one(n, target, vel, label, ask):
    """자리 잡기 → 계획 → (확인) → 실행. (True, '') / (False, 이유) / (None, '중단')."""
    n.settle()
    traj = n.plan_any(target, vel)
    if traj is None:
        return False, f'{label} 계획 실패'
    if ask:
        if input('  Enter = 실행 / q = 중단 : ').strip().lower() == 'q':
            return None, '중단'
        n.settle()                          # Enter 를 기다리는 사이 팔이 움직였을 수 있어 지금 자세에서 다시 계획
        traj = n.plan_any(target, vel)
        if traj is None:
            return False, f'{label} 계획 실패'
    res = n.execute(traj)
    if res is not True:                     # False = 실패, '충돌' = 전류가 커서 멈춤(press.Presser.execute)
        return False, f'{label} 실행 실패' + (' — 전류가 커서 멈춤(충돌 의심)' if res == '충돌' else '')
    return True, ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pose', choices=['home', 'front', 'right'])
    ap.add_argument('--cam', type=float, default=None,
                    help=f'관찰 자세 카메라 바닥 높이 m (기본 front {CAM_H["front"]}, right {CAM_H["right"]})')
    ap.add_argument('--yaw', type=float, default=None,
                    help='관찰 자세를 밑동에서 이만큼 더 돌림 도(음수 = 오른쪽, 기본 config/press.yaml yaw_deg)')
    ap.add_argument('--dry', action='store_true', help='계획만 보여 주고 움직이지 않는다')
    ap.add_argument('--vel', type=float, default=0.1, help='속도 배율 (기본 0.1)')
    ap.add_argument('--yes', action='store_true', help='Enter 확인 없이 바로 실행')
    ap.add_argument('--retract', action='store_true', help='home 전에 손끝을 6 cm 뒤로 뺀다')
    ap.add_argument('--path-speed', type=float, default=None,
                    help='기록한 길을 따라가는 관절 최대 속도 rad/s (기본 config/press.yaml floor.path_speed)')
    args = ap.parse_args()
    global PATH_SPEED
    if args.path_speed is not None:
        PATH_SPEED = args.path_speed

    cam = args.cam if args.cam is not None else CAM_H.get(args.pose, CAM_H['front'])
    yaw = args.yaw if args.yaw is not None else YAW.get(args.pose, 0.0)
    if args.pose == 'home':
        target = HOME
    elif args.pose in TAUGHT and args.cam is None and args.yaw is None:
        target = TAUGHT[args.pose]
        print('  손으로 기록한 길 끝(config/paths)' if PATH and target is PATH[-1] else '  손으로 잡아 둔 관찰 자세(config/press.yaml look_joints)')
    else:
        target = obs_pose(args.pose, cam, yaw)
    if args.pose != 'home' and target is not TAUGHT.get(args.pose):
        print(f'  카메라 높이 {cam:.2f} m (바닥에서)' + (f', 밑동 {yaw:+.0f}°' if yaw else ''))
    rclpy.init()
    n = Presser(args.vel)
    for _ in range(50):                 # 새 노드가 /joint_states 와 이어지는 데 1초 넘게 걸릴 때가 있다
        n.spin_for(0.1)
        if n.cur is not None:
            break
    if n.cur is None:
        sys.exit('/joint_states 가 없습니다. run/arm.sh 를 먼저 띄우세요')
    names, why = check_scene(n)
    if names is None:
        sys.exit(why)
    print('  지금 관절 (도) =', tuple(round(math.degrees(v), 1) for v in n.cur))
    print(f'  {args.pose:5s} 관절 (도) =', tuple(round(math.degrees(v), 1) for v in target))
    print('  움직일 양 (도) =', ', '.join(f'{j} {math.degrees(t - c):+.1f}' for j, t, c in zip(JOINTS, target, n.cur)))
    if args.dry:
        n.settle()
        kind, first, name = route(n.cur, target, PATH)[0]   # 여러 구간이면 첫 구간만 계획(기록한 길은 충돌 검사)
        why = n.path_blocked(first) if kind == 'path' else (None if n.plan_any(first, args.vel) else '계획 실패')
        if why:
            sys.exit(f'{why}. 움직이지 않습니다')
        n.spin_for(1.0)
        print('[--dry] 팔은 움직이지 않았습니다.' + (f' (첫 구간 {name}만 확인)' if name else ''))
        rclpy.shutdown()
        return

    note = ''
    if args.pose == 'home' and args.retract and max(abs(c - h) for c, h in zip(n.cur, HOME)) > NEAR_HOME:
        back = retract_target(n.cur, RETRACT_M)
        if back is None:
            note = ' (6 cm 빼기는 손끝이 닿지 않는 자리라 건너뜀)'
        else:
            print(f'  [1/2] 손끝을 {RETRACT_M * 100:.0f} cm 뒤로 빼기')
            ok, why = go(n, back, args.vel, '빼기', not args.yes)
            if ok is None:
                rclpy.shutdown()
                sys.exit('중단했습니다.')
            if not ok:
                note = f' ({why}, 바로 홈으로 감)'   # 펴진 채 버티면 과부하로 힘이 풀릴 수 있어 홈은 계속 간다
                print('  ' + note.strip(' ()'))
        print('  [2/2] 홈으로')
    ok, why = go(n, target, args.vel, args.pose, not args.yes)
    rclpy.shutdown()
    if ok is None:
        sys.exit('중단했습니다.')
    if not ok:
        sys.exit(f'{why}{note}')            # arm_service.py 가 종료 코드와 이 마지막 줄로 실패를 안다
    print(f'  완료{note}. 지금 관절 (도) =', tuple(round(math.degrees(v), 1) for v in n.cur))


if __name__ == '__main__':
    main()
