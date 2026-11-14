#!/usr/bin/env python3
"""시연용: 홈 → (관찰 자세) → 버튼 누르기 → 홈. 카메라·버튼 찾기 없이 손으로 기록한 길(record_path.py)만 쓴다.

  bash run/demo.sh down              config/paths/down.yaml 길로 한 번(시작 전 Enter)
  bash run/demo.sh down --yes        Enter 없이 바로
  bash run/demo.sh down --loop 3     세 번 되풀이(사이 쉬기 demo.rest 초)
  bash run/demo.sh down --dry        길의 점마다 충돌 검사만(팔 안 움직임)
  bash run/demo.sh down --reach 0.2  기록한 끝 자세보다 20 cm 더까지 밀어 본다(끝을 버튼 앞에서 멈추고 기록했을 때)
  (run/arm.sh 와 run/moveit.sh 가 떠 있어야 한다. ③④ 카메라는 없어도 된다)

길 가르치기: ① 을 끄고 python3 tools/record_path.py down --button down → 홈에서 Enter → 관찰 자세에서 2~3 초 멈춤 →
막대 끝을 버튼 가운데에 대고 Enter. 끝 자세 = 버튼에 닿은 자세.
10-22 사용자: 시연장에서 버튼 찾기가 안 돼도 보여 줄 수 있게.

재생: 길을 따라가다 기록할 때 1.5 초 넘게 멈췄던 자리(관찰 자세)에서 demo.look 초 멈춘다 → 버튼 앞 demo.back 까지 →
밑동에서 바깥쪽으로 곧게 밀다 전류가 demo.contact_ma 넘게 오르면 멈춤(닿음) → demo.push 더 밀고 demo.hold 초 →
demo.back 빼기 → 길을 거꾸로 → 홈. 손으로 받쳐 든 기록과 달리 모터로 가면 뻗은 팔이 처지므로 끝 자세를 demo.z_up 만큼 올린다.
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402
from press import (CONFIG, CONTACT_N, HOME, Presser, check_scene, forward_push, load_path,  # noqa: E402
                   off_home, path_traj, retract_target, same, wait_joints)

GAP_SEC = 1.5              # 기록에서 이보다 오래 멈춘 자리 = 관찰 자세(재생 때 look 초 멈춤)
DEFAULTS = dict(speed=0.5, z_up=0.03, back=0.03, reach=0.03, push=0.009, contact_ma=300, push_ma=300,
                press_vel=0.05, hold=1.5, look=1.0, rest=3.0)


def load_demo():
    with open(CONFIG) as f:
        return dict(DEFAULTS, **(yaml.safe_load(f).get("demo") or {}))


def tip(q):
    return geom.fk(q)[0]


def radial(q, dist):
    """q 에서 손끝 높이·숙임은 그대로 밑동에서 바깥쪽으로 dist m(음수 = 안쪽) 옮긴 관절값. 못 풀면 None."""
    (x, y, z), p = geom.fk(q)
    a = math.atan2(y, x - geom.J1_X)
    j, _ = geom.press_ik(x + dist * math.cos(a), y + dist * math.sin(a), z, p)
    return list(j) if j else None


def plan_parts(path, cfg):
    """기록한 길 → (멈출 자리마다 나눈 구간들, 버튼 앞 자세, 고친 끝 자세). 끝 자세는 z_up 올리고, 그 앞 back 만큼 안쪽이 버튼 앞."""
    keys = path["keypoints"]
    end = [float(v) for v in keys[-1][1:]]
    (x, y, z), p = geom.fk(end)
    j, why = geom.press_ik(x, y, z + cfg["z_up"], p)
    if j is None:
        sys.exit(f"끝 자세를 {cfg['z_up'] * 100:.0f} cm 올리면 못 갑니다: {why}")
    end = list(j)
    front = radial(end, -cfg["back"])
    if front is None:
        sys.exit("버튼 앞 자세를 못 풉니다")
    # 홈 근처 앞쪽 점은 버리고(press.off_home), 끝 자세 back 안쪽(버튼에 더 가까운) 점도 버린다 → 마지막은 버튼 앞
    pts = off_home([[float(v) for v in k[1:]] for k in keys])
    ts = {tuple(round(v, 4) for v in k[1:]): k[0] for k in keys}
    ex, ey, ez = tip(end)
    keep = [q for q in pts if math.dist(tip(q), (ex, ey, ez)) > cfg["back"] + 0.01]
    parts, cur = [], []
    for k, q in enumerate(keep):
        cur.append(q)
        t, nxt = ts.get(tuple(round(v, 4) for v in q)), keep[k + 1] if k + 1 < len(keep) else None
        tn = ts.get(tuple(round(v, 4) for v in nxt)) if nxt else None
        if t is not None and tn is not None and tn - t > GAP_SEC:
            parts.append(cur)
            cur = []
    parts.append(cur + [front])
    return [p for p in parts if p], front, end


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", nargs="?", default="down", help="기록한 길 이름(config/paths/<이름>.yaml)")
    ap.add_argument("--loop", type=int, default=1, help="되풀이 횟수")
    ap.add_argument("--yes", action="store_true", help="시작 전 Enter 확인 없이")
    ap.add_argument("--dry", action="store_true", help="충돌 검사만, 팔 안 움직임")
    ap.add_argument("--z-up", type=float, default=None, help="끝(누르는) 자세를 올릴 m (기본 config demo.z_up)")
    ap.add_argument("--speed", type=float, default=None, help="길 따라가는 관절 최대 rad/s (기본 config demo.speed)")
    ap.add_argument("--reach", type=float, default=None,
                    help="기록한 끝 자세보다 이만큼 더까지 밀어 본다 m(그 전에 닿으면 멈춤, 기본 config demo.reach)")
    ap.add_argument("--hold", type=float, default=None, help="누른 채 머무는 초 (기본 config demo.hold)")
    args = ap.parse_args()
    cfg = load_demo()
    if args.z_up is not None:
        cfg["z_up"] = args.z_up
    if args.speed is not None:
        cfg["speed"] = args.speed
    if args.reach is not None:
        cfg["reach"] = args.reach
    if args.hold is not None:
        cfg["hold"] = args.hold
    path_file = os.path.join(os.path.dirname(CONFIG), "paths", f"{args.name}.yaml")
    if not os.path.exists(path_file) or not load_path(args.name):
        sys.exit(f"기록한 길이 없습니다: {path_file}. ① 을 끄고 python3 tools/record_path.py {args.name} --button down 으로 먼저 기록하세요")
    with open(path_file) as f:
        path = yaml.safe_load(f)
    parts, front, end = plan_parts(path, cfg)
    print(f"[시연] 길 {args.name}({path.get('recorded')}): {len(parts)} 구간"
          + (" — 관찰 자세에서 멈춤" if len(parts) > 1 else "")
          + f", 누르는 자세 +{cfg['z_up'] * 100:.0f} cm, 버튼 앞 {cfg['back'] * 100:.0f} cm에서 곧게 밀기, 닿음 {cfg['contact_ma']:.0f} mA")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Presser(cfg["press_vel"])
    try:
        if not wait_joints(n):
            sys.exit("/joint_states 가 없습니다. run/arm.sh 가 떠 있는지 보세요")
        names, why = check_scene(n)
        if names is None:
            sys.exit(why)
        allpts = [q for p in parts for q in p] + [end]
        why = n.path_blocked(allpts)
        if why:
            sys.exit(f"길이 MoveIt 장애물에 걸립니다({why}). 움직이지 않습니다")
        if args.dry:
            print("[--dry] 충돌 검사 통과. 팔은 움직이지 않았습니다")
            return
        for k in range(args.loop):
            if not same(n.cur, HOME, 0.1):
                sys.exit("팔이 홈이 아닙니다. bash run/pose.sh home 으로 홈에 둔 뒤 다시 하세요")
            if not args.yes and k == 0 and input("Enter = 시작 / q = 중단 : ").strip().lower() == "q":
                return
            print(f"\n===== 시연 {k + 1}/{args.loop}")
            ok = run_once(n, cfg, parts, front, end)
            if ok is not True:
                sys.exit(f"여기서 멈춥니다: {ok}")
            if k + 1 < args.loop:
                n.spin_for(cfg["rest"])
    finally:
        rclpy.shutdown()


def run_once(n, cfg, parts, front, end):
    """한 번: 길 → (관찰에서 멈춤) → 버튼 앞 → 밀어 누르기 → 빼기 → 길 거꾸로 → 홈. True 또는 이유."""
    t0 = time.time()
    for k, part in enumerate(parts):
        print(f"[길 {k + 1}/{len(parts)}]" + (" → 버튼 앞" if k == len(parts) - 1 else " → 관찰 자세"))
        n.settle(timeout=1.0, window=0.2)
        ok = n.go_path(part, cfg["speed"])
        if ok is not True:
            return ok
        if k < len(parts) - 1:
            print(f"  관찰 자세에서 {cfg['look']:.1f}초")
            n.spin_for(cfg["look"])
    # 누르기: 버튼 앞에서 곧게 back + reach 까지 밀다 닿으면 멈춤 → push 더 → hold
    n.settle()
    target = radial(n.cur, cfg["back"] + cfg["reach"])
    print(f"  버튼 앞에서 곧게 최대 {(cfg['back'] + cfg['reach']) * 100:.0f} cm 밀기(닿으면 멈춤)")
    traj = n.plan_any(target, cfg["press_vel"]) if target else None
    if traj is None:
        return "누르기 계획 실패"
    base = n.amp_baseline()
    print("[누르기]")
    ok = n.execute(traj, base=base, stop_ma=cfg["contact_ma"], n_over=CONTACT_N, why="닿음")
    if ok == "닿음":
        q = forward_push(n.cur, cfg["push"])
        traj = n.plan_any(q, cfg["press_vel"])
        if traj is not None:
            n.execute(traj, base=base, stop_ma=cfg["contact_ma"] + cfg["push_ma"], n_over=CONTACT_N, why="너무 셈")
        print(f"  누른 채 {cfg['hold']:.1f}초")
        n.spin_for(cfg["hold"])
    elif ok is True:
        print(f"  ⚠ 버튼 앞 {cfg['back'] * 100:.0f} cm + {cfg['reach'] * 100:.0f} cm 를 다 가도록 안 닿음(전류 안 오름) — 판이 더 멀거나 높이가 어긋남")
    else:
        return "누르기 실행 실패"
    # 빼기 → 길 거꾸로 → 홈
    print("[빼기]")
    back = retract_target(n.cur, cfg["back"])
    traj = n.plan_any(back, 0.15) if back else None
    if traj is None or n.execute(traj) is not True:
        return "빼기 실패 — 팔이 버튼 앞에 펴져 있음"
    rev = [q for p in parts for q in p][::-1][1:] + [list(HOME)]
    print("[길 거꾸로 → 홈]")
    n.settle(timeout=1.0, window=0.2)
    ok = n.go_path(rev, cfg["speed"])
    if ok is not True:
        return ok
    print(f"완료 {time.time() - t0:.1f}초. 지금 관절 =", tuple(round(v, 3) for v in n.cur))
    return True


if __name__ == "__main__":
    main()
