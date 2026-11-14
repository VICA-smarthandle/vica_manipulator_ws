#!/usr/bin/env python3
"""버튼 자리를 손으로 가르친다: 토크를 푼 팔의 막대 끝을 버튼 가운데에 대고 Enter → 시연·NX 서비스(--fallback·--blind)가
쓰는 '성공한 버튼 자리'(~/vica_arm_logs/last_press_<버튼>.json)로 저장한다(팔은 안 움직인다).

  python3 tools/teach_button.py down             호출 버튼(down)
  python3 tools/teach_button.py f3               엘베 안 3층
  python3 tools/teach_button.py down --z-up 0.05 처짐 보정을 바꿔서
  (run/arm.sh(①)를 끈 상태에서. 팔 보드 포트를 이 도구가 직접 연다)

10-22 사용자: PB 에서 카메라로는 잘 못 눌러서 손으로 가르치고 싶다. 손으로 받쳐 댄 자세는 처지지 않지만 모터로 뻗으면
처지므로 저장 목표를 z_up 만큼 올린다. 호출 버튼 0.06: 카메라로 성공한 목표 높이(world 0.695~0.697) − 실측 버튼
높이(바닥 1.05 m = world 0.635). 엘베 안 0.05 는 [가정]. 막대는 버튼판에 거의 수직(수평)으로 대야 한다.
"""
import argparse
import json
import math
import os
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402

PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_83315A9250323952582E3120FF05251B-if00'
IDS = (11, 12, 13, 14)
FLOOR_Z = 0.415                       # world 원점(기둥 바닥)의 바닥 높이
Z_UP = {'up': 0.06, 'down': 0.06}     # 그 밖(엘베 안)은 0.05 [가정]


def read_q(p, h):
    q = []
    for i in IDS:
        v, r, e = h.read4ByteTxRx(p, i, 132)
        if r != COMM_SUCCESS:
            return None
        if v > 0x7fffffff:
            v -= 1 << 32
        q.append((v - 2048) * 2 * math.pi / 4096)
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('target', help='버튼 이름(down, up, f1~f4)')
    ap.add_argument('--z-up', type=float, default=None, help='저장할 목표를 막대 끝보다 올릴 m (처짐 보정)')
    args = ap.parse_args()
    z_up = args.z_up if args.z_up is not None else Z_UP.get(args.target, 0.05)

    p, h = PortHandler(PORT), PacketHandler(2.0)
    if not (p.openPort() and p.setBaudRate(1000000)):
        sys.exit('팔 보드 포트를 못 엽니다. ①(run/arm.sh)이 켜져 있으면 끄고 다시 하세요')
    torque = [h.read1ByteTxRx(p, i, 64)[0] for i in IDS]
    if read_q(p, h) is None:
        p.closePort()
        sys.exit('모터가 답하지 않습니다. 팔 보드 RESET 뒤 다시 하세요')
    if any(torque):
        p.closePort()
        sys.exit(f'토크가 켜진 모터가 있습니다 {torque}. 팔을 받치고 python3 ~/vica_arm_kit/torque_off.py 뒤 다시 하세요')
    print(f"막대 끝(둥근 끝)을 '{args.target}' 버튼 가운데에 살짝 대고, 막대는 버튼판에 수직(수평)으로 두세요.")
    print('대는 동안 막대 끝 자리가 계속 나옵니다. 댄 채로 Enter = 저장 / q + Enter = 그만')
    import select
    while True:
        q = read_q(p, h)
        if q:
            (x, y, z), pitch = geom.fk(q)
            print(f'\r  막대 끝: 밑동에서 {math.hypot(x - geom.J1_X, y):.3f} m, 바닥에서 {z + FLOOR_Z:.3f} m, '
                  f'좌우 {math.degrees(math.atan2(y, x - geom.J1_X)):+5.1f}°, 숙임 {math.degrees(pitch):+5.1f}°   ',
                  end='', flush=True)
        r, _, _ = select.select([sys.stdin], [], [], 0.3)
        if r:
            line = sys.stdin.readline().strip().lower()
            if line == 'q':
                p.closePort()
                print('\n저장하지 않았습니다')
                return
            break
    qs = [q for q in (read_q(p, h) for _ in range(10)) if q]      # 댄 채 10 번 읽어 평균
    p.closePort()
    q = [sum(v) / len(qs) for v in zip(*qs)]
    (x, y, z), pitch = geom.fk(q)
    print()
    if abs(math.degrees(pitch)) > 10:
        print(f'  ⚠ 막대가 {math.degrees(pitch):+.0f}° 숙여 있습니다. 누를 때는 수평으로 밀므로 닿는 자리가 조금 다를 수 있습니다')
    button = [round(x, 4), round(y, 4), round(z + z_up, 4)]
    path = os.path.expanduser(f'~/vica_arm_logs/last_press_{args.target}.json')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    old = None
    if os.path.exists(path):
        with open(path) as f:
            old = f.read()
        with open(path + '.prev', 'w') as f:
            f.write(old)
    with open(path, 'w') as f:
        json.dump({'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'target': args.target,
                   'result': f'손으로 가르침(막대 끝 + 처짐 보정 {z_up * 100:.0f} cm)', 'button': button,
                   'tip': [round(x, 4), round(y, 4), round(z, 4)], 'joints': [round(v, 4) for v in q], 'z_up': z_up},
                  f, ensure_ascii=False)
    print(f'저장: {path}' + ('  (이전 값은 .prev 로)' if old else ''))
    print(f'  막대 끝 world ({x:.3f}, {y:.3f}, {z:.3f}) = 바닥에서 {z + FLOOR_Z:.3f} m, 밑동에서 {math.hypot(x - geom.J1_X, y):.3f} m')
    print(f'  누를 목표 = 막대 끝 + {z_up * 100:.0f} cm 위(모터로 뻗으면 처지는 만큼)')
    print(f'  이제 bash run/press.sh {args.target} --observe --yes --blind 로 확인, NX 서비스는 못 찾으면 이 자리로 누릅니다')


if __name__ == '__main__':
    main()
