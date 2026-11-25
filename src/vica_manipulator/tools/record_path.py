#!/usr/bin/env python3
"""토크를 푼 팔을 손으로 옮기는 동안 관절값을 계속 읽어 길을 저장한다(팔은 안 움직인다).

  python3 tools/record_path.py right        홈 → 엘베 안 관찰 자세. Enter 로 시작, Enter 로 끝
  (run/arm.sh(①)를 끈 상태에서. 팔 보드 포트를 이 도구가 직접 연다)
  ③ astra.sh·④ buttons.sh 를 켜 두면 기록하는 동안 --button(기본 f3)이 손목 카메라 화면 어디에 보이는지 같이 찍는다
  — 관찰 자세에서 버튼이 화면 가운데 근처에 보일 때 Enter 로 끝내면 된다(10-22: 막대를 위로 든 자세는 3층이 화면 밖).

10-22 사용자: 짐칸에 큰 박스가 있어 홈 ↔ 오른쪽 관찰 자세를 사람이 손으로 옮긴 길 그대로 가고 싶다.
초당 hz 번 모터 4개의 현재 위치(주소 132)를 읽어 config/paths/<이름>.yaml 에 쓴다. 값은 ros2_control 과 같은
rad = (값 − 2048) × 2π / 4096. 마지막 자세가 관찰 자세다. 손떨림을 거른 점(keypoints)은 관절이 step_deg 넘게
바뀔 때마다 한 점이다. 재생은 press.py 가 한다(갈 때 순서대로, 홈으로 올 때 거꾸로).
"""
import argparse
import math
import os
import sys
import threading
import time

import yaml
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_83315A9250323952582E3120FF05251B-if00'
IDS = (11, 12, 13, 14)                      # joint1~4
HOME = [0.016, -1.535, 1.540, 0.724]        # press.HOME 과 같은 값
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'paths')
LATEST = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'latest.json')   # buttons.py 가 쓰는 찾은 버튼


def button_note(name):
    """buttons.py 가 지금 이 버튼을 화면(바로 세운 것) 어디에 보는지 한 줄. 버튼 찾기가 꺼져 있으면 그렇게."""
    try:
        import json
        with open(LATEST) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return f'{name}: 버튼 찾기 없음(③④ 꺼짐)'
    if time.time() - d.get('stamp', 0) > 1.5:
        return f'{name}: 버튼 찾기 멈춤(③④ 확인)'
    b = (d.get('buttons') or {}).get(name)
    if not b:
        return f'{name}: 화면에 안 보임'
    du, dv = (640 - b['u']) - 320, (480 - b['v']) - 240        # 원본(거꾸로) → 바로 세운 화면, 가운데 기준
    z = f'{b["z_m"] * 100:.0f} cm' if b.get('z_m') else '거리 모름(가운데에서 너무 비낌)'
    return (f'{name}: 가운데에서 {"오른쪽" if du >= 0 else "왼쪽"} {abs(du):.0f}px·{"아래" if dv >= 0 else "위"} {abs(dv):.0f}px, {z}')


def to_rad(v):
    if v > 0x7fffffff:
        v -= 1 << 32
    return (v - 2048) * 2 * math.pi / 4096


def read_all(p, h):
    q = []
    for i in IDS:
        v, r, e = h.read4ByteTxRx(p, i, 132)
        if r != COMM_SUCCESS:
            return None
        q.append(to_rad(v))
    return q


def keypoints(samples, step):
    """관절 중 하나라도 step rad 넘게 바뀔 때마다 한 점(처음·끝 포함)."""
    keep = [samples[0]]
    for s in samples[1:]:
        if max(abs(a - b) for a, b in zip(s[1:], keep[-1][1:])) >= step:
            keep.append(s)
    if keep[-1] is not samples[-1]:
        keep.append(samples[-1])
    return keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name', help='길 이름(config/paths/<이름>.yaml). 엘베 안 관찰 자세는 right')
    ap.add_argument('--hz', type=float, default=20.0, help='초당 읽는 번수')
    ap.add_argument('--step-deg', type=float, default=3.0, help='손떨림을 거른 점 간격(도)')
    ap.add_argument('--button', default='f3', help='기록하는 동안 화면 어디에 보이는지 같이 찍을 버튼(③④ 가 켜져 있을 때)')
    args = ap.parse_args()

    p, h = PortHandler(PORT), PacketHandler(2.0)
    if not (p.openPort() and p.setBaudRate(1000000)):
        sys.exit('팔 보드 포트를 못 엽니다. ①(run/arm.sh)이 켜져 있으면 끄고 다시 하세요')
    torque = [h.read1ByteTxRx(p, i, 64)[0] for i in IDS]
    q = read_all(p, h)
    if q is None:
        p.closePort()
        sys.exit('모터가 답하지 않습니다. 팔 보드 RESET 뒤 다시 하세요')
    if any(torque):
        p.closePort()
        sys.exit(f'토크가 켜진 모터가 있습니다 {torque}. 팔을 받치고 python3 ~/vica_arm_kit/torque_off.py 뒤 다시 하세요')
    print('지금 관절(도) =', [round(math.degrees(v), 1) for v in q])
    far = max(abs(a - b) for a, b in zip(q, HOME))
    if far > 0.1:
        print(f'  ⚠ 홈에서 {math.degrees(far):.0f}° 떨어져 있습니다. 홈에서 시작해야 홈 ↔ 관찰 자세 길로 쓸 수 있습니다')
    input('팔을 받친 채 Enter = 기록 시작 (끝낼 때 다시 Enter) : ')

    stop = threading.Event()
    threading.Thread(target=lambda: (input(), stop.set()), daemon=True).start()
    samples, t0, last_print, misses = [], time.time(), 0.0, 0
    while not stop.is_set():
        t = time.time()
        q = read_all(p, h)
        if q is None:
            misses += 1
        else:
            samples.append([round(t - t0, 3)] + [round(v, 4) for v in q])
            if t - last_print > 0.5:
                print(f'\r  {t - t0:5.1f}s  관절(도) ' + ' '.join(f'{math.degrees(v):+6.1f}' for v in q)
                      + f'  |  {button_note(args.button)}   ', end='', flush=True)
                last_print = t
        time.sleep(max(0.0, 1.0 / args.hz - (time.time() - t)))
    p.closePort()
    print()
    if len(samples) < 2:
        sys.exit('기록된 점이 없습니다')
    keys = keypoints(samples, math.radians(args.step_deg))
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f'{args.name}.yaml')
    data = {'recorded': time.strftime('%Y-%m-%d %H:%M:%S'), 'hz': args.hz, 'step_deg': args.step_deg,
            'note': '손으로 옮긴 길. [t, joint1, joint2, joint3, joint4] (s, rad). 마지막 = 관찰 자세',
            'keypoints': keys, 'samples': samples}
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False, allow_unicode=True)
    s, e = samples[0][1:], samples[-1][1:]
    print(f'저장: {os.path.abspath(path)}')
    print(f'  {samples[-1][0]:.1f}초, 읽은 점 {len(samples)}개(놓친 읽기 {misses}번), 거른 점 {len(keys)}개')
    print('  시작(도) =', [round(math.degrees(v), 1) for v in s], f'(홈과 최대 {math.degrees(max(abs(a - b) for a, b in zip(s, HOME))):.1f}°)')
    print('  끝 = 관찰 자세(도) =', [round(math.degrees(v), 1) for v in e])
    print('  끝 자세 화면:', button_note(args.button))


if __name__ == '__main__':
    main()
