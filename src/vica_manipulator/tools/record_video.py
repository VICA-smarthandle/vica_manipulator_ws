#!/usr/bin/env python3
"""YOLO 가 버튼을 찾는 손목 카메라 화면(/buttons/image, 상자 그림)을 나노에서 바로 mp4 로 저장한다(시연 영상용).

  bash run/video.sh              Ctrl+C 까지 녹화 → ~/vica_arm_logs/video/<시각>.mp4
  bash run/video.sh --sec 60     60 초만
  (③ astra.sh·④ buttons.sh 가 떠 있어야 한다. 누르기는 다른 창에서 평소대로)

10-22 사용자: 원격 화면(GUI)은 끊겨서 캡처가 힘들다 → 화면을 띄우지 않고 파일로 쓴다.
buttons.py 는 이 토픽을 듣는 쪽이 있을 때만 그림을 그려 낸다(초당 --hz, 기본 10). 화면은 바로 세우고
위에 시각과 찾은 버튼(이름·거리)을 적는다. 팔이 움직이는 동안의 관절값도 한 줄 적는다.
누르기(press.py)가 내는 /arm/press_state 로 지금 단계와 불빛 칸(lit.region) 빨강 점 수를 쓰고, 그 칸을 네모로 그린다.
불이 켜졌다고 판정하면 칸이 빨갛게 굵어지고 '불 켜짐' 이 크게 뜬다(10-22 사용자: 빨간불 감지도 영상에).
한글은 Noto CJK 글꼴(PIL)로 쓴다. 없으면 영어로.
"""
import argparse
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
LATEST = os.path.join(HERE, 'latest.json')
OUT = os.path.expanduser('~/vica_arm_logs/video')
CONFIG = os.path.join(HERE, '..', 'config', 'press.yaml')
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
LIT_SHOW_SEC = 3.0          # 불 켜짐 판정 뒤 이만큼 '불 켜짐' 을 띄운다


def lit_region():
    try:
        import yaml
        with open(CONFIG) as f:
            return (yaml.safe_load(f).get('lit') or {}).get('region')
    except (OSError, ValueError, ImportError):
        return None


def make_writer_text():
    """한글을 쓸 수 있으면 PIL 로, 아니면 cv2 영문으로 쓰는 함수 text(img, 글, (x, y), 크기, BGR)."""
    try:
        from PIL import Image as PImage, ImageDraw, ImageFont
        fonts = {}

        def text(img, s, xy, size, color):
            f = fonts.setdefault(size, ImageFont.truetype(FONT, size))
            pil = PImage.fromarray(img[:, :, ::-1])
            ImageDraw.Draw(pil).text(xy, s, font=f, fill=color[::-1])
            img[:] = np.asarray(pil)[:, :, ::-1]
        ImageFont.truetype(FONT, 12)
        return text
    except (ImportError, OSError):
        def text(img, s, xy, size, color):
            s = s.encode('ascii', 'replace').decode()
            cv2.putText(img, s, (xy[0], xy[1] + size), cv2.FONT_HERSHEY_SIMPLEX, size / 30, color, 1, cv2.LINE_AA)
        return text


def buttons_line():
    try:
        with open(LATEST) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return ''
    if time.time() - d.get('stamp', 0) > 1.5:
        return ''
    parts = []
    for name, b in (d.get('buttons') or {}).items():
        z = f' {b["z_m"] * 100:.0f}cm' if b.get('z_m') else ''
        parts.append(f'{name}{z}')
    return 'found: ' + ', '.join(parts) if parts else 'found: -'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sec', type=float, default=0.0, help='이만큼만 녹화(0 = Ctrl+C 까지)')
    ap.add_argument('--fps', type=float, default=10.0, help='파일의 초당 장 수(buttons.py --hz 와 같게)')
    ap.add_argument('--topic', default='/buttons/image', help='녹화할 화면(상자 없이 원본은 /camera/color/image_raw)')
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, time.strftime('%Y%m%d_%H%M%S') + '.mp4')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)     # Ctrl+C 에도 파일을 닫고 끝낸다
    n = rclpy.create_node('arm_video')
    st = {'img': None, 'n': 0, 'q': None}
    n.create_subscription(Image, args.topic, lambda m: st.update(img=m, n=st['n'] + 1), qos_profile_sensor_data)
    n.create_subscription(JointState, '/joint_states', lambda m: st.update(q=dict(zip(m.name, m.position))), 10)
    st.update(ps={}, lit_t=0.0)

    def on_state(m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if 'red' not in d:                       # 단계만 바뀐 알림이면 빨강 수는 지운다(다른 단계의 수가 남지 않게)
            st['ps'] = {'stage': d.get('stage', '')}
        else:
            st['ps'] = d
        if d.get('lit'):
            st['lit_t'] = time.time()
    n.create_subscription(String, '/arm/press_state', on_state, 10)
    region, text = lit_region(), make_writer_text()
    writer, seen, frames, t0, warned = None, 0, 0, time.time(), time.time()
    print(f'녹화: {path}  (Ctrl+C = 끝)', flush=True)
    try:
        while rclpy.ok() and (args.sec <= 0 or time.time() - t0 < args.sec):
            rclpy.spin_once(n, timeout_sec=0.05)
            if st['n'] == seen or st['img'] is None:
                if frames == 0 and time.time() - warned > 5:
                    print(f'  화면이 안 옵니다({args.topic}). ③ astra.sh·④ buttons.sh 를 보세요', flush=True)
                    warned = time.time()
                continue
            seen, m = st['n'], st['img']
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
            img = np.ascontiguousarray(cv2.rotate(a[:, :, ::-1], cv2.ROTATE_180))     # 원본(거꾸로) RGB → 바로 선 BGR
            if writer is None:
                writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (img.shape[1], img.shape[0]))
                if not writer.isOpened():
                    path = path[:-4] + '.avi'
                    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), args.fps, (img.shape[1], img.shape[0]))
                    print(f'  mp4 를 못 써서 {path} 로 씁니다', flush=True)
            lit_now = time.time() - st['lit_t'] < LIT_SHOW_SEC
            ps = st['ps']
            if region and ('red' in ps or lit_now):   # 불빛 칸: 불을 보는 동안만 그린다
                x1, y1, x2, y2 = region
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255) if lit_now else (0, 220, 0), 4 if lit_now else 1)
            cv2.rectangle(img, (0, 0), (img.shape[1], 66), (0, 0, 0), -1)
            text(img, time.strftime('%H:%M:%S') + '  VICA arm  ' + buttons_line(), (8, 2), 15, (255, 255, 255))
            q = st['q']
            if q and all(f'joint{i}' in q for i in range(1, 5)):
                text(img, '관절(도) ' + ' '.join(f'{math.degrees(q[f"joint{i}"]):+.0f}' for i in range(1, 5)),
                     (8, 22), 13, (200, 200, 200))
            line = ps.get('stage', '')
            if 'red' in ps:
                line += f"   빨간 점 {ps['red']} (켜짐 기준 {ps.get('need')})"
            if line:
                text(img, line, (8, 42), 15, (0, 255, 255))
            if lit_now:
                cv2.rectangle(img, (150, 380), (490, 440), (0, 0, 200), -1)
                text(img, '불 켜짐 — 눌림 확인', (175, 386), 32, (255, 255, 255))
            writer.write(img)
            frames += 1
            if frames % 50 == 0:
                print(f'  {frames}장, {time.time() - t0:.0f}초', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        rclpy.try_shutdown()
    print(f'저장: {path}  ({frames}장, {time.time() - t0:.0f}초)' if frames else '저장할 화면이 없었습니다', flush=True)


if __name__ == '__main__':
    main()
