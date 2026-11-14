#!/usr/bin/env python3
"""길목 신호등. 팔이 지나갈 '칸' 안에 뭔가 들어오면 빨간불을 켠다.

칸 = 깊이 화면의 길목 네모 × 거리 범위(카메라 앞 ~ far). 누르기 때 far 는 버튼 벽 거리 − 5 cm 다.
물체가 무엇인지는 보지 않는다. 있다/없다만 본다(OctoMap 처럼 돌아가는 길을 찾지 않는다).

  bash run/guard.sh                          가운데 네모, 칸 끝 0.6 m (팔은 꺼 둔 채 손을 넣었다 뺐다)
  bash run/guard.sh --far 0.8                칸 끝을 바꾼다. 네모 안 배경(벽·바닥)보다 가까워야 한다
  bash run/guard.sh --roi 240 160 400 320    네모를 직접 정한다 (돌리기 전 원본 화면 u0 v0 u1 v1)

빨간불/초록불은 /path_guard/blocked(Bool), 확인 화면은 /path_guard/image (원본 방향. rqt 회전 버튼 두 번).

판정 (장마다)
  칸 안   : 깊이가 0 보다 크고 far 보다 가까운 점이 네모의 FRAC 이상
  렌즈 가림: 깊이 0 인 점이 네모의 BLANK_FRAC 이상 → 0.36 m 보다 가까이 와서 렌즈를 가림
  둘 중 하나면 '막힌 장'. 막힌 장 ON_N 장 연속 → 빨간불, 뚫린 장 OFF_N 장 연속 → 초록불

처음엔 켤 때 찍은 깊이와 비교했으나(10-18) 꺼진 팔에 손이 닿아 카메라가 돌자 바닥이 통째로
'가까워진 점'이 됐다. 칸 방식은 처음 사진이 없어 카메라가 움직여도 영향이 없다(사용자 제안).
막대(누름 도구)는 원본 화면 맨 위 약 90줄에만 보이므로 네모를 그 아래에 둔다.
"""
import argparse
import time

import numpy as np

FAR_M = 0.6         # 혼자 시험할 때 칸 끝. 누르기 때는 버튼 벽 거리 − WALL_MARGIN_M
WALL_MARGIN_M = 0.05
FRAC = 0.02         # 네모 점 수의 2% (160 x 160 이면 512점. 40 cm 앞 손가락 하나가 약 1,200점)
BLANK_FRAC = 0.4    # 10-18 바닥을 볼 때 가운데 네모의 빈칸은 0~1%
ON_N = 3            # 30 Hz 에서 0.1초
OFF_N = 15          # 0.5초


class Guard:
    """막힌 장인지 판정하고, 깜빡임을 걸러 빨간불(True)/초록불(False)을 낸다."""

    def __init__(self, frac=FRAC, blank_frac=BLANK_FRAC, on_n=ON_N, off_n=OFF_N):
        self.frac, self.blank_frac = frac, blank_frac
        self.on_n, self.off_n = on_n, off_n
        self.red = False
        self._run = 0       # 지금 불과 다른 장이 몇 장 연속인가

    def frame_blocked(self, d, far_mm):
        """d 네모 안 깊이(mm, 0 = 빈칸). 돌려주는 값 (막혔나, 칸 안 점 수, 빈칸 비율)."""
        inside = int(((d > 0) & (d < far_mm)).sum())
        blank = float((d == 0).mean())
        return inside > self.frac * d.size or blank > self.blank_frac, inside, blank

    def update(self, blocked):
        if blocked != self.red:
            self._run += 1
            if self._run >= (self.on_n if blocked else self.off_n):
                self.red, self._run = blocked, 0
        else:
            self._run = 0
        return self.red


def depth_array(m):
    return np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]


def debug_image(depth, roi, far_mm, red, active=True):
    """회색 깊이 + 칸 안 점 빨강 + 네모 안 빈칸 보라 + 네모 테두리(빨간불/초록불 색, 안 볼 때 회색). 원본 방향 그대로.
    밝기: 0.3 m 흰색 → 4 m 검정(11-02: 2 m 까지만 그려 책상 시험에서 거의 까맣게 보였다). 깊이 0 은 검정."""
    g = np.clip((4000.0 - depth) * (255.0 / 3700.0), 0, 255).astype(np.uint8)
    g[depth == 0] = 0
    img = np.dstack([g, g, g])
    u0, v0, u1, v1 = roi
    d = depth[v0:v1, u0:u1]
    box = img[v0:v1, u0:u1]
    if active:
        box[(d > 0) & (d < far_mm)] = (255, 0, 0)
        box[d == 0] = (200, 0, 255)
    c = (255, 0, 0) if red else (0, 220, 0) if active else (128, 128, 128)
    t = 3
    img[v0:v0 + t, u0:u1] = c
    img[v1 - t:v1, u0:u1] = c
    img[v0:v1, u0:u0 + t] = c
    img[v0:v1, u1 - t:u1] = c
    return img


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool

    ap = argparse.ArgumentParser()
    ap.add_argument('--far', type=float, default=FAR_M, help=f'칸 끝 m (기본 {FAR_M})')
    ap.add_argument('--roi', type=int, nargs=4, metavar=('U0', 'V0', 'U1', 'V1'),
                    help='길목 네모 (원본 화면 픽셀). 기본은 가운데 160 x 160')
    ap.add_argument('--topic', default='/camera/depth/image_raw')
    args, _ = ap.parse_known_args()
    far_mm = args.far * 1000.0

    class GuardNode(Node):
        def __init__(self):
            super().__init__('path_guard')
            self.g = Guard()
            self.roi = args.roi
            self.count = 0
            self.last_print = 0.0
            self.pub = self.create_publisher(Bool, '/path_guard/blocked', 10)
            self.img_pub = self.create_publisher(Image, '/path_guard/image', 2)
            self.create_subscription(Image, args.topic, self.on_depth, qos_profile_sensor_data)
            print(f'{args.topic} 를 기다립니다. 칸 끝 {args.far:.2f} m')

        def on_depth(self, m):
            depth = depth_array(m)
            if self.roi is None:
                cu, cv = m.width // 2, m.height // 2
                self.roi = (cu - 80, cv - 80, cu + 80, cv + 80)
                print(f'네모 {self.roi}')
            u0, v0, u1, v1 = self.roi
            d = depth[v0:v1, u0:u1]
            was = self.g.red
            blocked, n, blank = self.g.frame_blocked(d, far_mm)
            red = self.g.update(blocked)
            self.pub.publish(Bool(data=red))
            now = time.time()
            msg = f'칸 안 점 {n:5d} / {d.size} ({100 * n / d.size:4.1f}%)  빈칸 {100 * blank:4.1f}%'
            if red != was:
                print(f'{"빨간불 — 막힘" if red else "초록불 — 뚫림"}  {msg}')
            elif now - self.last_print > 1.0:
                print(f'  {"빨간불" if red else "초록불"}  {msg}')
                self.last_print = now
            self.count += 1
            if self.count % 3 == 0 and self.img_pub.get_subscription_count():
                img = debug_image(depth, self.roi, far_mm, red)
                out = Image(header=m.header, height=img.shape[0], width=img.shape[1],
                            encoding='rgb8', step=img.shape[1] * 3, data=img.tobytes())
                self.img_pub.publish(out)

    rclpy.init()
    n = GuardNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
