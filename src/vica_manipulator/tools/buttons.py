#!/usr/bin/env python3
"""손목 Astra(색 + 깊이) + YOLO 로 엘리베이터 버튼을 찾고, 거리와 3D 위치를 낸다.

  bash run/buttons.sh            찾기만 (팔을 꺼 둬도 된다. 그때는 카메라 기준 위치만 나온다)
  bash run/buttons.sh --imgsz 960   버튼이 작게 보일 때 (느려진다)
  확인 화면: bash run/view.sh /buttons/image  (원본 방향. rqt 회전 버튼 두 번이면 글자가 바로 선다)
  찾은 결과는 tools/latest.json 에 계속 덮어쓴다. press.py 가 이 파일을 읽는다(옛 웹캠 detect.py 와 같은 꼴).

카메라가 거꾸로 달려 있어(선 꽂는 곳이 위) 원본 화면은 위아래가 뒤집혀 있다.
YOLO 에는 180도 돌려서 넣는다. 안 돌리면 up 과 down 을 바꿔 읽는다.
찾은 상자는 다시 원본 방향으로 되돌린다. 깊이와 카메라 정보(K)가 원본 기준이기 때문이다.

거리 Z = 버튼 상자 가운데 절반 안 깊이 칸들의 중앙값(0 = 모름 은 뺀다). 깊이는 색 화면에 정합돼 있다.
실제 엘리베이터 호출 버튼판은 거울 같은 스테인리스라 깊이가 안 나온다(10-20). 그때는 버튼 크기로 잰다:
  Z = fx × 상자 실제 크기(--button-mm) ÷ 화면 속 상자 크기(px)  — 상자가 화면 끝에 걸리거나 많이 비껴 있으면 안 잼
깊이와 크기가 둘 다 있는데 많이 다르면 가까운 쪽을 쓴다(멀게 잘못 알면 판을 세게 민다).
카메라 기준 3D (camera_color_optical_frame) = ((u - cx) Z / fx, (v - cy) Z / fy, Z)
팔이 켜져 있으면 TF 로 world(팔 기둥 바닥 중심) 위치까지 바꾼다.
"""
import argparse
import json
import os
import statistics
import time
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.expanduser('~/vica_arm_kit/models')
# 10-20 Astra 사진으로 다시 배운 네모 모델(yolo11s, 9 클래스, Roboflow 버전 4 그대로 학습한 B).
# 나노 TensorRT 640 고정 엔진이 있으면 그걸 쓴다. 나노 Astra 55장: B 엔진 200/200 · 23.8 ms, A(원본 재구성) 엔진 31.5 ms
MODEL = next((os.path.join(MODELS, f) for f in ('elev_v4_rf_best.engine', 'elev_v4_rf_best.pt')
              if os.path.exists(os.path.join(MODELS, f))), os.path.join(MODELS, 'elev_v4_rf_best.pt'))
BUTTONS = ('up', 'down', 'f1', 'f2', 'f3', 'f4', 'open', 'close')
MIN_CONF = 0.5
# 10-18 옛 모델(elev_v3_aug)은 작은 버튼(약 20 px)을 640 통째로 0 개 찾아서, 1280 으로 버튼판을 찾고 잘라 키워 봤다.
# 10-20 새 모델은 Astra 검증 55장에서 640 한 번 200/200 · 6 ms, 1280 188/200, 판 확대(옛 방식) 187/200,
# 전체+판 확대 합치기 197/200 · 57 ms 였다. 그래서 기본은 640 한 번, 판 확대는 --zoom 일 때만(멀리서 볼 때).
PANEL_IMGSZ = 1280
PANEL_CONF = 0.1        # 버튼판은 낮아도 받는다. 최종 버튼은 MIN_CONF 가 거른다
MIN_DEPTH_PX = 20       # 상자 가운데에 거리를 잰 칸이 이보다 적으면 깊이를 모른다고 한다
# 버튼 크기로 거리 재기. 10-20 실물 호출 버튼(앞면 3.5 cm). 처음엔 줄자(렌즈 569 mm, 33.2 px)로 37 mm 를 잡았으나
#   실제 누르기에서 37 mm 로 잰 57.4 cm(32.9 px)보다 4 cm 가까운 데서 막대가 버튼에 닿았다(카메라~막대 끝 실측
#   9~10.5 cm 는 모델 10.9 cm 와 맞음) → 53.4 cm × 32.9 px ÷ fx = 34.4 mm. 상자가 앞면에 꼭 맞게 그려진다.
#   같은 버튼을 28° 비껴 봤을 땐 크게 틀려 비낀 화면은 쓰지 않는다
BUTTON_MM = 34.5
SIZE_MAX_DEG = 15.0     # 화면 가운데에서 이보다 비껴 보이면 크기로 재지 않는다
EDGE_PX = 3             # 상자가 화면 끝에 이만큼 붙으면 잘린 것(작게 보여 멀다고 착각)으로 보고 크기로 재지 않는다
AGREE_M = 0.03          # 깊이와 크기가 이보다 다르면 가까운 쪽을 쓴다
CAM_FRAME = 'camera_color_optical_frame'
COLORS = {'up': (0, 200, 0), 'down': (0, 140, 255), 'panel': (160, 160, 160)}


class ButtonFinder:
    """ROS 없이 쓰는 부분. 원본(거꾸로 된) RGB 화면을 받아 원본 기준 상자를 돌려준다."""

    def __init__(self, model=MODEL, imgsz=640, panel_imgsz=PANEL_IMGSZ, zoom=False):
        from ultralytics import YOLO
        self.model = YOLO(model, task='detect')
        self.imgsz, self.panel_imgsz, self.zoom = imgsz, panel_imgsz, zoom

    def _predict(self, bgr, conf=0.25, imgsz=None):
        r = self.model.predict(bgr, conf=conf, imgsz=imgsz or self.imgsz, verbose=False)[0]
        return [(r.names[int(c)], float(p), tuple(b))
                for b, c, p in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist())]

    def detect_upright(self, bgr):
        """바로 선 화면에서 찾는다. zoom 이면 버튼판을 잘라 키워 한 번 더 보고 두 결과를 합친다."""
        if not self.zoom:
            return self._predict(bgr)
        full = self._predict(bgr, PANEL_CONF, self.panel_imgsz)
        return merge(full + [d for d in self._zoom_panel(bgr, full) if d[0] in BUTTONS])

    def _zoom_panel(self, bgr, dets):
        """가장 확실한 버튼판을 15% 여백을 두고 잘라 본다. 상자는 전체 화면 좌표로 돌려준다."""
        panels = [d for d in dets if d[0] == 'panel']
        if not panels:
            return []
        _, _, (x1, y1, x2, y2) = max(panels, key=lambda d: d[1])
        h, w = bgr.shape[:2]
        mx, my = (x2 - x1) * 0.15, (y2 - y1) * 0.15
        cx1, cy1 = int(max(0, x1 - mx)), int(max(0, y1 - my))
        cx2, cy2 = int(min(w, x2 + mx)), int(min(h, y2 + my))
        if cx2 - cx1 < 8 or cy2 - cy1 < 8:
            return []
        return [(n, p, (b[0] + cx1, b[1] + cy1, b[2] + cx1, b[3] + cy1))
                for n, p, b in self._predict(bgr[cy1:cy2, cx1:cx2])]

    def detect(self, rgb_raw):
        """원본 RGB → [(이름, 신뢰도, 원본 기준 상자 (x1, y1, x2, y2))]."""
        h, w = rgb_raw.shape[:2]
        upright = np.ascontiguousarray(rgb_raw[::-1, ::-1, ::-1])     # 180도 + RGB→BGR
        return [(n, p, (w - b[2], h - b[3], w - b[0], h - b[1]))
                for n, p, b in self.detect_upright(upright)]


def iou(a, b):
    w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - w * h
    return w * h / union if union > 0 else 0.0


def merge(dets, conf=0.0, overlap=0.5):
    """신뢰도 높은 순으로 받되, 이미 받은 같은 이름 상자와 겹치면(IoU > overlap) 버린다."""
    out = []
    for n, p, b in sorted(dets, key=lambda d: -d[1]):
        if p >= conf and not any(m[0] == n and iou(m[2], b) > overlap for m in out):
            out.append((n, p, b))
    return out


def best_per_class(dets):
    best = {}
    for n, p, b in dets:
        if n in BUTTONS and p >= MIN_CONF and (n not in best or p > best[n][0]):
            best[n] = (p, b)
    return best


def box_depth(depth, box):
    """상자 가운데 절반 안 깊이 중앙값(m). 잰 칸이 MIN_DEPTH_PX 보다 적으면 None."""
    x1, y1, x2, y2 = box
    qx, qy = (x2 - x1) / 4, (y2 - y1) / 4
    d = depth[int(y1 + qy):int(np.ceil(y2 - qy)), int(x1 + qx):int(np.ceil(x2 - qx))]
    d = d[d > 0]
    if d.size < MIN_DEPTH_PX:
        return None
    return float(np.median(d)) / 1000.0


def size_distance(box, k, shape, button_mm=BUTTON_MM):
    """상자 크기로 잰 거리 Z(m). 화면 끝에 걸렸거나 SIZE_MAX_DEG 넘게 비껴 있으면 None."""
    x1, y1, x2, y2 = box
    h, w = shape[:2]
    if x1 < EDGE_PX or y1 < EDGE_PX or x2 > w - EDGE_PX or y2 > h - EDGE_PX:
        return None
    u, v = (x1 + x2) / 2, (y1 + y2) / 2
    off = np.degrees(np.arctan(np.hypot((u - k[2]) / k[0], (v - k[5]) / k[4])))
    if off > SIZE_MAX_DEG:
        return None
    return k[0] * button_mm / 1000.0 / ((x2 - x1 + y2 - y1) / 2)


def button_distance(depth, box, k, button_mm=BUTTON_MM, use_depth=False):
    """(거리 m, 출처). 출처 'button' = 버튼 자리 깊이, 'size' = 상자 크기, 'closer' = 둘이 달라 가까운 쪽.

    기본은 크기로만 잰다(use_depth=False). 거울 같은 스테인리스 판에서 깊이는 비친 것까지의 거리가 섞인다 —
    10-21 실물: 크기로 잰 회차(60.0·61.9 cm)는 버튼 앞으로 갔고, 깊이로 잰 세 회차(63.0·62.8·52.9 cm)는
    옆으로 1.2~22 cm 틀리거나 접근점 화면에 버튼이 없었다. BUTTON_MM 34.5 도 크기 기준으로 맞춘 값이다.
    """
    zd = box_depth(depth, box) if use_depth else None
    zs = size_distance(box, k, depth.shape, button_mm)
    if zd is not None and zs is not None and abs(zd - zs) > AGREE_M:
        return min(zd, zs), 'closer'
    if zd is not None:
        return zd, 'button'
    if zs is not None:
        return zs, 'size'
    return None, None


def to_optical(u, v, z, k):
    """화면 (u, v) 와 거리 z → camera_color_optical_frame 3D (m). k = CameraInfo.k (3x3 를 펼친 9개)."""
    return ((u - k[2]) * z / k[0], (v - k[5]) * z / k[4], z)


def write_json_atomic(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)          # 읽는 쪽이 반쯤 쓴 파일을 보지 않게


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    import tf2_ros
    import cv2

    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--zoom', action='store_true', help='버튼판을 잘라 키워 한 번 더 본다(멀리서 볼 때, 느려짐)')
    ap.add_argument('--button-mm', type=float, default=BUTTON_MM,
                    help='크기로 거리 잴 때 상자 실제 크기 mm (건물마다 버튼이 다르면 바꾼다)')
    ap.add_argument('--use-depth', action='store_true',
                    help='깊이도 쓴다(기본은 버튼 크기로만 — 스테인리스 판에서 깊이가 틀려서)')
    ap.add_argument('--hz', type=float, default=10.0,
                    help='초당 몇 번 찾을지. 10 이면 최근 10 장이 1 초 — press.py 가 관찰 자세에서 그만큼 기다린다')
    args, _ = ap.parse_known_args()

    class ButtonsNode(Node):
        def __init__(self):
            super().__init__('buttons')
            print(f'모델을 불러옵니다: {args.model}')
            self.finder = ButtonFinder(args.model, args.imgsz, zoom=args.zoom)
            self.rgb = self.depth = self.k = None
            self.rgb_n, self.rgb_t, self.used_n, self.stale_print = 0, 0.0, 0, 0.0   # 새 화면만 쓰려고 센다
            self.hist = {}
            self.last_print = 0.0
            self.tf = tf2_ros.Buffer()
            tf2_ros.TransformListener(self.tf, self)
            self.create_subscription(Image, '/camera/color/image_raw', self.on_rgb, qos_profile_sensor_data)
            self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info, 5)
            self.img_pub = self.create_publisher(Image, '/buttons/image', 2)
            self.create_timer(1.0 / args.hz, self.tick)
            print('카메라를 기다립니다')

        def on_rgb(self, m):
            self.rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3] \
                .reshape(m.height, m.width, 3)
            self.rgb_header = m.header
            self.rgb_n, self.rgb_t = self.rgb_n + 1, time.time()

        def on_depth(self, m):
            self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]

        def on_info(self, m):
            self.k = list(m.k)

        def to_world(self, p):
            try:
                t = self.tf.lookup_transform('world', CAM_FRAME, Time())
            except tf2_ros.TransformException:
                return None
            q = t.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            tr = t.transform.translation
            return tuple(r @ np.array(p) + np.array([tr.x, tr.y, tr.z]))

        def tick(self):
            if self.rgb is None or self.depth is None or self.k is None:
                return
            if self.rgb_n == self.used_n:
                # 새 화면이 없다 = 카메라가 멈춤(10-21 저녁 실물: USB EMI 뒤 화면이 그대로 멈춰, 옛 화면으로 낸 버튼 자리를
                # 팔이 눌러 실패). 옛 화면으로는 latest.json 을 쓰지 않는다 → press.py 는 '나이' 로 거부한다
                if time.time() - self.rgb_t > 0.5 and time.time() - self.stale_print > 1.0:
                    self.stale_print = time.time()
                    print(f'  ⚠ 카메라 새 화면 없음 {time.time() - self.rgb_t:.1f}초 — 버튼 위치를 내지 않음(Astra USB 확인)')
                return
            self.used_n = self.rgb_n
            rgb, depth = self.rgb, self.depth
            t0 = time.time()
            dets = self.finder.detect(rgb)
            ms = (time.time() - t0) * 1000
            best = best_per_class(dets)
            now = self.rgb_t                                  # 화면을 받은 시각(창의 since·stamp 도 이 시각)
            for n, (p, b) in best.items():
                u, v = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
                z, src = button_distance(depth, b, self.k, args.button_mm, args.use_depth)
                size_px = (b[2] - b[0] + b[3] - b[1]) / 2
                self.hist.setdefault(n, deque(maxlen=10)).append((now, p, u, v, z, src, size_px))
            for n in list(self.hist):                       # 1초 넘게 안 보이면 잊는다
                if now - self.hist[n][-1][0] > 1.0:
                    del self.hist[n]
            buttons = self.summarize()
            write_json_atomic(os.path.join(HERE, 'latest.json'),
                              {'stamp': now, 'source': 'astra', 'calibrated': True, 'buttons': buttons})
            if now - self.last_print > 1.0:
                self.last_print = now
                self.report(ms, len(dets), buttons)
            if self.img_pub.get_subscription_count():
                self.publish_image(rgb, dets, best)

        def summarize(self):
            """최근 몇 장의 중앙값. 거리를 모르면 z_m·cam·world 는 None."""
            out = {}
            for n, h in self.hist.items():
                p = statistics.median(x[1] for x in h)
                u = statistics.median(x[2] for x in h)
                v = statistics.median(x[3] for x in h)
                zs = [x[4] for x in h if x[4] is not None]
                item = {'conf': round(p, 3), 'u': round(u, 1), 'v': round(v, 1), 'frames': len(h),
                        'since': round(h[0][0], 3),     # 창 안 가장 오래된 장 — 팔이 멈춘 뒤 장만 남았는지 본다
                        'size_px': round(statistics.median(x[6] for x in h), 1),
                        'z_m': None, 'z_src': None, 'cam': None, 'world': None}
                if zs:
                    z = statistics.median(zs)
                    item['z_src'] = statistics.mode(x[5] for x in h if x[4] is not None)
                    cam = to_optical(u, v, z, self.k)
                    w = self.to_world(cam)
                    item.update(z_m=round(z, 4), cam=[round(c, 4) for c in cam],
                                world=[round(float(c), 4) for c in w] if w else None)
                out[n] = item
            return out

        def report(self, ms, n_all, buttons):
            print(f'--- 찾기 {ms:.0f} ms, 상자 {n_all}개')
            for n, b in sorted(buttons.items()):
                line = f'  {n:6s} 신뢰도 {b["conf"]:.2f}  화면(원본) ({b["u"]:5.1f}, {b["v"]:5.1f})'
                if b['z_m'] is None:
                    print(line + f'  거리 모름(깊이 0, 크기로도 못 잼 — 화면 끝이거나 {SIZE_MAX_DEG:.0f}° 넘게 비낌)'
                          f'  상자 {b["size_px"]:.0f} px')
                    continue
                c, w = b['cam'], b['world']
                src = {'button': '', 'size': ' (크기로)', 'closer': ' (깊이·크기 달라 가까운 쪽)'}[b['z_src']]
                line += f'  거리 {b["z_m"] * 100:5.1f} cm{src}  카메라 ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})'
                line += f'  world ({w[0]:+.3f}, {w[1]:+.3f}, {w[2]:+.3f})' if w else '  world 모름(팔 꺼짐)'
                print(line)

        def publish_image(self, rgb, dets, best):
            h, w = rgb.shape[:2]
            img = np.ascontiguousarray(rgb[::-1, ::-1, ::-1])     # 바로 선 BGR 에 그린다
            for n, p, b in dets:
                x1, y1, x2, y2 = w - b[2], h - b[3], w - b[0], h - b[1]
                c = COLORS.get(n, (255, 255, 255)) if (n == 'panel' or n in best) else (90, 90, 90)
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), c, 2)
                cv2.putText(img, f'{n} {p:.2f}', (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
            out = np.ascontiguousarray(img[::-1, ::-1, ::-1])    # 다시 원본 방향 RGB
            msg = Image(header=self.rgb_header, height=h, width=w, encoding='rgb8', step=w * 3,
                        data=out.tobytes())
            self.img_pub.publish(msg)

    rclpy.init()
    n = ButtonsNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
