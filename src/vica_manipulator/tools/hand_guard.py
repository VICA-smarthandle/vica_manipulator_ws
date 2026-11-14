#!/usr/bin/env python3
"""손 인식 신호등. 손목 색 화면에서 사람 손을 찾고(MediaPipe Hands), 가까운 손이면 빨간불을 낸다.

  bash run/hand.sh                     혼자 시험(팔은 꺼 둬도 됨). 확인 화면 bash run/view.sh /hand_guard/image
  bash run/all.sh camera:=true         launch 가 카메라와 함께 띄운다. press.py 가 빨간불이면 어느 단계든 멈춘다

빨간불/초록불은 /hand_guard/detected(Bool, 처리한 장마다), 확인 화면 /hand_guard/image(바로 세운 방향, 늘 갱신).

판정 (장마다)
  손 찾기 : 카메라가 거꾸로 달려 있어 180° 돌려 MediaPipe 에 넣는다(바로 선 손이 더 잘 잡힌다)
  거리    : 손 관절 21점 자리의 깊이(색 화면에 정합) 중앙값. 0 = 못 잼(0.36 m 보다 가까움 등) → 가까운 손으로 본다
  가까운 손: 거리 < hand.max_dist(기본 0.8 m) 또는 못 잼. 11-02 관찰 자세 사진: 1 m 넘게 떨어진 책상 위 손도 0.99 로 잡혀
            거리로 거른다
  막힌 장 hand.on_n 장 연속 → 빨간불, 뚫린 장 hand.off_n 장 연속 → 초록불 (처리 속도 약 6~8 장/s, Orin Nano CPU)

모델: ~/vica_arm_kit/models/hand/hand_landmarker.task (MediaPipe 공식 float16, 11-02 받음).
mediapipe 는 pip --user --no-deps 로 설치(numpy 1.26·cv2 5.0 을 안 바꾸려고). 되돌리기 pip3 uninstall mediapipe
"""
import argparse
import json
import os
import threading
import time

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config", "press.yaml")
MODEL = os.path.expanduser("~/vica_arm_kit/models/hand/hand_landmarker.task")
DEFAULTS = dict(max_dist=0.8, on_n=2, off_n=3, min_score=0.5)
# 손 뼈대(그리기용): 손목 0, 엄지 1-4, 검지 5-8, 중지 9-12, 약지 13-16, 새끼 17-20
BONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
         (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]


def load_cfg():
    try:
        with open(CONFIG) as f:
            return {**DEFAULTS, **(yaml.safe_load(f).get("hand") or {})}
    except OSError:
        return dict(DEFAULTS)


def hand_distance(pts_raw, depth):
    """원본 화면 픽셀 점들 자리의 깊이 중앙값(m). 다 0 이면 None."""
    if depth is None:
        return None
    h, w = depth.shape
    vals = []
    for u, v in pts_raw:
        u, v = int(round(u)), int(round(v))
        if 0 <= u < w and 0 <= v < h:
            patch = depth[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3]
            nz = patch[patch > 0]
            if nz.size:
                vals.append(float(np.median(nz)))
    return float(np.median(vals)) / 1000.0 if vals else None


def main():
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, String

    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--max-dist", type=float, default=None, help="이보다 가까운 손만 빨간불 m (기본 config hand.max_dist)")
    args, _ = ap.parse_known_args()
    cfg = load_cfg()
    if args.max_dist is not None:
        cfg["max_dist"] = args.max_dist
    if not os.path.exists(args.model):
        raise SystemExit(f"손 인식 모델이 없습니다: {args.model}")

    lm = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=args.model), running_mode=vision.RunningMode.VIDEO, num_hands=2,
        min_hand_detection_confidence=cfg["min_score"], min_hand_presence_confidence=cfg["min_score"],
        min_tracking_confidence=0.5))

    class HandNode(Node):
        def __init__(self):
            super().__init__("hand_guard")
            self.lock = threading.Lock()
            self.rgb, self.rgb_n, self.depth = None, 0, None
            self.create_subscription(Image, "/camera/color/image_raw", self.on_rgb, qos_profile_sensor_data)
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)
            self.pub = self.create_publisher(Bool, "/hand_guard/detected", 10)
            self.state_pub = self.create_publisher(String, "/hand_guard/state", 10)
            self.img_pub = self.create_publisher(Image, "/hand_guard/image", 2)

        def on_rgb(self, m):
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
            with self.lock:
                self.rgb = a if m.encoding == "rgb8" else a[:, :, ::-1]    # MediaPipe 는 RGB
                self.rgb_header, self.rgb_n = m.header, self.rgb_n + 1

        def on_depth(self, m):
            d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
            with self.lock:
                self.depth = d

    rclpy.init()
    n = HandNode()
    def spin():
        try:
            rclpy.spin(n)
        except Exception:               # Ctrl+C 로 끌 때 ExternalShutdownException
            pass
    threading.Thread(target=spin, daemon=True).start()
    print(f"손 인식 준비: 가까운 손 = {cfg['max_dist']:.2f} m 안(또는 깊이 못 잼), "
          f"{cfg['on_n']}장 이어지면 빨간불 · {cfg['off_n']}장 비면 초록불. 카메라를 기다립니다")

    red, run, used, t0, last_print, ms_hist = False, 0, 0, time.monotonic(), 0.0, []
    try:
        while rclpy.ok():
            with n.lock:
                fresh = n.rgb is not None and n.rgb_n != used
                if fresh:
                    rgb, depth, header, used = n.rgb, n.depth, n.rgb_header, n.rgb_n
            if not fresh:
                time.sleep(0.005)
                continue
            h, w = rgb.shape[:2]
            up = np.ascontiguousarray(rgb[::-1, ::-1])                  # 180° (바로 세움)
            t1 = time.monotonic()
            res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=up), int((t1 - t0) * 1000))
            ms_hist = (ms_hist + [(time.monotonic() - t1) * 1000])[-30:]
            hands = []
            for marks, hd in zip(res.hand_landmarks, res.handedness):
                pts_up = [(p.x * w, p.y * h) for p in marks]
                pts_raw = [(w - 1 - x, h - 1 - y) for x, y in pts_up]        # 깊이는 원본 방향
                dist = hand_distance(pts_raw, depth)
                close = dist is None or dist < cfg["max_dist"]
                hands.append(dict(score=round(hd[0].score, 2), dist=None if dist is None else round(dist, 2),
                                  close=close, pts=pts_up))
            blocked = any(x["close"] for x in hands)
            if blocked != red:
                run += 1
                if run >= (cfg["on_n"] if blocked else cfg["off_n"]):
                    red, run = blocked, 0
                    desc = ", ".join(f"{x['score']:.2f}/{'?' if x['dist'] is None else x['dist']} m" for x in hands)
                    print(f"{'● 빨간불 — 손' if red else '○ 초록불 — 손 없음'}  (손 {len(hands)}개: {desc or '-'})", flush=True)
            else:
                run = 0
            n.pub.publish(Bool(data=red))
            n.state_pub.publish(String(data=json.dumps(
                {"red": red, "hands": [{k: v for k, v in x.items() if k != "pts"} for x in hands]})))
            now = time.monotonic()
            if now - last_print > 5.0:
                last_print = now
                print(f"  {'빨간불' if red else '초록불'}  손 {len(hands)}개, 처리 {np.median(ms_hist):.0f} ms/장", flush=True)
            if n.img_pub.get_subscription_count():
                img = np.ascontiguousarray(up[:, :, ::-1])               # BGR 로 그리기
                for x in hands:
                    c = (0, 0, 255) if x["close"] else (0, 200, 255)
                    for a, b in BONES:
                        pa, pb = x["pts"][a], x["pts"][b]
                        cv2.line(img, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])), c, 2)
                    xs, ys = [p[0] for p in x["pts"]], [p[1] for p in x["pts"]]
                    label = f"{x['score']:.2f} " + ("<0.36m?" if x["dist"] is None else f"{x['dist']:.2f}m")
                    cv2.putText(img, label, (int(min(xs)), max(int(min(ys)) - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
                col = (0, 0, 255) if red else (0, 200, 0)
                cv2.rectangle(img, (0, 0), (w - 1, h - 1), col, 8)
                cv2.putText(img, "HAND - STOP" if red else "no hand", (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2)
                out = img[:, :, ::-1].tobytes()
                n.img_pub.publish(Image(header=header, height=h, width=w, encoding="rgb8", step=w * 3, data=out))
    except KeyboardInterrupt:
        pass
    finally:
        lm.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
