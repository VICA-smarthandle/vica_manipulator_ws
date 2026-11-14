#!/usr/bin/env python3
"""웹캠 + YOLO 로 엘리베이터 up/down 버튼을 찾고, 거리와 world 좌표를 추정한다. (호스트에서 실행)

웹캠은 거리를 모른다. 그래서 "버튼이 화면에서 얼마나 크게 보이나"로 거리를 잰다.
멀면 작고 가까우면 크다. 처음 한 번 줄자로 잰 거리에서 크기를 기억시키는 것(--calib)이 보정이다.

  python3 detect.py                  화면 표시 + latest.json 기록 (press.py 가 이 파일을 읽는다)
  python3 detect.py --imgsz 1280     버튼이 작게 보일 때 (학습 때보다 작으면 확신도가 떨어진다)
  python3 detect.py --calib 0.50     버튼판을 카메라에서 줄자로 0.50 m 에 두고 실행 → c 키로 보정 저장

키: q 종료 / s 화면 저장 / c 보정 저장(--calib 일 때만)
"""
import argparse, json, os, statistics, time
from collections import deque

import cv2
import yaml
from ultralytics import YOLO

import geom

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = "/home/msk/runs/segment/runs/elev_v1/weights/best.pt"
TARGETS = ("up", "down")
MIN_CONF = 0.5
DOWN_GATED_CONF = 0.15   # up 바로 아래 자리에 있을 때만 받아 주는 down 하한
PANEL_CONF = 0.3         # 이 이상 확신하는 버튼판이 있으면 잘라서 한 번 더 본다
SMOOTH_N = 15              # 최근 몇 프레임의 중앙값을 쓸지 (10 FPS 면 1.5초. 크기 흔들림 줄이기)
FORGET_SEC = 1.0           # 이 시간 동안 안 보이면 기억을 비운다 (한두 프레임 놓쳐도 유지)
COLORS = {"up": (0, 200, 0), "down": (0, 140, 255), "panel": (160, 160, 160)}


def load_cam():
    with open(os.path.join(HERE, "camera.yaml")) as f:
        cam = yaml.safe_load(f)
    calib_path = os.path.join(HERE, "calib.json")
    cam["fx"] = None
    if os.path.exists(calib_path):
        with open(calib_path) as f:
            cam["fx"] = json.load(f)["fx"]
    cam["fy"] = cam["fx"]
    cam["cx"], cam["cy"] = cam["width"] / 2, cam["height"] / 2
    return cam


def _summary(d):
    """(이름, 신뢰도, 박스) → (신뢰도, 중심u, 중심v, 지름px)"""
    _, conf, (x1, y1, x2, y2) = d
    # 비스듬히 보면 한쪽만 줄어든다 → 긴 쪽이 실제 지름에 가깝다
    return (conf, (x1 + x2) / 2, (y1 + y2) / 2, max(x2 - x1, y2 - y1))


def pick_buttons(dets):
    """up/down 각각 하나씩 고른다.

    up 은 신뢰도 MIN_CONF 이상만. down 은 모델이 자주 자신 없어 하므로,
    "확실한 up 바로 아래, 버튼 1~2.5개 거리, 좌우로 거의 같은 줄"에 있으면 DOWN_GATED_CONF 까지 받아 준다.
    엉뚱한 곳의 낮은 신뢰도 오인식은 이 위치 조건에서 걸러진다.
    """
    best = {}
    ups = [d for d in dets if d[0] == "up" and d[1] >= MIN_CONF]
    if ups:
        best["up"] = _summary(max(ups, key=lambda d: d[1]))
    downs = []
    for d in dets:
        if d[0] != "down":
            continue
        if d[1] >= MIN_CONF:
            downs.append(d)
        elif d[1] >= DOWN_GATED_CONF and "up" in best:
            _, uu, uv, uw = best["up"]
            _, du, dv, _ = _summary(d)
            if 1.0 * uw <= dv - uv <= 2.5 * uw and abs(du - uu) <= 0.5 * uw:
                downs.append(d)
    if downs:
        best["down"] = _summary(max(downs, key=lambda d: d[1]))
    return best


def panel_width(hist):
    """보이는 버튼들의 크기(px) 중앙값을 평균. 하나보다 둘을 같이 쓰면 덜 흔들린다."""
    ws = [statistics.median(h[3] for h in hist[t]) for t in TARGETS if hist[t]]
    return sum(ws) / len(ws) if ws else 0.0


def detect_two_pass(model, frame, imgsz):
    """두 번 본다: 전체 화면에서 버튼판(panel)을 찾고, 그 부분만 잘라 확대해서 버튼을 다시 찾는다.

    버튼판이 멀면 버튼이 학습 사진보다 작게 보여 놓친다. 잘라서 키우면 학습 때 크기에 가까워진다.
    반환: [(이름, 신뢰도, (x1, y1, x2, y2))]  — 좌표는 원본 화면 기준
    """
    res = model.predict(frame, conf=0.25, imgsz=imgsz, verbose=False)[0]
    dets = [(res.names[int(c)], float(p), tuple(b))
            for b, c, p in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist())]
    panels = [d for d in dets if d[0] == "panel" and d[1] >= PANEL_CONF]
    if not panels:
        return dets
    _, _, (x1, y1, x2, y2) = max(panels, key=lambda d: d[1])
    H, W = frame.shape[:2]
    mx, my = (x2 - x1) * 0.15, (y2 - y1) * 0.15
    cx1, cy1 = int(max(0, x1 - mx)), int(max(0, y1 - my))
    cx2, cy2 = int(min(W, x2 + mx)), int(min(H, y2 + my))
    crop = frame[cy1:cy2, cx1:cx2]
    if crop.size == 0:
        return dets
    r2 = model.predict(crop, conf=0.25, imgsz=imgsz, verbose=False)[0]
    inner = [(r2.names[int(c)], float(p), (b[0] + cx1, b[1] + cy1, b[2] + cx1, b[3] + cy1))
             for b, c, p in zip(r2.boxes.xyxy.tolist(), r2.boxes.cls.tolist(), r2.boxes.conf.tolist())
             if r2.names[int(c)] in TARGETS]
    cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), (255, 255, 0), 1)     # 확대해서 본 영역
    # 버튼은 확대해서 본 결과를, 판은 전체 화면 결과를 쓴다
    return [d for d in dets if d[0] not in TARGETS] + inner


def write_json_atomic(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)          # 읽는 쪽이 반쯤 쓴 파일을 보지 않게


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", type=float, help="줄자로 잰 카메라~버튼판 거리 (m)")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="YOLO 입력 크기. 버튼이 작게 보이면 1280 (느려짐)")
    args = ap.parse_args()

    cam = load_cam()
    D = cam["button_diameter_m"]
    if D <= 0:
        print("button_diameter_m 이 비어 있어 인식만 보여 줍니다. 박스가 감싸는 부분을 자로 재서 camera.yaml 에 넣으세요")
        cam["fx"] = None
        if args.calib:
            print("보정은 지름이 있어야 합니다")
            return
    elif cam["fx"] is None and args.calib is None:
        print("아직 보정 전입니다. 거리·좌표 없이 인식만 보여 줍니다. (--calib 0.50 으로 보정)")

    model = YOLO(MODEL)
    cap = cv2.VideoCapture(cam["device"], cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
    if not cap.isOpened():
        print(f"카메라를 열 수 없습니다: {cam['device']}")
        return

    hist = {t: deque(maxlen=SMOOTH_N) for t in TARGETS}
    last_seen = {t: 0.0 for t in TARGETS}
    out_path = os.path.join(HERE, "latest.json")
    t_prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("프레임을 못 읽었습니다")
            break
        now = time.time()
        dets = detect_two_pass(model, frame, args.imgsz)

        # 클래스마다 신뢰도가 가장 높은 것 하나
        for name, conf, (x1, y1, x2, y2) in dets:
            color = COLORS.get(name, (255, 255, 255))
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(frame, f"{name} {conf:.2f}", (int(x1), int(y1) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        best = pick_buttons(dets)

        buttons = {}
        for t in TARGETS:
            if t in best:
                hist[t].append(best[t])
                last_seen[t] = now
            elif now - last_seen[t] > FORGET_SEC:
                hist[t].clear()
        w_panel = panel_width(hist)        # up·down 은 같은 판 위라 거리가 같다 → 크기를 같이 평균

        for t in TARGETS:
            if not hist[t]:
                continue
            conf = statistics.median(h[0] for h in hist[t])
            u = statistics.median(h[1] for h in hist[t])
            v = statistics.median(h[2] for h in hist[t])
            w = statistics.median(h[3] for h in hist[t])
            item = {"conf": round(conf, 3), "u": round(u, 1), "v": round(v, 1), "w_px": round(w, 1),
                    "z_m": None, "world": None, "frames": len(hist[t])}
            if cam["fx"]:
                z = geom.depth_from_size(w_panel, cam["fx"], D) + cam.get("z_offset_m", 0.0)
                xyz = geom.pixel_to_world(u, v, z, cam)
                item["z_m"] = round(z, 4)
                item["world"] = [round(c, 4) for c in xyz]
            buttons[t] = item

            # 화면 표시
            line = f"{t}: w={w:.0f}px"
            if item["z_m"]:
                x, y, zz = item["world"]
                line += f"  Z={item['z_m'] * 100:.1f}cm  world=({x:.3f}, {y:.3f}, {zz:.3f})"
            row = 30 if t == "up" else 60
            cv2.putText(frame, line, (10, row), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[t], 2)
            cv2.drawMarker(frame, (int(u), int(v)), COLORS[t], cv2.MARKER_CROSS, 20, 2)

        write_json_atomic(out_path, {"stamp": now, "image": [frame.shape[1], frame.shape[0]],
                                     "calibrated": bool(cam["fx"]), "buttons": buttons})

        fps = 1.0 / max(now - t_prev, 1e-6)
        t_prev = now
        status = f"{fps:.1f} FPS"
        if args.calib:
            status += f"   CALIB at {args.calib:.2f} m - press c"
        cv2.putText(frame, status, (10, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("buttons", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            p = os.path.join(HERE, f"snap_{time.strftime('%H%M%S')}.jpg")
            cv2.imwrite(p, frame)
            print("저장:", p)
        if key == ord("c") and args.calib:
            seen = [t for t in TARGETS if hist[t]]
            if not seen:
                print("보정 실패: up/down 버튼이 안 보입니다")
                continue
            t = "+".join(seen)
            w = panel_width(hist)
            fx = geom.fx_from_calibration(w, args.calib, D)
            write_json_atomic(os.path.join(HERE, "calib.json"),
                              {"fx": fx, "from": t, "w_px": w, "distance_m": args.calib,
                               "diameter_m": D, "image": [frame.shape[1], frame.shape[0]],
                               "date": time.strftime("%Y-%m-%d %H:%M")})
            cam["fx"] = cam["fy"] = fx
            print(f"보정 저장: fx = {fx:.1f} px  ({t} 버튼 {w:.1f}px 를 {args.calib} m 로)")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
