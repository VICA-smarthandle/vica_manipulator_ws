"""시험용 가짜 Astra(노트북 도메인 78): 찍어 둔 사진 한 장을 손목 카메라처럼 내보낸다.

사진은 바로 선 것(capture.py FLIP)이라 180도 돌려 원본(거꾸로 된) 방향으로 낸다. 깊이는 0(거울 스테인리스와 같음).
깃발 파일이 있으면 press.py 불빛 칸(lit.region, 바로 선 화면 기준)의 막대 양옆에 빨간불을 그린다 = 버튼에 불이 켜짐.
<깃발>.stall 파일이 있으면 화면을 안 낸다 = 카메라가 멈춤(10-21 저녁 실물의 USB EMI 흉내).
  python3 photo_cam.py <사진> <깃발 파일>

노트북 가짜 팔 리허설(도메인 78, 10-21 엘베 안 3층: 오른쪽 관찰 → YOLO 로 f3 찾기 → 누르기 → 3 cm 빼기 → 홈 12 s)
  VICA_ARM_DOMAIN=78 bash run/arm.sh use_mock_hardware:=true
  VICA_ARM_DOMAIN=78 bash run/moveit.sh
  source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=78 python3 tools/photo_cam.py ~/Downloads/elev_astra/20261019_test/images/20261019_test_0523.jpg /tmp/red.flag
  VICA_ARM_DOMAIN=78 bash run/buttons.sh --model /mnt/ssd/vica_arm_kit/models/elev_v4_rf_best.pt
  VICA_ARM_DOMAIN=78 bash run/press.sh f3 --observe --yes      (누르는 중에 touch /tmp/red.flag = 불 켜짐)
가짜 팔은 전류가 0 이라 불을 안 켜면 '닿음 감지 없음' 으로 끝난다.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

photo, flag = sys.argv[1], sys.argv[2]
up = cv2.imread(photo)                                   # 바로 선 BGR
lit = up.copy()
for x1, x2 in ((214, 244), (349, 379)):                  # 10-21 실물 불빛 자리(막대 양옆, y 354~385)
    lit[354:386, x1:x2] = (40, 20, 235)                  # BGR 밝은 빨강
raw = {False: np.ascontiguousarray(cv2.rotate(up, cv2.ROTATE_180)[:, :, ::-1]),
       True: np.ascontiguousarray(cv2.rotate(lit, cv2.ROTATE_180)[:, :, ::-1])}   # 원본 방향 RGB
depth = np.zeros((480, 640), np.uint16)
rclpy.init()
n = rclpy.create_node('photo_astra')
pc = n.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
pd = n.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
pi = n.create_publisher(CameraInfo, '/camera/color/camera_info', 5)
while rclpy.ok():
    if os.path.exists(flag + '.stall'):                  # <깃발>.stall 이 있으면 화면을 안 낸다 = 카메라가 멈춤
        time.sleep(1 / 30)
        continue
    stamp = n.get_clock().now().to_msg()
    img = raw[os.path.exists(flag)]
    m = Image(height=480, width=640, encoding='rgb8', step=640 * 3, data=img.tobytes())
    m.header.frame_id, m.header.stamp = 'camera_color_optical_frame', stamp
    pc.publish(m)
    d = Image(height=480, width=640, encoding='16UC1', step=640 * 2, data=depth.tobytes())
    d.header = m.header
    pd.publish(d)
    info = CameraInfo(width=640, height=480)
    info.header = m.header
    info.k = [510.7, 0.0, 321.5, 0.0, 510.7, 242.1, 0.0, 0.0, 1.0]
    pi.publish(info)
    rclpy.spin_once(n, timeout_sec=0.0)
    time.sleep(1 / 30)
