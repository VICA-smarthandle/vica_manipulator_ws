#!/usr/bin/env python3
"""팔 모터의 에러 기록·온도·전압·전류를 읽기만 한다. (호스트에서 실행, 모터에 쓰지 않음)

ROS bringup 을 끈 뒤(포트가 비어야 함), 12V 는 켠 채로 실행한다.
12V 를 끄면 에러 기록이 지워지므로 그 전에 읽어야 한다.

  python3 dxl_errors.py
"""
from dynamixel_sdk import PortHandler, PacketHandler

PORT, BAUD = "/dev/ttyACM0", 1000000
IDS = {11: "베이스", 12: "어깨", 13: "팔꿈치", 14: "손목", 15: "그리퍼"}
BITS = {0: "입력 전압", 2: "과열", 3: "엔코더", 4: "전기 충격", 5: "과부하"}

port, pk = PortHandler(PORT), PacketHandler(2.0)
if not port.openPort() or not port.setBaudRate(BAUD):
    raise SystemExit(f"{PORT} 를 열 수 없습니다. ROS bringup 이 꺼져 있는지 확인하세요")
print(" ID  부위      에러기록        온도   전압    전류   토크")
for i, name in IDS.items():
    err, r1, _ = pk.read1ByteTxRx(port, i, 70)          # Hardware Error Status
    tmp, r2, _ = pk.read1ByteTxRx(port, i, 146)         # Present Temperature (°C)
    vin, r3, _ = pk.read2ByteTxRx(port, i, 144)         # Present Input Voltage (0.1 V)
    cur, r4, _ = pk.read2ByteTxRx(port, i, 126)         # Present Current (2.69 mA)
    tq, r5, _ = pk.read1ByteTxRx(port, i, 64)           # Torque Enable
    if r1 != 0:
        print(f" {i}  {name:4}  응답 없음")
        continue
    cur = cur - 65536 if cur > 32767 else cur
    why = ", ".join(v for b, v in BITS.items() if err >> b & 1) or "없음"
    print(f" {i}  {name:4}  {why:12}  {tmp:3}°C  {vin / 10:4.1f}V  {cur * 2.69:6.0f}mA   {tq}")
port.closePort()
