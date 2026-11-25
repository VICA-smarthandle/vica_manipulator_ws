#!/usr/bin/env python3
"""NX(주행 보드)가 부르는 팔 서비스. 팔 도메인 77 에 떠 있고, NX 는 명령 한 줄만 77 로 보낸다.

  bash run/service.sh       나노에서 켜 둔다 (run/arm.sh·run/moveit.sh·run/buttons.sh 가 먼저 떠 있어야 한다)

NX 에서 (주행 프로그램은 7 에 그대로 두고 이 한 줄만 77 로 간다)
  ROS_DOMAIN_ID=77 ros2 service call /arm/status std_srvs/srv/Trigger       안 움직임. 연결·팔 상태 확인
  ROS_DOMAIN_ID=77 ros2 service call /arm/press_up std_srvs/srv/Trigger     관찰 자세 → 찾기 → 누르기 → 홈
  ROS_DOMAIN_ID=77 ros2 service call /arm/press_down std_srvs/srv/Trigger
  ROS_DOMAIN_ID=77 ros2 service call /arm/press_f3 std_srvs/srv/Trigger     엘베 안 층 버튼(f1~f4). 오른쪽 관찰 자세
  ROS_DOMAIN_ID=77 ros2 service call /arm/home std_srvs/srv/Trigger
  ROS_DOMAIN_ID=77 ros2 service call /arm/demo std_srvs/srv/Trigger         시연 한 번(카메라로 안 찾고 저장한 버튼 자리로)

/arm/demo(11-25 사용자: NX 앱 실행 버튼으로 시연) = run/demo_hand.sh 의 누르기 한 번과 같다:
press.py <demo.service_target> --observe --yes --blind --no-path-guard. 관찰 자세 → 저장한 버튼 자리로 뻗기 → 홈.
손 인식(hand_guard)으로만 멈추고 깊이 길목 신호등은 끈다(앞에 벽이 없는 시연장). 버튼판이 없어 '닿음 없음·불 안 켜짐'
으로 끝나도 끝까지 뻗고 홈에 왔으면 시연 성공으로 답한다. 손이 오래 있어 그만뒀거나 계획 실패·비상정지면 실패.

관찰 자세·높이 보정·누름 깊이·닿음 기준·속도는 config/press.yaml(hall = up·down, floor = f1~f4)을 pose.py·press.py 가
읽는다. 누르는 동안 버튼에 빨간불이 들어오면 바로 멈추고(먼저), 불을 못 보면 전류로 닿음을 본다.
답은 'down 누름 완료(불 켜짐 확인) → 홈' 또는 '…(전류로 닿음, 불 확인 못 함)'(카메라가 없을 때·층 버튼).
닿음이 없으면 '닿음 감지 없음', 호출 버튼이 닿았는데 불이 끝내 안 켜지면 '닿았지만 불이 안 켜짐' 실패로 답한다. 층 버튼은 두 번 누르면 취소되니
여기서 다시 누르지 않는다(NX 조율 #5 — 다시 누를지는 NX 가 엘베 출발을 보고 정한다).

NX 앱이 기대는 약속(11-25 NX 공유) — 바꾸면 NX 중계(arm_relay_node.py)·앱 버튼이 깨진다:
  · 서비스 이름·형식: 도메인 77 /arm/demo·/arm/home·/arm/status (std_srvs/Trigger)
  · NX 중계 대기: /arm/demo 180 s, /arm/home 60 s, /arm/status 5 s. 넘으면 앱에 '시간 초과'
  · message 는 앱 화면에 그대로 나간다. '홈 복귀 실패' 글자가 있어야 앱에 [홈으로](→ /arm/home)가 뜬다
  · 비상정지는 도메인 7 /estop_state 를 랜선으로 듣는다(NX 20 Hz)

한 번에 하나만 받는다(돌고 있으면 거절). 동작은 손으로 시험한 press.py 를 --observe --yes 로 그대로 부른다
(관찰 자세 → 찾기 → 누르기 → 홈). 비상정지 홈·/arm/home 은 pose.py.
응답은 성공 여부와 한 줄 설명이다. 버튼을 못 찾거나 중간에 실패하면 손끝을 6 cm 뺀 뒤 홈으로 가고,
홈에 실제로 갔는지까지 message 에 적는다(NX 조율 #7, 10-18):
  "up 누름 완료 → 홈 (NN초)"  /  "up 실패: <이유> → 홈 도착"
  "up 실패: <이유> → 홈 복귀 실패: <이유> — 팔이 펴져 있을 수 있음"

비상정지(NX 조율 #8, 사용자 결정 10-18): 주행 도메인(기본 7)의 /estop_state 를 듣기만 한다(쓰지 않는다).
비상정지가 켜져 있고 팔이 홈이 아니면 지금 동작을 멈추고(MoveIt 에 "stop") 6 cm 뺀 뒤 평소 속도로 홈.
펴진 채 멈춰 버티면 부하로 모터 과부하 보호가 걸려 힘이 풀릴 수 있어서다(10-18 팔꿈치 과부하).
비상정지 중에는 누르기 요청을 거절한다. 시험할 때는 VICA_ESTOP_DOMAIN=79 처럼 다른 도메인을 듣게 한다.
신호가 안 들리면('모름') 요청은 거절하고, 작업 중에 끊기면 비상정지와 똑같이 저속 홈(사용자 결정 10-18).
팔은 NX 요청으로만 움직이니 평소엔 '신호 끊김 = NX 와 끊김 = 요청 없음' 이지만, NX 비상정지 프로그램만
죽은 경우와 작업 도중 랜선이 빠진 경우를 막는다. 팔만 따로 시험할 때는 VICA_ESTOP_REQUIRED=0.
아직 허가·하트비트(주행 보드와 주고받는 안전 약속)는 없다. 사람이 지켜보는 시험용이다.
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import SetBool, Trigger
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
from press import HOME, load_profile  # noqa: E402

FLOORS = ['f1', 'f2', 'f3', 'f4']   # /arm/press_f1~f4 (NX 조율 #3). open·close 는 press.py 로만
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
ESTOP_DOMAIN = int(os.environ.get('VICA_ESTOP_DOMAIN', '7'))
ESTOP_TOPIC = '/estop_state'   # vica_safety emergency_stop_node 가 주기적으로 내는 래치 상태(Bool, 눌림 = True)
ESTOP_STALE_SEC = 2.0          # 이만큼 안 들리면 '모름'
ESTOP_WAIT_SEC = 3.0           # 11-25 사용자: 요청이 왔는데 '모름'이면 바로 거절하지 않고 이만큼 신호를 기다린다
                               # (나노를 막 켠 직후 NX 노드를 아직 못 찾았거나, NX safety 리셋 중 잠깐 끊긴 때)
# 비상정지 홈 속도 배율 = 평소 홈(pose.py 0.1)과 같다. 10-18 실물로 절반(0.05: 빼기 1.5 s·홈 6.6 s, 전류 최대
# 어깨 155·팔꿈치 −129)과 평소(빼기 1.1 s·홈 3.8 s, 213·−165)를 보고 사용자가 평소 속도로 정했다
ESTOP_VEL = 0.1
ESTOP_REQUIRED = os.environ.get('VICA_ESTOP_REQUIRED', '1') != '0'
HOME_TOL = 0.1                 # rad. 모든 관절이 홈과 이만큼 안이면 '홈' (pose.py NEAR_HOME 과 같음)
MY_FLOOR_GRACE = 2.0           # /arm/my_floor: 문이 열려도 불이 아직 켜져 있으면 이만큼 더 본다(꺼짐과 열림이 거의 같은 때)
WATCH_START_SEC = 120          # 엘베 안 누르기: 이 안에 '지켜보기 시작' 이 없으면 끊는다
# 11-25 사용자(모터 발열·과부하 걱정): 쉬는 동안은 홈(지지대 위)에서 힘을 뺀다. 명령이 오면 힘을 켜고 홈 → 동작 → 홈 → 다시 뺀다.
# 홈은 어깨·팔꿈치가 지지대보다 1도 위라 힘을 켜 두면 두 모터가 팔 무게를 계속 든다(11-25 홈 15분: 팔꿈치 42 ℃).
# 힘을 빼면 손목(막대)이 아래로 떨어지고, 켜면 홈으로 다시 올린다. 끄려면 VICA_IDLE_TORQUE_OFF=0
IDLE_TORQUE_OFF = os.environ.get('VICA_IDLE_TORQUE_OFF', '1') != '0'
TORQUE_SRV = '/dynamixel_hardware_interface/set_dxl_torque'
WAKE_TOL = 0.03                # rad. 깨운 뒤 이 안이면 홈에 온 것으로 본다. 아니면 MoveIt 으로 홈(pose.py)
# 11-25 실물: 그냥 힘을 켜면 모터가 마지막 목표(홈)로 한 번에 뛰어 손목이 0.3 s 에 35도(≈2 rad/s) 올라가며 14번이 심하게 떨렸다.
# → 켜기 전에 궤적 컨트롤러 목표를 지금(떨어진) 자세로 옮겨 두고(힘이 없어 안 움직임), 켠 뒤 홈까지 곧은 궤적으로 천천히 올린다.
WAKE_VEL = 0.4                 # rad/s. 깨울 때 가장 많이 도는 관절의 평균 속도(손목 35도 ≈ 1.5 s)
WAKE_MIN_SEC = 1.5
JTC_ACTION = '/arm_controller/follow_joint_trajectory'
DEMO_ARGS = ['--observe', '--yes', '--blind', '--no-path-guard']   # /arm/demo = run/demo_hand.sh 와 같은 옵션
DEMO_REACHED = ('누름 완료', '닿았지만 불이 안 켜짐', '닿음 감지 없음')   # press.py 마지막 줄 — 누름점까지 뻗었다
DEMO_TIMEOUT = 180             # 손 때문에 멈췄다 이어가면(최대 guard.wait_sec 20 s) 평소 24 s 보다 길어진다


class EstopListener:
    """주행 도메인의 /estop_state 를 듣기만 하는 별도 통로(context). 팔 노드는 77 에 그대로 있다."""

    def __init__(self, domain):
        self.state, self.stamp = None, 0.0
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=domain)
        self.node = rclpy.create_node('arm_estop_listener', context=self.ctx)
        self.node.create_subscription(Bool, ESTOP_TOPIC, self._cb, 10)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            self.ex.spin()
        except Exception:                  # 끌 때(Ctrl+C·stop.sh) 통로가 먼저 닫히며 나는 예외 — 조용히 끝낸다
            pass

    def _cb(self, m):
        self.state, self.stamp = bool(m.data), time.time()

    def on(self):
        """True = 눌림, False = 풀림, None = 안 들림(주행 프로그램 꺼짐·랜선 빠짐)."""
        return self.state if time.time() - self.stamp < ESTOP_STALE_SEC else None

    def text(self):
        return {True: '켜짐', False: '꺼짐', None: '모름'}[self.on()]


class ArmService(Node):
    def __init__(self):
        super().__init__('arm_service')
        cb = ReentrantCallbackGroup()
        self.busy = threading.Lock()
        self.doing = ''
        self.cur = None
        self.child = None                  # 지금 돌고 있는 도구 프로세스(비상정지 때 끊는다)
        self.child_lock = threading.Lock()
        self.estop_abort = False           # 비상정지로 지금 작업을 끊었다
        self.estop = EstopListener(ESTOP_DOMAIN)
        self.torque_off = False            # 쉬는 동안 힘을 뺐다(홈, 지지대 위)
        self.torque_cli = self.create_client(SetBool, TORQUE_SRV, callback_group=cb)
        self.jtc = ActionClient(self, FollowJointTrajectory, JTC_ACTION, callback_group=cb)
        self.ride = {}                     # 엘베 안 지켜보기 {target, state: pressing·watching·arrived·home·stopped, msg}
        self.ride_done = threading.Event()
        self.ride_pub = self.create_publisher(String, '/arm/ride', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))       # 늦게 붙은 NX 도 마지막 상태를 받는다
        self.stop_pub = self.create_publisher(String, '/trajectory_execution_event', 10)   # MoveIt "stop"
        self.create_timer(0.2, self.watch_estop, callback_group=cb)
        self.create_subscription(JointState, '/joint_states', self.js, 10, callback_group=cb)
        self.create_service(Trigger, '/arm/status', self.status, callback_group=cb)
        self.create_service(Trigger, '/arm/home', self.home, callback_group=cb)
        self.create_service(Trigger, '/arm/demo', self.demo, callback_group=cb)
        self.create_service(Trigger, '/arm/my_floor', self.my_floor, callback_group=cb)
        self.create_service(Trigger, '/arm/press_up', lambda q, r: self.press('up', r), callback_group=cb)
        self.create_service(Trigger, '/arm/press_down', lambda q, r: self.press('down', r), callback_group=cb)
        for f in FLOORS:
            self.create_service(Trigger, f'/arm/press_{f}', lambda q, r, f=f: self.press(f, r), callback_group=cb)
        if IDLE_TORQUE_OFF:
            threading.Thread(target=self._relax_at_start, daemon=True).start()
        print('팔 서비스 준비: /arm/status /arm/home /arm/demo /arm/press_up /arm/press_down /arm/press_f1~f4 /arm/my_floor '
              f'(도메인 {os.environ.get("ROS_DOMAIN_ID", "?")}), '
              f'비상정지는 도메인 {ESTOP_DOMAIN} {ESTOP_TOPIC} 를 듣는다', flush=True)

    def run(self, args, timeout):
        """도구 하나를 돌리고 (성공, 마지막 줄) 을 돌려준다. 출력은 그대로 이 터미널에도 찍는다."""
        cmd = [sys.executable, '-u', os.path.join(TOOLS, args[0])] + args[1:]
        print('$', ' '.join(cmd[2:]), flush=True)
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        with self.child_lock:
            self.child = p
        try:
            out, _ = p.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.kill()
            p.communicate()
            return False, f'{args[0]} 가 {timeout}초 안에 끝나지 않음'
        finally:
            with self.child_lock:
                self.child = None
        lines = [ln for ln in out.splitlines() if ln.strip() and 'Warn' not in ln]
        for ln in lines:
            print('  |', ln, flush=True)
        return p.returncode == 0, (lines[-1].strip() if lines else '')

    def away_from_home(self):
        if self.torque_off:                # 홈에서 힘을 빼 지지대에 얹혀 있다(손목만 떨어져 있음) = 홈으로 본다
            return False
        return self.cur is not None and self.home_err() > HOME_TOL

    def home_err(self):
        return max(abs(c - h) for c, h in zip(self.cur, HOME)) if self.cur else float('inf')

    def set_torque(self, on):
        """팔 드라이버의 힘 켜기·끄기 서비스. (성공, 이유). 가짜 팔처럼 서비스가 없으면 (None, 이유)."""
        if not self.torque_cli.wait_for_service(timeout_sec=1.0):
            return None, f'{TORQUE_SRV} 없음(가짜 팔?)'
        f = self.torque_cli.call_async(SetBool.Request(data=on))
        t = time.time()
        while not f.done() and time.time() - t < 3.0:
            time.sleep(0.02)
        if not f.done():
            return False, '힘 켜기·끄기 답 없음'
        return True, f.result().message

    def relax(self, why):
        """홈에 있으면 힘을 뺀다. 홈이 아니면(펴진 채) 빼지 않는다 — 팔이 떨어진다."""
        if not IDLE_TORQUE_OFF or self.torque_off or self.home_err() > HOME_TOL:
            return
        ok, msg = self.set_torque(False)
        if ok:
            self.torque_off = True
            self.get_logger().info(f'힘 뺌({why}) — 홈에서 쉬는 중')
        elif ok is False:
            self.get_logger().warn(f'힘 빼기 실패: {msg} — 힘을 켠 채 기다린다')

    def send_traj(self, target, sec):
        """궤적 컨트롤러에 지금 → target(관절값) 곧은 궤적 하나(끝 속도 0). 끝날 때까지 기다린다. (성공, 이유)."""
        if not self.jtc.wait_for_server(timeout_sec=2.0):
            return False, f'{JTC_ACTION} 없음'
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(JOINTS)
        whole = int(sec)
        g.trajectory.points = [JointTrajectoryPoint(positions=[float(v) for v in target], velocities=[0.0] * len(JOINTS),
                                                    time_from_start=Duration(sec=whole, nanosec=int((sec - whole) * 1e9)))]
        f = self.jtc.send_goal_async(g)
        t = time.time()
        while not f.done() and time.time() - t < 3.0:
            time.sleep(0.02)
        h = f.result() if f.done() else None
        if h is None or not h.accepted:
            return False, '궤적 거절'
        r = h.get_result_async()
        while not r.done() and time.time() - t < sec + 5.0:
            time.sleep(0.02)
        if not r.done():
            return False, '궤적 시간 넘김'
        code = r.result().result.error_code
        return code == FollowJointTrajectory.Result.SUCCESSFUL, f'궤적 결과 {code}'

    def wake(self):
        """쉬던 팔에 힘을 켜고 홈으로 천천히. (True, '') 또는 (False, 이유)."""
        if not self.torque_off:
            return True, ''
        here = list(self.cur)
        ok, msg = self.send_traj(here, 0.1)   # 힘이 꺼진 채 목표를 지금 자세로 → 켤 때 안 뛴다
        if not ok:
            self.get_logger().warn(f'켜기 전 목표 맞추기 실패({msg}) — 그대로 켠다')
        ok, msg = self.set_torque(True)
        if not ok:
            return False, f'힘을 못 켬: {msg}'
        self.torque_off = False
        t = time.time()
        far = max(abs(c - h) for c, h in zip(self.cur, HOME))
        sec = max(WAKE_MIN_SEC, far / WAKE_VEL)
        ok, msg = self.send_traj(HOME, sec)
        err = self.home_err()
        self.get_logger().info(f'힘 켬 → 홈까지 {sec:.1f}초 궤적({"" if ok else msg + ", "}'
                               f'{time.time() - t:.1f}초 뒤 홈과 최대 {math.degrees(err):.1f}도)')
        if err > WAKE_TOL:                 # 홈에 안 왔다 → MoveIt 으로 천천히 홈
            ok, last = self.run(['pose.py', 'home', '--yes'], 60)
            if not ok:
                return False, f'힘을 켠 뒤 홈으로 못 감: {last}'
        return True, ''

    def _relax_at_start(self):
        """서비스가 뜬 뒤(all.sh 는 홈 이동이 끝난 뒤 띄운다) 홈이면 힘을 뺀다."""
        t = time.time()
        while self.cur is None and time.time() - t < 10:
            time.sleep(0.2)
        time.sleep(1.0)
        if self.busy.acquire(blocking=False):
            try:
                self.relax('서비스 시작')
            finally:
                self.busy.release()

    def watch_estop(self):
        """0.2 s 마다. 비상정지가 켜져 있고 팔이 홈이 아니면 멈추고 저속으로 홈. 작업 중 신호가 끊겨도 같다."""
        on = self.estop.on()
        lost = on is None and ESTOP_REQUIRED
        if on is not True and not (lost and self.doing):
            return
        if self.doing and self.doing != '비상정지 홈' and not self.estop_abort:
            self.estop_abort = '비상정지 신호 끊김' if lost else '비상정지'
            self.get_logger().warn(f'{self.estop_abort} — "{self.doing}" 을 멈추고 저속으로 홈')
            self.stop_pub.publish(String(data='stop'))
            with self.child_lock:
                if self.child:
                    self.child.send_signal(signal.SIGINT)
        elif not self.doing and self.away_from_home():
            threading.Thread(target=self._estop_home_idle, daemon=True).start()

    def estop_home(self):
        ok, last = self.run(['pose.py', 'home', '--retract', '--yes', '--vel', str(ESTOP_VEL)], 150)
        return ok, ('홈 도착' if ok else f'홈 복귀 실패: {last} — 팔이 펴져 있을 수 있음')

    def _estop_home_idle(self):
        if not self.busy.acquire(blocking=False):
            return
        try:
            self.doing = '비상정지 홈'
            self.get_logger().warn('비상정지 중 팔이 홈이 아님 — 멈추고 저속으로 홈')
            self.stop_pub.publish(String(data='stop'))    # 서비스 밖(pose.py 를 손으로 등)에서 움직이는 중일 수 있다
            time.sleep(0.5)
            _, tail = self.estop_home()
            self.get_logger().warn(f'비상정지 홈: {tail}')
        finally:
            self.doing = ''
            try:
                self.relax('비상정지 홈 끝')
            finally:
                self.busy.release()

    def js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.cur = [d[j] for j in JOINTS]

    def status(self, req, res):
        res.success = self.cur is not None
        deg = ', '.join(f'{math.degrees(v):.1f}' for v in self.cur) if self.cur else '없음(run/arm.sh 꺼짐?)'
        res.message = (f'관절(도) [{deg}] / {"작업 중: " + self.doing if self.doing else "대기 중"}'
                       f' / 비상정지 {self.estop.text()}(도메인 {ESTOP_DOMAIN})'
                       + (' / 힘 꺼짐(홈에서 쉬는 중)' if self.torque_off else '')
                       + (f' / 엘베 안 {self.ride["target"]} {self.ride["state"]}' if self.ride else ''))
        return res

    def _start(self, what, res, allow_estop=False):
        on = self.estop.on()
        if on is None and ESTOP_REQUIRED and not allow_estop:
            t = time.time()
            while on is None and time.time() - t < ESTOP_WAIT_SEC:
                time.sleep(0.05)
                on = self.estop.on()
            if on is not None:
                self.get_logger().info(f'비상정지 신호를 {time.time() - t:.1f}초 기다려 들음({self.estop.text()})')
        if on is True and not allow_estop:
            res.success, res.message = False, '거절: 비상정지 중'
            return False
        if on is None and ESTOP_REQUIRED and not allow_estop:
            res.success, res.message = False, (f'거절: 비상정지 신호를 {ESTOP_WAIT_SEC:.0f}초 기다려도 못 들음(도메인 {ESTOP_DOMAIN} {ESTOP_TOPIC}). '
                                               'NX 비상정지 프로그램·랜선을 확인하세요')
            return False
        if not self.busy.acquire(blocking=False):
            res.success, res.message = False, f'거절: 이미 작업 중({self.doing})'
            return False
        if self.cur is None:
            self.busy.release()
            res.success, res.message = False, '거절: /joint_states 없음(run/arm.sh 꺼짐?)'
            return False
        self.doing = what
        self.get_logger().info(f'요청 받음: {what}')
        ok, why = self.wake()
        if not ok:
            self.doing = ''
            self.busy.release()
            res.success, res.message = False, f'거절: {why}'
            self.get_logger().warn(res.message)
            return False
        return True

    def _end(self):
        self.doing = ''
        self.estop_abort = False
        try:
            self.relax('작업 끝')
        finally:
            self.busy.release()

    def safe_home(self):
        """손끝을 6 cm 뺀 뒤 홈. message 꼬리('홈 도착' 또는 '홈 복귀 실패: …')와 성공 여부를 돌려준다."""
        ok, last = self.run(['pose.py', 'home', '--retract', '--yes'], 90)
        return ok, ('홈 도착' if ok else f'홈 복귀 실패: {last} — 팔이 펴져 있을 수 있음')

    def home(self, req, res):
        if self.ride.get('state') in ('pressing', 'watching', 'arrived'):
            # 엘베 안 지켜보기 중 — press.py 에 SIGUSR1(그만 보고 홈)을 보내고 그 프로세스가 홈까지 가길 기다린다
            with self.child_lock:
                if self.child:
                    self.child.send_signal(signal.SIGUSR1)
            self.ride_done.wait(timeout=40)
            home = not self.away_from_home()
            res.success = home
            res.message = f'{self.ride["target"]} 지켜보기 그만 → ' + ('홈 도착' if home else '홈 아님 — 팔이 펴져 있을 수 있음')
            return res
        if not self._start('홈', res, allow_estop=True):
            return res
        try:
            res.success, res.message = self.estop_home() if self.estop.on() is not False else self.safe_home()
        finally:
            self._end()
        return res

    def _aborted(self, target, res):
        """비상정지로 끊겼으면 저속 홈까지 하고 True."""
        if not self.estop_abort:
            return False
        _, tail = self.estop_home()
        res.success, res.message = False, f'{target} 중단: {self.estop_abort} → {tail}'
        return True

    def pub_ride(self):
        self.ride_pub.publish(String(data=f'{self.ride["target"]} {self.ride["state"]}'))

    def my_floor(self, req, res):
        """NX 가 엘베 문이 열리면 묻는다 — 여기가 내 층인가. 내 층이면 팔이 홈에 간 뒤 True(내려도 됨)."""
        r = self.ride
        if not r:
            res.success, res.message = False, '엘베 안 지켜보기 없음(층 버튼을 지켜보기로 누른 적 없음)'
            return res
        t0 = time.time()
        while r['state'] in ('pressing', 'watching') and time.time() - t0 < MY_FLOOR_GRACE:
            time.sleep(0.05)
        if r['state'] in ('pressing', 'watching'):
            res.success = False
            res.message = (f'{r["target"]} 아님 — 내 층 버튼 불이 켜져 있음. 계속 지켜봄' if r['state'] == 'watching'
                           else f'{r["target"]} 아직 누르는 중')
            return res
        self.ride_done.wait(timeout=15)    # 불이 꺼져 홈으로 가는 중이면 다 갈 때까지
        if r['state'] == 'home':
            res.success, res.message = True, f'{r["target"]} 내 층(불 꺼짐) → 홈 도착, 내려도 됨'
        else:
            res.success, res.message = False, f'{r["target"]} {r["state"]}: {r.get("msg", "")}'
        return res

    def _reply_press(self, target, ok, last, t0, res):
        """press.py 가 끝난 뒤의 답(호출 버튼·지켜보기 없는 엘베 안). res 를 채운다."""
        if ok:
            if '앞에서 멈춤' in last:               # config/press.yaml depth 가 음수 = 연습(안 누름)
                res.success, res.message = True, f'{target} 연습 완료(앞에서 멈춤, 안 누름) → 홈 ({time.time() - t0:.0f}초)'
                return
            how = '(불 켜짐 확인)' if '불 켜짐' in last else '(전류로 닿음, 불 확인 못 함)' if '전류로' in last else ''
            if '저장한 버튼 자리' in last:          # 카메라로 못 찾아 마지막으로 성공한 자리로 누름(--fallback)
                how += '(카메라로 못 찾아 저장한 버튼 자리로)'
            res.success, res.message = True, f'{target} 누름 완료{how} → 홈 ({time.time() - t0:.0f}초)'
            return
        # 버튼을 못 찾았거나 중간 실패. press.py 가 이미 홈까지 갔으면 다시 보내지 않는다
        # (10-21: 홈인데 또 보내다 장애물 목록 읽기가 비어 '홈 복귀 실패 — 팔이 펴져 있을 수 있음' 거짓 경보)
        tail = self.safe_home()[1] if self.away_from_home() else '홈 도착'
        res.success, res.message = False, f'{target} 실패: {last} → {tail}'

    def press_ride(self, target, res):
        """엘베 안 층 버튼: 누르고 불을 확인하면 '지켜보는 중' 으로 바로 답하고, 그 불이 꺼질 때(= 그 층 도착)까지는
        press.py 가 뒤에서 본다. 꺼지면 press.py 가 홈으로 가고, NX 는 문이 열리면 /arm/my_floor 로 묻는다."""
        if not self._start(f'{target} 누르기', res):
            return res
        t0 = time.time()
        self.ride = {'target': target, 'state': 'pressing', 'msg': ''}
        self.ride_done.clear()
        self.pub_ride()
        cmd = [sys.executable, '-u', os.path.join(TOOLS, 'press.py'), target, '--observe', '--yes', '--max-age', '2',
               '--watch']
        print('$', ' '.join(cmd[2:]), flush=True)
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        with self.child_lock:
            self.child = p
        started = threading.Event()
        timer = threading.Timer(WATCH_START_SEC, lambda: None if started.is_set() or p.poll() is not None else p.kill())
        timer.daemon = True
        timer.start()
        lines = []
        for ln in p.stdout:
            ln = ln.rstrip('\n')
            if not ln.strip() or 'Warn' in ln:
                continue
            print('  |', ln, flush=True)
            lines.append(ln)
            if ln.startswith('지켜보기 시작'):
                started.set()
                break
        if started.is_set() and not self.estop_abort:
            self.ride['state'] = 'watching'
            self.pub_ride()
            self.doing = f'{target} 지켜보기'
            res.success = True
            res.message = f'{target} 누름 완료(불 켜짐 확인) → 접근점에서 내 층 불 지켜보는 중 ({time.time() - t0:.0f}초)'
            self.get_logger().info(f'결과: {res.message}')
            threading.Thread(target=self._ride_rest, args=(target, p, lines), daemon=True).start()
            return res                         # busy 는 _ride_rest 가 놓는다
        try:                                   # 지켜보기 전에 끝남(불을 못 봄·실패·비상정지) = 평소 누르기와 같은 답
            p.wait()
            with self.child_lock:
                self.child = None
            if self._aborted(target, res):
                return res
            self._reply_press(target, p.returncode == 0, lines[-1].strip() if lines else '', t0, res)
            return res
        finally:
            self.ride.update(state='home' if not self.away_from_home() else 'stopped', msg=res.message)
            self.pub_ride()
            self.ride_done.set()
            self.get_logger().info(f'결과: {res.message}')
            self._end()

    def _ride_rest(self, target, p, lines):
        """지켜보는 press.py 가 끝날 때까지 줄을 읽는다. '내 층 불 꺼짐' 이면 arrived, 홈까지 끝나면 home."""
        try:
            for ln in p.stdout:
                ln = ln.rstrip('\n')
                if not ln.strip() or 'Warn' in ln:
                    continue
                print('  |', ln, flush=True)
                lines.append(ln)
                if ln.startswith('내 층 불 꺼짐'):
                    self.ride['state'] = 'arrived'
                    self.pub_ride()
            p.wait()
            with self.child_lock:
                self.child = None
            last = lines[-1].strip() if lines else ''
            if self.estop_abort:
                _, tail = self.estop_home()
                self.ride.update(state='stopped', msg=f'{self.estop_abort} → {tail}')
            elif p.returncode == 0:
                self.ride.update(state='home', msg=last)
            else:
                tail = self.safe_home()[1] if self.away_from_home() else '홈 도착'
                self.ride.update(state='stopped', msg=f'{last} → {tail}')
            self.get_logger().info(f'{target} 지켜보기 끝: {self.ride["state"]} — {self.ride["msg"]}')
        finally:
            self.pub_ride()
            self.ride_done.set()
            self._end()

    def press(self, target, res):
        if target in FLOORS and load_profile(target)[1].get('watch'):
            return self.press_ride(target, res)
        if not self._start(f'{target} 누르기', res):
            return res
        self.ride = {}
        t0 = time.time()
        try:
            # 관찰 자세 → 찾기 → 누르기 → 홈을 한 프로세스로(10-21: pose.py 를 따로 띄우던 시작 시간·고정 2.5 s 대기를 뺐다).
            # 관찰 자세·높이·속도는 config/press.yaml(hall = up·down, floor = f1~f4)
            cmd = ['press.py', target, '--observe', '--yes', '--max-age', '2']
            if load_profile(target)[1].get('service_fallback'):
                cmd.append('--fallback')            # 10-22 사용자: 카메라로 못 찾으면 마지막으로 성공한 버튼 자리로
            ok, last = self.run(cmd, 120)
            if self._aborted(target, res):
                return res
            self._reply_press(target, ok, last, t0, res)
            return res
        finally:
            self.get_logger().info(f'결과: {res.message}')
            self._end()

    def demo(self, req, res):
        """시연 한 번(NX 앱 실행 버튼). run/demo_hand.sh 의 누르기와 같다 — 저장한 버튼 자리로 뻗고 홈."""
        target = str((load_demo_cfg() or {}).get('service_target', 'down'))
        if not self._start(f'시연({target})', res):
            return res
        self.ride = {}
        t0 = time.time()
        try:
            ok, last = self.run(['press.py', target] + DEMO_ARGS, DEMO_TIMEOUT)
            if self._aborted('시연', res):
                return res
            reached = ok or any(s in last for s in DEMO_REACHED)
            tail = self.safe_home()[1] if self.away_from_home() else '홈 도착'
            if reached and tail == '홈 도착':
                touch = ('눌림 확인' if '누름 완료' in last else '닿음, 불 안 켜짐' if '닿았지만' in last
                         else '닿음 없음(버튼판 없음)')
                res.success, res.message = True, f'시연 완료({target}, {touch}) → 홈 ({time.time() - t0:.0f}초)'
            else:
                res.success, res.message = False, f'시연 실패({target}): {last} → {tail}'
            return res
        finally:
            self.get_logger().info(f'결과: {res.message}')
            self._end()


def load_demo_cfg():
    """config/press.yaml 의 demo 묶음(없으면 None)."""
    try:
        import yaml
        with open(os.path.join(TOOLS, '..', 'config', 'press.yaml')) as f:
            return yaml.safe_load(f).get('demo')
    except (OSError, ValueError, AttributeError):
        return None


def main():
    rclpy.init()
    n = ArmService()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
