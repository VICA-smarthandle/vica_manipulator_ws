#!/usr/bin/env python3
# VICA 2026-11-02 · 11-14 — 나노 팔을 한 번에 띄운다. camera 인자 하나로 두 가지:
#
#   camera:=false (기본) 시연 데모 — 팔(run/arm.sh) + MoveIt(run/moveit.sh) → 홈·장애물이 끝나면
#                        press down --observe --yes --blind 한 번(카메라 없이 저장한 버튼 자리로)
#   camera:=true         NX 명령 대기 — 팔 + MoveIt + 손목 카메라(run/astra.sh) + YOLO 버튼 찾기(run/buttons.sh)
#                        + 손 인식(run/hand.sh) + NX 서비스(run/service.sh). 홈·장애물이 끝나면 서비스가 뜨고,
#                        NX 가 /arm/press_up 등을 부르면 카메라로 버튼을 찾아 누른다(press.py --observe --yes).
#                        손이 보이면 어느 단계든 그 자리에서 멈추고 손이 빠지면 이어간다. 스스로 누르지는 않는다
#
#   bash run/all.sh                                  시연 데모(down 한 번)
#   bash run/all.sh press:=up                        시연 데모, 다른 버튼
#   bash run/all.sh camera:=true                     NX 명령 대기(카메라·버튼 찾기·손 인식·서비스)
#   bash run/all.sh camera:=true service:=false      카메라·버튼 찾기·손 인식까지만(누르기는 따로 run/press.sh)
#   bash run/all.sh press:=none                      팔·MoveIt 만 띄우기
#   bash run/all.sh use_mock_hardware:=true          팔 없이 가짜 하드웨어로
#   확인 화면: bash run/view.sh /hand_guard/image · /buttons/image · /path_guard/image
#
# 서비스·한 번 누르기는 팔이 홈에 간 뒤(go_home 끝)와 MoveIt 장애물을 넣은 뒤(load_scene 끝) 둘 다 끝나야 시작한다.
# 홈 이동이 실패하면 띄우지 않는다. 둘이 같이 팔을 움직이지 않게 서비스가 뜨면 press:= 는 무시한다.
# 누르기가 끝나도 나머지는 계속 떠 있다(Ctrl+C 로 모두 끔).

import os
import shlex

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WS = os.environ.get('VICA_ARM_WS', os.path.expanduser('~/vica_manipulator_ws'))
TOOLS = os.path.join(WS, 'src', 'vica_manipulator', 'tools')
PRESS_PY = os.path.join(TOOLS, 'press.py')
BUTTONS_PY = os.path.join(TOOLS, 'buttons.py')
HAND_PY = os.path.join(TOOLS, 'hand_guard.py')
SERVICE_PY = os.path.join(TOOLS, 'arm_service.py')
OFF = ('', 'none', 'false', '0', 'no')


def _on(value, camera):
    """'auto' 는 camera 를 따른다."""
    v = value.strip().lower()
    return camera if v == 'auto' else v not in OFF


def generate_launch_description():
    args = [
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0', description='OpenRB-150 포트'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false', description='팔 없이 가짜 하드웨어'),
        DeclareLaunchArgument('start_rviz', default_value='false', description='RViz'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='false = 시연 데모(저장 자리로 한 번). true = 카메라·버튼 찾기·손 인식 + NX 서비스'),
        DeclareLaunchArgument('buttons', default_value='auto',
                              description='YOLO 버튼 찾기(run/buttons.sh). auto = camera 를 따름'),
        DeclareLaunchArgument('service', default_value='auto',
                              description='NX 서비스(run/service.sh). auto = camera 를 따름. 홈·장애물 뒤 뜬다'),
        DeclareLaunchArgument('press', default_value='down',
                              description='서비스가 없을 때 홈·장애물 뒤 한 번 누를 버튼(up·down·f1~f4·open·close). none = 안 누름'),
        DeclareLaunchArgument('press_args', default_value='--observe --yes --blind',
                              description='press.py 에 넘길 옵션(--yes 가 없으면 붙인다 — launch 에서는 Enter 를 못 친다)'),
    ]
    pkg = FindPackageShare('vica_manipulator')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    # 팔(ros2_control). 홈 이동(go_home)은 여기서 따로 띄워 끝나는 때를 안다
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'arm.launch.py'])),
        launch_arguments={'port_name': LaunchConfiguration('port_name'),
                          'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
                          'start_rviz': LaunchConfiguration('start_rviz'),
                          'init_position': 'false'}.items())
    go_home = Node(package='vica_manipulator', executable='go_home.py', output='both',
                   parameters=[PathJoinSubstitution([pkg, 'config', 'home.yaml'])])   # 컨트롤러가 뜰 때까지 스스로 기다린다

    # MoveIt. 장애물 넣기(load_scene)도 여기서 따로 띄워 끝나는 때를 안다
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'moveit.launch.py'])),
        launch_arguments={'start_rviz': 'false', 'load_scene': 'false'}.items())
    scene = Node(package='vica_manipulator', executable='load_scene.py', output='screen')

    # 손목 카메라 — run/astra.sh 와 같은 인자
    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('astra_camera'), 'launch',
                                                         'astra.launch.xml'])),
        launch_arguments={'publish_tf': 'false', 'depth_registration': 'true',
                          'color_depth_synchronization': 'true', 'enable_ir': 'false',
                          'enable_point_cloud': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('camera')))

    # 손 인식 — camera:=true 일 때만
    hand = ExecuteProcess(cmd=['python3', '-u', HAND_PY], name='hand_guard', output='screen', emulate_tty=True,
                          condition=IfCondition(LaunchConfiguration('camera')))

    def setup(context):
        cam = LaunchConfiguration('camera').perform(context).strip().lower() not in OFF
        out = []

        # YOLO 버튼 찾기 — run/buttons.sh 와 같다. 카메라 화면을 기다렸다가 돈다
        if _on(LaunchConfiguration('buttons').perform(context), cam):
            if not cam:
                out.append(LogInfo(msg='[all] buttons:=true 인데 camera:=false — 다른 곳에서 카메라가 떠 있어야 찾습니다'))
            out.append(ExecuteProcess(cmd=['python3', '-u', BUTTONS_PY], name='buttons', output='screen',
                                      emulate_tty=True))

        # 홈·장애물 뒤에 띄울 것: NX 서비스, 또는 한 번 누르기
        later = []
        if _on(LaunchConfiguration('service').perform(context), cam):
            later.append(('NX 서비스(arm_service.py) — NX 명령을 기다립니다',
                          ExecuteProcess(cmd=['python3', '-u', SERVICE_PY], name='arm_service', output='screen',
                                         emulate_tty=True)))
        else:
            target = LaunchConfiguration('press').perform(context).strip()
            if target.lower() not in OFF:
                opts = shlex.split(LaunchConfiguration('press_args').perform(context))
                if '--yes' not in opts and '--dry' not in opts:
                    opts.append('--yes')
                later.append((f'press.py {target} {" ".join(opts)}',
                              ExecuteProcess(cmd=['python3', '-u', PRESS_PY, target] + opts, name='press',
                                             output='screen', emulate_tty=True)))
        if not later:
            out.append(LogInfo(msg='[all] 서비스·누르기 없이 띄우기만 합니다'))
            return out

        done = {}

        def on_exit(event, _context):
            what = '홈 이동(go_home)' if event.action is go_home else '장애물 넣기(load_scene)'
            done[what] = event.returncode
            if event.returncode != 0:
                return [LogInfo(msg=f'[all] {what} 실패(종료 코드 {event.returncode}) — 서비스·누르기를 띄우지 않습니다')]
            if len(done) < 2 or any(done.values()):
                return []
            acts = []
            for msg, act in later:
                acts += [LogInfo(msg=f'[all] 홈·장애물 끝 → {msg}'), act]
            return acts

        out += [RegisterEventHandler(OnProcessExit(target_action=go_home, on_exit=on_exit)),
                RegisterEventHandler(OnProcessExit(target_action=scene, on_exit=on_exit))]
        return out

    return LaunchDescription(args + [OpaqueFunction(function=setup), arm, go_home, moveit, scene, camera, hand])
