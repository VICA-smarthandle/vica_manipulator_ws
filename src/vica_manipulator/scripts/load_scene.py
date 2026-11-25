#!/usr/bin/env python3
"""config/scene.yaml 의 AMR 몸체·짐칸·마스트를 MoveIt planning scene 에 상자로 넣는다.

  ros2 run vica_manipulator load_scene.py            넣기 (moveit.launch.py 가 자동으로 부른다)
  ros2 run vica_manipulator load_scene.py --list     지금 들어 있는 장애물 이름만 본다

scene.yaml 의 숫자는 base_link 기준이다. mount(base_link → 팔 world)를 빼서 world 기준으로 바꾼다.
팔 보드와 주행 보드는 TF 를 잇지 않으므로(URDF 를 합치지 않는다) 여기서 직접 바꾼다.
"""
import argparse
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
import yaml

WAIT_SEC = 60.0
TRIES = 9                # 장애물 넣기 다시 보내기 최대 번수(번마다 10 초까지 기다림)


def to_world(p, mount):
    """base_link 점 p 를 팔 world 로. world 는 base_link 에서 mount 만큼 옮기고 yaw 만큼 돈 자리다."""
    yaw = math.radians(mount.get('yaw_deg', 0.0))
    dx, dy, dz = (p[i] - mount['xyz'][i] for i in range(3))
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy, dz), yaw


def load(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    objs = []
    for b in cfg['boxes']:
        (x, y, z), yaw = to_world(b['center'], cfg['mount'])
        o = CollisionObject()
        o.header.frame_id = 'world'
        o.id = b['name']
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in b['size']])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z, pose.orientation.w = math.sin(-yaw / 2), math.cos(-yaw / 2)
        o.primitives.append(prim)
        o.primitive_poses.append(pose)
        o.operation = CollisionObject.ADD
        objs.append(o)
    return objs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true', help='들어 있는 장애물 이름만 본다')
    ap.add_argument('--file', default=os.path.join(
        get_package_share_directory('vica_manipulator'), 'config', 'scene.yaml'))
    args, _ = ap.parse_known_args()

    rclpy.init()
    n = Node('vica_load_scene')
    if args.list:
        cli = n.create_client(GetPlanningScene, '/get_planning_scene')
        if not cli.wait_for_service(timeout_sec=WAIT_SEC):
            sys.exit('move_group 이 없습니다')
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        names = [o.id for o in fut.result().scene.world.collision_objects]
        print('장애물:', ', '.join(names) if names else '없음')
        return

    objs = load(args.file)
    cli = n.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not cli.wait_for_service(timeout_sec=WAIT_SEC):
        sys.exit(f'{WAIT_SEC:.0f}초 기다려도 move_group 이 없습니다')
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = objs
    # 11-25 부팅 자동 실행: move_group 이 서비스는 열었지만 아직 로딩 중이면(카메라·YOLO·손 인식이 같이 뜰 때) 10 초 안에
    # 답이 없어 '장애물을 넣지 못했습니다' → 팔 서비스가 안 떴다. 답이 없거나 실패면 다시 보낸다(최대 TRIES 번)
    for k in range(1, TRIES + 1):
        fut = cli.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
        if fut.result() is not None and fut.result().success:
            break
        print(f'장애물 넣기 {k}/{TRIES}번째 답 없음·실패 — move_group 이 아직 준비 중일 수 있어 다시 보냅니다', flush=True)
    else:
        sys.exit('장애물을 넣지 못했습니다')
    for o in objs:
        p = o.primitive_poses[0].position
        d = o.primitives[0].dimensions
        print(f'  {o.id:10s} world 중심 ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})'
              f'  크기 {d[0]:.3f} x {d[1]:.3f} x {d[2]:.3f}')
    print(f'장애물 {len(objs)}개를 MoveIt 에 넣었습니다 ({args.file})')


if __name__ == '__main__':
    main()
