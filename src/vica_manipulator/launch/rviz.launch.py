#!/usr/bin/env python3
# VICA 2026-10-18 — 보기 전용 RViz. 로봇·장애물(차체·짐칸·마스트)·MoveIt 이 세운 경로·버튼 표시만 보여 준다.
# MotionPlanning 패널을 넣지 않아 RViz 에서 실수로 팔을 움직일 수 없다.
# 경로(Trajectory) 표시가 로봇 모양을 알아야 해서 MoveIt 설정을 파라미터로 넘긴다(move_group 은 띄우지 않는다).

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name='vica_manipulator', package_name='vica_manipulator')
        .robot_description(file_path=str(Path('urdf') / 'vica_manipulator.urdf.xacro'))
        .robot_description_semantic(str(Path('config') / 'vica_manipulator.srdf'))
        .robot_description_kinematics(str(Path('config') / 'kinematics.yaml'))
        .to_moveit_configs()
    )
    rviz_config = str(Path(get_package_share_directory('vica_manipulator')) / 'rviz' / 'view.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_view',
            output='log',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        ),
    ])
