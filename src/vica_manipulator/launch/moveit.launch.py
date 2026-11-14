#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# VICA 2026-10-16 — open_manipulator_moveit_config/launch/open_manipulator_x_moveit.launch.py 를
# 옮겼다. 이 패키지는 Humble 전용이라 계획기(ompl·pilz·chomp) 설정은 따로 두지 않고
# moveit_configs_utils 의 Humble 기본값을 그대로 쓴다(Jazzy 꼴 설정으로 move_group 이 죽던
# 문제가 애초에 생기지 않는다).

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'load_scene', default_value='true',
            description='Add AMR body, cargo bin and mast from config/scene.yaml',
        ),
    ]
    start_rviz = LaunchConfiguration('start_rviz')
    load_scene = LaunchConfiguration('load_scene')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='vica_manipulator', package_name='vica_manipulator')
        .robot_description(file_path=str(Path('urdf') / 'vica_manipulator.urdf.xacro'))
        .robot_description_semantic(str(Path('config') / 'vica_manipulator.srdf'))
        .robot_description_kinematics(str(Path('config') / 'kinematics.yaml'))
        .joint_limits(str(Path('config') / 'joint_limits.yaml'))
        .trajectory_execution(str(Path('config') / 'moveit_controllers.yaml'))
        .pilz_cartesian_limits(str(Path('config') / 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # VICA 2026-10-16 — move_group 이 뜨면 AMR 몸체·짐칸·마스트를 장애물로 넣는다.
    # load_scene.py 가 /apply_planning_scene 이 생길 때까지 기다렸다가 넣고 끝난다.
    scene_node = Node(
        package='vica_manipulator',
        executable='load_scene.py',
        output='screen',
        condition=IfCondition(load_scene),
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node, scene_node])
