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
# VICA 2026-10-16 — open_manipulator_bringup/launch/open_manipulator_x.launch.py 를 옮겨
# vica_manipulator 하나만 보도록 줄였다. Gazebo 분기와 prefix 인자는 뺐다.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command (no real arm).',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='Serial port of the OpenRB-150 board.',
        ),
        DeclareLaunchArgument(
            'init_position',
            default_value='true',
            description='Move to the home pose (config/home.yaml) after start',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    port_name = LaunchConfiguration('port_name')
    init_position = LaunchConfiguration('init_position')

    pkg = FindPackageShare('vica_manipulator')
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg, 'urdf', 'vica_manipulator.urdf.xacro']),
        ' ',
        'use_mock_hardware:=',
        use_mock_hardware,
        ' ',
        'port_name:=',
        port_name,
    ])
    # 11-25: 문자열로 못 박는다. 안 하면 launch 가 URDF 를 YAML 로 한 번 읽어, xacro 주석에 '뒤: ' 같은 콜론+빈칸이 있으면
    # 'Unable to parse the value of parameter robot_description as yaml' 로 팔이 안 켜진다(부팅 자동 실행에서 발견)
    robot_description = ParameterValue(urdf_file, value_type=str)

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            PathJoinSubstitution([pkg, 'config', 'controllers.yaml']),
        ],
        output='both',
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', 'joint_state_broadcaster'],
        output='both',
        parameters=[{'robot_description': robot_description}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='both',
    )

    go_home_node = Node(
        package='vica_manipulator',
        executable='go_home.py',
        parameters=[PathJoinSubstitution([pkg, 'config', 'home.yaml'])],
        output='both',
        condition=IfCondition(init_position),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'arm.rviz'])],
        output='both',
        condition=IfCondition(start_rviz),
    )

    # 컨트롤러가 다 뜬 뒤에 RViz 와 홈 이동을 시작한다.
    after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node, go_home_node],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            after_controllers,
        ]
    )
