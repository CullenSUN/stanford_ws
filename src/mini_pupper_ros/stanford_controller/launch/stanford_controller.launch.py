#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 MangDang
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    orientation_from_imu = LaunchConfiguration("orientation_from_imu")
    orientation_from_imu_launch_arg = DeclareLaunchArgument(
        name='orientation_from_imu',
        default_value='False',
        description='if use imu for orientation'
    )

    return LaunchDescription([
        orientation_from_imu_launch_arg,
        Node(
            package='stanford_controller',
            executable='twist_to_command_node',
            name='twist_to_command_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='stanford_controller',
            executable='stanford_controller_node',
            name='stanford_controller_node',
            output='screen',
            parameters=[
                {'orientation_from_imu': orientation_from_imu}
            ]
        )
    ])
