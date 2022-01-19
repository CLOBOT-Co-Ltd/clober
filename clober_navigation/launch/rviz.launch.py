#!/usr/bin/env python3
#
# Copyright 2022 CLOBOT Co., Ltd
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

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',default='true')
    open_rviz = LaunchConfiguration('open_rviz',default='true')

    rviz_config_dir = os.path.join(get_package_share_directory('clober_navigation'),'rviz','navigation.rviz')

    return LaunchDescription([

        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value=use_sim_time,
        #     description='use simulation time'
        # ),

        # DeclareLaunchArgument(
        #     'open_rviz',
        #     default_value=open_rviz,
        #     description='open rviz'
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time':use_sim_time}],
            condition= IfCondition(LaunchConfiguration("open_rviz"))
        )

    ])