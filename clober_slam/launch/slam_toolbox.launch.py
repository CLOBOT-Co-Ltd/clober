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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    rviz_config_dir = os.path.join(get_package_share_directory('clober_slam'),'rviz','slam.rviz')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[{get_package_share_directory('clober_slam')+'/config/slam_toolbox_params.yaml'},
            {'use_sim_time':use_sim_time}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time':use_sim_time}],
            arguments=['-d',rviz_config_dir]
        )        
    ])