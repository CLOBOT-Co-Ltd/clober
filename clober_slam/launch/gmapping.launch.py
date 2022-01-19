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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory('clober_slam'),'rviz','slam.rviz')

    remappings = [('/tf','tf'),
                  ('/tf_static','tf_static')]

    use_sim_time = LaunchConfiguration('use_sim_time',default='true')

    params = {
        'use_sim_time': use_sim_time,
        'scan_topic': 'scan',
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'map_update_interval': 3.0,
        'maxUrange': 10.0,
        'sigma': 0.05,
        'kernelSize': 1,
        'lstep': 0.05,
        'astep': 0.05,
        'iterations': 5,
        'lsigma': 0.075,
        'ogain': 3.0,
        'lskip': 0,
        'srr': 0.1,
        'srt': 0.2,
        'str': 0.1,
        'stt': 0.2,
        'linearUpdate': 0.3,
        'angularUpdate': 3.14,
        'temporalUpdate': 5.0,
        'resampleThreshold': 0.5,
        'particles': 30,
        'xmin': -15.0,
        'ymin': -15.0,
        'xmax': 15.0,
        'ymax': 15.0,
        'delta': 0.025,
        'llsamplerange': 0.01,
        'llsamplestep': 0.01,
        'lasamplerange': 0.005,
        'lasamplestep': 0.005,
    }

    return LaunchDescription([        
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time':use_sim_time}],
            remappings=remappings
        )
    ])