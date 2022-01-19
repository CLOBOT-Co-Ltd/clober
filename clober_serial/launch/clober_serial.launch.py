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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    dir = get_package_share_directory('clober_serial')
    param_file = os.path.join(dir,'config','clober.yaml')

    return LaunchDescription([

        # DeclareLaunchArgument(
        #     'port', default_value='/dev/ttyUSB0',
        #     description='device port'),


        # Load the base controllers #
        Node(
            package='clober_serial',
            executable='clober_serial_node',
            name='clober_serial_node',
            output='screen',
            parameters=[param_file],
        ),
    ])