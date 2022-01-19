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
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    navi_dir = get_package_share_directory('clober_navigation')
    launch_dir = os.path.join(navi_dir,'launch')
    param_dir = os.path.join(navi_dir,'param')
    map_dir = os.path.join(navi_dir,'map')
    bt_dir = os.path.join(navi_dir,'bt')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam',default='false')
    param_file = LaunchConfiguration('param_file',default=os.path.join(param_dir,'clober_params.yaml'))
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename',default=os.path.join(bt_dir,'navigate_w_replanning_time.xml'))
    map_file = LaunchConfiguration('map_file',default=os.path.join(map_dir,'map.yaml'))

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use simulation time'
    ),

    DeclareLaunchArgument(
        'param_file',
        default_value=param_file,
        description='navigation parameter file'
    ),

    DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=default_bt_xml_filename,
        description='default xml filename'
    ),

    DeclareLaunchArgument(
        'use_slam',
        default_value=use_slam,
        description='use slam flag'
    ),

    DeclareLaunchArgument(
        'map_file',
        default_value=map_file,
        description='map yaml file'
    ),


    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,'clober_localization.launch.py')),
            condition=UnlessCondition(use_slam),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file':param_file
            }.items()
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir,'clober_navigation.launch.py')),
        #     launch_arguments={
        #         'use_sim_time' : use_sim_time,
        #         'params_file': param_file,
        #         'default_bt_xml_filename':default_bt_xml_filename,
        #         'map_subscribe_transient_local':'true'
        #     }.items()
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,'rviz.launch.py')),
            launch_arguments={
                'use_sim_time':use_sim_time,
                'map_subscribe_transient_local':'true'
            }.items()
        )

    ])