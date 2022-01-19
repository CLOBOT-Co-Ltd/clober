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
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    navi_dir = get_package_share_directory('clober_navigation')
    launch_dir = os.path.join(navi_dir,'launch')
    param_dir = os.path.join(navi_dir,'param')
    map_dir = os.path.join(navi_dir,'map')

    use_sim_time = LaunchConfiguration('use_sim_time')
    param_file = LaunchConfiguration('param_file',default=os.path.join(param_dir,'clober_params.yaml'))
    map_file = LaunchConfiguration('map_file',default=os.path.join(map_dir,'map.yaml'))
    autostart = LaunchConfiguration('autostart',default='false')

    lifecycle_nodes = ['map_server','amcl']

    remappings = [('/tf','tf'),('/tf_static','tf_static')]

    param_substitutions = {
        'use_sim_time' : use_sim_time,
        'yaml_filename' : map_file
    }

    params = RewrittenYaml(
        source_file= param_file,
        param_rewrites= param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params],
            remappings=remappings
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time':use_sim_time},
                        {'autostart':autostart},
                        {'node_names':lifecycle_nodes}]
        )


    ])