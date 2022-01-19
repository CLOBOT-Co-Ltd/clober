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
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    navi_dir = get_package_share_directory('clober_navigation')
    launch_dir = os.path.join(navi_dir,'launch')
    param_dir = os.path.join(navi_dir,'param')
    map_dir = os.path.join(navi_dir,'map')
    bt_dir = os.path.join(navi_dir,'bt')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam',default='false')
    param_file = LaunchConfiguration('param_file',default=os.path.join(param_dir,'clober_params.yaml'))
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename',default=os.path.join(bt_dir,'navigate_w_replanning_time.xml'))
    map_file = LaunchConfiguration('map_file',default=os.path.join(map_dir,'map.yaml'))
    autostart = LaunchConfiguration('autostart')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'waypoint_follower',
        'recoveries_server',
        'bt_navigator'
    ]

    remappings = [('/tf','tf'),
                  ('/tf_static','tf_static')]

    param_substitutions = {
        'use_sim_time' : use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart':autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    configured_params = RewrittenYaml(
        source_file = param_file,
        param_rewrites = param_substitutions,
        convert_types=True
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='use simulation time'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            description='automatically startup'
        ),

        DeclareLaunchArgument(
            'param_file',
            default_value=param_file,
            description='parameter file'
        ),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=default_bt_xml_filename,
            description='behavior tree file'
        ),

        DeclareLaunchArgument(
            'map_subscribe_transient_local',
            default_value='true',
            description='set map subscriber QoS to transient local'
        ),        

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[{'use_sim_time':use_sim_time}],
            remappings=remappings
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart':autostart,
                'node_names':lifecycle_nodes
            }]
        )

    ])