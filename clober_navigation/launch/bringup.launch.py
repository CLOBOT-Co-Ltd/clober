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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    nav_dir = get_package_share_directory('clober_navigation')
    launch_dir = os.path.join(nav_dir,'launch')
    param_dir = os.path.join(nav_dir,'param')
    param_file = 'clober_params.yaml'
    bt_file = 'BehaviorTree.xml'
    map_dir = os.path.join(nav_dir,'map','3x3')
    map_file = 'map.yaml'

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam', default='false')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    open_rviz = LaunchConfiguration('open_rviz')


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(map_dir, map_file),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether run a SLAM')    

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(param_dir, param_file),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(param_dir, bt_file),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_open_rviz_cmd = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Launch Rviz?')

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            # Run Localization only when we don't use SLAM
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'clober_localization.launch.py')),
            condition=UnlessCondition(use_slam),
            launch_arguments={'namespace': namespace,
                            'map': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': params_file,
                            'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'clober_navigation.launch.py')),
            launch_arguments={'namespace': namespace,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': params_file,
                            'default_bt_xml_filename': default_bt_xml_filename,
                            'use_lifecycle_mgr': 'false',
                            'map_subscribe_transient_local': 'true'}.items()),
                            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                            'open_rviz': open_rviz,
                            'map_subscribe_transient_local': 'true'}.items()),
                            
    ])


    # # Create the launch description and populate
    ld = LaunchDescription()

    # # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_open_rviz_cmd)

    # # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld