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

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',default='true')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
        default=os.path.join(get_package_share_directory('clober_slam'),'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',default='cartographer.lua')
    resolution = LaunchConfiguration('resolution',default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec',default='0.5')
    rviz_config_dir = os.path.join(get_package_share_directory('clober_slam'),'rviz','slam.rviz')


    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='use simulation time'
        ),

        DeclareLaunchArgument(
            'cartographer_config_dir', default_value= cartographer_config_dir, 
            description='cartographer config dir'
        ),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='cartographer config file'
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='map resolution'
        ),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='occupancy grid publishing period'
        ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time':use_sim_time}],
            arguments=['-configuration_directory',cartographer_config_dir,
                        '-configuration_basename',configuration_basename]
        ),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time':use_sim_time}],
            arguments=['-resolution',resolution,'-publish_period_sec',publish_period_sec],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time':use_sim_time}],
            output='screen'
        )
    ])