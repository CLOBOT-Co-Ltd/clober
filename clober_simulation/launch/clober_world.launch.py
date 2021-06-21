#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'clober.model'
    world = os.path.join(get_package_share_directory('clober_simulation'), 'worlds', world_file_name)

    launch_file_dir = os.path.join(get_package_share_directory('clober_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_model_path = os.path.join(get_package_share_directory('clober_simulation'), 'models')


    return LaunchDescription([

        ExecuteProcess(
            cmd=['gzserver', world , 
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                '--verbose'
                ],
            # additional_env=EnvironmentVariable('GAZEBO_MODEL_PATH'),
            output='screen'),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file' , gazebo_model_path + '/clober/' + 'model.sdf',
                '-entity', 'clober', '-spawn_service_timeout', '300', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen'),    

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        #     ),
        #     launch_arguments={'world':world}.items(),
        # ),


        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        #     ),
        # ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_file_dir, '/description_launch.py'
            ]), launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
    
    ])