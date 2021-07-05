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