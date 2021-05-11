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