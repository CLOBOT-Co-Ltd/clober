import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_dir = get_package_share_directory('clober_bringup')
    ekf_config_file = os.path.join(bringup_dir,'config'.'ekf.yaml')

    description_dir = os.path.join(get_package_share_directory('clober_description'),'launch')
    lidar_dir = os.path.join(get_package_share_directory('sick_scan2'),'launch')

    return LaunchDescription([

        # Node(
        #     package=''

        # ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            parameters=[ekf_config_file],
            # remappings=[('odometry/filtered','odom')]
        )

        # Load robot model #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir,'description_launch.py')
        )
        
        # Load lidar driver #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_dir,'sick_tim_5xx.launch.py'])
        )

    ])