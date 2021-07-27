import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_dir = get_package_share_directory('clober_bringup')
    print("bringup : ", bringup_dir)

    ekf_config_file = os.path.join(bringup_dir,'config','ekf.yaml')
    print("ekf_config_file : ", ekf_config_file)

    robot_dir = os.path.join(get_package_share_directory('clober_serial'),'launch')
    print("robot_dir : ", robot_dir)

    description_dir = os.path.join(get_package_share_directory('clober_description'),'launch')

    lidar_config_file = os.path.join(bringup_dir,'config','sick_tim_5xx.yaml')

    return LaunchDescription([

        # Load robot driver #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_dir,'/clober_serial.launch.py'])
        ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     output='screen',
        #     parameters=[ekf_config_file],
        #     # remappings=[('odometry/filtered','odom')]
        # ),

        # Load robot model #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir,'/description_launch.py'])
        ),
        

        Node(
                package='sick_scan2',
                executable='sick_generic_caller',
                output='screen',
                parameters=[lidar_config_file],
            ),
    ])