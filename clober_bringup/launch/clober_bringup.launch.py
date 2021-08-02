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

    imu_dir = get_package_share_directory('myahrs_ros2_driver')
    imu_config_file = os.path.join(imu_dir, 'config', 'config.yaml')

    lidar_config_file = os.path.join(bringup_dir,'config','sick_tim_5xx.yaml')

    return LaunchDescription([

        # Load robot driver #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_dir,'/clober_serial.launch.py'])
        ),

        # Load robot model #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir,'/description_launch.py'])
        ),
        
        # Load Myahrs+ Driver #
        Node(
            package='myahrs_ros2_driver',
            executable='myahrs_ros2_driver',
            name='myahrs_ros2_driver',
            output='screen',
            arguments=['/dev/ttyACM0', '115200'],
            parameters=[imu_config_file],
        ),

        # Load 2D Lidar #
        Node(
            package='sick_scan2',
            executable='sick_generic_caller',
            output='screen',
            parameters=[lidar_config_file],
        ),

        # ekf_localization_node #
        Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            parameters=[ekf_config_file],
            remappings=[('odometry/filtered','odom/ekf/enc_imu')]
        ),
    ])