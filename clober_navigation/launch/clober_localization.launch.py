import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


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