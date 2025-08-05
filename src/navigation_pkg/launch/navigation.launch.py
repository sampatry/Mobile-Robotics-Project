import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import TimerAction


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('navigation_pkg'),'config')
    map_file = os.path.join(config_dir,'my_map.yaml')
    param_file = os.path.join(config_dir,'nav2_params.yaml')
    rviz_config_dir = os.path.join(config_dir,'navigation.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('simulation_pkg'),'/launch','/custom_world.launch.py'])
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        #     #PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/localization_launch.py']),
        #     launch_arguments={
        #         'map': map_file,
        #         'params_file': param_file,
        #         'use_sim_time': 'true'
        #     }.items(),
        # ),

    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_dir],
        output='screen',
        parameters=[{'use_sim_time': True}],
    ),

    Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}],
    ),
    Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': True}],
    ),
    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
    ),


    TimerAction(
        period=10.0,  # adjust as needed
        actions=[
            Node(
                package='navigation_pkg',
                executable='single_goal_nav.py',
                name='single_goal_nav',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    ),

    ])