from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'True',
                'map': '/home/sam/my_map.yaml',
                'params_file': '/home/sam/Mobile-Robotics-Project/src/navigation_pkg/config/nav2_params.yaml'
            }.items()
        ),


        Node(
            package='navigation_pkg',
            executable='waypoint_follower.py',
            name='waypoint_follower',
            output='screen'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
        )

    ])
