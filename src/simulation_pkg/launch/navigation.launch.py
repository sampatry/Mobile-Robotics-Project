from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'map_subscribe_transient_local': True}
            ],
            arguments=['--ros-args', '--params-file', '/absolute/path/to/nav2_params.yaml']
        )
    ])
