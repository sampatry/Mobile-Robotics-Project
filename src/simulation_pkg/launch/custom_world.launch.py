import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation_pkg')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'turtlebot3_burger.urdf')
    world_path = os.path.join(pkg_sim, 'worlds', 'your_world.world')

    robot_description = {'robot_description': open(urdf_path).read()}

    return LaunchDescription([
        # Launch Gazebo with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_path}.items()
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        ),

        # Spawn robot after delay
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'turtlebot3_burger',
                        '-topic', 'robot_description'
                    ],
                    output='screen'
                )
            ]
        )
    ])
