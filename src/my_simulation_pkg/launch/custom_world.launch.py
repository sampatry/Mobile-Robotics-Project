from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_simulation_pkg')
    world_path = os.path.join(pkg_path, 'worlds', 'your_world.world')
    urdf_path = os.path.join(pkg_path, 'urdf', 'turtlebot3_burger.urdf')

    return LaunchDescription([
        # Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Publish transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3', '-file', urdf_path, '-x', '0.0', '-y', '0.0', '-z', '0.3'],
            output='screen'
        )
    ])
