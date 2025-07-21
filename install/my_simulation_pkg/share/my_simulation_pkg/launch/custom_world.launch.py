from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    world_path = '/home/sam/MR_Project/simulation/world/your_world.world'
    urdf_path = '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf'

    return LaunchDescription([
        # Launch Gazebo with ROS plugins
        ExecuteProcess(
            cmd=[
                'gzserver',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_path
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Spawn TurtleBot3
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', urdf_path,
                '-x', '0.0', '-y', '0.0', '-z', '0.3'
            ],
            output='screen'
        )

    ])
