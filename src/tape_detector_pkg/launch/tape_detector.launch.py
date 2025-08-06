from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tape_detector_pkg',
            executable='tape_detector',
            name='tape_detector_node',
            output='screen',
            parameters=[]
        ),
        # Optional: Launch RViz if needed
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', '/path/to/your/rviz/config.rviz']
        # )
    ])
