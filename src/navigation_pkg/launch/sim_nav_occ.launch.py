from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    print("Setting up launch paths...")

    # Source-space paths
    map_path = os.path.join(
        os.getenv('HOME'),
        'Mobile-Robotics-Project',
        'src',
        'navigation_pkg',
        'config',
        'my_map.yaml'
    )
    # Get simulation_pkg path
    try:
        sim_pkg_path = get_package_share_directory('simulation_pkg')
    except Exception as e:
        raise RuntimeError(f"[ERROR] Could not find simulation_pkg: {e}")

    simulation_launch_path = os.path.join(sim_pkg_path, 'launch', 'custom_world.launch.py')
    if not os.path.exists(simulation_launch_path):
        raise FileNotFoundError(f"[ERROR] Simulation launch file not found: {simulation_launch_path}")

    # Get turtlebot3_navigation2 path
    try:
        tb3_nav2_pkg_path = get_package_share_directory('turtlebot3_navigation2')
    except Exception as e:
        raise RuntimeError(f"[ERROR] Could not find turtlebot3_navigation2 package: {e}")

    tb3_nav2_launch_path = os.path.join(tb3_nav2_pkg_path, 'launch', 'navigation2.launch.py')
    if not os.path.exists(tb3_nav2_launch_path):
        raise FileNotFoundError(f"[ERROR] Nav2 launch file not found: {tb3_nav2_launch_path}")


    # Launch Gazebo simulation
    print("[INFO] Launching Gazebo simulation...")
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # Launch Nav2 stack (delayed)
    print("[INFO] Preparing Nav2 launch...")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_nav2_launch_path),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_path
        }.items()
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    # Launch occupancy updater node (delayed)
    # print("[INFO] Preparing occupancy updater...")
    # occupancy_updater_node = Node(
    #     package='navigation_pkg',
    #     executable='occupancy_updater.py',
    #     name='occupancy_updater',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'map_topic': '/map'}
    #     ]
    # )

    # delayed_updater = TimerAction(
    #     period=3.0,
    #     actions=[occupancy_updater_node]
    # )

    print("Launching!")
    # Final launch description
    return LaunchDescription([
        simulation_launch,
        # delayed_updater,
        # delayed_nav2,
    ])
