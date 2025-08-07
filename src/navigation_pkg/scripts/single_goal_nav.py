#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def create_pose(x, y, z=0.0, w=1.0, frame='map', navigator=None):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = create_pose(1.90, 2.40, navigator=navigator)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Define multiple goal poses
    goal_poses = [
        create_pose(0.9, 3.2, navigator=navigator),
        create_pose(1.0, 1.5, navigator=navigator),
        create_pose(3.0, 1.5, navigator=navigator),
        create_pose(1.9, 2.4, navigator=navigator)
    ]

    for goal_pose in goal_poses:
        print(f"Navigating to: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")
        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f'Estimated time of arrival: {eta:.0f} seconds')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
