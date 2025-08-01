#!/usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

rclpy.init()

nav = BasicNavigator()

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 1.5
goal_pose.pose.position.y = -0.77
goal_pose.pose.orientation.w = 1.0
goal_pose.pose.orientation.z = 0.0

nav.setInitialPose(init_pose)
nav.waitUntilNav2Active() # if autostarted, else use `lifecycleStartup()`

path = nav.getPath(init_pose, goal_pose)
smoothed_path = nav.smoothPath(path)

nav.goToPose(goal_pose)
while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback.navigation_duration > 600:
                nav.cancelTask()

result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
elif result == TaskResult.FAILED:
    print('Goal failed!')