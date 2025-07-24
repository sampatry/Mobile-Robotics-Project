import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.task_result import TaskResult

def make_pose(x, y, z, ox, oy, oz, ow):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = ox
    pose.pose.orientation.y = oy
    pose.pose.orientation.z = oz
    pose.pose.orientation.w = ow
    return pose

rclpy.init()
navigator = BasicNavigator()

# Set the starting pose
init_pose = make_pose(1.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0)
navigator.setInitialPose(init_pose)
navigator.waitUntilNav2Active()

# Define waypoints
waypoints = [
    make_pose(1.5, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0),
    make_pose(2.5, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0),
    make_pose(3.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0),
    make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
]

# Navigate through waypoints
for idx, pose in enumerate(waypoints):
    print(f"Navigating to waypoint {idx + 1}/{len(waypoints)}")
    task = navigator.goToPose(pose)

    while not navigator.isTaskComplete(task):
        feedback = navigator.getFeedback(task)
        print(f"⏱ Elapsed time: {feedback.navigation_duration.sec}s")

    result = navigator.getResult(task)
    if result == TaskResult.SUCCEEDED:
        print(f"Waypoint {idx + 1} reached successfully")
    elif result == TaskResult.CANCELED:
        print(f"Waypoint {idx + 1} was canceled")
    elif result == TaskResult.FAILED:
        print(f"Waypoint {idx + 1} failed")

navigator.lifecycleShutdown()
rclpy.shutdown()
