cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
#find_package(nav2_simple_commander REQUIRED)
find_package(geometry_msgs REQUIRED)



# Install Python scripts
install(PROGRAMS
  scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if you have one)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
