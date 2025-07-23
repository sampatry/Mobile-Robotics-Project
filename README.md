TurtleBot3 SLAM Project – Group 3

Welcome to our group project repository for ROB8224: Mobile Robotics. We're prototyping a TurtleBot3 Burger robot to explore a factory-like environment, detect obstacles, and return to base—all using ROS2 Humble and Gazebo simulation. This repository will track our code, configuration files, simulation assets, and documentation.

What We’re Building

Our robot will:
- Map the environment using **SLAM**
- Avoid lanes marked by **red tape**
- Avoid randomly placed **obstacles**
- **Return** to its starting cell at the end of its mission
- Work in **real life** *and* inside **Gazebo simulation**


To build;
- cd ~/Mobile-Robotics-Project
- colcon build --symlink-install

To start sim;
- ros2 launch simulation_pkg custom_world.launch.py

Other usefull ros commands;
- ros2 run turtlebot3_teleop teleop_keyboard
- ros2 launch turtlebot3_bringup rviz2.launch.py
- ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True //creates map from lidar scan

Dependencies

sudo apt install ros-humble-turtlebot3*

link to make turtlebot workspace (i need to verify if we need this)
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/


source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_description

reseting colcon;
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH



