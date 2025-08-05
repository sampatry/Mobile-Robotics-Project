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
Make sure to source Ros and the workspace before running
- source /opt/ros/humble/setup.bash
- source ~/Mobile-Robotics-Project/install/setup.bash


To start sim;
- ros2 launch simulation_pkg custom_world.launch.py #start only gazebo
- ros2 launch navigation_pkg navigation.launch.py #start gazebo with rviz

To manually generate map;
- ros2 run turtlebot3_teleop teleop_keyboard #manual control of bot
- ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True #creates the map as you move around
- ros2 run nav2_map_server map_saver_cli -f my_map #to save the map

- ros2 launch turtlebot3_bringup rviz2.launch.py #Use RViz2 to visualize robot sensors and motion

once scan is done, nav2 can be run with (set the proper path to map file)
- ros2 launch nav2_bringup localization_launch.py map:=/home/sam/my_map.yaml use_sim_time:=True
- ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/sam/Mobile-Robotics-Project/src/simulation_pkg/map/my_map.yaml


Dependencies;
make sure to have the following in .bashrc (so they load on new terminal)
source /opt/ros/humble/setup.bash #if it doenst compile you might need to run this
source ~/Mobile-Robotics-Project/install/setup.bash #if it doesnt run you might need to run this

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/sam/Mobile-Robotics-Project/src/simulation_pkg/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_description

you may need to install the following
sudo apt install ros-humble-turtlebot3*

link to make turtlebot workspace (i need to verify if we need this)
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

reseting colcon;
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

might be usefull for nav2
https://roboticsbackend.com/ros2-nav2-tutorial/#Make_the_robot_move_in_the_environment
https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py


