TurtleBot3 SLAM Project – Group 3

Welcome to our group project repository for ROB8224: Mobile Robotics. We're prototyping a TurtleBot3 Burger robot to explore a factory-like environment, detect obstacles, and return to base—all using ROS2 Humble and Gazebo simulation. This repository will track our code, configuration files, simulation assets, and documentation.

What We’re Building

Our robot will:
- Map the environment using **SLAM**
- Navigate lanes marked by **red tape**
- Avoid randomly placed **obstacles**
- **Return** to its starting cell at the end of its mission
- Work in **real life** *and* inside **Gazebo simulation**

Folder Structure

MR_Project/
├── src/
│   ├── my_simulation_pkg/
│   │   ├── launch/         # Launch files for simulation
│   │   ├── urdf/           # Robot description (TurtleBot3 URDF)
│   │   ├── worlds/         # Gazebo .world files
│   │   ├── models/         # Custom models (SDF + meshes)
│   │   ├── config/         # Config files (SLAM, Nav2, etc.)
│   │   ├── setup.py        # Python setup script
│   │   └── package.xml     # ROS2 manifest
│   ├── return_to_cell/     # Optional logic (future node or package)
│   └── vision_tape_detector/ # Vision system (future node or package)
├── README.md               # You’re reading it!


How to Run It (Once Fully Set Up)

# 1. Source ROS2 (if not already done)
source /opt/ros/humble/setup.bash

# 2. Build your workspace
cd ~/MR_Project
colcon build --symlink-install

# 3. Source the workspace
source install/setup.bash

# 4. Launch the simulation with your world + robot
ros2 launch my_simulation_pkg custom_world.launch.py

or all-together
source /opt/ros/humble/setup.bash
cd ~/MR_Project
colcon build --symlink-install
ros2 launch my_simulation_pkg custom_world.launch.py

Dependencies

sudo apt install ros-humble-turtlebot3-description
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$HOME/MR_Project/src/my_simulation_pkg/models:$GAZEBO_MODEL_PATH

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_description
source ~/MR_Project/install/setup.bash



