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

Here’s what each folder does:

| Folder | Purpose |
|--------|---------|
| `src/` | Contains all our ROS2 packages and custom Python/C++ nodes. Each subfolder is a separate ROS2 component (e.g. tape detection, return-to-cell logic). |
| `launch/` | ROS2 launch files that start our robot system. This is where we combine SLAM, navigation, and vision nodes into one runtime. |
| `config/` | YAML files used to configure ROS2 packages like Navigation2 (`nav2`) and SLAM Toolbox. |
| `simulation/` | Gazebo simulation assets: this includes the `.world` file, URDFs describing our environment layout, and SolidWorks-exported models. |
| └── `world/` | Gazebo `.world` file describing the map for simulation. |
| └── `urdf/` | URDF files used to define parts of our environment and robot setup. |
| └── `model/` | 3D mesh files (e.g. `.stl`, `.dae`) for Gazebo objects, including our custom map. |
| `maps/` | Saved map files (`.yaml`, `.pgm`) generated during SLAM runs in simulation and real-world testing. |
| `README.md` | You’re reading it! This file explains our project, repo structure, and how to get started. |

How to Run It (Once Fully Set Up)

Since we’re still learning ROS2 and Gazebo, setup steps will evolve. This is a draft roadmap for future updates.

```bash
# 1. Source the ROS2 environment
source /opt/ros/humble/setup.bash

# 2. Clone this repo and navigate into it
cd ~/turtlebot3_project

# 3. Build the workspace
colcon build

# 4. Source the workspace
source install/setup.bash

# 5. Launch the full robot system (example)
ros2 launch launch/full_simulation.launch.py


To run gazebo with the model run
export GAZEBO_MODEL_PATH=/home/sam/MR_Project/simulation/model:$GAZEBO_MODEL_PATH

gazebo --verbose /home/sam/MR_Project/simulation/world/your_world.world

