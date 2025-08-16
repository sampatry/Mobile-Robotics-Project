

- ros2 launch simulation_pkg custom_world.launch.py
- ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/$USER/Mobile-Robotics-Project/src/navigation_pkg/config/my_map.yaml

Use map_saver_cli to save your updated map from /updated_map
- ros2 run nav2_map_server map_saver_cli --map-topic /updated_map -f ~/maps/my_updated_map

Relaunch Nav2 with the new map, once saved, you can relaunch Nav2 with your updated map:
- ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/$USER/maps/my_updated_map.yaml
