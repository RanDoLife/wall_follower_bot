## Первый терминал:
```bash
source /opt/ros/foxy/setup.bash 
cd ros2_ws
sorce /install/setup.bash 

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

## Второй терминал:

```bash
source /opt/ros/foxy/setup.bash 
cd ros2_ws
sorce /install/setup.bash 

ros2 launch wall_follower_bot wall_follower.launch.py 
```
