This project realizes a robot simulation with ROS2 and Gazebo.

To launch the project the following commands must be ran in separate terminals:

```
ros2 launch ar_project launch_sim.launch.py (optional: world:=<path to world file>)
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<path to params file> use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
rviz2 # for visualization
```