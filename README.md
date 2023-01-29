# Delivery Robot using AgileX Scout Robot
This project will contain the ROS packages necessary to run the project.

We have built a gazebo simulation with the packages provided with the real scout robots. This will enable us to do the same things (exporting maps, navigation with RVIZ etc.) in gazebo as we did with the real robots. Follow the instructions inside `scout_mini_gazebo` package for testing.

## To Run the Navigation:

### Simulation
Launch the robot first
```
roslaunch scout_mini_gazebo scout_v2_minimal.launch
```

Then launch the node to covert laser_scan to a point cloud for octomap
```
rosrun scout_mini_gazebo laser_scan_to_point_cloud_node.py
```

Launch the mapping server using octomap
```
roslaunch scout_mini_gazebo octomap_nav.launch
```

Run the transformation if there is an error
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 1000
```

The simulation doesn't need EKF as odometry is perfect. We can comment the EKF nodes and only run navsat_transform
```
roslaunch outdoor_waypoint_nav localization_run.launch
```

To collect and send gps waypoints, launch the following file and follow onscreen instructions
```
roslaunch outdoor_waypoint_nav joy_launch_control.launch
```

To simulate a joystick, we can use `rostopic pub` and publish message to joy control topic. You can see inside the `joy_launch_control.launch` that which key belongs to what number and put 1 in the index of that number in buttons array of `sensor_msgs/Joy`. An example to press button number 4 on joystick which corresponds to `LB` is
```
rostopic pub /joy_teleop/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
axes:
- 0
buttons: [0, 0, 0, 0, 1, 0, 0, 0, 0]" 
```

Also launch the move_base node to plan and send `cmd_vel` commands
```
roslaunch scout_mini_gazebo move_base.launch
```

### Real Robot
For the real robot, we will follow almost the same steps but the nodes will be launched from the real robot. After setting up the can, launch the robot first
```
roslaunch scout_bringup scout_minimal.launch
```

Run the transformation
```
rosrun tf  static_transform_publisher 0 0 0.138 0 0 0 base_footprint velodyne 100
```

Then launch the laser and rf2o laser odometry
```
roslaunch scout_bringup open_rslidar.launch
```

Then launch the node to covert laser_scan to a point cloud for octomap
```
rosrun scout_mini_gazebo laser_scan_to_point_cloud_node.py
```

Launch the mapping server using octomap
```
roslaunch scout_mini_gazebo octomap_nav.launch
```

Launch the xsens drivers
```
roslaunch xsens_mti_driver xsens_mti_node.launch
```

Uncomment the EKF nodes and run the localization run launch file
```
roslaunch outdoor_waypoint_nav localization_run.launch
```

To collect and send gps waypoints, launch the following file and follow onscreen instructions
```
roslaunch outdoor_waypoint_nav joy_launch_control.launch
```

To simulate a joystick, we can use `rostopic pub` and publish message to joy control topic. You can see inside the `joy_launch_control.launch` that which key belongs to what number and put 1 in the index of that number in buttons array of `sensor_msgs/Joy`. An example to press button number 4 on joystick which corresponds to `LB` is
```
rostopic pub /joy_teleop/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
axes:
- 0
buttons: [0, 0, 0, 0, 1, 0, 0, 0, 0]" 
```

Also launch the move_base node to plan and send `cmd_vel` commands
```
roslaunch scout_mini_gazebo move_base.launch
```
