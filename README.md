# Delivery Robot using AgileX Scout Robot
This project will contain the ROS packages necessary to run the project.

We have built a gazebo simulation with the packages provided with the real scout robots. This will enable us to do the same things (exporting maps, navigation with RVIZ etc.) in gazebo as we did with the real robots. Follow the instructions inside `scout_mini_gazebo` package for testing.

## Port real robot packages to Gazebo
The first thing we tried to do is port all the packages from the real robot to the gazebo so we can easily do everything in simulation first. Using the `scout_mini_gazebo` package, you can do all the things, i.e. generated and save maps and navigation on the saved maps on Gazebo now. For example to run the navigation, launch the following two nodes:

```
roslaunch scout_mini_gazebo scout_v2_minimal.launch
roslaunch scout_mini_gazebo navigation_4wd.launch
```

You can now add the 2d Nav Goal points through RVIZ and the robot will follow it. We have also saved a map of gazebo playpen enviornment to make the testing easier. A demo is shown:

<img width="1157" alt="image" src="https://user-images.githubusercontent.com/31202659/215353318-edab9a1b-3876-4093-bd7e-61771f72a6de.png">

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

## Road Segmentation
### Simulation
In simulation, the camera will already be launched with the above robot launch files. So you just need to run the `road_segmentation` node. Before running the segmentation, you have to copy this weights file to your `road_segmentation/src` folder. The weights are available at: https://drive.google.com/file/d/1jnI16fkRYhd2dT8bm726cgkUNNwG6kFQ/view

```
rosrun road_segmentation road_segmentation.py
```

### Real Robot
For the real robot, first you have to launch the robot launch node and then camera node as well. 
```
roslaunch realsense2_camera rs_camera.launch
```

Then you just need to run the `road_segmentation` node. Before running the segmentation, you have to copy this weights file to your `road_segmentation/src` folder. The weights are available at: https://drive.google.com/file/d/1jnI16fkRYhd2dT8bm726cgkUNNwG6kFQ/view

```
rosrun road_segmentation road_segmentation.py
```

## Simulation Results:
The robot is launched in a small city world inside gazebo where we can use GPS to navigate. The city looks something like:
<img width="560" alt="gazebo_world" src="https://user-images.githubusercontent.com/31202659/215352225-07c63647-50b4-4b7b-97d1-df89d3c7c726.png">

We can see how the map is generated and the robot is following waypoints while exploring the enviornment
<img width="1156" alt="simulation resukt" src="https://user-images.githubusercontent.com/31202659/215352192-6bffc1b3-c6e5-4316-a82d-533e11f3f2d1.png">

## Real Robot Results:
The real robot was run in a closed loop outside the CLC building and the trajectories are shown below:
<img width="745" alt="GPS_odom_and_filter" src="https://user-images.githubusercontent.com/31202659/215352269-c4e9cfc7-24fb-4b6a-8c78-5e12b47ed876.png">

The red shows the filtered odom, orange shows the unfiltered odom and the cyan shows the gps odometry. The maps are generated by `rviz_satellite` package by running the `GPS_to_XY.py` file. The same results are shown in a different map.

<img width="749" alt="GPS_odom_and_filter_map2" src="https://user-images.githubusercontent.com/31202659/215352333-aa005113-b3be-4a0a-8375-2327d780bd65.png">

The map generataed by the real robot is also shown:
<img width="1280" alt="Real Robot" src="https://user-images.githubusercontent.com/31202659/215352350-b4091f11-db18-4c6f-a9ce-2a780a69a2c9.png">
