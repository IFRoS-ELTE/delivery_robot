# Scout Mini Robot at ELTE - Gazebo Simulation 

## Dependencies

### Robot Models
First clone the robot gazebo models and URDF files from the official source to your catkin workspace
```
git clone https://github.com/agilexrobotics/ugv_gazebo_sim.git
```

Go through the official docs once and install dependencies. Remember to install them for your own distro (noetic etc.) 


### Camera
```
git clone https://github.com/issaiass/realsense2_description
git clone https://github.com/issaiass/realsense_gazebo_plugin
```

### LiDAR
```
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
```

### Point Cloud to Laser Scan
```
git clone https://github.com/Umar-Senpai/pointcloud_to_laserscan.git
```

### RVIZ Navigation
```
git clone https://github.com/autolaborcenter/rviz_navi_multi_goals_pub_plugin.git
```
## Errors & External Packages need to be added:

Error : Could not find a package configuration file provided by "tf2_sensor_msgs"
```
Solution:$ cd /catkin_ws/src 
         $ git clone https://github.com/ros/geometry2.git
         $ cd catkin_ws 
         $ catkin_make 
```
Error : 'velocity_controllers/JointVelocityController' does not exist. 
``` 
Required pkg: $ sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```
Error: No module named "rospkg"
```
required pkg: $ pip install -U rospkg

```
### navigation_4wd.launch (Errors and pkgs): 

Error: yocs_velocity_smoother (Resource not found) 
```
$ cd catkin_ws/src
$ git clone https://github.com/Umar-Senpai/yujin_ocs.git
$ sudo apt install ros-noetic-ecl-threads
$ cd catkin_ws 
$ catkin_make
```
Error: missing packages: (move-base, map-server, amcl,dwa-Local-planner, global-planner)
```
$ sudo apt install ros-noetic-move-base
$ sudo apt install ros-noetic-map-server
$ sudo apt install ros-noetic-amcl
$ sudo apt install ros-noetic-dwa-local-planner
$ sudo apt install ros-noetic-global-planner
```
make sure to build and source 

### scout_v2_octomap (missing pkgs):
Error: missing pakage: gmapping, octomap-server
```
$ sudo apt install ros-noetic-octomap-server
$ sudo apt install ros-noetic-gmapping
```

## Usage
Spawn the scout_mini in a gazebo world first:
```
roslaunch scout_mini_gazebo scout_v2_minimal.launch
```

We can teleoperate the robot using:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Now, you can run the octomap server:
```
roslaunch scout_mini_gazebo scout_v2_octomap.launch
```

Or you can also run the Navigation module which navigates the robot to different points using RVIZ
```
roslaunch scout_mini_gazebo navigation_4wd.launch
```

These packages will work almost the same way as in the real robot. The only change is instead of using rf2o laser odometry, we are using odom.py file to generate odometry.
