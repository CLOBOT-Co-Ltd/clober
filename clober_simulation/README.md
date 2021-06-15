# Clober Simulations
This Gazebo Simulation utilizes the ROS Gazebo package, Gazebo version for ROS1 noetic has to be properly installed before running this instruction.

<!-- ## Clober ROS 1 Package Status
|develop|main|Noetic + Ubuntu Focal|
|:---:|:---:|:---:|
|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=noetic-devel)](https://travis-ci.com/clobot-git/clober)|

## Clober ROS 2 Package Status
|ros2-devel|ros2|Foxy + Ubuntu Focal|Galactic|
|:---:|:---:|:---:|:---:|
|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=foxy-devel)](https://travis-ci.com/clobot-git/clober)| TBD | -->
## 1. Installation
### 1.1 Install Clober Packages
The Clober Simulation Package requires `clober_msgs` package as a prerequisite. Without the package the simulation cannot be launched.
```bash
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/clobot-git/clober_msgs.git
git clone -b noetic-devel https://github.com/clobot-git/clober.git
cd ~/catkin_ws && catkin_make
```

### 1.2 Install Dependent ROS 1 Packages
```bash
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan \
  ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
  ros-noetic-rosserial-server ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
  ros-noetic-teb-local-planner
```

## 2. Launch Simulation World
Several Simulation environments are prepared, made accessible by each launch file.

### 2.1 Empty World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/empty_world.png">

```bash
roslaunch clober_simulation empty_world.launch
```

### 2.2 Clobot Logo World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/logo_world.png">

```bash
roslaunch clober_simulation logo_world.launch
```

### 2.3 Warehouse World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/warehouse_world.png">

```bash
roslaunch clober_simulation warehouse_env_world.launch
```

### 2.4 Grid World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/grid_world.png">

3x3 grid world
```bash
roslaunch clober_simulation 3x3_world.launch
```

4x4 grid world
```bash
roslaunch clober_simulation 4x4_world.launch
```

5x5 grid world
```bash
roslaunch clober_simulation 5x5_world.launch
```

10x10 grid world
```bash
roslaunch clober_simulation 10x10_world.launch
```


## 3. Operate Clober
To teleoperate the simulated Clober with the keyboard, launch the teleoperation node in a new terminal window.
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## 4. SLAM Simulation
For SLAM in Gazebo simulator, you can select or create various virtual environemnts and robot models. SLAM simulation works pretty similar to that of the real world under ordinary circumstances.
For fine tuning of the SLAM package please review the SLAM package README.md

### 4.1 Launch Simulation World
Out of the worlds prepared, we recommend using the Clobot Logo World.
```bash
roslaunch clober_simulation logo_world.launch
``` 

### 4.2 Run SLAM Node
On a new terminal run a SLAM node. Gmapping SLAM is used by default.
```bash
roslaunch clober_slam clober_slam.launch slam_methods:=gmapping
```

### 4.4 Run Teleoperation Node
On a new terminal run a teleoperation node to explore and map the Gazebo world.
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
![clober_slam](https://github.com/clobot-git/testrobot/blob/noetic-devel/gifs/clober_slam.gif)

### 4.5 Save Map
After successful SLAM and map creation, open a new terminal to save the map.
```bash
rosrun map_server map_saver -f ~/map
```
If you've saved your map successfully it should look like the following.

<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/map.png">

## 5. Navigation Simulation
### 5.1 Launch Simulation World
Out of the worlds prepared, we recommend using the Clobot Logo World.
```bash
roslaunch clober_simulation logo_world.launch
``` 

### 5.2 Run Navigation 
On a new terminal run a Navgitaion node.
```bash
roslaunch clober_navigation navigation.launch
```

### 5.3 Estimate Initial Pose
Initial Pose Estimation can be performed before Navigation to intialize AMCL parameters which are critical to Navigation quality. 
1. Click `2D Pose Estimate` Button on the RVIZ menu
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/2d_pose_estimate.png">

2. Click on the map where clober is located and drag the green arrow toward the dirction the robot is facing.

3. Repeat step 1 and 2 to increase data precision.

4. Launch keyboard teleoperation to further increase precision.

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

5. Move the robot back and forth in order to narrow down the estimated location.

![clober_amcl](https://github.com/clobot-git/testrobot/blob/noetic-devel/gifs/clober_amcl.gif)

6. Terminate the keyboard teleoperation(`Ctrl`+`C`) for the next navigation step. 

### 5.4 Publish 2D Navigation Goal
- On the RVIZ menu click `2D Nav Goal`
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/2d_nav_goal.png">

- Click on the destination and drag the green arrow toward the dirction of the robot on the map.
![clober_navigation](https://github.com/clobot-git/testrobot/blob/noetic-devel/gifs/clober_navigation.gif)

<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/gifs/clober_navigation.gif">