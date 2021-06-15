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

## 1. Install Clober Packages
The Clober Simulation Package requires `clober_msgs` package as a prerequisite. Without the package the simulation cannot be launched
```bash
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/clobot-git/clober_msgs.git
git clone -b noetic-devel https://github.com/clobot-git/clober.git
cd ~/catkin_ws && catkin_make
```
### 1.1 Install Dependent ROS 1 Packages
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
```

## 2. Launch Simulation World
Several Simulation environments are prepared, made accessible by each launch file.

### 2.1 Empty World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/empty_world.png" width="400">

```bash
roslaunch clober_simulation empty_world.launch
```

### 2.2 Clobot Logo World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/logo_world.png" width="400">

```bash
roslaunch clober_simulation logo_world.launch
```

### 2.3 Warehouse World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/warehouse_world.png" width="400">

```bash
roslaunch clober_simulation warehouse_env_world.launch
```

### 2.4 Grid World
<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/grid_world.png" width="400">

```bash
roslaunch clober_simulation 3x3_world.launch
```
```bash
roslaunch clober_simulation 4x4_world.launch
```
```bash
roslaunch clober_simulation 5x5_world.launch
```
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
On a new terminal run a teleoperation node.
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 4.5 Save Map
After successful SLAM and map creation, open a new terminal to save the map
```bash
rosrun map_server map_saver -f ~/map
```

## 5. Navigation Simulation
### 5.1 Launch Simulation World
Out of the worlds prepared, we recommend using the Clobot Logo World.
```bash
roslaunch clober_simulation logo_world.launch
``` 

### 5.2 Run SLAM Navigation
On a new terminal run a SLAM node. Gmapping SLAM is used by default.
```bash
roslaunch clober_navigation navigation.launch
```
