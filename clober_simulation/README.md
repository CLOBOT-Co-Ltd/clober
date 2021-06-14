# Clober Simulations
This Gazebo Simulation utilizes the ROS Gazebo package, Gazebo version for ROS1 noetic has to be properly installed before running this instruction.

## Clober ROS 1 Package Status
|develop|main|Noetic + Ubuntu Focal|
|:---:|:---:|:---:|
|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=noetic-devel)](https://travis-ci.com/clobot-git/clober)|

## Clober ROS 2 Package Status
|ros2-devel|ros2|Foxy + Ubuntu Focal|Galactic|
|:---:|:---:|:---:|:---:|
|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=main)](https://travis-ci.com/clobot-git/clober)|[![Build Status](https://travis-ci.com/clobot-git/clober.svg?branch=foxy-devel)](https://travis-ci.com/clobot-git/clober)| TBD |

## Install Clober Packages
- The Clober Simulation Package requires `clober_msgs` package as a prerequisite. Without the package the simulation cannot be launched
```bash
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/clobot-git/clober_msgs.git
cd ~/catkin_ws && catkin_make
```

## Launch Simulation World
- Several Simulation environments are prepared, made accessible by each launch file.

### Empty World
<img src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/empty.png" width="400">

```bash
roslaunch clober_simulation empty_world.launch
```

### Clobot Logo World
<img src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/logo.png" width="400">

```bash
roslaunch clober_simulation logo_world.launch
```

### Warehouse World
<img src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/warehouse.png" width="400">

```bash
roslaunch clober_simulation warehouse_env_world.launch
```

### Grid World
<img src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/grid.png" width="400">

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

## Operate Clober
To teleoperate the simulated Clober with the keyboard, launch the teleoperation node in a new terminal window.
```bash
roslaunch clober_teleop clober_teleop_keyboard.launch
```

## SLAM Simulation
For SLAM in Gazebo simulator, you can select or create various virtual environemnts and robot models. SLAM simulation works pretty similar to that of the real world under ordinary circumstances.
For fine tuning of the SLAM package please review the SLAM package README.md

### Launch Simulation World
Out of the worlds prepared, we recommend using the Clobot Logo World.
```bash
roslaunch clober_simulation logo_world.launch
``` 

### Run SLAM Node
On a new terminal run a SLAM node. Gmapping SLAM is used by default.
```bash
roslaunch clober_slam clober_slam.launch slam_methods:=gmapping
```

### Run Teleoperation Node
On a new terminal run a teleoperation node.
```bash
roslaunch clober_teleop clober_teleop_keyboard.launch
```

### Save Map
After successful SLAM and map creation, open a new terminal to save the map
```bash
rosrun map_server map_saver -f ~/map
```

## Navigation Simulation
- [ Clobot Homepage ](https://www.clobot.co.kr/)
- [ Clobot YouTube ](https://www.youtube.com/channel/UCau5FLJpMxhvW-IHZ8c8qKQ/featured/)

