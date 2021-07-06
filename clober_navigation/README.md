# Clober Navigation

## 1. Install Nav2 pakcage

```bash
sudo apt-get install ros-foxy-nav2*
```

## 2. Install Groot

### 2.1. Install BehaviorTree.CPP Package

original source : https://github.com/BehaviorTree/BehaviorTree.CPP

```bash
sudo apt-get install libzmq3-dev libboost-dev
```

### 2.2. Install Groot in ROS2

original source : https://github.com/BehaviorTree/Groot

```bash
mkdir -p groot_ws/src
cd groot_ws/src
git clone https://github.com/BehaviorTree/Groot.git
cd ..
colcon build --symlink-install
```

## 3. Run Clober (Simulation)
### 3.1. Launch Simulation World
```bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/clober_ws/src/clober/clober_simulation/models

source /opt/ros/foxy/setup.bash
source ~/clober_ws/install/setup.bash
ros2 launch clober_simulation clober_world.launch.py
```

## 4. Launch Navigation
```bash
source /opt/ros/foxy/setup.bash
source ~/clober_ws/install/setup.bash
ros2 launch clober_navigation bringup.launch.py
```

### 4.1. Set Initialpose

> Click "2D Pose Estimate", and set estimation to the approximate location of robot on the map.

<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/SetInitialPose.gif">

### 4.2. Set Goal

> Click "2D Goal Pose", and set goal to any free space on the map.

<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/SetGoal.gif">

### 4.3. Waypoint Mode

> To Start the Waypoint Mode, click "Waypoint mode" on the bottom left.
Click "Navigation2 Goal", and set the waypoint goals to any free space on the map.
Than, click "Start Navigation" on the bottom left.

<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/WaypointMode.gif">

## 5. Launch Groot

``` bash
cd groot_ws/build/groot
./Groot
```
### 5.1. Start Monitor

> Select "Monitor", and click "START".
Click "Connect" on the left side of the Groot, than you can see the Behavior Tree!

<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/clober_navigation.gif">

## 6. Tuning Guide

### 6.1. Set Navigation2 Parameters

Navigation2 stack has many parameters to change the performances. If you want to improve the navigation's performances, tune the followed basic parameters in the [/param/clober_params.yaml](https://github.com/clobot-git/clober/blob/foxy-devel/clober_navigation/param/clober_params.yaml).
You can get more information about Navigation2's parameters from [Navigation2 Configuration Guide](https://navigation.ros.org/configuration/index.html#).

#### 6.1.1. min_x_veloticy_threshold
- This parameter sets the linear velocity to 0.0 if the odometry values below this threshold(m/s).
#### 6.1.2. min_theta_velocity_threshold
- This parameter sets the angular velocity to 0.0 if the odometry values below this threshold(rad/s).
#### 6.1.3. min_vel_x
- This parameter sets the minimum value of linear velocity(m/s).
#### 6.1.4. max_vel_x
- This parameter sets the maximum value of linear velocity(m/s).
#### 6.1.5. max_vel_theta
- This parameter sets the maximum value of angular velocity(rad/s).
#### 6.1.6. acc_lim_x
- This parameter sets the maximum value of linear acceleration(m/s^2).
#### 6.1.7. acc_lim_theta
- This parameter sets the maximum value of angular acceleration(rad/s^2).
#### 6.1.8. sim_time
- This parameter sets the time to simulate ahead by(s).
- If this parameter is too low, it can't pass the narrow area, and too high, it can't rotate rapidly.
#### 6.1.9. critics
- List of critic plugins to use. These critic plugins decide the controller's behavior according to the each critic's parameters.
- The information of these critics get from [Navigation2 Configuration Guide - DWB Controller's Trajectory Critics](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html#trajectory-critics)
#### 6.1.10. required_movement_radius
- This parameter is used to check the navgation progress.
- This parameter sets the minimum amount a robot must move to be progressing to goal(m).
#### 6.1.11. movement_time_allowance
- This parameter is used to check the navgation progress
- This parameter set the maximum amount of time a robot has to move the minimum radius(s).
#### 6.1.12. xy_goal_tolerance
- This parameter is used to check the goal state.
- This parameter sets the value of tolerance to meet goal completion criteria(m).
#### 6.1.13. yaw_goal_tolerance
- This parameter is used to check the goal state.
- This parameter sets the value of tolerance to meet goal completion criteria(rad).
#### 6.1.14. resolution
- This parameter sets the resolution of 1 pixel of the costmap(m).
#### 6.1.15. robot_radius
- This parameter sets the robot's radius.
- If this parameter is too smaller than the robot's radius, it occurs collision, and too larger, it occurs inefficient path planning.
#### 6.1.16. inflation_radius
- This parameter sets the radius to inflate costmap around lethal obstacles.
- This parameter should be set larger than the robot_radius

### 6.2. Set Navigation2 Behavior Tree

Behavior Tree Navigator module implements the NavigateToPos task interface. This based the [Behavior Tree](https://www.behaviortree.dev/) and the Navigation2 provies the plugins for navigation. To make a [BehaviorTree.xml](https://github.com/clobot-git/clober/blob/foxy-devel/clober_navigation/param/BehaviorTree.xml) file, you can customize the Navigator based on the BT.
You can get more information about Navigation2's plugins for BT from [Navigation2 Configuration Guide - Behavior Tree XML Nodes](https://navigation.ros.org/configuration/packages/configuring-bt-xml.html#)