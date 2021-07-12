# Install development tools
Install development tools for a smooth installation.
```
sudo apt install curl git
```

# Clober Simulations
This Gazebo Simulation utilizes the ROS Gazebo package, Gazebo version 11 for ROS1 noetic has to be properly installed before running this instruction, such that we recommend you installing the full version of ROS noetic.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 1. Installation
### 1.1 Install Required ROS 1 Packages
Install the needed packages used in the simulation environment. 
  ```bash
  sudo apt-get install ros-noetic-teleop-twist-keyboard \
    ros-noetic-gmapping \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-navigation \
    ros-noetic-move-base \
    ros-noetic-dwa-local-planner
  ```

### 1.2 Install Clober Packages
The Clober Simulation Package requires `clober` & `clober_msgs` packages. Without the package the simulation cannot be launched.
  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src/
  git clone -b noetic-devel https://github.com/clobot-git/clober.git
  git clone -b noetic-devel https://github.com/clobot-git/clober_msgs.git
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

## 2. Launch Simulation World
Several Simulation environments are prepared, made accessible by each launch file.

### 2.1 Empty World
<img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/empty_world.png">

  ```bash
  roslaunch clober_simulation empty_world.launch
  ```

### 2.2 Clobot Logo World
<img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/logo_world.png">

  ```bash
  roslaunch clober_simulation logo_world.launch
  ```

### 2.3 Warehouse World
<img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/warehouse_world.png">

  ```bash
  roslaunch clober_simulation warehouse_env_world.launch
  ```

### 2.4 Grid World
<img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/grid_world.png">

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

  [*For fine tuning of the SLAM package please review the SLAM package README.md*](https://github.com/clobot-git/clober/tree/noetic-devel/clober_slam)

### 4.1 Launch Simulation World
Out of the worlds prepared, we recommend using the Clobot Logo World.
  ```bash
  roslaunch clober_simulation logo_world.launch
  ``` 

### 4.2 Launch SLAM Node
On a new terminal run a SLAM node. Gmapping SLAM is used by default.
  ```bash
  roslaunch clober_slam clober_slam.launch slam_methods:=gmapping
  ```

### 4.3 Run Teleoperation Node
On a new terminal run a teleoperation node to explore and map the Gazebo world.
  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/gifs/clober_slam.gif">


### 4.4 Save Map
After successful SLAM and map creation, open a new terminal to save the map.
  ```bash
  rosrun map_server map_saver -f ~/map
  ```
  If you've saved your map successfully it should look like the following.

  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/map.png" width=400>

## 5. Navigation Simulation
For Navigation in Gazebo simulator navigation works pretty similar under low velocity circumstances.

  [*For fine tuning of the Navigation package please review the Navgiation package README.md*](https://github.com/clobot-git/clober/tree/noetic-devel/clober_navigation)

### 5.1 Launch Simulation World
We recommend using the Clobot Logo World for Navigation simulation
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
    <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/2d_pose_estimate.png">

  2. Click on the map where clober is located and drag the green arrow toward the dirction the robot is facing.

  3. Repeat step 1 and 2 to increase data precision.

  4. Launch keyboard teleoperation to further increase precision.
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

  5. Move the robot back and forth in order to narrow down the estimated location.
    <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/gifs/clober_amcl.gif">

  6. Terminate the keyboard teleoperation(`Ctrl`+`C`) for the next navigation step. 

### 5.4 Publish 2D Navigation Goal
- On the RVIZ menu click `2D Nav Goal`
  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/2d_nav_goal.png">

- Click on the destination and drag the green arrow toward the dirction of the robot on the map.
  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/gifs/clober_navigation.gif">
