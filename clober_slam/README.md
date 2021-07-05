# Clober SLAM

## 1. Run Clober (Simulation)
### 1.1 Launch Simulation World
```bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/clober_ws/src/clober/clober_simulation/models

source /opt/ros/foxy/setup.bash
source ~/clober_ws/install/setup.bash
ros2 launch clober_simulation clober_world.launch.py
```

### 1.2 Operate Clober
To teleoperate the simulated Clober with the keyboard, launch the teleoperation node in a new terminal window.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2. Launch SLAM
### 2.1  Cartographer SLAM

install cartographer package :
```bash
sudo apt-get install ros-foxy-cartographer*
```

Launch Cartographer SLAM :

```bash
source /opt/ros/foxy/setup.bash
source ~/clober_ws/install/setup.bash
ros2 launch clober_slam cartographer.launch.py
```

<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/clober_slam.png" width=400>

### 2.2  Gmapping SLAM

install slam_gmapping package :
```bash
git clone https://github.com/clobot-git/slam_gmapping.git
```

* original slam_gmapping package source : https://github.com/Adlink-ROS/slam_gmapping

Launch Gmapping SLAM :

```bash
source /opt/ros/foxy/setup.bash
source ~/clober_ws/install/setup.bash
ros2 launch clober_slam gmapping.launch.py
```
<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/clober_slam.gif">


## 3. Save the map
```bash
source /opt/ros/foxy/setup.bash
ros2 run nav2_map_server map_saver_cli -f (filename) --ros-args -p save_map_timeout:=10000
```

<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/map_3x3.png" width=400>


