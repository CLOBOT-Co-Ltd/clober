# Clober Navigation

## 1. Run Clober (Simulation)
### 1.1 Launch Simulation World
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/clober_ws/src/clober/clober_simulation/models

source /opt/ros/foxy/setup.bash
source ~/clober_ws/install/setup.bash
ros2 launch clober_simulation clober_world.launch.py
```

## 2. Launch Navigation
```bash
source /opt/ros/foxy/setup.bash
source ~/clober_ws/install/setup.bash
ros2 launch clober_navigation bringup.launch.py
```

### 2.1 Set Initialpose

> Click "2D Pose Estimate", and set estimation to the approximate location of robot on the map.

### 2.2 Set Goal

> Click "2D Nav Goal", and set goal to any free space on the map.


<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/clober_navigation.gif">
