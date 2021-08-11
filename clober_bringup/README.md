# Install development tools
Install development tools for a smooth installation.
```
sudo apt install curl git
```

# 1. Installation
### 1.1 Install ROS 1.0 (Noetic)
http://wiki.ros.org/noetic/Installation


### 1.3 Install related packages
```
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt-get install libgeographic-dev
sudo apt-get install ros-noetic-geographic-msgs
```

### 1.4 Install Clober Packages
```
$ mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
wget https://raw.githubusercontent.com/CLOBOT-Co-Ltd/clober/noetic-devel/clober.repos
vcs import src < clober.repos
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

### 1.5 Install other dependencies
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
```

## 2. Launch Bringup package
```bash
cd ~/catkin_ws/src/clober/clober_bringup/rules
./create_udev_rules.sh
roslaunch clober_bringup base.launch
```

