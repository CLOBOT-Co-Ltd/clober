# Clober Simulation

## 1. Installation
### 1.1 Install ROS 2.0 (Foxy)
https://docs.ros.org/en/foxy/Installation.html

### 1.2 Install development tool
```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  openssh-server \
  wget
```

### 1.3 Install Gazebo (gazebo 11)
http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros

### 1.4 Install xacro packages

```
sudo apt-get install ros-foxy-xacro
```

### 1.5 Install Clober Packages
```
$ mkdir -p ~/clober_ws/src
cd ~/clober_ws/
wget https://raw.githubusercontent.com/clobot-git/clober/foxy-devel/clober.repos
vcs import src < clober.repos
```

### 1.6 Install other dependencies
```
cd ~/clober_ws/
source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
```

### 1.7 Colcon build the packages
```
cd ~/clober_ws/
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source ~/clober_ws/install/setup.bash
```

## 2. Launch Simulation World
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/clober_ws/src/clober/clober_simulation/models

ros2 launch clober_simulation clober_world.launch.py
```

<img align="center" src="https://github.com/clobot-git/clober/blob/foxy-devel/images/clober_gazebo.png">

## 3. Operate Clober
To teleoperate the simulated Clober with the keyboard, launch the teleoperation node in a new terminal window.
  ```
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
