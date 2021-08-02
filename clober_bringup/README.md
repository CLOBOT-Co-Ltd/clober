# clober_serial

# Install development tools
Install development tools for a smooth installation.
```
sudo apt install curl git
```

# 1. Installation
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

### 1.3 Install related packages
```
sudo apt-get install ros-foxy-xacro
sudo apt-get install ros-foxy-teleop-twist-keyboard
sudo apt-get install ros-foxy-diagnostic-updater
sudo apt-get install ros-foxy-geographic-msgs
sudo apt-get install libgeographic-dev
```

### 1.4 Install Clober Packages
```
$ mkdir -p ~/clober_ws/src
cd ~/clober_ws/
wget https://raw.githubusercontent.com/CLOBOT-Co-Ltd/clober/foxy-devel/clober.repos
vcs import src < clober.repos
colcon build --symlink-install
```

### 1.5 Install other dependencies
```
cd ~/clober_ws/
source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
```

## 2. Launch Bringup package
```bash
ros2 launch clober_bringup clober_bringup.launch.py
```

