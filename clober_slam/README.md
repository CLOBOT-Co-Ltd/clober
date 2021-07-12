# Clober SLAM
**Clober may move & rotate for navigation. Make sure you are operating in a safe environment**

The SLAM (Simultaneous Localization and Mapping) is a technique to draw a map by estimating current location in an arbitrary space. The SLAM is a well-known feature of Clober from its predecessors. 

## 1. Run SLAM Nodes
### 1.1 Bringup Robot
1. Run a `Bringup` for the Clober.
  ```bash
  roslaunch clober_bringup base.launch
  ```
- This can be substituted by running a simulation node
  ```bash
  roslaunch clober_simulation logo_world.launch
  ```

### 1.2 Launch SLAM Node
On a new terminal run a SLAM node. Gmapping SLAM is used by default.
  ```bash
  roslaunch clober_slam clober_slam.launch slam_methods:=gmapping
  ```

### 1.3 Run Teleoperation Node
On a new terminal run a teleoperation node to explore and map the world.
  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/clober_slam.gif">


### 1.4 Save Map
After successful SLAM and map creation, open a new terminal to save the map.
  ```bash
  rosrun map_server map_saver -f ~/map
  ```
  If you've saved your map successfully it should look like the following. The map uses two-dimensional Occupancy Grid Map (OGM), which is commonly used in ROS and Navigation.

  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/map.png" width=400>
  White area is collision free area while black area is occupied and inaccessible, and gray area represents the unknown area. 


## 2. Tuning Guide
Gmapping has many parameters to change performances for different environments. You can get an information about whole parameters in [ROS WiKi](http://wiki.ros.org/gmapping). This tuning guide provides tips when configuring gmapping parameters. If you want to optimize SLAM performances for your environments, this section might be helpful.

**Parameters below are defined in clober_slam/config/gmapping_params.yaml file.**

### 2.1 maxUrange
- This parameter is set the maximum usable range of the lidar sensor.

  default :
  ```bash
  maxUrange: 3.0
  ```


### 2.2 map_update_interval
- This parameter defines time between updating the map.
  The smaller the value, the more frequent the map is updated.
  However, setting this too small will be require more processing power for the map calculation. Set this parameter depending on the map environment.

  default :
  ```bash
  map_update_interval: 2.0
  ```

### 2.3 minimumScore
- This parameter sets the minimum score value that determines the success or failure of the sensor’s scan data matching test. This can reduce errors in the expected position of the robot in a large area. 

  default :
  ```bash
  minimumScore: 50
  ```

  If the parameter is set properly, you will see information similar to one shown below.

  ```bash
  Average Scan Matching Score=278.965
  neff= 100
  Registering Scans:Done
  update frame 6
  update ld=2.95935e-05 ad=0.000302522
  Laser Pose= -0.0320253 -5.36882e-06 -3.14142
  ```
  If set too high, you might see below warnings.

  ```bash
  Scan Matching Failed, using odometry. Likelihood=0
  lp:-0.0306155 5.75314e-06 -3.14151
  op:-0.0306156 5.90277e-06 -3.14151
  ```

### 2.4 linearUpdate
- When the robot translates longer distance than this value, it will run the scan process.

  default :
  ```bash
  linearUpdate: 1.0
  ```

### 2.5 angularUpdate
- When the robot rotates more than this value, it will run the scan process. It is recommended to set this value less than linearUpdate.

  default :
  ```bash
  angularUpdate: 0.2
  ```