# Clober Navigation
**Clober may move & rotate for navigation. Make sure you are operating in a safe environment**

Navigation is to move a robot from one location to a specified destination of the given environemnt. For the purpose, a map containing the geometrical information of the robots surroundings is required.

[*SLAM of the environment allow you to acquire a map*](https://github.com/clobot-git/clober/tree/noetic-devel/clober_slam)

## 1. Run Navigation Nodes
### 1.1 Bringup Robot
1. Run a `Bringup` for the Clober.
  ```bash
  roslaunch clober_bringup base.launch
  ```
- This can be substituted by running a simulation node
  ```bash
  roslaunch clober_simulation logo_world.launch
  ```

### 1.2 Launch Navigation
```bash
roslaunch clober_navigation navigation.launch
```

## 2. Estimate Initial Pose
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
    <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/clober_amcl.gif">

  6. Terminate the keyboard teleoperation(`Ctrl`+`C`) for the next navigation step. 

## 3. Publish 2D Navigation Goal
- On the RVIZ menu click `2D Nav Goal`
  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/2d_nav_goal.png">

- Click on the destination and drag the green arrow toward the dirction of the robot on the map.
  <img align="center" src="https://github.com/clobot-git/clober/blob/noetic-devel/images/clober_navigation.gif">


---

## 4. Tuning Guide
You can get more information about Navigation tuning from [Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide), [ROS Navigation Tuning Guide by Kaiyu Zheng](https://kaiyuzheng.me/documents/navguide.pdf), [Dynamic Window Approach local planner wiki](http://wiki.ros.org/dwa_local_planner).

### 4.1 footprint
- Defined in clober_navigation/config/costmap_common.yaml

  This parameter describes the footprint of the robot used for navigation. Composed of four [x,y] coordinates representing the four corners of the base of the robot

  default: 

    ```bash
    footprint: [[-0.202, -0.202], [-0.202, 0.202], [0.202, 0.202], [0.202, -0.202]]
    ```

### 4.2 inflation_radius
- Defined in clober_navigation/config/global_costmap.yaml & clober_navigation/config/local_costmap.yaml

  This parameter makes inflation area from the obstacle. Path would be planned in order that it don’t across this area. It is safe that to set this to be bigger than robot radius. For more information, please refer to the [costmap_2d wiki](http://wiki.ros.org/costmap_2d).

  default:

    global_costmap.yaml: 

      inflation_radius: 0.3

    local_costmap.yaml:

      inflation_radius: 0.15
      

### 4.3 cost_scaling_factor
- Defined in clober_navigation/config/global_costmap.yaml & clober_navigation/config/local_costmap.yaml

  This factor is multiplied by cost value. Because it is an reciprocal propotion, this parameter is increased, the cost is decreased.

  The best path is for the robot to pass through a center of between obstacles. Set this factor to be smaller in order to far from obstacles. For more information, please refer to the [costmap_2d wiki](http://wiki.ros.org/costmap_2d)

  default:

    global_costmap.yaml: 

      inflation_radius: 3.0

    local_costmap.yaml:

      inflation_radius: 1.5

### 4.4 base_global_planner
- Defined in clober_navigation/config/move_base.yaml
  
  This parameter allows you to select which global planner to use as a plugin for path planning

  default: 
    ```bash
    base_global_planner: navfn/NavfnROS
    ```

### 4.5 base_local_planner
- Defined in clober_navigation/config/move_base.yaml
  
  This parameter allows you to select which local planner to use as a plugin for path planning

  default:
    ```bash
    base_local_planer: dwa_local_planner/DWAPlannerROS
    ```

### 4.6 max_vel_x
- Defined in clober_navigation/config/dwa_local_planner.yaml

  This factor is set the maximum value of translational velocity.

  default: 
  
    ```bash
    max_vel_x: 0.26
    ```

### 4.7 max_vel_x_backwards
- Defined in clober_navigation/config/dwa_local_planner.yaml

  Maximum absolute translational velocity of the robot while driving backwards in meters/sec. 
  
  See optimization parameter weight_kinematics_forward_drive

  default: 

    ```bash
    max_vel_x_backwards: -0.26
    ```

### 4.8 acc_lim_x
- Defined in clober_navigation/config/dwa_local_planner.yaml
  
  Maximum translational acceleration of the robot in meters/sec^2

  default: 

    ```bash
    acc_lim_x: 2.5
    ```

### 4.9 acc_lim_theta
- Defined in clober_navigation/config/dwa_local_planner.yaml
  
  Maximum angular acceleration of the robot in radians/sec^2

  default: 

    ```bash
    acc_lim_theta: 3.2
    ```

### 4.10 xy_goal_tolerance
- Defined in clober_navigation/config/dwa_local_planner.yaml
  
  Allowed final euclidean distance to the goal position in meters

  default: 

    ```bash
    xy_goal_tolerance: 0.05
    ```

### 4.11 yaw_goal_tolerance
- Defined in clober_navigation/config/dwa_local_planner.yaml
  
  Allowed final orientation error in radians

  default:

    ```bash
    yaw_goal_tolerance: 0.17
    ```
