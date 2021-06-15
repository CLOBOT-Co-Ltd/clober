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
    <img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/2d_pose_estimate.png">

  2. Click on the map where clober is located and drag the green arrow toward the dirction the robot is facing.

  3. Repeat step 1 and 2 to increase data precision.

  4. Launch keyboard teleoperation to further increase precision.
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

  5. Move the robot back and forth in order to narrow down the estimated location.
    [<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/gifs/clober_amcl.gif">](https://youtube.com)

  6. Terminate the keyboard teleoperation(`Ctrl`+`C`) for the next navigation step. 

## 3. Publish 2D Navigation Goal
- On the RVIZ menu click `2D Nav Goal`
  <img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/images/2d_nav_goal.png">

- Click on the destination and drag the green arrow toward the dirction of the robot on the map.
  [<img align="center" src="https://github.com/clobot-git/testrobot/blob/noetic-devel/gifs/clober_navigation.gif">](https://youtube.com/)


---

## 4. Tuning Guide
Navigation stack has many parameters to change performances for different robots.

You can get more information about Navigation tuning from [Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide), [ROS Navigation Tuning Guide by Kaiyu Zheng](https://kaiyuzheng.me/documents/navguide.pdf).

### 4.1 inflation_radius
- Defined in clober_navigation/param/costmap_common_param_${TB3_MODEL}.yaml
  This parameter makes inflation area from the obstacle. Path would be planned in order that it don’t across this area. It is safe that to set this to be bigger than robot radius. For more information, please refer to the costmap_2d wiki.


### 4.2 cost_scaling_factor
- Defined in turtlebot3_navigation/param/costmap_common_param_${TB3_MODEL}.yaml
  This factor is multiplied by cost value. Because it is an reciprocal propotion, this parameter is increased, the cost is decreased.


  The best path is for the robot to pass through a center of between obstacles. Set this factor to be smaller in order to far from obstacles.

### 4.3 max_vel_x
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  This factor is set the maximum value of translational velocity.

### 4.4 min_vel_x
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  This factor is set the minimum value of translational velocity. If set this negative, the robot can move backwards.

### 4.5 max_trans_vel
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  Actual value of the maximum translational velocity. The robot can not be faster than this.

### 4.6 min_trans_vel
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  Actual value of the minimum translational velocity. The robot can not be slower than this.

#### 4.7 max_rot_vel
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  Actual value of the maximum rotational velocity. The robot can not be faster than this.

### 4.8 min_rot_vel
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  Actual value of the minimum rotational velocity. The robot can not be slower than this.

### 4.9 acc_lim_x
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  Actual value of the translational acceleration limit.

### 4.10 acc_lim_theta
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  Actual value of the rotational acceleration limit.

### 4.11 xy_goal_tolerance
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  The x,y distance allowed when the robot reaches its goal pose.

### 4.12 yaw_goal_tolerance
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  The yaw angle allowed when the robot reaches its goal pose.

### 4.13 sim_time
- Defined in turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml
  This factor is set forward simulation in seconds. Too low value is in sufficient time to pass narrow area and too high value is not allowed rapidly rotates. You can watch defferences of length of the yellow line in below image.