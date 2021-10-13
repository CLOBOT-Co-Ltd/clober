# Clober MBF(Move Base Flex)
**Clober may move & rotate for navigation. Make sure you are operating in a safe environment**

MBF is a highly flexible navigation framework. See this [*link*](http://wiki.ros.org/move_base_flex) to have more information of MBF

clober support mbf based navigation, so that it makes Clober can deal with more complex tasks.

[*SLAM of the environment allow you to acquire a map*](https://github.com/clobot-git/clober/tree/noetic-devel/clober_slam)

## 1. Run MBF Navigation Nodes
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
roslaunch clober_mbf navigation.launch
```
