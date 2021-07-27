cmake_minimum_required(VERSION 3.0.2)
project(clober_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  tf
)

find_package(gazebo REQUIRED)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 DEPENDS gazebo_ros roscpp std_msgs sensor_msgs geometry_msgs tf
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY launch models worlds map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

