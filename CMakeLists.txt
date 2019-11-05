cmake_minimum_required(VERSION 2.8.3)
project(lidar_txt2bag)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rosbag
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_txt2bag
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lidar_txt2bag_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(lidar_change_frame_node src/lidar_change_frame.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_change_frame_node
  ${catkin_LIBRARIES}
)
