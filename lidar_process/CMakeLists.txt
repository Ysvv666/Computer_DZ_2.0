cmake_minimum_required(VERSION 3.0.2)
project(lidar_process)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs move_base_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lidar_loc
  src/lidar_loc.cpp
)
add_dependencies(lidar_loc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_loc
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(lidar_filter_node
  src/lidar_filter_node.cpp
)
add_dependencies(lidar_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_filter_node
  ${catkin_LIBRARIES}
)