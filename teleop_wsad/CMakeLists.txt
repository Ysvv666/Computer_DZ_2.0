cmake_minimum_required(VERSION 2.8.3)
project(teleop_wsad)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/wsad_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
