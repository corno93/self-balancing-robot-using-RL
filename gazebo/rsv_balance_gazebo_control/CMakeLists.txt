cmake_minimum_required(VERSION 2.8.3)
project(rsv_balance_gazebo_control)

find_package(catkin REQUIRED COMPONENTS
  roslint
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES rsv_balance_gazebo_control
)

add_library(rsv_balance_gazebo_control src/balance_gazebo_control.cpp src/control.cpp)

install(
  TARGETS 
    rsv_balance_gazebo_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

roslint_cpp()

