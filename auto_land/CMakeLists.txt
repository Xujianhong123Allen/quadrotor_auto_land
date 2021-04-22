cmake_minimum_required(VERSION 2.8.3)
project(auto_land)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(auto_land src/auto_land.cpp)
target_link_libraries(auto_land ${catkin_LIBRARIES})