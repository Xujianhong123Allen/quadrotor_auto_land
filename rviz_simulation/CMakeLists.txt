cmake_minimum_required(VERSION 3.5)
project(rviz_simulation)

find_package(catkin_simple REQUIRED)
catkin_simple()
# Default to C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

add_executable(rviz_simulation src/rviz_simulation.cpp src/rviz_simulation_node.cpp)

target_link_libraries(rviz_simulation
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS
  rviz_simulation
  DESTINATION lib/${PROJECT_NAME}
)


