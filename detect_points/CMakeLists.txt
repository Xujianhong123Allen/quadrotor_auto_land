cmake_minimum_required(VERSION 2.8.3)
project(detect_points)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native -O3")

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package( G2O REQUIRED)
find_package( CSparse REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    sensor_msgs
  )

include_directories(include ${catkin_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
include_directories( 
  ${OpenCV_INCLUDE_DIRS} 
  ${G2O_INCLUDE_DIRS}
${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/")

# make the executable
cs_add_executable(detect_points src/detect_points.cpp  src/detect_points_node.cpp)

# link the executable to the necesarry libs
target_link_libraries(detect_points
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

cs_install()

# Install other support files for installation
install(TARGETS
detect_points
  DESTINATION lib/${PROJECT_NAME}
)
