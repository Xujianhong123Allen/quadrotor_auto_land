# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(libuvc_camera)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs)

find_package(libuvc REQUIRED) #原因是libuvc库存在问题,没有libuvc.cmake文件,find_package()无法找到libuvc库文件
#解决办法手动添加库文件位置
set(libuvc_INCLUDE_DIRS "/usr/local/include/libuvc")
set(libuvc_LIBRARIES "/usr/local/lib/libuvc.so")

generate_dynamic_reconfigure_options(cfg/UVCCamera.cfg)

find_package(libuvc REQUIRED)
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    nodelet
    sensor_msgs
  LIBRARIES libuvc_camera_nodelet
  )

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})
include_directories(include ${libuvc_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(camera_node src/main.cpp src/camera_driver.cpp)
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

add_library(libuvc_camera_nodelet src/nodelet.cpp src/camera_driver.cpp)
add_dependencies(libuvc_camera_nodelet ${libuvc_camera_EXPORTED_TARGETS})
target_link_libraries(libuvc_camera_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(libuvc_camera_nodelet ${PROJECT_NAME}_gencfg)

install(TARGETS camera_node libuvc_camera_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES libuvc_camera_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  

