cmake_minimum_required(VERSION 2.8.3)
project(libuvc_ros)
find_package(catkin REQUIRED)
catkin_metapackage()

set(libuvc_INCLUDE_DIRS “/usr/local/include/libuvc”)
set(libuvc_LIBRARIES “/usr/local/lib/libuvc.so”)
