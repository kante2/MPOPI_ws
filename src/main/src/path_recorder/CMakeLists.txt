cmake_minimum_required(VERSION 3.0.2)
project(path_recorder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  morai_msgs
  geodesy
  geographic_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp morai_msgs geodesy geographic_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_recorder_node
  path_recorder.cpp
)

target_link_libraries(path_recorder_node
  ${catkin_LIBRARIES}
)