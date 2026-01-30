cmake_minimum_required(VERSION 3.0.2)
project(gps_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  morai_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ========================================
# GPS Control Node (WIP - 구조체 및 전역변수 정의 필요)
# ========================================

# add_executable(gps_control_node
#   src/gps_control.cpp
# )

# target_link_libraries(gps_control_node
#   ${catkin_LIBRARIES}
# )

# add_dependencies(gps_control_node
#   ${catkin_EXPORTED_TARGETS}
#   morai_msgs_generate_messages_cpp
# )
