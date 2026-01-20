cmake_minimum_required(VERSION 3.0.2)
project(lattice_test)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  morai_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(lattice_test_node
  src/main.cpp
  src/LatticePlanning.cpp
  # 만약 변환/유틸 구현 cpp가 따로 있으면 여기에 추가!
  # src/transform_utils.cpp
)

target_link_libraries(lattice_test_node
  ${catkin_LIBRARIES}
)

# 헤더 의존성 (optional)
add_dependencies(lattice_test_node
  ${catkin_EXPORTED_TARGETS}
)
