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
  INCLUDE_DIRS global
)

include_directories(
  global
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(lattice_test_node
  src/main.cpp
  src/LatticePlanning.cpp
  src/Util_Coordinates.cpp
)

target_link_libraries(lattice_test_node
  ${catkin_LIBRARIES}
)
target_include_directories(lattice_test_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 헤더 의존성 (optional)
add_dependencies(lattice_test_node
  ${catkin_EXPORTED_TARGETS}
)
