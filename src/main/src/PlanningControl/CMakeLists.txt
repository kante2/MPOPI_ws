cmake_minimum_required(VERSION 3.0.2)
project(PlanningControl)

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
  INCLUDE_DIRS Global Planning Control Visualizer  # 수정!
)

include_directories(
  Global
  Planning     
  Control      
  Visualizer   
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ========================================
# Lattice Planning Node
# ========================================

add_executable(lattice_test_node
  main.cpp
  Global/parameter_loader.cpp 
  Global/Global.cpp             
  Planning/LatticePlanning.cpp
  Planning/JammingPlanning.cpp
  Planning/mpopisPlanning.cpp
  Planning/Util_Coordinates.cpp
  Control/Control.cpp
  Visualizer/Visualizer.cpp
)

target_link_libraries(lattice_test_node
  ${catkin_LIBRARIES}
)

# 헤더 의존성
add_dependencies(lattice_test_node
  ${catkin_EXPORTED_TARGETS}
  morai_msgs_generate_messages_cpp
)