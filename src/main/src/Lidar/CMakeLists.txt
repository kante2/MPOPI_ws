cmake_minimum_required(VERSION 3.0.2)
project(Lidar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  morai_msgs
  pcl_ros
  pcl_conversions
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  nav_msgs
  geometry_msgs
  visualization_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
  pcl_ros pcl_conversions 
  laser_geometry 
  tf2 tf2_ros tf2_sensor_msgs
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}include
  ../../Global
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}

  ${CMAKE_CURRENT_SOURCE_DIR}/Visualizer
  ${CMAKE_CURRENT_SOURCE_DIR}/RoiVoxel
  ${CMAKE_CURRENT_SOURCE_DIR}/Ransac
  ${CMAKE_CURRENT_SOURCE_DIR}/Euclidean
  ${CMAKE_CURRENT_SOURCE_DIR}/Lshapefitting
  ${CMAKE_CURRENT_SOURCE_DIR}/Kalman
  ${CMAKE_CURRENT_SOURCE_DIR}/Costmap
  ${CMAKE_CURRENT_SOURCE_DIR}/LidarProcess
)

add_definitions(${PCL_DEFINITIONS})

add_executable(Lidar
  Lidar_main.cpp
  Visualizer/Visualizer.cpp 
  RoiVoxel/RoiVoxel.cpp 
  Ransac/Ransac.cpp 
  Euclidean/Euclidean.cpp 
  Lshapefitting/Lshapefitting.cpp 
  Kalman/Kalman.cpp
  Costmap/Costmap.cpp
  LidarProcess/LidarProcess.cpp 
)

target_link_libraries(Lidar
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_include_directories(Lidar PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)
