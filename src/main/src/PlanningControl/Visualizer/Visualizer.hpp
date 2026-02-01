#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// 시각화 함수 선언
void publishCandidatePaths();
void publishVehicleFootprint();
void publishLocalPath();

#endif // VISUALIZER_HPP