#ifndef LIDARPROCESS_HPP
#define LIDARPROCESS_HPP

#include <Global/Global.hpp>
#include <Lidar/RoiVoxel/RoiVoxel.hpp>
#include <Lidar/Ransac/Ransac.hpp>
#include <Lidar/Euclidean/Euclidean.hpp>
#include <Lidar/Lshapefitting/Lshapefitting.hpp>
#include <Lidar/Costmap/Costmap.hpp>
#include <Lidar/Kalman/Kalman.hpp>

// extern MultiObjectTracker g_tracker;

void LidarProcess(Lidar& st_Lidar, LidarCluster& st_LidarCluster, const std_msgs::Header& header);

#endif // LIDARPROCESS_H