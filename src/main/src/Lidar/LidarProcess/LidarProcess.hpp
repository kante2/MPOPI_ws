#ifndef LIDARPROCESS_HPP
#define LIDARPROCESS_HPP

#include "../Global/Global.hpp"
#include "../RoiVoxel/RoiVoxel.hpp"
#include "../Ransac/Ransac.hpp"
#include "../Euclidean/Euclidean.hpp"
#include "../Lshapefitting/Lshapefitting.hpp"
#include "../Costmap/Costmap.hpp"
#include "../Kalman/Kalman.hpp"

// extern MultiObjectTracker g_tracker;

void LidarProcess(Lidar& st_Lidar, LidarCluster& st_LidarCluster, const std_msgs::Header& header);

#endif // LIDARPROCESS_H