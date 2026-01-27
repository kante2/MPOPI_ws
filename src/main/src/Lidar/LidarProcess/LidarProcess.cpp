#include <LidarProcess/LidarProcess.hpp>

MultiObjectTracker kalman;

void LidarProcess(Lidar& st_Lidar, LidarCluster& st_LidarCluster, const std_msgs::Header& header) 
{
    FilterRange(st_Lidar);
    CropBox(st_Lidar);
    Voxel(st_Lidar);
    Ransac(st_Lidar);
    Euclidean(st_Lidar);
    LShapeFitting(st_Lidar);

    for (LidarCluster& cluster : st_Lidar.vec_clusters)
    {
        cluster.header = header;
    }
    
    Costmap(st_Lidar, header.stamp);

    double current_time = header.stamp.toSec();
    kalman.updateTracks(st_Lidar, current_time);

}