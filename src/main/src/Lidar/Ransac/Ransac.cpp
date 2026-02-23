#include "Ransac.hpp"

void Ransac(Lidar& st_Lidar)
{
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    if (!st_Lidar.pcl_voxel_cloud || st_Lidar.pcl_voxel_cloud -> empty()) {
        return;
    }
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(st_LidarParam.ransac_dist_thresh);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));  // z축에 수직(=지면)
    seg.setEpsAngle(static_cast<float>(st_LidarParam.ransac_eps_angle_deg * M_PI / 180.0));
    seg.setMaxIterations(st_LidarParam.ransac_max_iter);
    seg.setInputCloud(st_Lidar.pcl_voxel_cloud);
    seg.segment(*inliers, *coefficients);

    if (!inliers->indices.empty()) {
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(st_Lidar.pcl_voxel_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);   // outlier(비지면)만 남김
        extract.filter(*st_Lidar.pcl_ransac_cloud);
    }
}