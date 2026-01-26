#include <RoiVoxel/RoiVoxel.hpp>

void FilterRange (Lidar& st_Lidar)
{
    pcl::CropBox<pcl::PointXYZI> cropbox;
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    cropbox.setInputCloud(st_Lidar.pcl_input_cloud);

    Eigen::Vector4f min_pt(st_LidarParam.lidar_range_xmin, st_LidarParam.lidar_range_ymin, st_LidarParam.lidar_range_zmin, 1.0f);
    cropbox.setMin(min_pt);

    Eigen::Vector4f max_pt(st_LidarParam.lidar_range_xmax, st_LidarParam.lidar_range_ymax, st_LidarParam.lidar_range_zmax, 1.0f);
    cropbox.setMax(max_pt);

    cropbox.setNegative(false);

    cropbox.filter(*st_Lidar.pcl_filterrange_cloud);
}

void CropBox (Lidar& st_Lidar)
{
    pcl::CropBox<pcl::PointXYZI> cropbox;
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;
    
    cropbox.setInputCloud(st_Lidar.pcl_filterrange_cloud);
 
    Eigen::Vector4f min_pt(st_LidarParam.ego_xmin, st_LidarParam.ego_ymin, st_LidarParam.ego_zmin, 1.0f);
    cropbox.setMin(min_pt);

    Eigen::Vector4f max_pt(st_LidarParam.ego_xmax, st_LidarParam.ego_ymax, st_LidarParam.ego_zmax, 1.0f);
    cropbox.setMax(max_pt);

    cropbox.setNegative(true);

    cropbox.filter(*st_Lidar.pcl_cropbox_cloud);
}

void Voxel (Lidar& st_Lidar)
{
    pcl::VoxelGrid<pcl::PointXYZI> pcl_voxel_filter;
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    pcl_voxel_filter.setInputCloud(st_Lidar.pcl_cropbox_cloud);
    pcl_voxel_filter.setLeafSize(st_LidarParam.voxel_leaf, st_LidarParam.voxel_leaf, st_LidarParam.voxel_leaf);
    pcl_voxel_filter.filter(*st_Lidar.pcl_voxel_cloud);
}