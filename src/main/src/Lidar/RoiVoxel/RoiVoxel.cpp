#include <RoiVoxel/RoiVoxel.hpp>

void FilterHeight (Lidar& st_Lidar)
{
    pcl::PassThrough<pcl::PointXYZI> filterheight;

    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    filterheight.setInputCloud(st_Lidar.pcl_input_cloud);
    filterheight.setFilterFieldName("z");
    filterheight.setFilterLimits(st_LidarParam.min_height, st_LidarParam.max_height);
    filterheight.filter(*st_Lidar.pcl_filterheight_cloud);
} 

void FilterRange (Lidar& st_Lidar)
{
    
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;
    st_Lidar.pcl_filterrange_cloud -> clear();

    st_Lidar.pcl_filterrange_cloud -> reserve(st_Lidar.pcl_filterheight_cloud->size());
    for (const pcl::PointXYZI &point : *(st_Lidar.pcl_filterheight_cloud))
    {
        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance <= st_LidarParam.lidar_range)
        {
            st_Lidar.pcl_filterrange_cloud->push_back(point);
        }
    }
}

// void Passthrough (Lidar& st_Lidar)
// {
//     pcl::PassThrough<pcl::PointXYZI> x_filter;
//     pcl::PassThrough<pcl::PointXYZI> y_filter;

//     LidarParam st_LidarParam = st_Lidar.st_LidarParam;


//     x_filter.setInputCloud(st_Lidar.pcl_filterrange_cloud);
//     x_filter.setFilterFieldName("x");
//     x_filter.setFilterLimits(st_LidarParam.ego_xmin, st_LidarParam.ego_xmax);
//     x_filter.setFilterLimitsNegative(false);
//     x_filter.filter(*st_Lidar.x_inside);

//     x_filter.setFilterLimitsNegative(true);
//     x_filter.filter(*st_Lidar.x_outside);

//     y_filter.setInputCloud(st_Lidar.x_inside);
//     y_filter.setFilterFieldName("y");
//     y_filter.setFilterLimits(st_LidarParam.ego_ymin, st_LidarParam.ego_ymax);
//     y_filter.setFilterLimitsNegative(true);
//     y_filter.filter(*st_Lidar.pcl_passthrough_cloud);

//     *st_Lidar.pcl_passthrough_cloud += *st_Lidar.x_outside;
// }

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