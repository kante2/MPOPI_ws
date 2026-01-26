#include <Global/Global.hpp>
#include <LidarProcess/LidarProcess.hpp>
#include <Visualizer/Visualizer.hpp>
#include <Costmap/Costmap.hpp>

Lidar st_Lidar;
LidarCluster st_LidarCluster;

ros::Publisher pub_filtered_range;
ros::Publisher pub_cropbox;
ros::Publisher pub_voxel;
ros::Publisher pub_ransac;
ros::Publisher pub_cluster_all;
ros::Publisher pub_bounding_box;
ros::Publisher pub_OBB_bounding_box;
ros::Publisher pub_kalman;
ros::Publisher pub_heading;

ros::Subscriber sub;

void LidarCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg (*msg, *st_Lidar.pcl_input_cloud);

    LidarProcess(st_Lidar, st_LidarCluster, msg -> header);

    PublishPointCloud (pub_filtered_range, st_Lidar.pcl_filterrange_cloud, msg -> header);
    PublishPointCloud (pub_cropbox, st_Lidar.pcl_cropbox_cloud, msg -> header);
    PublishPointCloud (pub_voxel, st_Lidar.pcl_voxel_cloud, msg -> header);
    PublishPointCloud (pub_ransac, st_Lidar.pcl_ransac_cloud, msg -> header);
    PublishCluster (pub_cluster_all, st_Lidar.vec_clusters, msg -> header);
    PublishBoundingBox (pub_bounding_box, st_Lidar, msg -> header);
    PublishOBBLineStrip (pub_OBB_bounding_box, st_Lidar, msg -> header);
    PublishHeading(pub_heading, st_Lidar, msg->header);
    PublishKalman(pub_kalman, st_Lidar.vec_kalman_clusters, msg->header);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LidarNode");
    ros::NodeHandle nh;

    initCostmapModule(nh); 

    ros::Rate loop_rate(10);

    pub_filtered_range = nh.advertise<sensor_msgs::PointCloud2>("/filtered_range", 1);
    pub_cropbox = nh.advertise<sensor_msgs::PointCloud2>("/cropbox", 1);
    pub_voxel = nh.advertise<sensor_msgs::PointCloud2>("/voxel", 1);
    pub_ransac = nh.advertise<sensor_msgs::PointCloud2>("/ransac", 1);
    pub_cluster_all = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 1);
    pub_bounding_box = nh.advertise<visualization_msgs::MarkerArray>("/aabb", 1);
    pub_OBB_bounding_box = nh.advertise<visualization_msgs::MarkerArray>("/obb", 1);
    pub_costmap = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
    pub_heading = nh.advertise<visualization_msgs::MarkerArray>("/heading", 1);
    pub_kalman = nh.advertise<visualization_msgs::MarkerArray>("/kalman", 1);

    sub = nh.subscribe("/lidar3D", 1, LidarCallback);

    ros::spin();

    return 0;
}