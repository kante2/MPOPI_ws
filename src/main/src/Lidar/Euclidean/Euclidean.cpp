#include "Euclidean.hpp"
#include "../Visualizer/Visualizer.hpp"

void Euclidean (Lidar &st_Lidar) 
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterrange_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean;
    vector<pcl::PointIndices> vec_cluster_indices;
    
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cluster_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cluster_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    
    vector<LidarCluster> vec_cluster;
    LidarCluster st_LidarCluster;

    pcl::PointXYZI pcl_point;
    
    int i, j;
    int pointidx;
    int intensity = 0;

    if (!st_Lidar.pcl_ransac_cloud -> empty())
    {
        kdtree->setInputCloud(st_Lidar.pcl_ransac_cloud);

        euclidean.setInputCloud(st_Lidar.pcl_ransac_cloud);
        euclidean.setSearchMethod(kdtree);
        euclidean.setClusterTolerance(st_LidarParam.euclidean_tolerance);
        euclidean.setMinClusterSize(st_LidarParam.euclidean_min_size);
        euclidean.setMaxClusterSize(st_LidarParam.euclidean_max_size);
        euclidean.extract(vec_cluster_indices); 
    }
    
    // 인덱스를 포인트 클라우드로 변환해서 클러스터 단위로 저장
    for (i = 0; i < vec_cluster_indices.size(); i++)
    {
        st_LidarCluster.pcl_cluster_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
        for (j = 0; j < vec_cluster_indices[i].indices.size(); j++)
        {
            pointidx = vec_cluster_indices[i].indices[j];

            pcl::PointXYZI pt = st_Lidar.pcl_ransac_cloud->points[pointidx];

            // pt.intensity = (float)(i % 10); 

            st_LidarCluster.pcl_cluster_point->push_back(pt);
        }
        vec_cluster.push_back(st_LidarCluster);
    }
    st_Lidar.vec_clusters = vec_cluster;
}

// euclidean 반환값 : 포인트 클라우드 좌표로 이루어진 클러스터를 묶어서 vector에 담아 반환
