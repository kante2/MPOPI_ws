#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>
#include <limits>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <string> 

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// costmap
#include <pcl_ros/transforms.h>
#include <tf2/utils.h>

using namespace std;

// ========================= Type ========================= //
// typedef unsigned char uint8;
// typedef unsigned short uint16;
// typedef unsigned int uint32;
// typedef uint64_t uint64;
// typedef short int16;
// typedef int int32;
// typedef float float32;
// typedef double float64;
// typedef char char8;

struct Point2D 
{
  double x;
  double y;
};

struct LidarParam
{
  // lidar prefilter
  float min_height = -10.0f; // ransac에 방해되지 않도록
  float max_height = 5.0f;
  float lidar_range = 25.0f;

  // ego 제거 ROI (cropbox) 
  float ego_xmin = - 1.0f; 
  float ego_xmax = 1.0f; 
  float ego_ymin = - 1.0f;
  float ego_ymax = 1.0f; 
  float ego_zmin = - 10.0f;
  float ego_zmax = 5.0f; 

  // voxel downsample
  float voxel_leaf = 0.10f;

  // RANSAC
  float ransac_dist_thresh = 0.30f; // 평면에서 0.30m 이내면 지면(inlier)로 간주
  float ransac_eps_angle_deg = 12.0f; // 지면 평면의 법선이 z축에서 10도 이내면 지면으로 인정 
  int ransac_max_iter = 200; 

  // clustering
  float euclidean_tolerance = 0.8f; 
  int euclidean_min_size = 5;
  int euclidean_max_size = 2000; 

};

// Lshapefitting에서 사용하는 파라미터, L피팅 결과
struct LidarCluster
{
  std_msgs::Header header;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cluster_point; // 포인트 클라우드 클러스터 1개

  vector<Point2D> vec_Corners; // obb 직사각형 위치

  float theta; // 최적 angle

  float rotate_rect_min_x; // 직사각형을 축정렬했을 때 나오는 꼭짓점
  float rotate_rect_max_x;
  float rotate_rect_min_y;
  float rotate_rect_max_y;

  float min_z;
  float max_z;

  // ---- extended kalman filter 에서 사용하는 obb 값-----

  int id = -1;           // 객체 고유 ID (초기값 -1)
  float centroid_x;
  float centroid_y;
  float width;
  float length;
  float velocity; // 추정된 속도 (EKF 결과값)
  bool is_updated = false; // 이번 프레임에서 추적 성공 여부
};

// ========================================
// Costmap 파라미터
// ========================================
struct CostmapParams {
  float resolution = 0.05f;
  float width = 50.0f;
  float height = 50.0f;
  int8_t unknown_cost = -1;
  int8_t free_cost = 0;
  int8_t obstacle_cost = 100;
  float inflation_radius = 0.0f;
};

// ========================================
// 런타임 상태 (프레임마다 갱신)
// ========================================
struct CostmapState {
  bool tf_ok{false};
  LidarCluster baselink_cloud; //::PointCloud2 baselink_cloud;
  // std::vector<Detection> detections;
  nav_msgs::OccupancyGrid costmap;
};


// ========================================
// costmap_3D AABB (aabb 대신 obb)
// ========================================
struct Detection 
{

  // -------------- AABB -----------------
  int id{0};
  int num_points; // 클러스터 내 포인트 개수
  ros::Time stamp; // 메시지 타임 스탬프

  Eigen::Vector3f min_pt{0,0,0};
  Eigen::Vector3f max_pt{0,0,0};
  Eigen::Vector3f centroid{0,0,0};
  Eigen::Vector3f size{0,0,0};

  // -------------- OBB ------------------
  // OBB를 costmap에 Eigen::Vector3f 가 아니라 float로 x, y 좌표를 주면?
  LidarCluster st_LidarCluster; 
  // -> st_LidarCluster.vec_Corners 에 obb 위치 저장되어 있음
  
  // Eigen::Vector3f vec_obb_corners[4];
  
};

// ========================================
// kalman filter 결과값
// ========================================

struct KalmanDetection 
{
  int id;
  double x;
  double y;
  double yaw;
  double v;
  double w;
  double l;
  bool is_confirmed;
};


// ======================================================
// 
// ======================================================

struct Lidar
{
  LidarParam st_LidarParam;
  LidarCluster st_LidarCluster;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input_cloud;

  // ========================
  // vector 객체 정보
  // ========================

  vector<LidarCluster> vec_clusters; // 포인트 클라우드 클러스터
  vector<Detection> vec_aabb;


  // ========================
  // 중간 처리 단계별 PointCloud
  // ========================

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filterheight_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filterrange_cloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cropbox_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_voxel_cloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ransac_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_euclidean_cloud;

  Lidar()
  {
    pcl_input_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  
    pcl_filterheight_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_filterrange_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl_cropbox_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_voxel_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl_ransac_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_euclidean_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

  }
};



#endif // GLOBAL_HPP
