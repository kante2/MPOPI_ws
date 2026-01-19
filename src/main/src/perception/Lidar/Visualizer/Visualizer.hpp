#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

// #include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Global/Global.hpp>

using namespace std;

void PublishPointCloud (const ros::Publisher& pub, 
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                        const std_msgs::Header& header);

void PublishCluster (const ros::Publisher& pub, 
                    const vector<LidarCluster>& vec_clusters, 
                    const std_msgs::Header& header);

vector<Detection> ClusterInfo (
  const vector<LidarCluster> &vec_clusters, 
  const std_msgs::Header& header);

void PublishBoundingBox (const ros::Publisher &pub_bounding_box,
                        Lidar& st_Lidar,
                        const std_msgs::Header& header);

void PublishOBBLineStrip (const ros::Publisher &pub_bounding_box,
                          Lidar& st_Lidar,
                          const std_msgs::Header& header);

void PublishKalman(const ros::Publisher& pub, 
                  const std::vector<KalmanDetection>& detections, 
                  const std_msgs::Header& header);
#endif