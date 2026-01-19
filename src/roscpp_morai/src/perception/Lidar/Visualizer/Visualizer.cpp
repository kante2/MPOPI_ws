#include <Visualizer/Visualizer.hpp>

// ===============================================================================
// pointcloud 시각화
// ===============================================================================

void PublishPointCloud (const ros::Publisher& pub, 
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                        const std_msgs::Header& header)
{
    if (cloud->empty()) return;
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud, ros_msg);
    ros_msg.header = header;
    pub.publish(ros_msg);
}

// ===============================================================================
// cluster 시각화
// ===============================================================================

void PublishCluster(const ros::Publisher& pub, 
                    const vector<LidarCluster>& vec_clusters, 
                    const std_msgs::Header& header)
{
    if (vec_clusters.empty()) return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < vec_clusters.size(); ++i)
    {
        // 2. 클러스터 번호(i)를 이용해 고유 색상 생성 (간단한 수식)
        uint8_t r = (i * 70) % 255;
        uint8_t g = (i * 150) % 255;
        uint8_t b = (i * 210) % 255;

        for (const pcl::PointXYZI& pt_in : vec_clusters[i].pcl_cluster_point->points)
        {
            pcl::PointXYZRGB pt_out;
            pt_out.x = pt_in.x;
            pt_out.y = pt_in.y;
            pt_out.z = pt_in.z;
            pt_out.r = r; pt_out.g = g; pt_out.b = b;
            colored_cloud->push_back(pt_out);
        }
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = header;

    pub.publish(output);
}

// =========================================================
// 클러스터 정보 계산
// =========================================================

// pointcloud로 구성된 클러스터
vector<Detection> ClusterInfo (
  const vector<LidarCluster> &vec_clusters, 
  const std_msgs::Header& header)
{
  vector<Detection> vec_cluster_info;
  vec_cluster_info.reserve(vec_clusters.size());

  // ============== 가드레일, 건물 제거 =================
  // const float max_limit = 9.0f; // 6m 이상 제외

  // =========== 장애물이 아닌 클러스터 제거 ==============
//   const float min_limit = 0.1f; 

  for (int i = 0; i < (int)vec_clusters.size(); ++i) {

    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud = vec_clusters[i].pcl_cluster_point;
    if (cluster_cloud->empty()) continue;

    float min_x =  numeric_limits<float>::infinity();
    float min_y =  numeric_limits<float>::infinity();
    float min_z =  numeric_limits<float>::infinity();
    float max_x = -numeric_limits<float>::infinity();
    float max_y = -numeric_limits<float>::infinity();
    float max_z = -numeric_limits<float>::infinity();

    for (const pcl::PointXYZI& p : *cluster_cloud) {
      min_x = min(min_x, p.x); min_y = min(min_y, p.y); min_z = min(min_z, p.z);
      max_x = max(max_x, p.x); max_y = max(max_y, p.y); max_z = max(max_z, p.z);
    }

    // ================ 가드레일, 건물, 극소형 장애물 제거 위해 ==============
    // 1. 크기 계산
    // float size_x = max_x - min_x;
    // float size_y = max_y - min_y;
    // float size_z = max_z - min_z;

    // 2. 구조물 필터링 조건
    // if (size_x > max_limit || size_y > max_limit) continue;
    // if (size_z < min_limit) continue;
    // ================================================================

    Detection d;
    d.id = i;
    d.stamp = header.stamp;
    d.centroid = Eigen::Vector3f((min_x + max_x)/2, (min_y + max_y)/2, (min_z + max_z)/2);
    d.size = Eigen::Vector3f(max_x - min_x, max_y - min_y, max_z - min_z);
    d.max_pt = {max_x, max_y, max_z};
    d.min_pt = {min_x, min_y, min_z};

    vec_cluster_info.push_back(d);
  }
  return vec_cluster_info;
}



// =========================================================
// bounding box 시각화
// =========================================================

void PublishBoundingBox (const ros::Publisher &pub_bounding_box,
                        Lidar& st_Lidar,
                        const std_msgs::Header& header) 
{
    visualization_msgs::MarkerArray msg_marker_array;
    visualization_msgs::Marker delete_all_marker;
    delete_all_marker.header.frame_id = "Lidar3D";
    delete_all_marker.header.stamp = ros::Time::now();
    delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
    msg_marker_array.markers.push_back(delete_all_marker);

    vector<Detection> vec_aabb = ClusterInfo(st_Lidar.vec_clusters, header);

    for (const Detection &d : vec_aabb) {
        visualization_msgs::Marker m;
        m.header.frame_id = "Lidar3D";
        m.header.stamp = d.stamp;
        m.ns = "all_clusters";
        m.id = d.id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;

        // 위치 설정 (Centroid)
        m.pose.position.x = d.centroid.x();
        m.pose.position.y = d.centroid.y();
        m.pose.position.z = d.centroid.z();
        m.pose.orientation.w = 1.0;

        // 크기 설정 (Size)
        m.scale.x = max(0.1f, d.size.x());
        m.scale.y = max(0.1f, d.size.y());
        m.scale.z = max(0.1f, d.size.z());

        // 색상 설정 (초록색 반투명 예시)
        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f;
        m.color.a = 0.4f;

        m.lifetime = ros::Duration(0.0);
        msg_marker_array.markers.push_back(m);
    }
    // 이전 프레임 마커 삭제를 위해 클러스터가 없을 때도 빈 배열이라도 퍼블리시
    pub_bounding_box.publish(msg_marker_array);
}

// =========================================================
// line strip 시각화
// =========================================================

void PublishOBBLineStrip (const ros::Publisher &pub_bounding_box,
                          Lidar& st_Lidar,
                          const std_msgs::Header& header) 
{
    visualization_msgs::MarkerArray msg_marker_array;

    // 1. 이전 마커 삭제
    visualization_msgs::Marker delete_all_marker;
    delete_all_marker.header.frame_id = "Lidar3D";
    delete_all_marker.header.stamp = ros::Time::now();
    delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
    msg_marker_array.markers.push_back(delete_all_marker);

    // 필터링 임계값 설정
    // const float max_limit = 9.0f; // 6m 이상 구조물 제외
    // const float min_limit = 0.1f; // 너무 작은 노이즈 제외

    for (size_t i = 0; i < st_Lidar.vec_clusters.size(); ++i) 
    {
        const LidarCluster& cluster = st_Lidar.vec_clusters[i];
        
        if (cluster.pcl_cluster_point->empty()) continue;

        // OBB 기준 실제 크기 계산
        // float rotate_size_x = cluster.rotate_rect_max_x - cluster.rotate_rect_min_x;
        // float rotate_size_y = cluster.rotate_rect_max_y - cluster.rotate_rect_min_y;
        // float rotate_size_z = cluster.max_z - cluster.min_z;

        // if (size_x > max_limit || size_y > max_limit) continue; // 너무 긴 가드레일이나 건물
        // if (rotate_size_z < min_limit) continue;                       // 바닥 노이즈

        visualization_msgs::Marker m;
        m.header = header;
        // m.header.frame_id = "Lidar3D";
        m.header.frame_id = "base_link";

        m.ns = "obb_filtered";
        m.id = i;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;

        m.pose.orientation.w = 1.0; 
        m.scale.x = 0.05; // 선 두께
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; // 노란색
        m.color.a = 1.0f;

        // 4개 꼭짓점 추가 (바닥면 기준)
        for (int j = 0; j < 4; j++) {
            geometry_msgs::Point p;
            p.x = cluster.vec_Corners[j].x;
            p.y = cluster.vec_Corners[j].y;
            // p.z = cluster.min_z; 
            p.z = 1.5; 

            m.points.push_back(p);
        }
        m.points.push_back(m.points[0]); // 사각형 닫기

        msg_marker_array.markers.push_back(m);
    }
    pub_bounding_box.publish(msg_marker_array);
}

// =========================================================
// kalman tracking 시각화
// =========================================================

// void PublishKalman(const ros::Publisher& pub, 
//                   const std::vector<KalmanDetection>& detections, 
//                   const std_msgs::Header& header) 
// {

//     ROS_INFO("Detections size: %ld", detections.size()); 

//     if (detections.empty()) return; // 데이터가 없으면 바로 리턴

    
//     visualization_msgs::MarkerArray marker_array;
    
//     // 이전에 그려진 마커들을 지우기 위해 '전체 삭제' 액션을 먼저 보낼 수도 있습니다.
//     // 하지만 ID 기반 업데이트를 위해 바로 ADD를 수행합니다.

//     for (const auto& det : detections) 
//     {
//         // --- 1. ID 및 속도 텍스트 마커 ---
//         visualization_msgs::Marker text_marker;
//         text_marker.header = header;
//         text_marker.ns = "track_info";
//         text_marker.id = det.id; // 고유 ID 부여
//         text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//         text_marker.action = visualization_msgs::Marker::ADD;
        
//         // 위치: 객체 위쪽 (z축으로 약간 띄움)
//         text_marker.pose.position.x = det.x;
//         text_marker.pose.position.y = det.y;
//         text_marker.pose.position.z = 1.0; 
        
//         text_marker.scale.z = 0.6; // 글자 크기
//         text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; // 흰색
//         text_marker.color.a = 1.0;
        
//         // 표시할 텍스트 (ID와 추정 속도)
//         text_marker.text = "ID: " + std::to_string(det.id) + " | V: " + std::to_string(det.v).substr(0,4) + " m/s";
//         marker_array.markers.push_back(text_marker);

//         // --- 2. 진행 방향 및 속도 화살표 마커 ---
//         if (std::abs(det.v) > 0.1) // 어느 정도 속도가 있을 때만 표시
//         {
//             visualization_msgs::Marker arrow_marker;
//             arrow_marker.header = header;
//             arrow_marker.ns = "velocity_arrow";
//             arrow_marker.id = det.id + 1000; // ID 중복 방지
//             arrow_marker.type = visualization_msgs::Marker::ARROW;
//             arrow_marker.action = visualization_msgs::Marker::ADD;
            
//             arrow_marker.pose.position.x = det.x;
//             arrow_marker.pose.position.y = det.y;
//             arrow_marker.pose.position.z = 0.2; // 바닥보다 살짝 위

//             // Yaw 각도를 쿼터니언으로 변환하여 방향 설정
//             tf2::Quaternion q;
//             q.setRPY(0, 0, det.yaw);
//             // 에러 방지를 위해 직접 값 대입
//             arrow_marker.pose.orientation.x = q.x();
//             arrow_marker.pose.orientation.y = q.y();
//             arrow_marker.pose.orientation.z = q.z();
//             arrow_marker.pose.orientation.w = q.w();

//             // 화살표 크기 (x는 길이, y/z는 두께)
//             arrow_marker.scale.x = det.v * 1.5; // 속도에 비례한 길이
//             arrow_marker.scale.y = 0.15;
//             arrow_marker.scale.z = 0.15;

//             // 색상 (움직이는 물체는 노란색 계열)
//             arrow_marker.color.r = 1.0; arrow_marker.color.g = 0.8; arrow_marker.color.a = 1.0;

//             marker_array.markers.push_back(arrow_marker);
//         }
//     }

//     pub.publish(marker_array);
// }