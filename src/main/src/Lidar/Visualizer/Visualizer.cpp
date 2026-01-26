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

    for (size_t i = 0; i < st_Lidar.vec_clusters.size(); ++i) 
    {  
        const LidarCluster& cluster = st_Lidar.vec_clusters[i];
        
        if (cluster.pcl_cluster_point->empty()) continue;

        visualization_msgs::Marker m;
        m.header = header;
        m.header.frame_id = "Lidar3D";

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
            p.z = cluster.min_z; 
            // p.z = 1.5; 

            m.points.push_back(p);
        }
        m.points.push_back(m.points[0]); // 사각형 닫기

        msg_marker_array.markers.push_back(m);
    }
    pub_bounding_box.publish(msg_marker_array);
}

// =========================================================
// OBB heading vector 시각화
// =========================================================

void PublishHeading(const ros::Publisher& pub_heading, 
                    Lidar& st_Lidar, 
                    const std_msgs::Header& header)
{
    visualization_msgs::MarkerArray msg_marker_array;

    //1. 이전 마커 삭제

    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "Lidar3D"; 
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    msg_marker_array.markers.push_back(delete_marker);

    for (int i = 0; i < st_Lidar.vec_clusters.size(); ++i)
    {
      const LidarCluster& cluster = st_Lidar.vec_clusters[i];
      visualization_msgs::Marker arrow;
      arrow.points.clear();

      // ------- data 출력 확인 ----------
      // cout << "heading 시각화 : " << cluster.heading_direction_x << endl; 
      // gooood

      if (cluster.pcl_cluster_point->empty()) continue;
      
      arrow.header.frame_id = "Lidar3D"; 
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "heading_arrows";
      arrow.id = i;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;

      arrow.pose.orientation.w = 1.0;

      // 1. 화살표의 시작점 (OBB의 중심점)
      geometry_msgs::Point start;
      start.x = cluster.centroid_x;
      start.y = cluster.centroid_y;
      start.z = 0.5; // 바닥에 묻히지 않게 살짝 띄움

      // 2. 화살표의 끝점 (중심점에서 heading 방향)
      geometry_msgs::Point end;
      end.x = start.x + cluster.heading_direction_x * cluster.length * 0.5;
      end.y = start.y + cluster.heading_direction_y * cluster.length * 0.5;
      end.z = 0.5;

      arrow.points.push_back(start);
      arrow.points.push_back(end);

      // ------- data 출력 확인 ----------
      // cout << "arrow point : " << arrow << endl;
      // goooood

      // 3. 화살표 두께 및 색상 설정
      arrow.scale.x = 0.1; // 축 두께
      arrow.scale.y = 0.2; // 화살표 머리 두께
      arrow.scale.z = 0.2; // 화살표 머리 길이
      
      arrow.color.r = 1.0f; // 빨간색 화살표
      arrow.color.g = 1.0f;
      arrow.color.b = 0.0f;
      arrow.color.a = 1.0f;

      msg_marker_array.markers.push_back(arrow);

    }
    pub_heading.publish(msg_marker_array);

    // ------- data 출력 확인 ----------
    // cout << "pub_heading (visualizer.cpp) : " << msg_marker_array << endl;
    // goood
}

// =========================================================
// kalman tracking 시각화
// =========================================================

void PublishKalman(const ros::Publisher& pub_kalman, 
                  const std::vector<KalmanDetection>& detections, 
                  const std_msgs::Header& header) 
{
    if (detections.empty()) return;

    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();
    
    for (const KalmanDetection& det : detections) 
    {
        // --- 1. 장애물 본체 마커 (CUBE) ---
        visualization_msgs::Marker cube_marker;
        cube_marker.header = header;
        cube_marker.header.frame_id = "Lidar3D";
        cube_marker.header.stamp = now;
        cube_marker.ns = "track_cube";
        cube_marker.id = det.id; 
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;
        
        // 위치 설정
        cube_marker.pose.position.x = det.x;
        cube_marker.pose.position.y = det.y;
        cube_marker.pose.position.z = 0.5; // 지면에서 0.5m 높이
        
        // 방향 설정 (Yaw 반영)
        tf2::Quaternion q;
        q.setRPY(0, 0, det.yaw);
        cube_marker.pose.orientation = tf2::toMsg(q);

        // 크기 설정 (가로, 세로, 높이 0.6m)
        cube_marker.scale.x = 0.6; cube_marker.scale.y = 0.6; cube_marker.scale.z = 0.6;
        // 크기를 OBB 그대로 가져와서. 
        
        // 색상 (확정 트랙은 초록색 계열)
        cube_marker.color.r = 0.0; cube_marker.color.g = 1.0; cube_marker.color.b = 0.0;
        cube_marker.color.a = 0.6; // 약간 투명하게
        
        cube_marker.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(cube_marker);

        // --- 2. 정보 텍스트 마커 (ID & 속도) ---
        visualization_msgs::Marker text_marker;
        text_marker.header = header;
        text_marker.header.frame_id = "Lidar3D";
        text_marker.header.stamp = now;
        text_marker.ns = "track_info";
        text_marker.id = det.id + 10000; // 큐브 ID와 겹치지 않게 오프셋 부여
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        // 위치: 큐브보다 약간 위쪽
        text_marker.pose.position.x = det.x;
        text_marker.pose.position.y = det.y;
        text_marker.pose.position.z = 1.2; 
        
        // 글자 크기 (Z축 값이 글자 높이가 됨)
        text_marker.scale.z = 0.4; 
        
        // 색상 (흰색)
        text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        // 표시할 텍스트 설정
        text_marker.text = "ID: " + std::to_string(det.id) + 
                          " | V: " + std::to_string(det.v).substr(0, 4) + " m/s";
        
        text_marker.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(text_marker);
    }
    pub_kalman.publish(marker_array);
}