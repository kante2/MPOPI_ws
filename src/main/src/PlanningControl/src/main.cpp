#include "global.hpp"
#include "Planning.hpp"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>

using namespace std;

// ========================================
// 전역 변수 정의
// ========================================
std::vector<Waypoint> waypoints;
VehicleState ego;
LatticeControl lattice_ctrl;
CoordinateReference coord_ref;

PlannerParams planner_params;
CostmapInfo costmap_info;
double last_selected_offset = 0.0;

ros::Publisher marker_pub;

bool coord_ref_initialized = false;

// ========================================
// Waypoint 로드
// ========================================
// ========================================
// Waypoint 로드 (절대경로 하드코딩: ref.txt + track csv)
// ========================================
bool loadWaypoints() {
    waypoints.clear();

    const std::string ref_file_name  = "/home/autonav/aim_ws/src/main/config/ref.txt";
    const std::string path_file_name = "/home/autonav/aim_ws/src/main/config/track_log_recorded_right.csv";

    // -------------------------
    // 1) 기준점(ref.txt) 읽기
    // -------------------------
    ROS_INFO("Opening ref file: %s", ref_file_name.c_str());
    std::ifstream ref_file(ref_file_name);
    if (!ref_file.is_open()) {
        ROS_ERROR("Failed to open ref file: %s", ref_file_name.c_str());
        return false;
    }

    // ref.txt: lat0 lon0 h0 형태라고 가정
    ref_file >> coord_ref.lat0 >> coord_ref.lon0 >> coord_ref.h0;
    ref_file.close();

    wgs84ToECEF(coord_ref.lat0, coord_ref.lon0, coord_ref.h0,
                coord_ref.x0_ecef, coord_ref.y0_ecef, coord_ref.z0_ecef);

    coord_ref_initialized = true;
    ROS_INFO("[Ref] lat0=%.8f lon0=%.8f h0=%.3f", coord_ref.lat0, coord_ref.lon0, coord_ref.h0);

    // -------------------------
    // 2) 경로(csv) 읽기
    // -------------------------
    ROS_INFO("Opening path file: %s", path_file_name.c_str());
    std::ifstream path_file(path_file_name);
    if (!path_file.is_open()) {
        ROS_ERROR("Failed to open path file: %s", path_file_name.c_str());
        return false;
    }

    std::string line;
    while (std::getline(path_file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);

        // 케이스 A: 공백 구분 "x y (z)" 형태
        double x, y, z;
        if (ss >> x >> y) {
            Waypoint wp;
            wp.x = x;
            wp.y = y;
            wp.curvature = 0.0;
            waypoints.push_back(wp);
            continue;
        }

        // 케이스 B: 콤마 구분 "x,y,..." 형태
        ss.clear();
        ss.str(line);

        std::string val;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) {
            // trim
            val.erase(0, val.find_first_not_of(" \t\r\n"));
            val.erase(val.find_last_not_of(" \t\r\n") + 1);

            try {
                row.push_back(std::stod(val));
            } catch (...) {
                // 헤더나 문자열이면 스킵
            }
        }

        if (row.size() >= 2) {
            Waypoint wp;
            wp.x = row[0];
            wp.y = row[1];
            wp.curvature = 0.0;
            waypoints.push_back(wp);
        }
    }

    path_file.close();

    if (waypoints.size() < 3) {
        ROS_ERROR("Not enough waypoints! (size: %zu)", waypoints.size());
        return false;
    }

    ROS_INFO("[Waypoints] Loaded %zu waypoints from %s", waypoints.size(), path_file_name.c_str());
    return true;
}

// ========================================
// Callback 함수들
// ========================================

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    costmap_info.msg = msg; // 안전하게 shared_ptr 저장

    costmap_info.origin_x = msg->info.origin.position.x;
    costmap_info.origin_y = msg->info.origin.position.y;
    costmap_info.resolution = msg->info.resolution;
    costmap_info.width = (int)msg->info.width;
    costmap_info.height = (int)msg->info.height;

    ROS_INFO_ONCE("[Costmap] Received! %dx%d, res: %.2fm",
                  costmap_info.width, costmap_info.height, costmap_info.resolution);
}


void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
    if (!coord_ref_initialized) {
        coord_ref.lat0 = msg->latitude;
        coord_ref.lon0 = msg->longitude;
        coord_ref.h0 = msg->altitude;
        wgs84ToECEF(coord_ref.lat0, coord_ref.lon0, coord_ref.h0,
                    coord_ref.x0_ecef, coord_ref.y0_ecef, coord_ref.z0_ecef);
        coord_ref_initialized = true;
        ROS_INFO("[GPS] Reference point set: lat=%.6f, lon=%.6f", 
                 coord_ref.lat0, coord_ref.lon0);
    }
    
    double x, y, z;
    wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
               coord_ref, x, y, z);
    ego.x = x;
    ego.y = y;
}

void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    ego.vel = msg->velocity.x;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ego.yaw = quaternionToYaw(msg->orientation.x,
                              msg->orientation.y,
                              msg->orientation.z,
                              msg->orientation.w);
}

// ========================================
// RViz 시각화
// ========================================
void publishCandidatePaths() {
    visualization_msgs::MarkerArray marker_array;
    
    // 현재 선택된 최적 경로의 offset (비교 및 시각화용)
    double best_offset = lattice_ctrl.best_path.offset;
    
    for (size_t i = 0; i < lattice_ctrl.candidates.size(); i++) {
        const auto& candidate = lattice_ctrl.candidates[i];
        
        visualization_msgs::Marker marker;

        // [중요 1] 내부에서 좌표를 Map 기준으로 변환하므로, 프레임도 "bas"이어야 안 밀립니다.
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "candidate_paths";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        // [중요 2] 쿼터니언 초기화 (이게 없으면 RViz에서 안 보임)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0; 
        
        // 색상 및 굵기 설정 (Best 경로는 빨간색/두껍게, 나머지는 초록색/얇게)
        bool is_best = (fabs(candidate.offset - best_offset) < 0.01);
        
        if (is_best) {
            marker.scale.x = 0.15; 
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        } else {
            marker.scale.x = 0.05;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.6;
        }
        
        // 포인트 좌표 변환 및 입력
        for (const auto& pt : candidate.points) {
            geometry_msgs::Point p;
            p.x = pt.x;
            p.y = pt.y;
            // Best 경로는 겹침 방지를 위해 살짝 띄움
            p.z = is_best ? 0.1 : 0.0; 
            marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub.publish(marker_array);
}

void latticeTestLoop(const ros::TimerEvent&) {
    LatticePlanningProcess();
    publishCandidatePaths();
}

// ========================================
// Main.
// ========================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_test_node");
    ros::NodeHandle nh;
    
    ROS_INFO("========================================");
    ROS_INFO("  Lattice Planning Test (No Control)");
    ROS_INFO("========================================");
    
    // Waypoints 로드
    if (!loadWaypoints()) {
        ROS_FATAL("Failed to load waypoints!");
        return -1;
    }
    
    // 파라미터
    planner_params.num_offsets = 9;
    planner_params.lateral_offset_step = 0.5;
    planner_params.sample_spacing = 0.2;
    planner_params.lethal_cost_threshold = 90.0;
    
    ROS_INFO("[Params] Offsets: %d, Step: %.2fm", 
             planner_params.num_offsets, 
             planner_params.lateral_offset_step);
    
    // 초기값
    ego.x = 0.0;
    ego.y = 0.0;
    ego.yaw = 0.0;
    ego.vel = 0.0;
    
    // Subscriber
    ros::Subscriber gps_sub = nh.subscribe("/gps", 1, gpsCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCallback);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, egoCallback);
    ros::Subscriber costmap_sub = nh.subscribe("/costmap", 1, costmapCallback);
    
    // Publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/lattice/paths", 1);
    
    // Timer (10Hz)
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), latticeTestLoop);
    
    ROS_INFO("Node Started! (Visualization Only - No Control)");
    ROS_INFO("========================================");
    
    ros::spin();
    return 0;
}