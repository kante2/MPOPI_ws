#include "global.hpp"
#include "Planning.hpp"

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>
#include <morai_msgs/CtrlCmd.h>

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
ros::Publisher local_path_pub;

bool coord_ref_initialized = false;

// =========================================================
// [추가 2] 링킹 에러 해결을 위한 변수 실체 정의 (여기 꼭 필요!)
// =========================================================
ControlData ctrl;          // 제어 데이터
double Kp = 0.2;           // PID 게인 초기값
double Ki = 0.0;
double Kd = 0.0;
double k_gain = 1.0;       // 조향 게인
double curve_standard = 0.0;
double curve_vel = 30.0 / 3.6;
double target_vel = 40.0 / 3.6;   // 목표 속도

ros::Publisher cmd_pub;    // 제어 명령 Publisher

// ========================================
// Waypoint 로드 (상대경로: config 폴더)
// ========================================
bool loadWaypoints() {
    waypoints.clear();

    const std::string ref_file_name  = "src/main/config/ref.txt";
    const std::string path_file_name = "src/main/config/track_log_recorded_final.csv";

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
// [수정된 루프 함수] 중복 없이 이거 하나만 있어야 합니다!
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
// main.cpp

void publishCandidatePaths() {
    visualization_msgs::MarkerArray marker_array;
    
    // 현재 선택된 최적 경로의 offset
    double best_offset = lattice_ctrl.best_path.offset;

    for (size_t i = 0; i < lattice_ctrl.candidates.size(); i++) {
        const auto& candidate = lattice_ctrl.candidates[i];
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "candidate_paths"; // 기존과 동일
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.orientation.w = 1.0; 
        
        // [색상 설정]
        if (!candidate.valid) {
            // 충돌한 경로는 회색
            marker.scale.x = 0.03; 
            marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5; marker.color.a = 0.5;
        } 
        else if (fabs(candidate.offset - best_offset) < 0.01) {
            // Best 경로는 빨간색
            marker.scale.x = 0.15; 
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
        } 
        else {
            // 나머지 후보는 초록색
            marker.scale.x = 0.05; 
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 0.6;
        }
        
        // Rear axle 기준 경로 표시 (front_offset 변환 제거)
        for (size_t j = 0; j < candidate.points.size(); j++) {
            double cx = candidate.points[j].x;
            double cy = candidate.points[j].y;

            geometry_msgs::Point p_visual;
            p_visual.x = cx;
            p_visual.y = cy;
            p_visual.z = 0.0; 
            
            marker.points.push_back(p_visual);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub.publish(marker_array);
}
// 내 차(노란 박스) 그리기
void publishVehicleFootprint() {
    visualization_msgs::Marker car_marker;
    car_marker.header.frame_id = "base_link";
    car_marker.header.stamp = ros::Time::now();
    car_marker.ns = "ego_shape";
    car_marker.id = 0;
    car_marker.type = visualization_msgs::Marker::CUBE; // 박스 모양
    car_marker.action = visualization_msgs::Marker::ADD;

    // 파라미터 적용 확인 (4.0m)
    double front_len = planner_params.vehicle_front_offset; 
    double rear_len  = 1.0; 
    double width     = 2.0; 

    // 박스 중심 위치 (Base_link 기준)
    car_marker.pose.position.x = (front_len - rear_len) / 2.0;
    car_marker.pose.position.y = 0.0;
    car_marker.pose.position.z = 0.5; 
    car_marker.pose.orientation.w = 1.0;

    car_marker.scale.x = front_len + rear_len; 
    car_marker.scale.y = width;
    car_marker.scale.z = 1.5; 

    car_marker.color.r = 1.0; // 노란색
    car_marker.color.g = 1.0;
    car_marker.color.b = 0.0;
    car_marker.color.a = 0.5; 

    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(car_marker);
    marker_pub.publish(arr);
}

// [추가된 함수] 로컬 경로를 파란색 MarkerArray로 발행
void publishLocalPath() {
    if (lattice_ctrl.best_path.points.empty()) {
        return;
    }
    
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "base_link";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "local_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.1; // 선의 두께
    
    // 파란색
    path_marker.color.r = 0.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 1.0;
    path_marker.color.a = 1.0;
    
    // 최종 선택된 경로의 포인트 추가
    for (const auto& point : lattice_ctrl.best_path.points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        path_marker.points.push_back(p);
    }
    
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(path_marker);
    local_path_pub.publish(arr);
}

void latticeTestLoop(const ros::TimerEvent&) {
    // planning -> coontrol
    LatticePlanningProcess();     // 1. 경로 계산
    ControlProcess();             // 1-2. 제어 계산
    // rviz,, 
    publishCandidatePaths();      // 2. 경로 그리기
    publishVehicleFootprint();    // 3. 내 차 박스 그리기
    publishLocalPath();           // 4. 선택된 경로 발행
}

// ========================================
// Main.
// ========================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_planning_node");
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
    planner_params.num_offsets = 7; // 9 -> 15
    planner_params.lateral_offset_step = 1.5; 
    planner_params.sample_spacing = 0.2;
    planner_params.lethal_cost_threshold = 90.0;
    planner_params.vehicle_front_offset = 4.0; 
    
    ROS_INFO("========================================");
    ROS_INFO("[Params] Front Offset: %.2fm (Check This!)", planner_params.vehicle_front_offset);
    ROS_INFO("========================================");
    
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
    local_path_pub = nh.advertise<visualization_msgs::MarkerArray>("/local_path", 1);
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);

    // Timer (10Hz)
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), latticeTestLoop);
    
    ROS_INFO("Node Started! (Visualization Only - No Control)");
    ROS_INFO("========================================");
    
    ros::spin();
    return 0;
}