#include "Global.hpp"
#include "Planning.hpp"
#include "Visualizer.hpp"
#include "Control.hpp"
#include "parameter_loader.hpp"

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
#include <std_msgs/Float32MultiArray.h>

using namespace std;
mutex costmap_mutex;

// ========================================
// Callback
// ========================================

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

    lock_guard<std::mutex> lock(costmap_mutex);

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

    if (std::abs(msg->latitude) > 0.000001) { 
            last_gps_time = ros::Time::now();
        }

    // 2. 좌표계 초기화 (기존 코드 유지)
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

    // 3. [삭제됨] 과거의 'msg->latitude == 0' 확인 로직은 삭제합니다.
    // (이제 mainControlLoop에서 시간 차이로 재밍을 판단합니다)

    // 4. 좌표 변환 (기존 코드 유지)
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

void laneCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // Lane 정보를 처리하는 로직 추가
    lane.offset = msg->data[0];
    lane.angle = msg->data[1];
}

void mainControlLoop(const ros::TimerEvent&) {

    // 현재 시간 - 마지막 수신 시간
    double time_diff = (ros::Time::now() - last_gps_time).toSec();

    // 1.0초 이상 연락 없으면 재밍으로 판단
    if (time_diff > 0.5) { 
        if (!is_gps_jamming) {
            ROS_WARN("GPS Signal Lost! (Timeout: %.2f sec) -> Jamming Mode ON", time_diff);
        }
        is_gps_jamming = true;
    } 
    else {
        // 연락이 잘 오고 있으면 정상
        is_gps_jamming = false;
    }

    // (기존 로직) 분기 처리
    if (is_gps_jamming) {
       JammingPlanningProcess();    
    }
    else {
        LatticePlanningProcess();     
        ControlProcess();             
        publishCandidatePaths();      
        publishVehicleFootprint();    
        publishLocalPath();           
    }
}

// ========================================
// main
// ========================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_planning_node");
    ros::NodeHandle nh;

    // Waypoints 로드
    if (!loadWaypoints()) {
        ROS_FATAL("Failed to load waypoints!");
        return -1;
    }
    // 파라미터 초기화 
    initializePlannerParameters();    
    initializeControlParameters();
    
    // 초기값
    ego.x = 0.0;
    ego.y = 0.0;
    ego.yaw = 0.0;
    ego.vel = 0.0;
    
    // Subscriber
    ros::Subscriber gps_sub = nh.subscribe("/gps", 1, gpsCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCallback);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, egoCallback);
    // lidar_costmap
    ros::Subscriber costmap_sub = nh.subscribe("/costmap", 1, costmapCallback);
    // camera_costmap
    // ros::Subscriber camera_costmap_sub = nh.subscribe("/camera_costmap", 1, costmapCallback);` <- 

    // ros::Subscriber lane_sub = nh.subscribe<camera::LaneInfo>("/lane/path", 1, laneCallback);
    // path_msg = Float32MultiArray()
    ros::Subscriber lane_sub = nh.subscribe<std_msgs::Float32MultiArray>("/lane/path", 1, laneCallback);

    // Publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/lattice/paths", 1);
    local_path_pub = nh.advertise<visualization_msgs::MarkerArray>("/local_path", 1);
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);

    // Timer (10Hz)
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), mainControlLoop);
    
    ROS_INFO("Node Started! (Visualization Only - No Control)");
    ROS_INFO("========================================");
    
    ros::AsyncSpinner spinner(4); // 일꾼 4명 고용 // 4개의 스레드를 할당하여 전역 콜백 큐(Global Callback Queue)를 병렬로 처리하는 스피너 생성
    spinner.start();              // 일꾼들 투입 (백그라운드에서 돔) // 스피너를 백그라운드 스레드에서 비동기적으로 시작 (메인 스레드는 차단되지 않음)
    ros::waitForShutdown();       // 메인 스레드는 여기서 프로그램 안 꺼지게 대기
 
    return 0;
}