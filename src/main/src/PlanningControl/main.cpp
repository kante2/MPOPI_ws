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

using namespace std;

// ========================================
// Callback
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

void mainControlLoop(const ros::TimerEvent&) {
    
    // 
    // planning -> coontrol
    LatticePlanningProcess();     // 1. 경로 계산
    ControlProcess();             // 1-2. 제어 계산
    // rviz,, 
    publishCandidatePaths();      // 2. 경로 그리기
    publishVehicleFootprint();    // 3. 내 차 박스 그리기
    publishLocalPath();           // 4. 선택된 경로 발행
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
    ros::Subscriber costmap_sub = nh.subscribe("/costmap", 1, costmapCallback);
    
    // Publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/lattice/paths", 1);
    local_path_pub = nh.advertise<visualization_msgs::MarkerArray>("/local_path", 1);
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);

    // Timer (10Hz)
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), mainControlLoop);
    
    ROS_INFO("Node Started! (Visualization Only - No Control)");
    ROS_INFO("========================================");
    
    ros::spin();
    return 0;
}