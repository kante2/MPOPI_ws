#include "Global.hpp"   // 파일명이 GLOBAL_HPP 헤더인 그대로 include

// 재밍용
GPSJammingState gps_state;
LaneData lane;
JammingParams jamming_params;
Jamming_offset offset;
VehicleState gps_ego;
    
bool gps_jamming_perception = false;


// 기본
std::vector<Waypoint> waypoints;
VehicleState ego;
CoordinateReference coord_ref;
bool coord_ref_initialized = false;

// Lattice용
LatticeControl lattice_ctrl;
Point2D best_waypoint;               // Point2D 안에 curvature도 있어서 초기값 필요하면 {0,0,0} 가능
PlannerParams planner_params;
CostmapInfo costmap_info;
double last_selected_offset = 0.0;
//추월
std::vector<Point2D> overtaking_zone; 
bool is_in_overtaking_zone = false;
//카메라 코스트맵
CostmapInfo Camera_costmap_info;
std::vector<Point2D> no_camera_zones;

// 임시(Control)
ControlData ctrl;
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
double k_gain = 0.0;
double ld_gain = 0.0;
double target_vel = 0.0;
double obstacle_vel = 0.0;
double curve_vel = 0.0;
double curve_standard = 0.0;
int lookahead_idx = 0;
double min_ld = 0.0;
double gain_ld = 0.0;

// Publishers
ros::Publisher marker_pub;
ros::Publisher local_path_pub;
ros::Publisher cmd_pub;
