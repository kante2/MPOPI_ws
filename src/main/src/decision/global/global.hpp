#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

// ========================================
// 기본 구조체
// ========================================

struct Point2D {
    double x{0.0};
    double y{0.0};
};

struct Waypoint {
    double x{0.0};
    double y{0.0};
    double curvature{0.0};
};

struct VehicleState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double vel{0.0};
};

struct ControlData {
    int close_idx{0};
    int target_idx{0};
    double ld{5.0};
    double steering{0.0};
    double accel{0.0};
    double brake{0.0};
};

// ========================================
// 목표점 구조체
// ========================================

struct GoalPoint {
    Point2D global;           // Global(ENU) 좌표
    Point2D baselink;         // Base_link 좌표
    double yaw_global{0.0};   // Global yaw
    double yaw_baselink{0.0}; // Base_link yaw
    double offset{0.0};       // Lateral offset
};

// ========================================
// 방향 벡터 구조체
// ========================================

struct DirectionVector {
    double x{0.0};
    double y{0.0};
};

// ========================================
// 5차 다항식 계수 구조체
// ========================================

struct PolynomialCoefficients {
    double a0{0.0};
    double a1{0.0};
    double a2{0.0};
    double a3{0.0};
    double a4{0.0};
    double a5{0.0};
};

// ========================================
// 후보 경로 구조체
// ========================================

struct CandidatePath {
    std::vector<Point2D> points;  // Base_link 좌표계 점들
    double cost{1e10};
    double obstacle_cost{0.0};
    double offset_cost{0.0};
    double curvature_cost{0.0};
    double offset{0.0};
    bool valid{true};
};

// ========================================
// Planner 파라미터 구조체
// ========================================

struct LatticePlannerParams {
    double LD{5.0};
    double lateral_offset_step{0.5};
    int num_offsets{9};
    double sample_spacing{0.5};
    double lethal_cost_threshold{100.0};
};

// ========================================
// Costmap 정보 구조체
// ========================================

struct CostmapInfo {
    nav_msgs::OccupancyGrid::ConstPtr data;
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    std::string frame_id{"base_link"};
};

// ========================================
// 좌표 기준점 구조체
// ========================================

struct CoordinateReference {
    double lat0{0.0};
    double lon0{0.0};
    double h0{0.0};
    double x0_ecef{0.0};
    double y0_ecef{0.0};
    double z0_ecef{0.0};
};

// ========================================
// 전역 변수 선언
// ========================================

extern std::vector<Waypoint> waypoints;
extern VehicleState ego;
extern ControlData ctrl;
extern LatticePlannerParams planner_params;
extern CostmapInfo costmap_info;
extern CoordinateReference coord_ref;

extern double last_selected_offset;

extern ros::Publisher cmd_pub;
extern ros::Publisher local_path_pub;
extern ros::Publisher candidate_paths_pub;

extern std::string path_file_name;
extern std::string ref_file_name;
extern double target_vel;
extern double curve_vel;
extern double curve_standard;
extern double k_gain;
extern double ld_gain;
extern int lookahead_idx;
extern double Kp, Ki, Kd;

#endif // GLOBAL_HPP