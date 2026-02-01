#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <nav_msgs/OccupancyGrid.h>

// ========================================
// 재밍 구조체
// ========================================
struct LaneData
{
    double offset;
    double angle;
};

struct GPSJammingState {
    bool is_jamming = false;
    ros::Time last_valid_gps_time;
    ros::Time jamming_start_time;

    double gps_timeout_threshold = 0.3;

    int jamming_count = 0;
    double total_jamming_duration = 0.0; // total duration
    double current_jamming_duration = 0.0; //실시간 duration
};

struct Jamming_offset{
    double filtered_offset =0.0;
    double pp_steering ;
    double offset_steering ;
    double last_offset = 0.0;
};

struct JammingParams
{
    const double Ld = 30.0;
    const float wheel_base = 3.0;
    double gps_steering = 0.0;
    const double gps_alpha = 0.1; 
    const double pid_Kp = 0.05;
    const double pid_Ki = 0.0;
    const double pid_Kd = 0.0;
    const double dt = 0.02;
    const double gps_target_vel = 50.0/3.6;
};

// ========================================
// 기본 구조체
// ========================================

struct Point2D {
    double x;
    double y;
    double curvature;
};

struct Point3D {
    double x;
    double y;
    double z;
};

struct Waypoint {
    double x, y;
    double curvature;
};


// 차량 상태
struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vel = 0.0;
    double max_curvature = 0.0;
    double accel = 0.0;
    double brake = 0.0;

};


// Control 구조체 (Base Planning용)
struct ControlData {
    int close_idx = 0;
    int target_idx = 0;
    int lookahead_idx = 0;
    double target_vel = 0.0;
    double accel = 0.0;
    double brake = 0.0;
    double steering = 0.0;
    double ld = 8.0;
};

// 좌표 변환용
struct CoordinateReference {
    double lat0 = 0.0;
    double lon0 = 0.0;
    double h0 = 0.0;
    double x0_ecef = 0.0;
    double y0_ecef = 0.0;
    double z0_ecef = 0.0;
};

// // Mission 추가
enum Mission {
    LATTICE,
    JAMMING,
    END
};

// ========================================
// Lattice Planning 구조체
// ========================================

// Offset 목표점 (Global 좌표계)
struct OffsetGoal {
    double global_x;
    double global_y;
    double global_yaw;
    double offset;
};

// Baselink 목표점 (차량 좌표계)
struct BaselinkGoal {
    Point2D point;
    double yaw;
    double offset;
};

// 5차 다항식 계수
struct PolynomialCoefficients {
    double a0, a1, a2, a3, a4, a5;
};

// 후보 경로 (points는 baselink 좌표계 유지)
struct CandidatePath {
    std::vector<Point2D> points;  // baselink frame
    double offset; // 옆으로 얼마나 이동한 경로인지 (vertical offset)
    // scores
    double obstacle_cost;
    double curvature_cost;
    double offset_cost;
    double cost;

    bool valid;

    // 생성자(constructor)
    // 구조체 변수(객체)를 만드는 순간 자동으로 한 번 실행되는 초기화 함수
    CandidatePath()
        : offset(0.0),
          obstacle_cost(0.0),
          curvature_cost(0.0),
          offset_cost(0.0),
          cost(0.0),
          valid(true) {}
};

// Lattice Control 구조체
struct LatticeControl {
    int close_idx = 0;
    int target_idx = 0;
    int target_idx_very_long = 0;
    int target_idx_long = 0;
    int target_idx_short = 0;
    int target_idx_medium = 0;
    // double ld = 0.0;
    double ld_short = 5.0;
    double ld_medium = 10.0;
    double ld_long = 15.0;
    double ld_very_long = 25.0;
    // LIDAR costmap with local path --> dynamic velocity decision
    double valid_path_ratio = 1.0;

    std::vector<OffsetGoal> offset_goals;
    std::vector<BaselinkGoal> baselink_goals;
    std::vector<PolynomialCoefficients> coefficients;

    std::vector<CandidatePath> candidates;
    CandidatePath best_path;

    Point2D lookahead_point;  
};

// Planner 파라미터
struct PlannerParams {
    int num_offsets = 15; // 9 -> 15
    double lateral_offset_step = 0.7; // 0.5 -> 1.0
    double sample_spacing = 0.2;
    double lethal_cost_threshold = 70.0;
    double vehicle_front_offset = 4.0;
};

// Costmap 정보 (안전: shared_ptr 보관)
struct CostmapInfo {
    nav_msgs::OccupancyGrid::ConstPtr msg;  // holds latest costmap safely
    double origin_x = 0.0;
    double origin_y = 0.0;
    double resolution = 0.0;
    int width = 0;
    int height = 0;
};

// ========================================
// 전역 변수 (extern) 
// ========================================

//재밍용
extern GPSJammingState gps_state;
extern LaneData lane;
extern JammingParams jamming_params;
extern Jamming_offset offset;
extern bool gps_first_received;
extern VehicleState gps_ego;
extern bool is_gps_jamming;

//기본
extern std::vector<Waypoint> waypoints;
extern VehicleState ego;
extern CoordinateReference coord_ref;
extern bool coord_ref_initialized;

// Lattice용
extern LatticeControl lattice_ctrl;
extern Point2D best_waypoint;
extern PlannerParams planner_params;
extern CostmapInfo costmap_info;
extern double last_selected_offset;

// 임시
extern ControlData ctrl;
extern double Kp, Ki, Kd;
extern double k_gain;
extern double ld_gain;
extern double target_vel;
extern double obstacle_vel;
extern double curve_vel;
extern double curve_standard;
extern int lookahead_idx;


// ROS Publishers
extern ros::Publisher marker_pub;
extern ros::Publisher local_path_pub;
extern ros::Publisher cmd_pub;
extern std::mutex costmap_mutex;

#endif // GLOBAL_HPP
