#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <vector>
#include <string>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>

// ========================================
// 재밍 구조체
// ========================================



// ========================================
// 기본 구조체
// ========================================

struct Point2D {
    double x;
    double y;
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

// ========================================
// 차량 상태
// ========================================

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vel = 0.0;
    double max_curvature = 0.0;
};

// ========================================
// Control 구조체 (Base Planning용)
// ========================================

struct ControlData {
    int close_idx = 0;
    int target_idx = 0;
    int lookahead_idx = 0;
    double target_vel = 0.0;
    double accel = 0.0;
    double brake = 0.0;
    double steering = 0.0;
    double ld = 0.0;
};

// ========================================
// 좌표 변환용
// ========================================

struct CoordinateReference {
    double lat0 = 0.0;
    double lon0 = 0.0;
    double h0 = 0.0;
    double x0_ecef = 0.0;
    double y0_ecef = 0.0;
    double z0_ecef = 0.0;
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
    double offset;

    double obstacle_cost;
    double curvature_cost;
    double offset_cost;
    double cost;

    bool valid;

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
    double ld = 0.0;

    std::vector<OffsetGoal> offset_goals;
    std::vector<BaselinkGoal> baselink_goals;
    std::vector<PolynomialCoefficients> coefficients;

    std::vector<CandidatePath> candidates;
    CandidatePath best_path;
};

// Planner 파라미터
struct PlannerParams {
    int num_offsets = 9;
    double lateral_offset_step = 0.5;
    double sample_spacing = 0.2;
    double lethal_cost_threshold = 90.0;
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

extern std::vector<Waypoint> waypoints;
extern VehicleState ego;
extern LatticeControl lattice_ctrl;
extern CoordinateReference coord_ref;

// Lattice용
extern PlannerParams planner_params;
extern CostmapInfo costmap_info;
extern double last_selected_offset;

// (안 쓰면 제거해도 됨)
extern ControlData ctrl;
extern double Kp, Ki, Kd;
extern double k_gain;
extern double ld_gain;
extern double target_vel;
extern double curve_vel;
extern double curve_standard;
extern int lookahead_idx;

#endif // GLOBAL_HPP
