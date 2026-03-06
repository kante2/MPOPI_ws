#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <nav_msgs/OccupancyGrid.h>
#include <new>


// ========================================
// MPOPI 핵심 구조체
// ========================================

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

// 제어 입력
struct ControlInput {
    double v;          // 속도 (m/s)
    double delta;       // 스티어링 (rad)
};

// MPOPI 예측 궤적
struct MPOPITrajectory {
    std::vector<VehicleState> states;   // t=0~t=N 의 상태들
    double total_cost = 0.0;
    double weight = 0.0;
};

// MPOPI 파라미터를 하나의 구조체로 정리
struct MPOPIParams {
    // ========== 샘플링 파라미터 ==========
    int K = 500;              // 샘플 수
    int N = 20;               // 예측 스텝 수
    double DT = 0.1;          // 샘플링 시간 (20 × 0.1 = 2초)
    double L = 2.7;           // 차량 축간거리 (wheelbase)
    
    // ========== 분포 파라미터 ==========
    double sigma_v = 1.0;            // 초기 속도 분산
    double sigma_delta = 0.1;        // 초기 조향 분산
    double sigma_v_min = 0.1;        // 최소 속도 분산
    double sigma_delta_min = 0.01;   // 최소 조향 분산
    double temperature = 50.0;        // 소프트맥스 온도 (λ)
    
    // ========== 제어 제한 ==========
    double v_max = 20.0;       // 최대 속도
    double v_min = 0.0;        // 최소 속도
    double delta_max = 0.5;    // 최대 조향각
    double delta_min = -0.5;   // 최소 조향각
    
    // ========== 비용 가중치 ==========
    double w_path = 30.0;       // 경로 추종
    double w_obstacle = 2.0;   // 장애물 회피
    double w_drivable = 1.5;   // 운전가능 영역
    double w_velocity = 0.5;   // 속도 추종
    double w_goal = 5.0;       // 목표 진행
};

// MPOPI 상태를 하나의 구조체로 정리
struct MPOPIState {
    // 제어 입력 샘플들 [k][n]
    std::vector<std::vector<ControlInput>> U_samples; // U_samples[k][n]
    
    // 예측 궤적들 [k]
    std::vector<MPOPITrajectory> trajectories;
    
    // 평가 결과 [k]
    std::vector<double> costs;
    std::vector<double> weights;
    
    // 최적 제어 시퀀스 [n]
    std::vector<ControlInput> U_nominal;
    
    // 분포 파라미터들 [n]
    std::vector<double> mean_v;
    std::vector<double> mean_delta;
    std::vector<double> std_v;
    std::vector<double> std_delta;

    // cloest waypoint index
    int closest_wp_idx = 0;
    
    // 최종 출력
    ControlInput cmd;
    
    // AIS 반복 카운터
    int current_iteration = 0;
};




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
    const double gps_k_p = 0.15;
    const double gps_k_d = 0.02; 
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
    double lane_cost;
    double curvature_cost;
    double offset_cost;
    double offset_change_cost;  // 이전 경로와의 일관성 코스트
    double cost;

    bool valid;

    // 생성자(constructor)
    CandidatePath()
        : offset(0.0),
          obstacle_cost(0.0),
          lane_cost(0.0),
          curvature_cost(0.0),
          offset_cost(0.0),
          offset_change_cost(0.0),
          cost(0.0),
          valid(true) {}
};

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

// Lattice Control 구조체 (유일한 정의)
struct LatticeControl {
    int close_idx = 0;
    int target_idx = 0;
    int target_idx_very_long = 0;
    int target_idx_long = 0;
    int target_idx_short = 0;
    int target_idx_medium = 0;
    double ld_short;
    double ld_medium;
    double ld_long;
    double ld_very_long;
    double valid_path_ratio = 1.0;
    double very_long_path_ratio = 1.0; // 매우 긴 경로의 유효 경로 비율
    double ego_path_ratio = 1.0; // 내 차선의 유효 경로 비율

    std::vector<OffsetGoal> offset_goals;
    std::vector<BaselinkGoal> baselink_goals;
    std::vector<PolynomialCoefficients> coefficients;

    std::vector<CandidatePath> candidates;
    CandidatePath best_path;

    Point2D lookahead_point;  
};

// Planner 파라미터
struct PlannerParams {
    int num_offsets = 9; // 13 -> 9
    double lateral_offset_step = 1.0;
    double sample_spacing = 0.2;
    double lethal_cost_threshold = 90.0;
    double vehicle_front_offset = 4.0;
};

// Costmap 정보
struct CostmapInfo {
    nav_msgs::OccupancyGrid::ConstPtr msg;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double resolution = 0.0;
    int width = 0;
    int height = 0;
};

// ========================================
// 전역 변수 (extern) 
// ========================================


// MPOPI 관련
extern ControlData mpopi_ctrl;
extern MPOPIParams mpopi_params;
extern MPOPIState mpopi_state;
extern ControlInput mpopi_cmd;
extern VehicleState mpopi_vehicle_state;  // 현재 차량 상태 <- enu 좌표계

//재밍용
extern GPSJammingState gps_state;
extern LaneData lane;
extern JammingParams jamming_params;
extern Jamming_offset offset;
extern VehicleState gps_ego;
extern bool gps_jamming_perception;    // 마지막으로 GPS 들어온 시간


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
//노라이다 존
extern std::vector<Point2D> no_lidar_zones;
//추월
extern std::vector<Point2D> overtaking_zone;
extern bool is_in_overtaking_zone;
//카메라 코스트맵
extern CostmapInfo Camera_costmap_info;
extern std::vector<Point2D> no_camera_zones;

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

// driving mode
extern int driving_mode; // 0: Normal, 1: Overtaking
extern int prev_driving_mode; // 모드 변경 감지용

// ROS Publishers
extern ros::Publisher marker_pub;
extern ros::Publisher local_path_pub;
extern ros::Publisher cmd_pub;
extern std::mutex costmap_mutex;

#endif // GLOBAL_HPP
