#ifndef LATTICE_STRUCTS_HPP
#define LATTICE_STRUCTS_HPP

#include <vector>
// ======================================== 재밍 구조체 ============================================


// ======================================== 레티스플레너 구조체 ============================================

// 기본 좌표 구조체
struct Point2D {
    double x;
    double y;
};


// 차량 상태
struct VehicleState {
    double x;        // Global X
    double y;        // Global Y
    double yaw;      // Global Yaw (rad)
    double vel;      // 속도
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

// 5차 다항식 계수
struct PolynomialCoefficients {
    double a0, a1, a2, a3, a4, a5;
};


// 후보 경로
struct CandidatePath {
    std::vector<Point2D> points;
    double offset;
    
    // 비용들
    double obstacle_cost;
    double curvature_cost;
    double offset_cost;
    double cost;
    
    bool valid;
    
    CandidatePath() : 
        offset(0.0),
        obstacle_cost(0.0),
        curvature_cost(0.0),
        offset_cost(0.0),
        cost(0.0),
        valid(true) {}
};


// Control 구조체 (모든 상태 저장)
struct Control {
    // Waypoint 인덱스
    int close_idx;
    int target_idx;
    double ld;  // Lookahead distance
    
    // Lattice Planning 중간 결과들
    std::vector<OffsetGoal> offset_goals;
    std::vector<BaselinkGoal> baselink_goals;
    std::vector<PolynomialCoefficients> coefficients;
    
    // 최종 후보 경로들
    std::vector<CandidatePath> candidates;
    CandidatePath best_path;
    
    Control() : close_idx(0), target_idx(0), ld(0.0) {}
};


// Planner 파라미터
struct PlannerParams {
    int num_offsets;              // 오프셋 개수 (예: 9)
    double lateral_offset_step;   // 오프셋 간격 (예: 0.5m)
    double sample_spacing;        // 경로 샘플링 간격 (예: 0.2m)
    double lethal_cost_threshold; // 치명적 비용 임계값 (예: 90)
    
    PlannerParams() : 
        num_offsets(9),
        lateral_offset_step(0.5),
        sample_spacing(0.2),
        lethal_cost_threshold(90.0) {}
};


// Costmap 정보
struct CostmapInfo {
    const nav_msgs::OccupancyGrid* data;
    double origin_x;
    double origin_y;
    double resolution;
    
    CostmapInfo() : data(nullptr), origin_x(0.0), origin_y(0.0), resolution(0.0) {}
};

#endif // LATTICE_STRUCTS_HPP