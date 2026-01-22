#include "global.hpp"
#include "Planning.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

// ========================================
// Entry
// ========================================
void LatticePlanningProcess() {

    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(1.0, "[Lattice] Waiting for costmap...");
        return;
    }

    // main 기준: lattice_ctrl 사용
    findClosestWaypoint(ego, lattice_ctrl.close_idx);
    findLookaheadGoal(ego, lattice_ctrl.close_idx, lattice_ctrl.target_idx, lattice_ctrl.ld);

    generateOffsetGoals(lattice_ctrl.target_idx, lattice_ctrl);
    transformOffsetGoalsToBaselink(lattice_ctrl, ego);
    computeAllPolynomialPaths(lattice_ctrl);
    sampleAllCandidatePaths(lattice_ctrl);
    evaluateAllCandidates(lattice_ctrl);
    selectBestPath(lattice_ctrl);
}

// ========================================
// Costmap ready check ( msg ptr 방식)
// ========================================
bool checkCostmapAvailable() {
    return (costmap_info.msg != nullptr);
}

// ========================================
// Close Waypoint 찾기
// ========================================
void findClosestWaypoint(const VehicleState& ego, int& out_idx) {
    static int last_idx = 0;
    double best_dist = 1e10;
    int close_idx = last_idx;

    int start = std::max(0, last_idx - 10);
    int end   = std::min((int)waypoints.size() - 1, last_idx + 50);

    for (int i = start; i <= end; i++) {
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist < best_dist) {
            best_dist = dist;
            close_idx = i;
        }
    }

    last_idx = close_idx;
    out_idx = close_idx;
}

// ========================================
// Lookahead Goal 찾기
// ========================================
void findLookaheadGoal(const VehicleState& ego, int close_idx,
                       int& out_target_idx, double& ld) {
    ld = 10.0;  // 고정 10m
    int target_idx = close_idx;

    for (int i = close_idx; i < (int)waypoints.size(); i++) {
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist >= ld) {
            target_idx = i;
            break;
        }
    }

    out_target_idx = target_idx;
}

// ========================================
// 오프셋 후보 생성
// ========================================
void generateOffsetGoals(int goal_idx, LatticeControl& lattice_ctrl) {
    lattice_ctrl.offset_goals.clear();

    double goal_ref_x = waypoints[goal_idx].x;
    double goal_ref_y = waypoints[goal_idx].y;

    // 방향 벡터 계산
    double dx = 0.0, dy = 0.0;
    if (goal_idx < (int)waypoints.size() - 1) {
        dx = waypoints[goal_idx + 1].x - waypoints[goal_idx].x;
        dy = waypoints[goal_idx + 1].y - waypoints[goal_idx].y;
    }

    double len = std::sqrt(dx*dx + dy*dy);
    if (len < 1e-6) return;

    // 단위 방향 벡터
    double dir_x = dx / len;
    double dir_y = dy / len;

    // 법선 벡터 (좌측이 양수)
    double norm_x = -dir_y;
    double norm_y = dir_x;

    double yaw_global = std::atan2(dir_y, dir_x); //목표점 요값 ,, 여기가 잘못된거 같은게 목표점 요값 -차량 헤당값한거 뒤에 그걸로해야함

    for (int i = 0; i < planner_params.num_offsets; i++) {
        double offset = -planner_params.lateral_offset_step *
                        (planner_params.num_offsets - 1) / 2.0 +
                        planner_params.lateral_offset_step * i;

        OffsetGoal goal;
        goal.global_x = goal_ref_x + offset * norm_x;
        goal.global_y = goal_ref_y + offset * norm_y;
        goal.global_yaw = yaw_global;
        goal.offset = offset;

        lattice_ctrl.offset_goals.push_back(goal);
    }
}

// ========================================
// Baselink 변환
// ========================================
void transformOffsetGoalsToBaselink(LatticeControl& lattice_ctrl, const VehicleState& ego) {
    lattice_ctrl.baselink_goals.clear();

    for (const auto& goal : lattice_ctrl.offset_goals) {
        Point2D global_pt{goal.global_x, goal.global_y};

        BaselinkGoal bl_goal;
        mapToBaseLink(global_pt, ego, bl_goal.point);
        globalYawToBaselink(goal.global_yaw, ego, bl_goal.yaw);
        bl_goal.offset = goal.offset;

        lattice_ctrl.baselink_goals.push_back(bl_goal);
    }
}

// ========================================
// 다항식 계수 계산
// ========================================
void computeAllPolynomialPaths(LatticeControl& lattice_ctrl) {
    lattice_ctrl.coefficients.clear();

    for (const auto& bl_goal : lattice_ctrl.baselink_goals) {
        PolynomialCoefficients coeff = {0,0,0,0,0,0}; // 0으로 초기화

    
        double X = bl_goal.point.x;
        double Y = bl_goal.point.y;
        double target_yaw = bl_goal.yaw; // normalizeAngle(Goal - Ego)된 값


        // 거리(X)가 너무 작으면 행렬 폭발 -> 스킵
        if (std::fabs(X) < 0.1) {
            // 너무 가까우면 경로 생성 안 함 (직전 경로 유지하거나 정지)
            lattice_ctrl.coefficients.push_back(coeff);
            continue; 
        }

        // 각도(yaw)가 90도면 tan() 폭발 -> 클램핑
        double limit = 85.0 * M_PI / 180.0; // 85도 제한
        if (target_yaw > limit) target_yaw = limit;
        if (target_yaw < -limit) target_yaw = -limit;

        //안전하게 계산
        double yaw_prime = std::tan(target_yaw);
        
        // 5차 다항식 행렬 풀이 (start: x=0, y=0, yaw=0 가정)
        double X2 = X*X, X3 = X2*X, X4 = X3*X, X5 = X4*X;

        Eigen::Matrix3d A;
        Eigen::Vector3d b;

        A <<   X3,    X4,    X5,
             3*X2,  4*X3,  5*X4,
             6*X,  12*X2, 20*X3;

        b << Y, yaw_prime, 0.0;

        // FullPivLu가 역행렬 계산 시 가장 안정적임
        Eigen::Vector3d sol = A.fullPivLu().solve(b);

        coeff.a0 = 0.0;
        coeff.a1 = 0.0;
        coeff.a2 = 0.0;
        coeff.a3 = sol(0);
        coeff.a4 = sol(1);
        coeff.a5 = sol(2);

        lattice_ctrl.coefficients.push_back(coeff);
    }
}

// ========================================
// 경로 샘플링 (candidate.points는 baselink 유지)
// ========================================
void sampleAllCandidatePaths(LatticeControl& lattice_ctrl) {
    lattice_ctrl.candidates.clear();

    for (size_t i = 0; i < lattice_ctrl.coefficients.size(); i++) {
        const auto& coeff   = lattice_ctrl.coefficients[i];
        const auto& bl_goal = lattice_ctrl.baselink_goals[i];

        CandidatePath candidate;
        candidate.offset = bl_goal.offset;

        double X = bl_goal.point.x;
        
        if (X < 0.1) {
            // 0.1m 보다 작으면 계산할 가치가 없음 -> 그냥 빈 경로 처리
            candidate.points.clear(); // 빈 경로
            lattice_ctrl.candidates.push_back(candidate);
            continue;
        }

        if (std::fabs(X) < 1e-6) {
            candidate.points.push_back({0.0, 0.0});
        } else {
            int num_points = (int)(std::fabs(X) / planner_params.sample_spacing) + 1;
            if (num_points < 1) num_points = 1;

            for (int j = 0; j <= num_points; j++) {
                double x = X * ((double)j / (double)num_points);

                double y = coeff.a0 +
                           coeff.a1 * x +
                           coeff.a2 * x*x +
                           coeff.a3 * x*x*x +
                           coeff.a4 * x*x*x*x +
                           coeff.a5 * x*x*x*x*x;

                candidate.points.push_back({x, y}); // baselink
            }
        }

        lattice_ctrl.candidates.push_back(candidate);
    }
}


// ========================================
// 경로 평가 (costmap query는 map 좌표로 변환 후)
// ========================================
void evaluateAllCandidates(LatticeControl& lattice_ctrl) {
    // 차량 앞 범퍼 오프셋 (global.hpp의 planner_params에 정의된 값 사용)
    // 만약 파라미터가 없다면 여기서 const double VEHICLE_FRONT_OFFSET = 1.3; 처럼 선언해서 쓰세요.
    double front_offset = planner_params.vehicle_front_offset;

    for (auto& path : lattice_ctrl.candidates) {
        path.obstacle_cost = 0.0;
        path.curvature_cost = 0.0;
        path.offset_cost = 0.0;
        path.valid = true;

        int lethal_count = 0;
        int valid_point_count = 0;

        for (const auto& pt_bl : path.points) {
            // ------------------------------------
            // 1. 뒷바퀴(Base_link) 위치 체크
            // ------------------------------------
            int grid_x, grid_y;
            bool is_rear_valid = BaseLinkToCostmap(pt_bl, grid_x, grid_y);

            // ------------------------------------
            // 2. 앞 범퍼(Front Bumper) 위치 체크
            // ------------------------------------
            // 경로의 진행 방향(Heading)을 몰라도, Lattice 경로는 차체 정렬이 되어있으므로 
            // 단순히 x축 방향으로 offset을 더해도 근사적으로 맞습니다.
            Point2D pt_front;
            pt_front.x = pt_bl.x + front_offset;
            pt_front.y = pt_bl.y;

            int grid_fx, grid_fy;
            bool is_front_valid = BaseLinkToCostmap(pt_front, grid_fx, grid_fy);

            // 둘 다 맵 밖이면 스킵
            if (!is_rear_valid && !is_front_valid) {
                continue;  
            }

            valid_point_count++;
            
            // 비용 조회: 맵 안인 경우만 조회, 아니면 0
            int cost_rear = is_rear_valid ? getCostmapCostFromGrid(grid_x, grid_y) : 0;
            int cost_front = is_front_valid ? getCostmapCostFromGrid(grid_fx, grid_fy) : 0;

            // 둘 중 더 위험한 비용 선택 (보수적 판단)
            int max_cost = std::max(cost_rear, cost_front);

            if (max_cost >= (int)planner_params.lethal_cost_threshold) {
                lethal_count++;
            }

            path.obstacle_cost += max_cost / 100.0;
        }

        // Lethal cost(충돌)가 있으면 경로 폐기
        if (lethal_count > 0) {
            path.valid = false;
            path.cost = 1e10; // 비용 무한대
            continue;
        }

        // 평균 장애물 비용 계산
        if (valid_point_count > 0) {
            path.obstacle_cost /= valid_point_count;
        }

        // 곡률 비용
        if (path.points.size() >= 3) {
            for (size_t i = 1; i < path.points.size() - 1; i++) {
                double x0 = path.points[i-1].x, y0 = path.points[i-1].y;
                double x1 = path.points[i].x,   y1 = path.points[i].y;
                double x2 = path.points[i+1].x, y2 = path.points[i+1].y;

                double dx1 = x1 - x0, dy1 = y1 - y0;
                double dx2 = x2 - x1, dy2 = y2 - y1;
                double denom = (std::sqrt(dx1*dx1 + dy1*dy1) * std::sqrt(dx2*dx2 + dy2*dy2) + 1e-6);
                double curvature = std::fabs((dx1*dy2 - dy1*dx2) / denom);

                path.curvature_cost = std::max(path.curvature_cost, curvature);
            }
        }

        // 오프셋 비용
        path.offset_cost = std::fabs(path.offset) * 0.5;
        double offset_change_cost = std::fabs(path.offset - last_selected_offset) * 0.3;

        // 총 비용 계산
        path.cost = path.obstacle_cost * 100.0 +
                    path.offset_cost +
                    path.curvature_cost * 10.0 +
                    offset_change_cost;
    }
}
// ========================================
// 최적 경로 선택
// ========================================
void selectBestPath(LatticeControl& lattice_ctrl) {
    if (lattice_ctrl.candidates.empty()) {
        lattice_ctrl.best_path.valid = true;
        lattice_ctrl.best_path.offset = 0.0;
        return;
    }

    double best_cost = 1e10;
    int best_idx = 0;

    for (int i = 0; i < (int)lattice_ctrl.candidates.size(); i++) {
        if (lattice_ctrl.candidates[i].cost < best_cost) {
            best_cost = lattice_ctrl.candidates[i].cost;
            best_idx = i;
        }
    }

    lattice_ctrl.best_path = lattice_ctrl.candidates[best_idx];
    last_selected_offset = lattice_ctrl.best_path.offset;
}
