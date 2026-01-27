#include "global.hpp"
#include "Planning.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
using namespace std;

// ========================================
// LatticePlanningProcess
// ========================================
void LatticePlanningProcess() {

    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(1.0, "[Lattice] Waiting for costmap...");
        return;
    }

    findClosestWaypoint(ego, lattice_ctrl.close_idx);
    findLookaheadGoal(ego, lattice_ctrl.close_idx, lattice_ctrl.target_idx, lattice_ctrl.ld);
    generateOffsetGoals(lattice_ctrl.target_idx, lattice_ctrl);
    transformOffsetGoalsToBaselink(lattice_ctrl, ego);
    computeAllPolynomialPaths(lattice_ctrl);
    sampleAllCandidatePaths(lattice_ctrl);
    evaluateAllCandidates(lattice_ctrl);
    selectBestPath(lattice_ctrl);
    // ctrl.ld = 5.0; //ctrl.ld가 설정안되어있어서 임시로 여기에 놨어요 이거 다른데서 설정되게 해야함,,ㄴ

    // closeWaypointsIdx(ego, ctrl.close_idx); => getLocalPathIdx 로 변경
    getTargetLocalPathIdx(lattice_ctrl, ctrl.ld, ctrl.lookahead_idx);

    // getTargetWaypoint(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx, ego.max_curvature);
    getTargetSpeed(ego.max_curvature, ctrl.target_vel);
    

}

//--------------------------함수 정의----------------------------------------------------------


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
    ld = 15.0;  // 고정 10m
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

    // goal_idx ~ goal_idx +
    // GPS 좌표계의 목표점
    double goal_ref_x = waypoints[goal_idx].x;
    double goal_ref_y = waypoints[goal_idx].y;

    // 방향 벡터 계산
    // now ~ goal vector (dx, dy)
    // 경로의 진행 방향 = 목표 yaw 각도
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
        // global yaw -> baselink yaw
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
        } 
        else {
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
// LatticePlanning.cpp
void evaluateAllCandidates(LatticeControl& lattice_ctrl) {
    double front_offset = planner_params.vehicle_front_offset; // 4.0m
    double width = 2.0; // 차폭
    double half_width = width / 2.0;

    double consistency_weight = 0.5;  // 히스테리시스 무시// ← 매우 감소 (특정 경로에 갇히지 않도록) 

    for (auto& path : lattice_ctrl.candidates) {
        path.obstacle_cost = 0.0;
        path.valid = true;
        int lethal_count = 0;
        int valid_point_count = 0;

        for (size_t i = 0; i < path.points.size(); ++i) {
            Point2D pt_rear = path.points[i]; // base_link 좌표

            // 헤딩 계산
            double heading = 0.0;
            if (i + 1 < path.points.size()) {
                heading = atan2(path.points[i+1].y - pt_rear.y, 
                               path.points[i+1].x - pt_rear.x);
            } else if (i > 0) {
                heading = atan2(pt_rear.y - path.points[i-1].y, 
                               pt_rear.x - path.points[i-1].x);
            }
            double cos_heading = cos(heading);
            double sin_heading = sin(heading);

            // ====================================================
            // [핵심 수정] 몸통 전체 검사 - 첫 번째 코드 스타일
            // ====================================================
            int max_cost_in_step = 0;
            bool inside_map = false;

            // d는 차량 길이 방향 거리 (0m ~ 4.0m)
            for (double front_dx = 0.0; front_dx <= front_offset; front_dx += 0.5) {
                
                // 검사할 가상의 차량 중심점 (뒷바퀴에서 d만큼 앞)
                double cx = pt_rear.x + front_dx * cos_heading;
                double cy = pt_rear.y + front_dx * sin_heading;

                // 좌/우/중앙 3점 검사
                std::vector<Point2D> body_points;
                body_points.push_back({cx, cy}); // 중앙
                body_points.push_back({cx - half_width * sin_heading, cy + half_width * cos_heading}); // 왼쪽
                body_points.push_back({cx + half_width * sin_heading, cy - half_width * cos_heading}); // 오른쪽

                // 핵심: 직접 costmap 조회 (변환 함수 없음)
                for (const auto& body_point : body_points) {
                    int grid_x, grid_y;
                    
                    // base_link → grid 직접 계산
                    if (worldToCostmapCoord(body_point.x, body_point.y, grid_x, grid_y)) {
                        inside_map = true;
                        
                        // grid에서 직접 비용 읽기
                        int cost = getCostmapCost(body_point.x, body_point.y);
                        
                        if (cost > max_cost_in_step) {
                            max_cost_in_step = cost;
                        }
                    }
                }
            }

            if (!inside_map) continue;

            valid_point_count++;

            if (max_cost_in_step >= (int)planner_params.lethal_cost_threshold) {
                lethal_count++;
            }
            path.obstacle_cost += max_cost_in_step / 100.0;
        }
        // fatal point --> not valid
        if (lethal_count > 0) {
            path.valid = false;
            path.cost = 1e10;
            continue;
        }

        if (valid_point_count > 0) {
            path.obstacle_cost /= valid_point_count;
        }
        
        // 곡률 비용 계산 - curvature_cost
        if (path.points.size() >= 3) {
            for (size_t i = 1; i < path.points.size() - 1; i++) {
                double x0 = path.points[i-1].x, y0 = path.points[i-1].y;
                double x1 = path.points[i].x,   y1 = path.points[i].y;
                double x2 = path.points[i+1].x, y2 = path.points[i+1].y;

                double dx1 = x1 - x0, dy1 = y1 - y0;
                double dx2 = x2 - x1, dy2 = y2 - y1;

                double denom = (std::sqrt(dx1*dx1 + dy1*dy1) * 
                               std::sqrt(dx2*dx2 + dy2*dy2) + 1e-6);
                double curvature = std::fabs((dx1*dy2 - dy1*dx2) / denom);

                path.curvature_cost = std::max(path.curvature_cost, curvature);
            }
        }

        path.offset_cost = std::fabs(path.offset) * 1.0;
        
        double offset_change_cost = std::fabs(path.offset - last_selected_offset) * consistency_weight;

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

    // select path that has the lowest cost
    for (int i = 0; i < (int)lattice_ctrl.candidates.size(); i++) {
        if (lattice_ctrl.candidates[i].cost < best_cost) {
            best_cost = lattice_ctrl.candidates[i].cost;
            best_idx = i;
        }
    }


    lattice_ctrl.best_path = lattice_ctrl.candidates[best_idx];
    last_selected_offset = lattice_ctrl.best_path.offset;
    
}

// lattice_ctrl.best_path.points == local_path
// local path 에서 일정 ld이상인 index 인 out_idx 생성 ==> closewaypointsIdx 대체 
//
void getTargetLocalPathIdx(LatticeControl& lattice_ctrl, double ld, int& out_idx){
    int target_idx = 0;
    for(int i = 0; i < lattice_ctrl.best_path.points.size(); ++i){
        double local_path_x = lattice_ctrl.best_path.points[i].x;
        double local_path_y = lattice_ctrl.best_path.points[i].y;
        double local_dist = sqrt(local_path_x*local_path_x + local_path_y*local_path_y); // <- ego ~ local_path point 거리
        if(local_dist > ld){
            target_idx = i;
            break;
        }
    }
    out_idx = target_idx;
}


// ========================================
// 최대 곡률 계산
// ========================================
void getMaxCurvature(int close_idx, int lookahead_idx, double& max_curvature){
    
    double max_kappa = 0.0;
    int end_idx = min((int)lattice_ctrl.best_path.points.size(), close_idx + lookahead_idx);
    // close_idx부터 end_idx 전까지만 탐색
    for (int i = close_idx; i < end_idx; ++i) {
        double now_kappa = lattice_ctrl.best_path.points[i].curvature;
        if (now_kappa > max_kappa) {
            max_kappa = now_kappa;
        }
    } 
    max_curvature = max_kappa;

}
// ========================================
// 타겟 속도
// ========================================
void getTargetSpeed(double max_curvature, double& out_target_vel){ 
    // smoothing needed 
    if(max_curvature > curve_standard){
        out_target_vel = curve_vel;
    }
    else {out_target_vel = target_vel;}
}
