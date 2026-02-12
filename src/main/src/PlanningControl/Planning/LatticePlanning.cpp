#include "Global.hpp"
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

    lock_guard<std::mutex> lock(costmap_mutex);
    
    checkOvertakingZone(ego);
    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(1.0, "[Lattice] Waiting for costmap...");
        return;
    }
    findClosestWaypoint(ego, lattice_ctrl.close_idx);
    findLookaheadGoal(ego, lattice_ctrl.close_idx, lattice_ctrl);
    generateOffsetGoals(lattice_ctrl);
    transformOffsetGoalsToBaselink(lattice_ctrl, ego);
    computeAllPolynomialPaths(lattice_ctrl);
    sampleAllCandidatePaths(lattice_ctrl);
    evaluateAllCandidates(lattice_ctrl);
    selectBestPath(lattice_ctrl);
    getTargetLocalPathIdx(lattice_ctrl, ctrl.ld, ctrl.lookahead_idx);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx * 3, ego.max_curvature);
    getTargetSpeed(ego.max_curvature, ctrl.target_vel, ctrl.lookahead_idx);
}

//--------------------------함수 정의----------------------------------------------------------

void checkOvertakingZone(const VehicleState& ego) {
    if (overtaking_zone.empty()) {
        is_in_overtaking_zone = false;
        
        return;
    }
    if (!overtaking_zone.empty()) {
    ROS_INFO_THROTTLE(1.0, "Ego: (%.2f, %.2f) | First Zone Pt: (%.2f, %.2f)", 
                     ego.x, ego.y, overtaking_zone[0].x, overtaking_zone[0].y);
    }

    double min_dist = 1e10;
    
    // 가장 가까운 추월 차선 포인트와의 거리 계산 (L2 Distance)
    for (const auto& pt : overtaking_zone) {
        double dx = pt.x - ego.x;
        double dy = pt.y - ego.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }

    // 도로 폭이 양옆 2m이므로, 경로 중심에서 2m 이내면 영역 안으로 판단
    const double OVERTAKING_ROI_THRESHOLD = 5.0; 

    if (min_dist < OVERTAKING_ROI_THRESHOLD) {
        is_in_overtaking_zone = true;
    } else {
        is_in_overtaking_zone = false;
    }
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
void findLookaheadGoal(const VehicleState& ego, int close_idx, LatticeControl& lattice_ctrl) {
    double ld_short = lattice_ctrl.ld_short + ego.vel * 0.5;        // 5m
    double ld_medium = lattice_ctrl.ld_medium + ego.vel * 0.5;      // 10m
    double ld_long = lattice_ctrl.ld_long + ego.vel * 0.5;          // 15m
    double ld_very_long = lattice_ctrl.ld_very_long + ego.vel * 0.5; // 20m

    int target_idx_short = close_idx;
    int target_idx_medium = close_idx;
    int target_idx_long = close_idx;
    int target_idx_very_long = close_idx;


    for (int i = close_idx; i < (int)waypoints.size(); i++) {
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // ld_short보다 큰 첫 번째 idx
        if (dist >= ld_short && target_idx_short == close_idx) {
            target_idx_short = i;
        }

        // ld_medium보다 큰 첫 번째 idx
        if (dist >= ld_medium && target_idx_medium == close_idx) {
            target_idx_medium = i;
        }

        // ld_long보다 큰 첫 번째 idx
        if (dist >= ld_long && target_idx_long == close_idx) {
            target_idx_long = i;
        }

        // ld_very_long보다 큰 첫 번째 idx (이것을 찾으면 종료)
        if (dist >= ld_very_long && target_idx_very_long == close_idx) {
            target_idx_very_long = i;
            break;
        }
    }

    lattice_ctrl.target_idx_short = target_idx_short;
    lattice_ctrl.target_idx_medium = target_idx_medium;
    lattice_ctrl.target_idx_long = target_idx_long;
    lattice_ctrl.target_idx_very_long = target_idx_very_long;
}

// ========================================
// 오프셋 후보 생성
// ========================================
void generateOffsetGoals(LatticeControl& lattice_ctrl) {
    lattice_ctrl.offset_goals.clear(); // 초기화
    int n = planner_params.num_offsets; // 오프셋 후보 개수

    // 공통 목표점 생성 로직 (람다 함수)
    auto add_set = [&](int idx) {
        double goal_ref_x = waypoints[idx].x;
        double goal_ref_y = waypoints[idx].y;
        double dx = 0.0, dy = 0.0;
        if (idx < (int)waypoints.size() - 1) {
            dx = waypoints[idx + 1].x - waypoints[idx].x;
            dy = waypoints[idx + 1].y - waypoints[idx].y;
        }
        double len = std::max(1e-6, std::sqrt(dx*dx + dy*dy));
        double dir_x = dx / len, dir_y = dy / len;
        double norm_x = -dir_y, norm_y = dir_x;
        double yaw_global = std::atan2(dir_y, dir_x);

        //    // 예: n=5, offset_step=0.5m이면
        // i=0: offset = -1.0m  (맨 왼쪽)
        // i=1: offset = -0.5m
        // i=2: offset =  0.0m  (중앙)
        // i=3: offset = +0.5m
        // i=4: offset = +1.0m  (맨 오른쪽)
        for (int i = 0; i < n; i++) {
            double offset = -planner_params.lateral_offset_step * (n - 1) / 2.0 + 
                             planner_params.lateral_offset_step * i;
            OffsetGoal goal;
            goal.global_x = goal_ref_x + offset * norm_x;
            goal.global_y = goal_ref_y + offset * norm_y;
            goal.global_yaw = yaw_global;
            goal.offset = offset;
            lattice_ctrl.offset_goals.push_back(goal); // final pushback in this function
        }
    };

    add_set(lattice_ctrl.target_idx_very_long); // 후보 생성
    add_set(lattice_ctrl.target_idx_medium);    // 후보: 중간 경로
    add_set(lattice_ctrl.target_idx_long);      // 후보: 긴 경로
    add_set(lattice_ctrl.target_idx_short);     // 후보: 짧은 경로
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
// LatticePlanning.cpp

void evaluateAllCandidates(LatticeControl& lattice_ctrl) {
    double front_offset = planner_params.vehicle_front_offset; // 4.0m
    double width = 2.0; // 차폭
    double half_width = width / 2.0;

    double consistency_weight = 0.5;  

    for (auto& path : lattice_ctrl.candidates) {
        path.obstacle_cost = 0.0;
        path.lane_cost = 0.0; // [추가] 차선 비용 초기화
        path.valid = true;
        
        int lethal_count = 0;
        int valid_point_count = 0;
        double lane_penalty_sum = 0.0; // [추가] 차선 페널티 합산용 변수

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
            // 1. LiDAR 기반 물리적 장애물 검사 (Hard Constraint)
            // ====================================================
            int max_cost_in_step = 0;
            bool inside_map = false;

            // d는 차량 길이 방향 거리 (0m ~ 4.0m)
            for (double front_dx = 0.0; front_dx <= front_offset; front_dx += 0.5) {
                double cx = pt_rear.x + front_dx * cos_heading;
                double cy = pt_rear.y + front_dx * sin_heading;

                // 좌/우/중앙 3점 검사
                std::vector<Point2D> body_points;
                body_points.push_back({cx, cy}); // 중앙
                body_points.push_back({cx - half_width * sin_heading, cy + half_width * cos_heading}); // 왼쪽
                body_points.push_back({cx + half_width * sin_heading, cy - half_width * cos_heading}); // 오른쪽

                for (const auto& body_point : body_points) {
                    // LiDAR Costmap 직접 조회
                    if (checkCostmapAvailable()) { // 안전 장치
                         int cost = getCostmapCost(body_point.x, body_point.y);
                         if (cost > max_cost_in_step) {
                             max_cost_in_step = cost;
                         }
                         // 하나라도 맵 안에 있으면 유효한 포인트로 간주
                         if (cost >= 0) inside_map = true;
                    }
                }
            }

            // ====================================================
            // 2. Camera 기반 차선 이탈 검사 (Soft Constraint)
            // ====================================================
            // 차량 중심점 하나만 체크해도 충분 (연산 최적화)
            // 혹은 필요 시 위처럼 body_points 루프를 돌려도 됨.
            int cam_cost = getCameraCost(pt_rear.x, pt_rear.y); 
            lane_penalty_sum += cam_cost; // 차선 밖이면 5~10점 누적

            if (!inside_map) continue;

            valid_point_count++;

            // [LiDAR] 치명적 장애물(100)이면 즉시 카운트
            if (max_cost_in_step >= (int)planner_params.lethal_cost_threshold) {
                lethal_count++;
            }
            path.obstacle_cost += max_cost_in_step / 100.0;
        }

        // [LiDAR] 장애물 충돌 시 즉시 경로 폐기 (Hard Constraint)
        if (lethal_count > 0) {
            path.valid = false;
            path.cost = 1e10;
            continue;
        }

        if (valid_point_count > 0) {
            path.obstacle_cost /= valid_point_count;
            // [Camera] 차선 비용 평균 계산
            path.lane_cost = lane_penalty_sum / valid_point_count; 
        }
        
        // 곡률 비용 계산 - curvature_cost
        path.curvature_cost = 0.0;
        if (path.points.size() >= 3) {
            for (size_t i = 1; i < path.points.size() - 1; i++) {
                double x0 = path.points[i-1].x, y0 = path.points[i-1].y;
                double x1 = path.points[i].x,   y1 = path.points[i].y;
                double x2 = path.points[i+1].x, y2 = path.points[i+1].y;

                double dx1 = x1 - x0, dy1 = y1 - y0;
                double dx2 = x2 - x1, dy2 = y2 - y1;

                double denom = (std::sqrt(dx1*dx1 + dy1*dy1) * std::sqrt(dx2*dx2 + dy2*dy2) + 1e-6);
                double curvature = std::fabs((dx1*dy2 - dy1*dx2) / denom);

                path.points[i].curvature = curvature;
                path.curvature_cost = std::max(path.curvature_cost, curvature);
            }
        }

        path.offset_cost = std::fabs(path.offset);
        path.offset_change_cost = std::fabs(path.offset - last_selected_offset);
        
        path.cost = 0.0; 
    }
    
    // ========================================
    // 모든 코스트 정규화 (0~1 범위)
    // ========================================
    double max_obstacle = 0.0;
    double max_lane = 0.0; // [추가] 차선 비용 최대값
    double max_offset = 0.0;
    double max_curvature = 0.0;
    double max_offset_change = 0.0;
    
    for (const auto& path : lattice_ctrl.candidates) {
        if (!path.valid) continue;
        max_obstacle = std::max(max_obstacle, path.obstacle_cost);
        max_lane = std::max(max_lane, path.lane_cost); // [추가]
        max_offset = std::max(max_offset, path.offset_cost);
        max_curvature = std::max(max_curvature, path.curvature_cost);
        max_offset_change = std::max(max_offset_change, path.offset_change_cost);
    }
    
    // 0으로 나누기 방지
    if (max_obstacle < 1e-6) max_obstacle = 1.0;
    if (max_lane < 1e-6) max_lane = 1.0; // [추가]
    if (max_offset < 1e-6) max_offset = 1.0;
    if (max_curvature < 1e-6) max_curvature = 1.0;
    if (max_offset_change < 1e-6) max_offset_change = 1.0;
    
    // 정규화된 코스트 계산 (각 항목 0~1 범위, 가중치 합 = 1.0)
    for (auto& path : lattice_ctrl.candidates) {
        if (!path.valid) {
            path.cost = 1e10;
            continue;
        }
        
        double norm_obstacle = path.obstacle_cost / max_obstacle;
        double norm_lane = path.lane_cost / max_lane; // [추가] 차선 비용 정규화
        double norm_offset = path.offset_cost / max_offset;
        double norm_curvature = path.curvature_cost / max_curvature;
        double norm_offset_change = path.offset_change_cost / max_offset_change;
        
        // [최종 가중치 튜닝]
        // 장애물(40%) > 차선유지(30%) > 부드러움(15%) > 직진성(10%) > 안정성(5%)
        path.cost = norm_obstacle * 0.40 +        // 장애물 회피 (LiDAR)
                    norm_lane * 0.25 +            // 주행 가능 영역 (Camera)
                    norm_curvature * 0.05 +       // 급커브 방지
                    norm_offset * 0.15 +          // 중앙 차선 선호
                    norm_offset_change * 0.15;    // 경로 급변경 방지
    }
}

// ========================================
// 최적 경로 선택 (계층적 선택 로직)
// ========================================
void selectBestPath(LatticeControl& lattice_ctrl) {
    if (lattice_ctrl.candidates.empty()) return;
    int n = planner_params.num_offsets;  // 13

    double best_cost = 1e10;
    int best_idx = -1;

    // 🆕 계층적 경로 선택 (1~4단계)
    std::vector<std::pair<int, int>> path_groups = {
        {0, 2*n}, 
        // {n, 2*n},   // 1단계: VeryLong (0~12)
        {2*n, 4*n},         // 2단계: Medium (26~38)
    };

    // 각 그룹에서 유효한 경로 찾기
    for (const auto& group : path_groups) {
        best_cost = 1e10;
        best_idx = -1;

        for (int i = group.first; i < group.second && i < (int)lattice_ctrl.candidates.size(); i++) {
            if (lattice_ctrl.candidates[i].valid) {
                double cost = lattice_ctrl.candidates[i].cost;
                
                // 🆕 히스테리시스: 기존 경로 유지 (같은 그룹 내에서)
                // 현재 선택된 경로와 다른 경로의 비용 차이가 작으면 기존 유지
                static int last_selected_idx = -1;
                static double last_selection_cost = 1e10;
                
                // 같은 그룹에 속하는 기존 경로면 비용에 패널티 감소 (히스테리시스)
                double hysteresis_threshold = 2.0;  // 2% 이내 차이면 기존 유지
                
                if (last_selected_idx != -1 && 
                    last_selected_idx >= group.first && 
                    last_selected_idx < group.second &&
                    std::fabs(cost - last_selection_cost) < hysteresis_threshold) {
                    // 비용 차이가 작으면 기존 경로 선호
                    if (i == last_selected_idx) {
                        best_cost = cost;  // 기존 경로에 보너스
                        best_idx = i;
                    }
                    continue;
                }
                
                if (cost < best_cost) {
                    best_cost = cost;
                    best_idx = i;
                }
            }
        }

        // 현재 그룹에서 경로를 찾으면 선택
        if (best_idx != -1) {
            break;
        }
    }

    // 모든 경로가 막혔을 때의 예외 처리
    if (best_idx == -1) {
        ROS_ERROR_THROTTLE(0.5, "[Lattice] ALL PATHS BLOCKED! Emergency stop...");
        lattice_ctrl.best_path.points.clear();
        lattice_ctrl.best_path.valid = false;
        return;
    }

    // 최저 비용의 경로 선택
    lattice_ctrl.best_path = lattice_ctrl.candidates[best_idx];
    last_selected_offset = lattice_ctrl.best_path.offset;
    
    // 🆕 선택된 경로 주변만 유효성 계산
    int search_radius = 2;
    int start_idx = std::max(0, best_idx - search_radius);
    int end_idx = std::min((int)lattice_ctrl.candidates.size() - 1, best_idx + search_radius);
    
    int local_valid_count = 0;
    int local_search_range = end_idx - start_idx + 1;
    
    for (int i = start_idx; i <= end_idx; i++) {
        if (lattice_ctrl.candidates[i].valid) {
            local_valid_count++;
        }
    }
    
    lattice_ctrl.valid_path_ratio = (double)local_valid_count / (double)local_search_range;

    // VeryLong 그룹 평가
    // int very_long_valid = 0;
    // int very_long_range = std::min(n, (int)lattice_ctrl.candidates.size());
    // for (int i = 0; i < very_long_range; i++) {
    //     if (lattice_ctrl.candidates[i].valid) {
    //         very_long_valid++;
    //     }
    // }
    // // 모든 Very Long이 아니라, '내 차선 주변'의 Very Long만 검사
    // int center_idx = n / 2; // 보통 6 (13개일 경우)
    // int ego_vl_indices[] = {center_idx - 1, center_idx, center_idx + 1}; 
    
    // int ego_vl_valid_count = 0;
    // for (int idx : ego_vl_indices) {
    //     if (idx >= 0 && idx < n && lattice_ctrl.candidates[idx].valid) {
    //         ego_vl_valid_count++;
    //     }
    // }

    // // // 내 차선 Very Long 3개 중 하나라도 막히면(valid가 아니면) 1.0보다 작아짐
    // lattice_ctrl.very_long_path_ratio = (double)ego_vl_valid_count / 3.0;

    // 내 차선의 그룹 평가 (중앙 ± 1 범위)
    // 각 그룹(VeryLong, Long, Medium, Short)에서 중앙 경로(i%n==4)와 좌우 경로의 유효성 확인
    int center_remainder = n / 2;  // 중앙 인덱스의 나머지 값
    int ego_lane_valid = 0;   // 유효한 경로 개수 (center + left + right)
    int ego_lane_range = 0;   // 전체 체크 경로 개수
    
    for (int i = 0; i < (int)lattice_ctrl.candidates.size(); i++) {
        if (i % n == 4) {  // 각 그룹의 중앙 인덱스
            int ego_lane_center = i;
            int ego_lane_right = ego_lane_center - 1;
            int ego_lane_left = ego_lane_center + 1;

            // 좌우 범위 체크 (같은 그룹 내에서만)
            if (ego_lane_right >= 0 && ego_lane_right / n == i / n) {
                ego_lane_range++;
                if (lattice_ctrl.candidates[ego_lane_right].valid) {
                    ego_lane_valid++;
                }
            }
            
            ego_lane_range++;
            if (lattice_ctrl.candidates[ego_lane_center].valid) {
                ego_lane_valid++;
            }
            
            if (ego_lane_left < (int)lattice_ctrl.candidates.size() && ego_lane_left / n == i / n) {
                ego_lane_range++;
                if (lattice_ctrl.candidates[ego_lane_left].valid) {
                    ego_lane_valid++;
                }
            }
        }
    }

    // lattice_ctrl.very_long_path_ratio = (double)very_long_valid / (double)very_long_range;
    // 주행 차선을 포함하여, 주변부를 포함한, 내 차선의 비율 ( 속도에 이용할 예정) 
    lattice_ctrl.ego_path_ratio = ego_lane_range > 0 ? (double)ego_lane_valid / (double)ego_lane_range : 0.0; // 내 차선 + 양옆 1개씩 총 3개
}
// lattice_ctrl.best_path.points == local_path
// local path 에서 일정 ld이상인 index 인 out_idx 생성 ==> closewaypointsIdx 대체 


// ========================================
// 타겟 로컬 경로 인덱스 계산
// ========================================
void getTargetLocalPathIdx(LatticeControl& lattice_ctrl, double ld, int& out_idx) {
    
    int target_idx = 0;
    for(int i = 0; i < lattice_ctrl.best_path.points.size(); ++i){
        double local_path_x = lattice_ctrl.best_path.points[i].x;
        double local_path_y = lattice_ctrl.best_path.points[i].y;
        double local_dist = sqrt(local_path_x*local_path_x + local_path_y*local_path_y);

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
// 타겟 속도 (곡률 + 장애물 회피율 기반)
// ========================================
void getTargetSpeed(double max_curvature, double& out_target_vel, int lookahead_idx) {
    // [강력 수정] 추월 구간이면 곡률이고 장애물이고 뭐고 무조건 target_vel 고정
    if (is_in_overtaking_zone) {
        out_target_vel = target_vel; 
        ROS_INFO_THROTTLE(1.0, ">> [OVERTAKING ACTIVE] Forcing Target Speed: %.1f km/h", out_target_vel * 3.6);
        return; // 아래의 모든 감속 로직을 완전히 무시하고 즉시 종료
    }

    double base_vel = target_vel; 

    // 1. 코너 감속 (일반 주행 시에만 적용)
    if (max_curvature > curve_standard) {
        base_vel = curve_vel; 
        ROS_WARN_THROTTLE(1.0, "[Speed] Curve reduction active.");
    } 
    // 2. 일반 주행 구간 (장애물에 따른 감속 적용)
    else {
        double ego_ratio = lattice_ctrl.ego_path_ratio;
        double valid_ratio = lattice_ctrl.valid_path_ratio;
        // double very_long_ratio = lattice_ctrl.very_long_path_ratio;

        // if (very_long_ratio < 1.0) {
        //     base_vel *= 0.5; // 70% 수준으로 감속 (수치 조절 가능)
        //     ROS_WARN_THROTTLE(1.0, "[Speed] Distant obstacle detected (Very Long). Slowing down...");
        // }

        if (ego_ratio < 0.4) base_vel *= 0.3;
        else if (ego_ratio < 0.66) base_vel *= 0.4;
        else if (ego_ratio < 1.0) base_vel *= 0.5;

        if (valid_ratio < 0.1) base_vel = 5.0 / 3.6; 
        else if (valid_ratio < 0.3) base_vel *= 0.5;
        else if (valid_ratio < 0.6) base_vel *= 0.75;
    }

    out_target_vel = base_vel;
}
