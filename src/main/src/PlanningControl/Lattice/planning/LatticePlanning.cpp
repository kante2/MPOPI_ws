// #include "global.hpp"
// #include "Planning.hpp"

// #include <ros/ros.h>
// #include <algorithm>
// #include <cmath>
// #include <Eigen/Dense>

// // ========================================
// // LatticePlanningProcess
// // ========================================
// void LatticePlanningProcess() {

//     if (!checkCostmapAvailable()) {
//         ROS_WARN_THROTTLE(1.0, "[Lattice] Waiting for costmap...");
//         return;
//     }

//     findClosestWaypoint(lattice_ctrl, ego);
//     findLookaheadGoal(lattice_ctrl, ego);
//     generateOffsetGoals(lattice_ctrl, ego);
//     computeAllPolynomialPaths(lattice_ctrl);
//     sampleAllCandidatePaths(lattice_ctrl);
//     evaluateAllCandidates(lattice_ctrl);
//     selectBestPath(lattice_ctrl); //is_first_global 여기서 플래그 변환
//     // closeWaypointsIdx(ego, ctrl.close_idx);
//     // getTargetwaypoint(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
//     // getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx, ego.max_curvature);
//     // getTargetSpeed(ego.max_curvature, ctrl.target_vel);
// }

// //--------------------------함수 정의----------------------------------------------------------

// // ========================================
// // Costmap ready check ( msg ptr 방식)
// // ========================================
// bool checkCostmapAvailable() {
//     return (costmap_info.msg != nullptr);
// }


// // ========================================
// // Close Waypoint 찾기 (전역/baselink 자동 판단)
// // ========================================
// void findClosestWaypoint(LatticeControl& lattice_ctrl, const VehicleState& ego) {
//     static int last_idx = 0;
//     double best_dist = 1e10;
//     int close_idx = last_idx;

//     if (lattice_ctrl.is_first_global) {
//         // 전역 waypoints에서 찾기
//         int start = std::max(0, last_idx - 10);
//         int end   = std::min((int)waypoints.size() - 1, last_idx + 50);

//         for (int i = start; i <= end; i++) {
//             double dx = waypoints[i].x - ego.x;
//             double dy = waypoints[i].y - ego.y;
//             double dist = std::sqrt(dx*dx + dy*dy);

//             if (dist < best_dist) {
//                 best_dist = dist;
//                 close_idx = i;
//             }
//         }
        
//         last_idx = close_idx;
//         lattice_ctrl.close_idx = close_idx;
//         ROS_INFO("[Lattice] Global close_idx: %d", close_idx);
        
//     } else {
//         // baselink best_path에서 찾기
//         if (lattice_ctrl.best_path.points.empty()) {
//             ROS_WARN("[Lattice] No baselink path available for close point search!");
//             lattice_ctrl.close_idx = 0;
//             return;
//         }
        
//         // baselink 좌표계에서 차량은 (0, 0)
//         int start = std::max(0, last_idx - 5);
//         int end   = std::min((int)lattice_ctrl.best_path.points.size() - 1, last_idx + 20);
        
//         for (int i = start; i <= end; i++) {
//             double dx = lattice_ctrl.best_path.points[i].x;
//             double dy = lattice_ctrl.best_path.points[i].y;
//             double dist = std::sqrt(dx*dx + dy*dy);
            
//             if (dist < best_dist) {
//                 best_dist = dist;
//                 close_idx = i;
//             }
//         }
        
//         last_idx = close_idx;
//         lattice_ctrl.close_idx = close_idx;
//         ROS_INFO("[Lattice] Baselink close_idx: %d (dist: %.2fm)", close_idx, best_dist);
//     }
// }

// // ========================================
// // Lookahead Goal 찾기 (전역/baselink 자동 판단)
// // ========================================
// void findLookaheadGoal(LatticeControl& lattice_ctrl, const VehicleState& ego) {
//     double ld = 15.0;
    
//     if (lattice_ctrl.is_first_global) {
//         // 전역 waypoints 사용
//         ROS_INFO("[Lattice] Finding lookahead from global waypoints");
        
//         int target_idx = lattice_ctrl.close_idx;
//         for (int i = lattice_ctrl.close_idx; i < (int)waypoints.size(); i++) {
//             double dx = waypoints[i].x - ego.x;
//             double dy = waypoints[i].y - ego.y;
//             double dist = std::sqrt(dx*dx + dy*dy);
            
//             if (dist >= ld) {
//                 target_idx = i;
//                 break;
//             }
//         }
        
//         lattice_ctrl.target_idx = target_idx;
//         lattice_ctrl.ld = ld;
        
//     } else {
//         // 이전 baselink path 사용
        
//         for (size_t i = 0; i < lattice_ctrl.best_path.points.size(); i++) {
//             const auto& pt = lattice_ctrl.best_path.points[i];
//             double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            
//             if (dist >= ld) {
//                 lattice_ctrl.lookahead_point = pt;
//                 lattice_ctrl.ld = ld;
//                 ROS_INFO("[Lattice] Baselink lookahead: (%.2f, %.2f)", pt.x, pt.y);
//                 return;
//             }
//         }
        
//         // 경로 끝까지 도달 → 마지막 점 사용
//         if (!lattice_ctrl.best_path.points.empty()) {
//             lattice_ctrl.lookahead_point = lattice_ctrl.best_path.points.back();
//         } 
//     }
// }

// // ========================================
// // 오프셋 후보 생성 + Baselink 변환 통합
// // ========================================
// void generateOffsetGoals(LatticeControl& lattice_ctrl, const VehicleState& ego) {
//     lattice_ctrl.offset_goals.clear();
//     lattice_ctrl.baselink_goals.clear(); // 여기서 baselink도 같이 생성

//     double goal_x, goal_y, yaw_ref;
    
//     if (lattice_ctrl.is_first_global) {
//         // 전역 waypoints 사용
//         int goal_idx = lattice_ctrl.target_idx;
//         goal_x = waypoints[goal_idx].x;
//         goal_y = waypoints[goal_idx].y;
        
//         // 방향 벡터 계산
//         double dx = 0.0, dy = 0.0;
//         if (goal_idx < (int)waypoints.size() - 1) {
//             dx = waypoints[goal_idx + 1].x - waypoints[goal_idx].x;
//             dy = waypoints[goal_idx + 1].y - waypoints[goal_idx].y;
//         }
        
//         double len = std::sqrt(dx*dx + dy*dy);
//         if (len < 1e-6) return;
        
//         dx /= len;
//         dy /= len;
//         yaw_ref = std::atan2(dy, dx);
        
//     } else {
//         // baselink path 사용
//         goal_x = lattice_ctrl.lookahead_point.x;
//         goal_y = lattice_ctrl.lookahead_point.y;
        
//         // 이전 경로 방향 계산
//         double dx = 1.0, dy = 0.0;
//         if (lattice_ctrl.best_path.points.size() >= 2) {
//             size_t n = lattice_ctrl.best_path.points.size();
//             dx = lattice_ctrl.best_path.points[n-1].x - lattice_ctrl.best_path.points[n-2].x;
//             dy = lattice_ctrl.best_path.points[n-1].y - lattice_ctrl.best_path.points[n-2].y;
//         }
        
//         double len = std::sqrt(dx*dx + dy*dy);
//         if (len < 1e-6) {
//             dx = 1.0; dy = 0.0;
//         } else {
//             dx /= len; dy /= len;
//         }
//         yaw_ref = std::atan2(dy, dx);
//     }


//     // 공통: offset 후보 생성 + baselink 변환
//     double norm_x = -std::sin(yaw_ref);
//     double norm_y = std::cos(yaw_ref);
    
//     for (int i = 0; i < planner_params.num_offsets; i++) {
//         double offset = -planner_params.lateral_offset_step *
//                         (planner_params.num_offsets - 1) / 2.0 +
//                         planner_params.lateral_offset_step * i;
        
//         // offset_goals 생성 (기록용)
//         OffsetGoal goal;
//         goal.global_x = goal_x + offset * norm_x;
//         goal.global_y = goal_y + offset * norm_y;
//         goal.global_yaw = yaw_ref;
//         goal.offset = offset;
//         lattice_ctrl.offset_goals.push_back(goal);
        
//         // 동시에 baselink_goals도 생성
//         BaselinkGoal bl_goal;
        
//         if (lattice_ctrl.is_first_global) {
//             // 첫 실행: 전역 → baselink 변환 필요
//             Point2D global_pt{goal.global_x, goal.global_y};
//             mapToBaseLink(global_pt, ego, bl_goal.point);
//             globalYawToBaselink(goal.global_yaw, ego, bl_goal.yaw);
            
//         } else {
//             // 이후: 이미 baselink → 그대로 사용
//             bl_goal.point.x = goal.global_x;
//             bl_goal.point.y = goal.global_y;
//             bl_goal.yaw = goal.global_yaw;
//         }
        
//         bl_goal.offset = offset;
//         lattice_ctrl.baselink_goals.push_back(bl_goal);
//     }
    
//     ROS_INFO("[Lattice] Generated %zu offset goals (mode: %s)", 
//              lattice_ctrl.baselink_goals.size(),
//              lattice_ctrl.is_first_global ? "GLOBAL" : "BASELINK");
// }

// // ========================================
// // 다항식 계수 계산
// // ========================================
// void computeAllPolynomialPaths(LatticeControl& lattice_ctrl) {
//     lattice_ctrl.coefficients.clear();

//     for (const auto& bl_goal : lattice_ctrl.baselink_goals) {
//         PolynomialCoefficients coeff = {0,0,0,0,0,0}; // 0으로 초기화

//         double X = bl_goal.point.x;
//         double Y = bl_goal.point.y;
//         double target_yaw = bl_goal.yaw; // normalizeAngle(Goal - Ego)된 값

//         // 거리(X)가 너무 작으면 행렬 폭발 -> 스킵
//         if (std::fabs(X) < 0.1) {
//             // 너무 가까우면 경로 생성 안 함 (직전 경로 유지하거나 정지)
//             lattice_ctrl.coefficients.push_back(coeff);
//             continue; 
//         }

//         // 각도(yaw)가 90도면 tan() 폭발 -> 클램핑
//         double limit = 85.0 * M_PI / 180.0; // 85도 제한
//         if (target_yaw > limit) target_yaw = limit;
//         if (target_yaw < -limit) target_yaw = -limit;

//         //안전하게 계산
//         double yaw_prime = std::tan(target_yaw);
        
//         // 5차 다항식 행렬 풀이 (start: x=0, y=0, yaw=0 가정)
//         double X2 = X*X, X3 = X2*X, X4 = X3*X, X5 = X4*X;

//         Eigen::Matrix3d A;
//         Eigen::Vector3d b;

//         A <<   X3,    X4,    X5,
//              3*X2,  4*X3,  5*X4,
//              6*X,  12*X2, 20*X3;

//         b << Y, yaw_prime, 0.0;

//         // FullPivLu가 역행렬 계산 시 가장 안정적임
//         Eigen::Vector3d sol = A.fullPivLu().solve(b);

//         coeff.a0 = 0.0;
//         coeff.a1 = 0.0;
//         coeff.a2 = 0.0;
//         coeff.a3 = sol(0);
//         coeff.a4 = sol(1);
//         coeff.a5 = sol(2);

//         lattice_ctrl.coefficients.push_back(coeff);
//     }
// }

// // ========================================
// // 경로 샘플링 (candidate.points는 baselink 유지)
// // ========================================
// void sampleAllCandidatePaths(LatticeControl& lattice_ctrl) {
//     lattice_ctrl.candidates.clear();

//     for (size_t i = 0; i < lattice_ctrl.coefficients.size(); i++) {
//         const auto& coeff   = lattice_ctrl.coefficients[i];
//         const auto& bl_goal = lattice_ctrl.baselink_goals[i];

//         CandidatePath candidate;
//         candidate.offset = bl_goal.offset;

//         double X = bl_goal.point.x;
        
//         if (X < 0.1) {
//             // 0.1m 보다 작으면 계산할 가치가 없음 -> 그냥 빈 경로 처리
//             candidate.points.clear(); // 빈 경로
//             lattice_ctrl.candidates.push_back(candidate);
//             continue;
//         }

//         if (std::fabs(X) < 1e-6) {
//             candidate.points.push_back({0.0, 0.0});
//         } else {
//             int num_points = (int)(std::fabs(X) / planner_params.sample_spacing) + 1;
//             if (num_points < 1) num_points = 1;

//             for (int j = 0; j <= num_points; j++) {
//                 double x = X * ((double)j / (double)num_points);

//                 double y = coeff.a0 +
//                            coeff.a1 * x +
//                            coeff.a2 * x*x +
//                            coeff.a3 * x*x*x +
//                            coeff.a4 * x*x*x*x +
//                            coeff.a5 * x*x*x*x*x;

//                 candidate.points.push_back({x, y}); // baselink
//             }
//         }

//         lattice_ctrl.candidates.push_back(candidate);
//     }
// }

// // ========================================
// // 경로 평가 (costmap query는 map 좌표로 변환 후)
// // ========================================
// void evaluateAllCandidates(LatticeControl& lattice_ctrl) {
//     double front_offset = planner_params.vehicle_front_offset; // 4.0m
//     double width = 2.0; // 차폭
//     double half_width = width / 2.0;

//     // [튜닝 1] 경로 변경 페널티 강화 (기존 0.3 -> 50.0)
//     double consistency_weight = 50.0; 

//     for (auto& path : lattice_ctrl.candidates) {
//         path.obstacle_cost = 0.0;
//         path.valid = true;
//         int lethal_count = 0;
//         int valid_point_count = 0;

//         for (size_t i = 0; i < path.points.size(); ++i) {
//             Point2D pt_rear = path.points[i]; // 뒷바퀴 위치

//             // 헤딩 계산
//             double heading = 0.0;
//             if (i + 1 < path.points.size()) {
//                 heading = atan2(path.points[i+1].y - pt_rear.y, path.points[i+1].x - pt_rear.x);
//             } else if (i > 0) {
//                 heading = atan2(pt_rear.y - path.points[i-1].y, pt_rear.x - path.points[i-1].x);
//             }
//             double c = cos(heading);
//             double s = sin(heading);

//             // ====================================================
//             // [핵심 수정] 몸통 전체 검사 (Body Collision Check)
//             // ====================================================
//             int max_cost_in_step = 0;
//             bool inside_map = false;

//             // d는 차량 길이 방향 거리 (0m ~ 4.0m)
//             for (double d = 0.0; d <= front_offset; d += 0.5) {
                
//                 // 검사할 가상의 차량 중심점 (뒷바퀴에서 d만큼 앞)
//                 double cx = pt_rear.x + d * c;
//                 double cy = pt_rear.y + d * s;

//                 // 좌/우/중앙 3점 검사 (std::vector 사용)
//                 std::vector<Point2D> body_points;
//                 body_points.push_back({cx, cy}); // 중앙
//                 body_points.push_back({cx - half_width * s, cy + half_width * c}); // 왼쪽
//                 body_points.push_back({cx + half_width * s, cy - half_width * c}); // 오른쪽

//                 for (const auto& p : body_points) {
//                     int gx, gy;
//                     if (BaseLinkToCostmap(p, gx, gy)) {
//                         inside_map = true;
//                         int cost = getCostmapCostFromGrid(gx, gy);
//                         if (cost > max_cost_in_step) max_cost_in_step = cost;
//                     }
//                 }
//             }

//             if (!inside_map) continue;

//             valid_point_count++;

//             // [튜닝 2] 충돌 임계값 확인
//             if (max_cost_in_step >= (int)planner_params.lethal_cost_threshold) {
//                 lethal_count++;
//             }
//             path.obstacle_cost += max_cost_in_step / 100.0;
//         }

//         if (lethal_count > 0) {
//             path.valid = false;
//             path.cost = 1e10;
//             continue;
//         }

//         if (valid_point_count > 0) path.obstacle_cost /= valid_point_count;
        
//         // 곡률 비용 계산
//         if (path.points.size() >= 3) {
//             for (size_t i = 1; i < path.points.size() - 1; i++) {
//                 double x0 = path.points[i-1].x, y0 = path.points[i-1].y;
//                 double x1 = path.points[i].x,   y1 = path.points[i].y;
//                 double x2 = path.points[i+1].x, y2 = path.points[i+1].y;

//                 double dx1 = x1 - x0, dy1 = y1 - y0;
//                 double dx2 = x2 - x1, dy2 = y2 - y1;

//                 double denom = (std::sqrt(dx1*dx1 + dy1*dy1) * std::sqrt(dx2*dx2 + dy2*dy2) + 1e-6);
//                 double curvature = std::fabs((dx1*dy2 - dy1*dx2) / denom);

//                 path.curvature_cost = std::max(path.curvature_cost, curvature);
//             }
//         }

//         path.offset_cost = std::fabs(path.offset) * 0.5;
        
//         // [적용] 강력해진 변덕 방지 비용
//         double offset_change_cost = std::fabs(path.offset - last_selected_offset) * consistency_weight;

//         path.cost = path.obstacle_cost * 100.0 + 
//                     path.offset_cost + 
//                     path.curvature_cost * 10.0 + 
//                     offset_change_cost;
//     }
// }


// // ========================================
// // 최적 경로 선택 + waypoints 갱신
// // ========================================
// void selectBestPath(LatticeControl& lattice_ctrl) {
//     if (lattice_ctrl.candidates.empty()) {
//         lattice_ctrl.best_path.valid = true;
//         lattice_ctrl.best_path.offset = 0.0;
//         return;
//     }

//     double best_cost = 1e10;
//     int best_idx = 0;

//     for (int i = 0; i < (int)lattice_ctrl.candidates.size(); i++) {
//         if (lattice_ctrl.candidates[i].cost < best_cost) {
//             best_cost = lattice_ctrl.candidates[i].cost;
//             best_idx = i;
//         }
//     }

//     lattice_ctrl.best_path = lattice_ctrl.candidates[best_idx];
//     last_selected_offset = lattice_ctrl.best_path.offset;
    
//     // best_path를 waypoints에 복사 (baselink 그대로)
//     waypoints.clear();
//     for (const auto& pt : lattice_ctrl.best_path.points) {
//         waypoints.push_back({pt.x, pt.y, 0.0}); // baselink 좌표 그대로
//     }
    
//     // 곡률 계산 (baselink 경로 기준)
//     if (waypoints.size() >= 3) {
//         for (size_t i = 1; i < waypoints.size() - 1; i++) {
//             double dx1 = waypoints[i].x - waypoints[i-1].x;
//             double dy1 = waypoints[i].y - waypoints[i-1].y;
//             double dx2 = waypoints[i+1].x - waypoints[i].x;
//             double dy2 = waypoints[i+1].y - waypoints[i].y;

//             double alpha1 = atan2(dy1, dx1);
//             double alpha2 = atan2(dy2, dx2);
//             waypoints[i].curvature = fabs(alpha1 - alpha2);
//         }
//         waypoints[0].curvature = waypoints[1].curvature;
//         waypoints.back().curvature = waypoints[waypoints.size()-2].curvature;
//     }
    
//     ROS_INFO("[Lattice] Best path selected: %zu points (offset: %.2f, cost: %.2f)", 
//              waypoints.size(), lattice_ctrl.best_path.offset, best_cost);
    
//     // 첫 경로 선택 완료 플래그 바꾸는 거 is_first_global = false로 바꿈
//     if (lattice_ctrl.is_first_global) {
//         lattice_ctrl.is_first_global = false;
//         ROS_INFO("[Lattice] First path selected, switched to baselink recursive mode");
//     }
// }

// // ========================================
// // 가까운 인덱스잡기
// // ========================================
// void closeWaypointsIdx(const VehicleState& ego, int& out_idx){

//     static int last_close_idx = 0;
//     double best_close_dist = 10000000000;
//     int close_idx = last_close_idx;
//     int start = std::max(0,last_close_idx - 10);
//     int end = std::min((int)waypoints.size() - 1,last_close_idx + 30);
//     for(int i = start; i <= end ; ++i){
//         double dx = waypoints[i].x;
//         double dy = waypoints[i].y;
//         double dist = sqrt(dx*dx + dy*dy);
//         if (dist < best_close_dist){
//                 best_close_dist = dist;
//                 close_idx = i;
//             }
           
//         }
//     last_close_idx = close_idx;
//     out_idx = close_idx;
//     ROS_INFO("close_idx: %d",close_idx);
// }

// // ========================================
// // 타겟 인덱스 잡기
// // ========================================
// void getTargetwaypoint(const VehicleState& ego, int close_idx, int& out_target_idx, double& ld){
//     ld = 5.0 + ld_gain * ego.vel;
//     int target_idx = close_idx;
//     int i = close_idx;
//     for(; i <= waypoints.size()-1; ++i ){
//         double dx = waypoints[i].x;
//         double dy = waypoints[i].y;
//         double dist = sqrt(dx*dx + dy*dy);
//         if(dist > ld){
//             target_idx = i;
//             break;
//           }
//         }
//     out_target_idx = target_idx;
// }

// // ========================================
// // 최대 곡률 계산
// // ========================================
// void getMaxCurvature(int close_idx, int lookahead_idx, double& max_curvature){
    
//     double max_kappa = 0.0;
//     int end_idx = min((int)waypoints.size(), close_idx + lookahead_idx);
//     // close_idx부터 end_idx 전까지만 탐색
//     for (int i = close_idx; i < end_idx; ++i) {
//         double now_kappa = waypoints[i].curvature; 
//         if (now_kappa > max_kappa) {
//             max_kappa = now_kappa;
//         }
//     } 
//     max_curvature = max_kappa;

// }

// // ========================================
// // 타겟 속도
// // ========================================
// void getTargetSpeed(double max_curvature, double& out_target_vel){
//     if(max_curvature > curve_standard){
//         out_target_vel = curve_vel;
//     }
//     else {out_target_vel = target_vel;}
// }