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

    double yaw_global = std::atan2(dir_y, dir_x);

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
        PolynomialCoefficients coeff;

        coeff.a0 = 0.0;
        coeff.a1 = 0.0;
        coeff.a2 = 0.0;

        double X = bl_goal.point.x;
        double Y = bl_goal.point.y;

        if (std::fabs(X) < 1e-3) {
            coeff.a3 = coeff.a4 = coeff.a5 = 0.0;
        } else {
            double yaw_prime = std::tan(bl_goal.yaw);
            double X2 = X*X, X3 = X2*X, X4 = X3*X, X5 = X4*X;

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A(0,0) = X3;     A(0,1) = X4;      A(0,2) = X5;
            b(0)   = Y;

            A(1,0) = 3.0*X2; A(1,1) = 4.0*X3;  A(1,2) = 5.0*X4;
            b(1)   = yaw_prime;

            A(2,0) = 6.0*X;  A(2,1) = 12.0*X2; A(2,2) = 20.0*X3;
            b(2)   = 0.0;

            Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
            coeff.a3 = sol(0);
            coeff.a4 = sol(1);
            coeff.a5 = sol(2);
        }

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

bool worldToCostmapCoord(double world_x, double world_y,
                         int& grid_x, int& grid_y)
{
    if (!checkCostmapAvailable()) return false;

    const auto& cm = *costmap_info.msg;

    // resolution 0 보호
    if (cm.info.resolution <= 1e-9) return false;

    grid_x = (int)std::floor((world_x - cm.info.origin.position.x) / cm.info.resolution);
    grid_y = (int)std::floor((world_y - cm.info.origin.position.y) / cm.info.resolution);

    if (grid_x < 0 || grid_x >= (int)cm.info.width ||
        grid_y < 0 || grid_y >= (int)cm.info.height) {
        return false;
    }

    return true;
}

int getCostmapCost(double world_x, double world_y)
{
    if (!checkCostmapAvailable()) return 0;

    const auto& cm = *costmap_info.msg;

    int grid_x, grid_y;
    if (!worldToCostmapCoord(world_x, world_y, grid_x, grid_y)) {
        // costmap 바깥은 위험으로 처리 (기존 로직 유지)
        return (int)planner_params.lethal_cost_threshold;
    }

    const int width = (int)cm.info.width;
    const int height = (int)cm.info.height;

    // data 크기 sanity
    const int expected = width * height;
    if ((int)cm.data.size() < expected || expected <= 0) {
        return (int)planner_params.lethal_cost_threshold;
    }

    const int idx = grid_y * width + grid_x;
    if (idx < 0 || idx >= (int)cm.data.size()) {
        return (int)planner_params.lethal_cost_threshold;
    }

    const int8_t raw = cm.data[idx];

    // -1: unknown (ROS occupancygrid 관례)
    if (raw < 0) return 30;  // 기존처럼 medium cost

    int cost = (int)raw;
    if (cost < 0) cost = 0;
    if (cost > 100) cost = 100;
    return cost;
}


void evaluateAllCandidates(LatticeControl& lattice_ctrl) {
    for (auto& path : lattice_ctrl.candidates) {
        path.obstacle_cost = 0.0;
        path.curvature_cost = 0.0;
        path.offset_cost = 0.0;
        path.valid = true;

        int lethal_count = 0;
        int valid_point_count = 0;

        for (const auto& pt_bl : path.points) {
            //  baselink -> map 변환
            Point2D map_pt;
            baselinkToMap(pt_bl, ego, map_pt);

            int grid_x, grid_y;
            if (!worldToCostmapCoord(map_pt.x, map_pt.y, grid_x, grid_y)) {
                continue;
            }

            valid_point_count++;
            int cost = getCostmapCost(map_pt.x, map_pt.y);

            if (cost >= (int)planner_params.lethal_cost_threshold) {
                lethal_count++;
            }

            path.obstacle_cost += cost / 100.0;
        }

        if (lethal_count > 0) {
            path.valid = false;
            path.cost = 1e10;
            continue;
        }

        if (valid_point_count > 0) {
            path.obstacle_cost /= valid_point_count;
        }

        // 곡률 비용 (baselink 기준으로 계산해도 OK)
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

        path.offset_cost = std::fabs(path.offset) * 0.5;
        double offset_change_cost = std::fabs(path.offset - last_selected_offset) * 0.3;

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
