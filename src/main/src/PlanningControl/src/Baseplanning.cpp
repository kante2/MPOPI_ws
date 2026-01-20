#include "Planning.hpp"
#include <cmath>
#include <algorithm>

using namespace std;

// ========================================
// Base Planning Process
// ========================================
void BasePlanningProcess(VehicleState& ego, ControlData& ctrl)
{
    closeWaypointsIdx(ego, ctrl.close_idx);
    getTargetwaypoint(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx, ego.max_curvature);
    getTargetSpeed(ego.max_curvature, ctrl.target_vel);
}

// ========================================
// Close Waypoint 찾기
// ========================================
void closeWaypointsIdx(const VehicleState& ego, int& out_idx)
{
    static int last_close_idx = 0;
    double best_close_dist = 10000000000;
    int close_idx = last_close_idx;
    int start = std::max(0, last_close_idx - 10);
    int end = std::min((int)waypoints.size() - 1, last_close_idx + 30);
    
    for (int i = start; i <= end; ++i) {
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < best_close_dist) {
            best_close_dist = dist;
            close_idx = i;
        }
    }
    
    last_close_idx = close_idx;
    out_idx = close_idx;
    ROS_INFO("close_idx: %d", close_idx);
}

// ========================================
// Target Waypoint 찾기
// ========================================
void getTargetwaypoint(const VehicleState& ego, int close_idx, int& out_target_idx, double& ld)
{
    ld = 5.0 + ld_gain * ego.vel;
    int target_idx = close_idx;
    int i = close_idx;
    
    for (; i <= waypoints.size() - 1; ++i) {
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist > ld) {
            target_idx = i;
            break;
        }
    }
    
    out_target_idx = target_idx;
}

// ========================================
// 최대 곡률 계산
// ========================================
void getMaxCurvature(int close_idx, int lookahead_idx, double& max_curvature)
{
    double max_kappa = 0.0;
    int end_idx = min((int)waypoints.size(), close_idx + lookahead_idx);
    
    for (int i = close_idx; i < end_idx; ++i) {
        double now_kappa = waypoints[i].curvature;
        if (now_kappa > max_kappa) {
            max_kappa = now_kappa;
        }
    }
    
    max_curvature = max_kappa;
}

// ========================================
// 목표 속도 결정
// ========================================
void getTargetSpeed(double max_curvature, double& out_target_vel)
{
    if (max_curvature > curve_standard) {
        out_target_vel = curve_vel;
    } else {
        out_target_vel = target_vel;
    }
}

// ========================================
// 곡률 전처리
// ========================================
void preprocessCurvature()
{
    for (int i = 0; i < waypoints.size() - 2; ++i) {
        double dx1 = waypoints[i+1].x - waypoints[i].x;
        double dx2 = waypoints[i+2].x - waypoints[i+1].x;
        double dy1 = waypoints[i+1].y - waypoints[i].y;
        double dy2 = waypoints[i+2].y - waypoints[i+1].y;
        double alpha1 = std::atan2(dy1, dx1);
        double alpha2 = std::atan2(dy2, dx2);
        double k_val = fabs(alpha1 - alpha2);
        waypoints[i+1].curvature = k_val;
    }
    
    cout << "=== End Preprocessing ===" << endl;
    waypoints[0].curvature = waypoints[1].curvature;
    waypoints.back().curvature = waypoints[waypoints.size()-2].curvature;
}