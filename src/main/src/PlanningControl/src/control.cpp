#include "Planning.hpp"
#include <morai_msgs/CtrlCmd.h>
#include <cmath>
#include <algorithm>

using namespace std;

// ========================================
// Control Process
// ========================================
void ControlProcess() {

    if (current_mission == Mission::JAMMING) return;  // Jamming은 Planning에서 제어까지 함
    if (current_mission == Mission::END) {
        PublishStopCommand();
        return;
    }
    getsteering(ego, ctrl);
    computePID(ego.vel, ctrl.target_vel, ctrl.accel, ctrl.brake);
    pubCmd(ctrl);
    
    // 로깅
    ROS_INFO_THROTTLE(1.0, "[Control] V:%.1f(%.1f) | Steer:%.2f | Acc:%.2f | Brk:%.2f", 
                     ego.vel, ctrl.target_vel, ctrl.steering, 
                     ctrl.accel, ctrl.brake);
}

//--------------- 함수정의 ---------------------------------------------------------

// ========================================
// 조향각 계산 (Stanley)
// ========================================
void getsteering(const VehicleState& ego, ControlData& ctrl)
{
    double path_e = lateralPathError(ctrl.target_idx, ego.x, ego.y);
    double heading_e = headingError(ego.yaw, ctrl.target_idx);
    double v = std::max(1.0, ego.vel);
    
    double steering_raw = heading_e + atan(k_gain * path_e / v);
    
    const double MAX_STEERING = 30.0 * M_PI / 180.0;
    ctrl.steering = std::max(-MAX_STEERING, std::min(MAX_STEERING, steering_raw));
    
    ROS_INFO("[Steering] path_e: %.3f, heading_e: %.3f, raw: %.3f, limited: %.3f deg",
             path_e, heading_e, steering_raw, ctrl.steering * 180.0 / M_PI);
}

// ========================================
// Lateral Path Error
// ========================================
double lateralPathError(int target_idx, double x, double y) {
    
    const std::vector<Waypoint>* target_waypoints = nullptr;
    
    if (current_mission == Mission::NORMAL) {
        target_waypoints = &waypoints;
    } 
    else if (current_mission == Mission::LATTICE) {
        target_waypoints = &lattice_waypoints;
    } 
    else {
        return 0.0;
    }
    
    if (target_waypoints->empty() || target_idx >= target_waypoints->size() - 1) {
        return 0.0;
    }
    
    int i = target_idx;
    double a = (*target_waypoints)[i].x;
    double b = (*target_waypoints)[i].y;
    double B = (*target_waypoints)[i+1].x - (*target_waypoints)[i].x;
    double A = (*target_waypoints)[i+1].y - (*target_waypoints)[i].y;
    double C = -a*A + b*B;
    
    if (A == 0 && B == 0) return 0.0;
    
    return (x*A - y*B + C) / sqrt(A*A + B*B);
}
// ========================================
// Heading Error
// ========================================
double headingError(double yaw, int target_idx) {
    
    const std::vector<Waypoint>* target_waypoints = nullptr;
    
    if (current_mission == Mission::NORMAL) {
        target_waypoints = &waypoints;
    } 
    else if (current_mission == Mission::LATTICE) {
        target_waypoints = &lattice_waypoints;
    } 
    else {
        return 0.0;
    }
    
    if (target_waypoints->empty() || target_idx >= target_waypoints->size() - 1) {
        return 0.0;
    }
    
    int i = target_idx;
    double dx = (*target_waypoints)[i+1].x - (*target_waypoints)[i].x;
    double dy = (*target_waypoints)[i+1].y - (*target_waypoints)[i].y;
    double path_heading = atan2(dy, dx);
    double diff = path_heading - yaw;
    
    return atan2(sin(diff), cos(diff));
}

// ========================================
// PID 속도 제어
// ========================================
void computePID(double vel, double target_vel, double& out_accel, double& out_brake)
{
    static double prev_error = 0.0;
    static double integral_error = 0.0;
    
    double error = target_vel - vel;
    integral_error += error * 0.02;
    
    if (integral_error > 10.0) integral_error = 10.0;
    if (integral_error < -10.0) integral_error = -10.0;
    
    double p_error = Kp * error;
    double i_error = Ki * integral_error;
    double d_error = Kd * ((error - prev_error) / 0.02);
    prev_error = error;
    
    double total_output = p_error + i_error + d_error;
    
    if (total_output > 0) {
        out_accel = min(total_output, 1.0);
        out_brake = 0.0;
    } else {
        out_accel = 0.0;
        out_brake = min(-total_output, 1.0);
    }
}

// ========================================
// 제어 명령 발행
// ========================================
void pubCmd(const ControlData& data)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    cmd.accel = data.accel;
    cmd.brake = data.brake;
    cmd.steering = data.steering;
    cmd_pub.publish(cmd);
}