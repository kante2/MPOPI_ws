#include "Global.hpp"
#include "Control.hpp"
#include "Planning.hpp"
#include <morai_msgs/CtrlCmd.h>
#include <cmath>
#include <algorithm>

using namespace std;

// ========================================
// Control ControlProcess
// ========================================
void ControlProcess() {
    decideSteering(ego, ctrl, jamming_params);
    computePID(ego.vel, ctrl.target_vel, ctrl.accel, ctrl.brake);
    pubCmd(ctrl);

}

void decideSteering(const VehicleState& ego, ControlData& ctrl, const JammingParams& jamming_params){
 if (gps_jamming_perception) {
        // GPS 재밍 시 → 재밍 조향 사용
        ctrl.steering = jamming_params.gps_steering;
        ctrl.target_vel = jamming_params.gps_target_vel;
    } else {
        // 정상 주행 → 기존 Stanley
        getsteering(ego, ctrl);
    }
}

// ========================================
// 조향각 계산 (Stanley)
// ========================================
void getsteering(const VehicleState& ego, ControlData& ctrl)
{
    double path_e = lateralPathError(ctrl.lookahead_idx, ego.x, ego.y);
    double heading_e = headingError(ego.yaw, ctrl.lookahead_idx);
    double v = std::max(1.0, ego.vel);
    
    double steering_raw = heading_e + atan(k_gain * path_e / v);
    
    const double MAX_STEERING = 45.0 * M_PI / 180.0; // 30 -> 45 TUNE
    ctrl.steering = std::max(-MAX_STEERING, std::min(MAX_STEERING, steering_raw));

}

// ========================================
// PID 속도 제어
// ========================================
// void computePID(const VehicleState& ego, ControlData& ctrl)
void computePID(double vel, double target_vel, double& out_accel, double& out_brake)
{
    static double prev_error = 0.0;
    static double integral_error = 0.0;

    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    double error = target_vel - vel;
    integral_error += error * dt;

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

// ========================================
// Lateral Path Error
// ========================================
double lateralPathError(int target_idx, double x, double y) {
    x = 0;
    y = 0; //** */

    if (lattice_ctrl.best_path.points.empty() || target_idx >= lattice_ctrl.best_path.points.size() - 1) {
        return 0.0;
    }
    
    int i = target_idx;
    double a = lattice_ctrl.best_path.points[i].x;
    double b = lattice_ctrl.best_path.points[i].y;
    double B = lattice_ctrl.best_path.points[i+1].x - lattice_ctrl.best_path.points[i].x;
    double A = lattice_ctrl.best_path.points[i+1].y - lattice_ctrl.best_path.points[i].y;
    double C = -a*A + b*B;
    
    if (A == 0 && B == 0) return 0.0;
    
    return (x*A - y*B + C) / sqrt(A*A + B*B);
}

// ========================================
// Heading Error
// ========================================
double headingError(double yaw,int target_idx){
    yaw =0;
    int i = target_idx;
    if(i< (int)lattice_ctrl.best_path.points.size()-1){
    double dx = lattice_ctrl.best_path.points[i+1].x - lattice_ctrl.best_path.points[i].x;
    double dy = lattice_ctrl.best_path.points[i+1].y - lattice_ctrl.best_path.points[i].y;
    double path_heading = atan2(dy,dx);
    double diff = path_heading - yaw;
    return atan2(sin(diff),cos(diff));
    }
    else {return 0.0;}
}