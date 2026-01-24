#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <cmath>
#include <vector>
#include <ros/package.h>
#include <utility>
#include <iostream>
#include "hybrid_function.hpp"
#include <std_msgs/Bool.h>
#include <roscpp_morai/LaneInfo.h>
#include <morai_msgs/CtrlCmd.h>
#include <cmath>

// ========================================
// JammingProcess
// ========================================
void JammingProcess()
{   
    if (!isLaneVectorValid(lane, norm)) return;
    
    computePurePursuitTarget(lane, norm, x_t, y_t);

    double filtered_offset = filterOffset(lane);
    y_t -= filtered_offset;

    double pp_steering = computePurePursuitSteering(x_t, y_t);
    double offset_steering = computeOffsetPD(filtered_offset);

    gps_steering = pp_steering - offset_steering;

    printGPSGammingStatistics();
    publishCtrlCmd();
}


bool isLaneVectorValid(const LaneData& lane, double& norm)
{
    norm = std::hypot(lane.vector_x, lane.vector_y);
    return norm >= 1e-3;
}

void computePurePursuitTarget(const LaneData& lane, double norm, double& x_t, double& y_t)
{
    x_t = Ld * ( lane.vector_y / norm);
    y_t = Ld * (-lane.vector_x / norm);
}

double filterOffset(const LaneData& lane)
{
    static double filtered_offset = 0.0;
    filtered_offset = gps_alpha * lane.offset + (1.0 - gps_alpha) * filtered_offset;

    return filtered_offset;
}

double computePurePursuitSteering(double x_t, double y_t)
{
    double alpha_pp = atan2(y_t, x_t);
    return atan2(2.0 * wheel_base * sin(alpha_pp), Ld);
}

double computeOffsetPD(double filtered_offset)
{
    static double last_offset = 0.0;
    double d_offset = (filtered_offset - last_offset) / dt;
    double offset_steering = gps_k_p * filtered_offset + gps_k_d * d_offset;

    last_offset = filtered_offset;
    return offset_steering;
}


// ====================== STATISTICS ======================
void printGPSGammingStatistics() {
    ROS_INFO("\n\n========== GPS GAMMING STAT ==========");
    ROS_INFO("Count           : %d", gps_state.gamming_count);
    ROS_INFO("Total duration  : %.3f sec", gps_state.total_gamming_duration);
    if (gps_state.is_gamming) {
        ROS_INFO("Current duration: %.3f sec (ONGOING)",
                 gps_state.current_gamming_duration);
    } else {
        ROS_INFO("Current state   : NORMAL");
    }
    cout << "===== LanePath Received =====" << endl;
    cout << "offset    : " << lane.offset    << endl;
    cout << "vector_x  : " << lane.vector_x  << endl;
    cout << "vector_y  : " << lane.vector_y  << endl;
    cout << "angle_deg : " << lane.angle << endl;
    cout << "target_x  : " << lane.target_x  << endl;
    cout << "target_y  : " << lane.target_y  << endl;
}

// ====================== TIMER CALLBACK ======================
void jammingTimerCallback(const ros::TimerEvent&) {
    
    ros::Time current_time = ros::Time::now();
    double gps_dt = (current_time - gps_state.last_valid_gps_time).toSec();
    //current_time - last_valid_gps_time : ros::Duration
    //.toSec() : ros::Duration → double (초)

    if (gps_dt > gps_state.gps_timeout_threshold) {
        if (!gps_state.is_jamming) { // is_jamming 이 false일때
            gps_state.is_jamming = true;
            gps_state.jamming_start_time = gps_state.last_valid_gps_time;
            //last_valid_gps_time : GPS가 마지막으로 정상 수신된 시각
            gps_state.jamming_count++;
            gps_state.current_jamming_duration = 0.0;

            ROS_WARN("[GPS JAMMING] ===== GPS SIGNAL LOST =====");
            ROS_WARN("[GPS JAMMING] Count: %d", gps_state.jamming_count);
        }
        else {//이미 is_jamming = true 일때
            gps_state.current_jamming_duration =
                (current_time - gps_state.jamming_start_time).toSec();
                //jamming 시작 시점부터 지금까지 경과한 시간 계산
        }
    }
    // ---- GPS RECOVER ----
    else {
        if (gps_state.is_jamming) {
            gps_state.is_jamming = false; //false =정상상태로 전환

            double duration =
                (current_time - gps_state.jamming_start_time).toSec();

            gps_state.total_jamming_duration += duration;

            gps_state.current_gamming_duration = 0.0;

            ROS_WARN("[GPS GAMMING] ===== GPS RECOVERED =====");
            ROS_WARN("[GPS GAMMING] Duration: %.3f sec", duration);
        }
    }
}

void publishCtrlCmd()
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.steering = gps_steering;
    cmd.velocity = 60.0;   // 고정 속도
    cmd.accel = 0.0;
    cmd.brake = 0.0;

    cmd_pub.publish(cmd);
}


void controlTimerCallback(const ros::TimerEvent&)
{
    if (!gps_state.is_gamming) return;
    JammingProcess();
}
