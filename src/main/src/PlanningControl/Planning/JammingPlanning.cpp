#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <cmath>
#include <vector>
#include <ros/package.h>
#include <utility>
#include <iostream>
#include "Global.hpp" // ** 
#include <std_msgs/Bool.h>
#include "Planning.hpp"
#include <morai_msgs/CtrlCmd.h>
#include <cmath>
#include <algorithm>
using namespace std;
// ========================================
// JammingPlanningProcess
// ========================================
void JammingPlanningProcess()
{
    double accel = 0.0;
    double brake = 0.0;
    filterOffset(lane, offset,jamming_params);
    computePurePursuitSteering(lane, offset, jamming_params);
    computeOffsetPD(offset,jamming_params);
    computePID(accel, brake, ego, jamming_params);
    publishCtrlCmd(accel, brake, ego);
}

//--------------------------함수 정의----------------------------------------------------------

void filterOffset(const LaneData& lane, Jamming_offset& offset,const JammingParams& jamming_params)
{   
    offset.filtered_offset = jamming_params.gps_alpha * lane.offset + (1.0 - jamming_params.gps_alpha) * offset.filtered_offset;
}

void computePurePursuitSteering(const LaneData& lane, Jamming_offset& offset,const JammingParams& jamming_params)
{  
    double alpha_pp = -lane.angle * M_PI / 180.0;
    offset.pp_steering =  atan2(2.0 * jamming_params.wheel_base * sin(alpha_pp), jamming_params.Ld);
}

void computeOffsetPD(Jamming_offset& offset,const JammingParams& jamming_params)
{
    double d_offset = (offset.filtered_offset - offset.last_offset) / jamming_params.dt;
    offset.offset_steering = jamming_params.pid_Kp * offset.filtered_offset + jamming_params.pid_Kd * d_offset;
    offset.last_offset = offset.filtered_offset;
}
void computeLastSterring(const Jamming_offset& offset,JammingParams& jamming_params)
{
    jamming_params.gps_steering = offset.pp_steering-offset.offset_steering;
}


void computePID(double& accel, double& brake, const VehicleState& ego,const JammingParams& jamming_params)
{   
    static double prev_error = 0.0;
    static double integral_error = 0.0;

    double error = jamming_params.gps_target_vel - ego.vel;
    integral_error += error * 0.02;
    
    if (integral_error > 10.0) integral_error = 10.0;
    if (integral_error < -10.0) integral_error = -10.0;
    
    double p_error = jamming_params.pid_Kp * error;
    double i_error = jamming_params.pid_Ki * integral_error;
    double d_error = jamming_params.pid_Kd * ((error - prev_error) / 0.02);
    prev_error = error;
    
    double total_output = p_error + i_error + d_error;
    morai_msgs::CtrlCmd cmd;
    
    if (total_output > 0) {
        accel = min(total_output, 1.0);
        brake = 0.0;

    } else {
        accel = 0.0;
        brake = min(-total_output, 1.0);
    }
}

void publishCtrlCmd(double& accel, double& brake,const VehicleState& ego)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    cmd.accel = accel;
    cmd.brake = brake;
    cmd.steering = jamming_params.gps_steering;
    cmd_pub.publish(cmd);
}