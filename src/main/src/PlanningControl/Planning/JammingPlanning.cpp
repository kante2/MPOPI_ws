#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <cmath>
#include <vector>
#include <ros/package.h>
#include <utility>
#include <iostream>
#include "Global.hpp" 
#include <std_msgs/Bool.h>
#include "Planning.hpp"
#include <morai_msgs/CtrlCmd.h>
#include <cmath>
#include <algorithm>
using namespace std;

void JammingPlanningProcess()
{       
    filterOffset(lane, offset,jamming_params);
    computePurePursuitSteering(lane, offset, jamming_params);
    computeOffsetPD(offset,jamming_params);
    computeLastSteering(offset,jamming_params);
    printGPSJammingStatistics();
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
    offset.offset_steering = jamming_params.gps_k_p * offset.filtered_offset + jamming_params.gps_k_d * d_offset;
    offset.last_offset = offset.filtered_offset;
}
void computeLastSteering(const Jamming_offset& offset,JammingParams& jamming_params)
{
    jamming_params.gps_steering = offset.pp_steering-offset.offset_steering;
}

void printGPSJammingStatistics() 
{
    ROS_INFO("\n\n========== GPS JAMMING STAT ==========");
    ROS_INFO("Count           : %d", gps_state.jamming_count);
    ROS_INFO("Total duration  : %.3f sec", gps_state.total_jamming_duration);
    if (gps_state.is_jamming) {
        ROS_INFO("Current duration: %.3f sec (ONGOING)",
                 gps_state.current_jamming_duration);
    } else {
        ROS_INFO("Current state   : NORMAL");
    }
    cout << "===== LanePath Received =====" << endl;
    cout << "offset    : " << lane.offset    << endl;
    cout << "angle_deg : " << lane.angle << endl;
}