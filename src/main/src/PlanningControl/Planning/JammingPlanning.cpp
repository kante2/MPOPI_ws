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
    offset.offset_steering = jamming_params.gps_k_p * offset.filtered_offset;
}
void computeLastSteering(const Jamming_offset& offset,JammingParams& jamming_params)
{
    double w_pp = 0.7;
    double w_offset = 0.3;

    jamming_params.gps_steering =
    w_pp * offset.pp_steering -
    w_offset * offset.offset_steering;

}

void printGPSJammingStatistics() 
{
    static bool prev_jamming = false;

    // 재밍 시작 순간에만 count 증가
    if (!prev_jamming && gps_jamming_perception) {
        gps_state.jamming_count++;
    }
    prev_jamming = gps_jamming_perception;

    ROS_WARN("\n\n========== GPS JAMMING STAT ==========");
    ROS_WARN("\nCount           : %d", gps_state.jamming_count);
    ROS_WARN("\n===== LanePath Received =====");
    ROS_WARN("\noffset    : %.6f", lane.offset);
    ROS_WARN("\nangle_deg : %.6f", lane.angle);
}
