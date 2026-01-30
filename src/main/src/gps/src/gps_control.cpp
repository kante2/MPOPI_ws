#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/CtrlCmd.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <vector>
#include <ros/package.h>
#include <utility>
#include <iostream>
#include <algorithm>

// ========================================
// JammingProcess
// ========================================
void JammingProcess()
{       
    filterOffset(lane, offset,jamming_params);
    computePurePursuitSteering(lane, offset, jamming_params);
    computeOffsetPD(offset,jamming_params);
    computeLastSterring(offset,jamming_params);
    printGPSJammingStatistics();
    computePID(ego, ctrl, jamming_params);
    publishCtrlCmd(ctrl);
}

//--------------------------함수 정의----------------------------------------------------------

void filterOffset(const LaneData& lane, Jamming_offset& offset,JammingParams& jamming_params)
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
void computeLastSterring(const Jamming_offset& offset,JammingParams& jamming_params)
{
    jamming_params.gps_steering = offset.pp_steering-offset.offset_steering;
}

// ====================== STATISTICS ======================
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
    cout << "vector_x  : " << lane.vector_x  << endl;
    cout << "vector_y  : " << lane.vector_y  << endl;
    cout << "angle_deg : " << lane.angle << endl;
    cout << "target_x  : " << lane.target_x  << endl;
    cout << "target_y  : " << lane.target_y  << endl;
}

void computePID(const VehicleState& ego, ControlData& ctrl, const JammingParams& jamming_params)
{
    static double prev_error = 0.0;
    static double integral_error = 0.0;

    double error = target_vel - ego.vel;

    integral_error += error * 0.02;
    
    if (integral_error > 10.0) integral_error = 10.0;
    if (integral_error < -10.0) integral_error = -10.0;
    
    double p_error = Kp * error;
    double i_error = Ki * integral_error;
    double d_error = Kd * ((error - prev_error) / jamming_params.dt);
    prev_error = error;
    
    double total_output = p_error + i_error + d_error;
    
    if (total_output > 0) {
       ctrl.accel = std::min(total_output, 1.0);
        ctrl.brake = 0.0;
    } else {
        ctrl.accel = 0.0;
        ctrl.brake = std::min(-total_output, 1.0);
    }
}

void publishCtrlCmd(const ControlData& data)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    cmd.accel = data.accel;
    cmd.brake = data.brake;
    cmd.steering = jamming_params.gps_steering;
    cmd_pub.publish(cmd);
}

// ====================== TIMER CALLBACK ======================
void JammingTimerCallback(const ros::TimerEvent&) 
{  
    if (!gps_first_received) return;

    ros::Time current_time = ros::Time::now();

    if (gps_jamming_perception == true) {
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

            gps_state.current_jamming_duration = 0.0;

            ROS_WARN("[GPS JAMMING] ===== GPS RECOVERED =====");
            ROS_WARN("[GPS JAMMING] Duration: %.3f sec", duration);
        }
    }
}

void controlTimerCallback(const ros::TimerEvent&)
{
    if (!gps_state.is_jamming)
        return;

    JammingProcess();
}

// ====================== MAIN ======================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_control_node");
    ros::NodeHandle nh;
    
    ROS_INFO("GPS Control Node Started");
    
    ros::spin();
    return 0;
}