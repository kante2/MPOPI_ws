// #include "global.hpp"
// #include "Planning.hpp"

// // ========================================
// // PlanningProcess
// // ========================================

// void PlanningProcess() {
    
//     DecisionMaking(gps_state, current_mission);
//     SelectMission();
//     if (current_mission == Mission::JAMMING || current_mission == Mission::END) return;
//     closeWaypointsIdx(ego, ctrl.close_idx);
//     getTargetwaypoint(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
//     getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx, ego.max_curvature);
//     getTargetSpeed(ego.max_curvature, ctrl.target_vel);
// }

// //--------------- 함수정의 --------------------------------------------------

// // ========================================
// // 미션 결정
// // ========================================
// void DecisionMaking(const GPSJammingState& gps_state, Mission& current_mission) {

//     if (current_mission == Mission::END) {
//         return;
//     }
    
//     Mission determined_mission = gps_state.is_jamming ? Mission::JAMMING : Mission::LATTICE;
    
//     // Debouncing
//     static uint32_t timer = 0;
//     static Mission candidate = Mission::LATTICE;
    
//     if (determined_mission != current_mission) {
//         if (determined_mission != candidate) {
//             candidate = determined_mission;
//             timer = GetMillisecond();
//         }
//         else if (GetMillisecond() - timer > 100) {
//             current_mission = candidate;
//             ROS_WARN("[Mission] %s", MissionToString(current_mission).c_str());
//         }
//     }
//     else {
//         candidate = current_mission;
//         timer = 0;
//     }
// }


// // ========================================
// // GetMillisecond (시간 함수)
// // ========================================
// uint32_t GetMillisecond() {
//     struct timespec ts;
//     clock_gettime(CLOCK_MONOTONIC, &ts);
//     return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
// }

// // ========================================
// // Mission → String 변환 (디버깅용)
// // ========================================
// std::string MissionToString(Mission mission) {
//     switch (mission) {
//         case Mission::LATTICE:  return "LATTICE";
//         case Mission::JAMMING:  return "JAMMING";
//         case Mission::END:      return "END";
//         default:                return "UNKNOWN";
//     }
// }

// // ========================================
// // 미션 고르기
// // ========================================
// void SelectMission() {
//     switch (current_mission) {
    
//         case Mission::LATTICE:
//             LatticePlanningProcess();
//             break;
            
//         case Mission::JAMMING:
//             JammingProcess();
//             return;
            
//         case Mission::END:
//             return;
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
//         double dx = waypoints[i].x - ego.x;
//         double dy = waypoints[i].y - ego.y;
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
//         double dx = waypoints[i].x-ego.x;
//         double dy = waypoints[i].y-ego.y;
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