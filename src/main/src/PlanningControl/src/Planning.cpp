#include "global.hpp"
#include "Planning.hpp"

// ========================================
// PlanningProcess
// ========================================

void PlanningProcess() {
    DecisionMaking
    SelectMission(lattice_ctrl, current_mission);
    closeWaypointsIdx(ego, ctrl.close_idx);
    getTargetwaypoint(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
    getMaxCurvature(ctrl.close_idx, ctrl.lookahead_idx, ego.max_curvature);
    getTargetSpeed(ego.max_curvature, ctrl.target_vel);
}

//--------------- 함수정의 ---------------------------------------------------------


// ========================================
// 미션 결정 (Costmap 기반)
// ========================================
void DecisionMaking() {
    
    if (current_mission == Mission::END) {
        return;
    }
    
    Mission determined_mission;
  
    // GPS Jamming 체크
    if (gps_state.is_jamming) {
        determined_mission = Mission::JAMMING;
        
        // Stopline 0.5초 이상 감지 시 종료
        static uint32_t stopline_detect_time = 0;
        
        if (lane.stopline_detected) {  // 카메라에서 stopline 감지 (구조체 필드명은 실제에 맞게 수정)
            if (stopline_detect_time == 0) {
                stopline_detect_time = GetMillisecond();
            }
            else if (GetMillisecond() - stopline_detect_time >= 500) {
                determined_mission = Mission::END;
            }
        }
        else {
            stopline_detect_time = 0;  // stopline 미감지 시 타이머 리셋
        }
    }
    // Costmap 기반 장애물 감지
    else {
        bool has_obstacle = CheckObstacleInCostmap();
        
        if (has_obstacle) {
            determined_mission = Mission::LATTICE;
            ROS_INFO_THROTTLE(2.0, "[DecisionMaking] Obstacle detected → LATTICE mode");
        }
        else {
            determined_mission = Mission::NORMAL;
            ROS_INFO_THROTTLE(2.0, "[DecisionMaking] Path clear → NORMAL mode");
        }
    }
    
    // Debouncing 메커니즘 (100ms)
    static uint32_t mode_change_request_time = 0;
    static Mission candidate_mission = Mission::NORMAL;
    static bool is_initialized = false;
    
    // 초기화
    if (!is_initialized) {
        candidate_mission = current_mission;
        is_initialized = true;
    }
    
    const uint32_t DEBOUNCE_TIME_MS = 100;
    
    // Case 1: 결정된 미션 ≠ 현재 미션
    if (determined_mission != current_mission) {
        
        // Case 1-1: 새로운 후보 등장
        if (determined_mission != candidate_mission) {
            candidate_mission = determined_mission;
            mode_change_request_time = GetMillisecond();
            ROS_INFO("[DecisionMaking] New candidate mission: %s", 
                     MissionToString(candidate_mission).c_str());
        }
        // Case 1-2: 기존 후보 유지 중
        else {
            // 100ms 이상 유지되었는지 확인
            if (mode_change_request_time > 0 && 
                (GetMillisecond() - mode_change_request_time > DEBOUNCE_TIME_MS)) {
                
                // 미션 변경 확정!
                ROS_WARN("[DecisionMaking] Mission changed: %s → %s",
                        MissionToString(current_mission).c_str(),
                        MissionToString(candidate_mission).c_str());
                
                current_mission = candidate_mission;
                mode_change_request_time = 0;
            }
        }
    }
    // Case 2: 결정된 미션 = 현재 미션 (안정 상태)
    else {
        candidate_mission = current_mission;
        mode_change_request_time = 0;
    }
}

// ========================================
// Costmap에서 장애물 감지
// ========================================
bool CheckObstacleInCostmap() {
    
    // Costmap 유효성 체크
    if (!checkCostmapAvailable()) {
        ROS_WARN_THROTTLE(2.0, "[CheckObstacle] Costmap unavailable");
        return false;
    }
    
    const double CHECK_DISTANCE = 30.0;      // 전방 30m 확인
    const double PATH_WIDTH = 2.5;           // 차량 폭 + 여유 (2.5m)
    const int LETHAL_THRESHOLD = 70;         // 장애물로 판단할 cost 임계값
    const int MIN_OBSTACLE_POINTS = 10;      // 최소 장애물 포인트 개수
    
    // 현재 위치에서 전방 체크 포인트 생성
    int num_check_points = (int)(CHECK_DISTANCE / 0.5);  // 0.5m 간격
    int obstacle_count = 0;
    
    for (int i = 0; i < num_check_points; i++) {
        // 차량 기준 전방 거리
        double forward_dist = 0.5 * i;
        
        // 차량 좌표계에서 전방 점
        Point2D baselink_pt = {forward_dist, 0.0};
        
        // Global 좌표계로 변환
        Point2D global_pt;
        BaseLinkToMap(baselink_pt, global_pt);
        
        // Costmap grid 좌표로 변환
        int gx, gy;
        if (!MapToCostmap(global_pt, gx, gy)) {
            continue;  // Costmap 범위 밖
        }
        
        // 좌우 확장 체크 (차량 폭 고려)
        int lateral_cells = (int)(PATH_WIDTH / (2.0 * costmap_info.resolution));
        
        for (int dy = -lateral_cells; dy <= lateral_cells; dy++) {
            int check_x = gx;
            int check_y = gy + dy;
            
            // 범위 체크
            if (check_x < 0 || check_x >= costmap_info.width ||
                check_y < 0 || check_y >= costmap_info.height) {
                continue;
            }
            
            // Cost 확인
            int cost = getCostmapCostFromGrid(check_x, check_y);
            
            if (cost >= LETHAL_THRESHOLD) {
                obstacle_count++;
                
                // 일정 개수 이상 감지 시 장애물 있음으로 판단
                if (obstacle_count >= MIN_OBSTACLE_POINTS) {
                    ROS_INFO_THROTTLE(1.0, "[CheckObstacle] Obstacle detected at %.1fm (count: %d)",
                             forward_dist, obstacle_count);
                    return true;
                }
            }
        }
    }
    
    return false;
}

// ========================================
// GetMillisecond (시간 함수)
// ========================================
uint32_t GetMillisecond() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// ========================================
// Mission → String 변환 (디버깅용)
// ========================================
std::string MissionToString(Mission mission) {
    switch (mission) {
        case Mission::NORMAL:   return "NORMAL";
        case Mission::LATTICE:  return "LATTICE";
        case Mission::JAMMING:  return "JAMMING";
        case Mission::END:      return "END";
        default:                return "UNKNOWN";
    }
}

// ========================================
// 미션 고르기
// ========================================
void SelectMission(const LatticeControl& lattice_ctrl, const Mission& current_mission) {

    switch (current_mission) {
        case Mission::NORMAL: 
            break;
            
        case Mission::LATTICE:
            LatticePlanningProcess();
            break;
            
        case Mission::JAMMING:
            JammingProcess();
            return;
            
        case Mission::END:
            return;
    }
}


// ========================================
// 가까운 인덱스잡기
// ========================================
void closeWaypointsIdx(const VehicleState& ego, int& out_idx) { 

    // 미션별 waypoints 선택
    const std::vector<Waypoint>* target_waypoints = nullptr;
    
    if (current_mission == Mission::NORMAL) {
        target_waypoints = &waypoints;
    } 
    else if (current_mission == Mission::LATTICE) {
        target_waypoints = &lattice_waypoints;
    } 
    else {
        // JAMMING, END 등 waypoints 사용 안 하는 미션
        out_idx = 0;
        return;
    }
    
    // 빈 경로 체크
    if (target_waypoints->empty()) {
        ROS_WARN("[closeWaypointsIdx] No waypoints available!");
        out_idx = 0;
        return;
    }
    
    // Static 변수를 미션별로 분리
    static int last_close_idx_normal = 0;
    static int last_close_idx_lattice = 0;
    
    int& last_close_idx = (current_mission == Mission::NORMAL) 
                          ? last_close_idx_normal 
                          : last_close_idx_lattice;
    
    // 범위 안전 체크
    if (last_close_idx >= target_waypoints->size()) {
        last_close_idx = 0;
    }
    
    // 탐색
    double best_close_dist = 1e10;
    int close_idx = last_close_idx;
    int start = std::max(0, last_close_idx - 10);
    int end = std::min((int)target_waypoints->size() - 1, last_close_idx + 30);

    for (int i = start; i <= end; ++i) {
        double dx = (*target_waypoints)[i].x - ego.x;
        double dy = (*target_waypoints)[i].y - ego.y;
        double dist = sqrt(dx*dx + dy*dy);
        
        if (dist < best_close_dist) {
            best_close_dist = dist;
            close_idx = i;
        }
    }
    
    last_close_idx = close_idx;
    out_idx = close_idx;
    
    ROS_INFO_THROTTLE(1.0, "[%s] close_idx: %d / %zu", 
                     (current_mission == Mission::NORMAL) ? "NORMAL" : "LATTICE",
                     close_idx, target_waypoints->size());
}
// ========================================
// 타겟 인덱스 잡기
// ========================================
void getTargetwaypoint(const VehicleState& ego, int close_idx, 
                       int& out_target_idx, double& ld) {
    
    // 미션별 waypoints 선택
    const std::vector<Waypoint>* target_waypoints = nullptr;
    
    if (current_mission == Mission::NORMAL) {
        target_waypoints = &waypoints;
    } 
    else if (current_mission == Mission::LATTICE) {
        target_waypoints = &lattice_waypoints;
    } 
    else {
        out_target_idx = close_idx;
        ld = 5.0;
        return;
    }
    
    if (target_waypoints->empty()) {
        out_target_idx = close_idx;
        ld = 5.0;
        return;
    }
    
    ld = 5.0 + ld_gain * ego.vel;
    int target_idx = close_idx;
    
    for (int i = close_idx; i < target_waypoints->size(); ++i) {
        double dx = (*target_waypoints)[i].x - ego.x;
        double dy = (*target_waypoints)[i].y - ego.y;
        double dist = sqrt(dx*dx + dy*dy);
        
        if (dist >= ld) {
            target_idx = i;
            break;
        }
    }
    
    out_target_idx = target_idx;
}
// ========================================
// 최대 곡률 계산
// ========================================
void getMaxCurvature(int close_idx, int lookahead_idx, double& max_curvature) {
    
    const std::vector<Waypoint>* target_waypoints = nullptr;
    
    if (current_mission == Mission::NORMAL) {
        target_waypoints = &waypoints;
    } 
    else if (current_mission == Mission::LATTICE) {
        target_waypoints = &lattice_waypoints;
    } 
    else {
        max_curvature = 0.0;
        return;
    }
    
    if (target_waypoints->empty()) {
        max_curvature = 0.0;
        return;
    }
    
    double max_kappa = 0.0;
    int end_idx = std::min((int)target_waypoints->size(), close_idx + lookahead_idx);
    
    for (int i = close_idx; i < end_idx; ++i) {
        double now_kappa = (*target_waypoints)[i].curvature;
        if (now_kappa > max_kappa) {
            max_kappa = now_kappa;
        }
    }
    
    max_curvature = max_kappa;
}
// ========================================
// 타겟 속도
// ========================================
void getTargetSpeed(double max_curvature, double& out_target_vel){

    if (current_mission == Mission::NORMAL) {
        if(max_curvature > curve_standard){
            out_target_vel = curve_vel;
        }
        else {out_target_vel = target_vel;}
    }

    if (current_mission == Mission::LATTICE) {
        out_target_vel = obstacle_vel;
    }
}