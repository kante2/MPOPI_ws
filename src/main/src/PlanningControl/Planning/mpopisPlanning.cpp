#include "Global.hpp"
#include "Planning.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <random>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <morai_msgs/CtrlCmd.h>
using namespace std;

// ========================================
// Algorithm 1: mpopisPlanningProcess
// 0.1초마다 (10Hz) 실행되는 메인 mpopis 경로 계획 함수
// 
// MPOPI (Model Predictive Optimized Path Integral) 알고리즘:
// - 상태 추정 (x0)
// - L번 반복: 샘플링 → 동역학 전파 → 비용 평가 → 가중치 계산 → 분포 업데이트
// - 최적 제어 첫 단계 실행
// - 시간 시프트 (Receding Horizon)
//
// [좌표계 정책]
// trajectories는 rollout() 이후 끝까지 ENU(map frame) 고정
// costmap 조회 시에만 ENU → base_link 로 포인트 1개씩 변환
// ========================================
void mpopisPlanningProcess() {

    // Guard: Costmap이 준비될 때까지 대기
    // if (!checkCostmapAvailable()) {
    //     ROS_WARN_THROTTLE(1.0, "[MPOPI] Waiting for costmap...");
    //     return;
    // }

    // 초기화
    initializeMPOPIState();

    // ego 기준 closest waypoint 인덱스 계산 (매 루프 한 번만)
    findClosestWaypoint(ego, mpopi_ctrl.close_idx); // ego -> close_idx

    // Algorithm 1 라인 4: for ℓ ← 1 to L do
    // L=5 반복: 이번 타임스텝에서 최적화를 5번 수행
    // 각 반복마다 분포가 개선되어 점진적으로 더 좋은 경로로 수렴
    for (int iteration = 0; iteration < 5; iteration++) {
        mpopi_state.current_iteration = iteration;
        
        // Algorithm 1 라인 6: Sample Ek from N(0, Σ')
        sampleKcontrols(mpopi_state);
        
        // Algorithm 1 라인 8: x_t ← F(x_{t-1}, g(u'_{t-1} + ε_k_{t-1}))
        // rollout 결과는 ENU 좌표로 저장됨
        rollout(ego, mpopi_state.U_samples, mpopi_state.trajectories);

        // [좌표계] trajectoriesToBaseLink() 호출 제거
        // trajectories는 ENU 그대로 유지
        // obstacle cost 계산 시에만 내부에서 ENU → base_link 변환

        // Algorithm 1 라인 9: s_k ← c(X) + φ(X) + λ(1-α)U'Σ⁻¹(ε_k + U' - U)
        computeCostMPOPI(mpopi_state.trajectories, mpopi_ctrl.close_idx, ego);
        
        // Algorithm 1 라인 15: w_k ← 1/η × exp(...)
        computeWeight(mpopi_state.costs);
        
        // Algorithm 1 라인 10-11: if ℓ < L then U', Σ' ← PerformAIS(...)
        if (iteration < 4) {
            updateDistribution(mpopi_state.weights);
        }
    }
    
    // Algorithm 1 라인 17-20: SendToActuators(u₀) + Time Shift
    shiftSequence();
    
    // MPOPI 경로 시각화
    visualizeMPOPITrajectories();
}


// ========================================
// 라인 5-6: 샘플링 (sampleKcontrols)
// K개의 제어 입력 샘플 생성
// ε_k ~ N(0, Σ')
// ========================================
void sampleKcontrols(MPOPIState& mpopi_state) {
    const int K = mpopi_params.K;
    const int N = mpopi_params.N;
    // 참고 : 
    // int K = 500;              // 샘플 수
    // int N = 20;               // 예측 스텝 수
    
    static std::default_random_engine generator(42);  // 난수 생성기 -> 디버깅 시 재현성을 확보
    
    // Algorithm 1 라인 6: for k = 1 to K
    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        // Algorithm 1 라인 7: for t = 1 to T
        for (int timeStep = 0; timeStep < N; timeStep++) {
            // updateDistribution()이 관리하는 표준편차 직접 사용
            double std_v_current = mpopi_state.std_v[timeStep];
            double std_delta_current = mpopi_state.std_delta[timeStep];
            
            // 최소 분산 보장
            // sigma_min 을 통해, 최적화가 너무 진행되어 표준편차가 0 에 가까워지면 새로운 상황 대응 유연성이 떨어짐
            std_v_current = std::max(std_v_current, mpopi_params.sigma_v_min);
            std_delta_current = std::max(std_delta_current, mpopi_params.sigma_delta_min);
            
            // 가우시안 분포에서 샘플링
            std::normal_distribution<double> normalDistV(0.0, std_v_current);
            std::normal_distribution<double> normalDistDelta(0.0, std_delta_current);
            
            // 노이즈 샘플: ε_k ~ N(0, Σ')
            double noise_v = normalDistV(generator);
            double noise_delta = normalDistDelta(generator);
            
            // 실제 제어: u_t = U'_t + ε_t
            double sampled_v = mpopi_state.U_nominal[timeStep].v + noise_v;
            double sampled_delta = mpopi_state.U_nominal[timeStep].delta + noise_delta;
            
            // 제어 입력 범위 제한 (Clipping)
            sampled_v = std::max(mpopi_params.v_min, 
                                std::min(mpopi_params.v_max, sampled_v));
            sampled_delta = std::max(mpopi_params.delta_min, 
                                    std::min(mpopi_params.delta_max, sampled_delta));
            
            // 샘플 저장: U_samples[k][t]
            mpopi_state.U_samples[sampleIdx][timeStep].v = sampled_v;
            mpopi_state.U_samples[sampleIdx][timeStep].delta = sampled_delta;
        }
    }
}


// ========================================
// 라인 7-9: Rollout
// x_t = F(x_{t-1}, g(u'_t + ε_k_t))
//
// [좌표계] 출력 trajectories는 ENU(map frame) 기준
// ========================================
void rollout(const VehicleState& ego,
             const std::vector<std::vector<ControlInput>>& U_samples,
             std::vector<MPOPITrajectory>& trajectories) {
    const int K = mpopi_params.K;
    const int N = mpopi_params.N;

    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {

        trajectories[sampleIdx].states.clear();
        trajectories[sampleIdx].states.resize(N + 1);  // t=0~t=N
        
        // 현재 상태 x_0 설정 (ENU 좌표)
        trajectories[sampleIdx].states[0] = ego;

        for (int timeStep = 0; timeStep < N; timeStep++) {
            VehicleState& prev_state = trajectories[sampleIdx].states[timeStep];
            VehicleState& next_state = trajectories[sampleIdx].states[timeStep + 1];
            
            const ControlInput& control = U_samples[sampleIdx][timeStep];
            
            // Bicycle Model (동역학 함수 F)
            // 조향각 포화 (g 함수)
            double steered_angle = std::max(mpopi_params.delta_min,
                                           std::min(mpopi_params.delta_max, control.delta));
            
            double dt = mpopi_params.DT;       // 0.1초
            double wheelbase = mpopi_params.L;  // 2.7m
            
            // 차량 전진 방향(base_link X축) 이동량
            double rel_dx = control.v * dt;
            // 조향으로 인한 횡이동 (base_link Y축)
            double rel_dy = control.v * std::tan(steered_angle) * dt / wheelbase;
            
            // base_link → ENU 좌표 변환 (회전 행렬 적용)
            double cos_yaw = std::cos(prev_state.yaw);
            double sin_yaw = std::sin(prev_state.yaw);

            double enu_dx = rel_dx * cos_yaw - rel_dy * sin_yaw;
            double enu_dy = rel_dx * sin_yaw + rel_dy * cos_yaw;
            
            // ENU 좌표 업데이트
            next_state.x   = prev_state.x + enu_dx;
            next_state.y   = prev_state.y + enu_dy;
            next_state.yaw = prev_state.yaw + (control.v / wheelbase) * std::tan(steered_angle) * dt;
            next_state.vel = control.v;
        }
    }
}


// ========================================
// 라인 9: 비용 함수 계산
// s_k = c(X) + φ(X) + λ(1-α)U'Σ⁻¹(ε_k + U' - U)
//
// [좌표계]
// trajectories는 ENU 기준
// lateral / terminal cost → ENU 직접 사용 (waypoints도 ENU)
// obstacle cost           → ENU → base_link 변환 후 costmap 조회
// ========================================
void computeCostMPOPI(const std::vector<MPOPITrajectory>& trajectories,
                      int ego_closest_wp_idx,
                      const VehicleState& ego) {
    const int K = mpopi_params.K;
    const int N = mpopi_params.N;

    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        double pathCost = 0.0;
        double terminalCost = 0.0;
        double regularizationCost = 0.0;
        
        // <1. 경로 비용 c(X)> =====================================================
        for (int timeStep = 0; timeStep < N; timeStep++)
        {
            // [좌표계] state는 ENU 좌표 (rollout 출력 그대로)
            const VehicleState& state = mpopi_state.trajectories[sampleIdx].states[timeStep];

            // (1) lateral error — waypoints도 ENU이므로 직접 비교
            double lateral_error = computeLateralError(state.x, state.y, ego_closest_wp_idx);
            pathCost += mpopi_params.w_path * (lateral_error * lateral_error);
            
            // (2) obstacle cost — costmap은 base_link 기준이므로, 이 포인트만 변환
            Point2D enu_pt{state.x, state.y};
            Point2D bl_pt;
            mapToBaseLink(enu_pt, ego, bl_pt);  // ENU → base_link
            double obstacle_cost = getMPOPICostmapCost(bl_pt.x, bl_pt.y) / 100.0;
            pathCost += mpopi_params.w_obstacle * obstacle_cost;
            
            // 충돌 즉시 패널티 후 해당 샘플 조기 종료
            if (obstacle_cost >= 0.99) {
                pathCost += 1e6;
                break;
            }
            
            // (3) 속도 추종 cost
            double vel_error = state.vel - target_vel;
            pathCost += mpopi_params.w_velocity * (vel_error * vel_error);
        }
        
        // <2. 최종 상태 비용 φ(X)> ===============================================
        // [좌표계] final_state는 ENU → waypoints와 직접 비교 가능
        {
            const VehicleState& final_state = mpopi_state.trajectories[sampleIdx].states[N];
            
            if (!waypoints.empty()) {
                int close_idx = ego_closest_wp_idx;
                double min_dist = 1e10;
                
                int search_start = std::max(0, ego_closest_wp_idx - 5);
                int search_end   = std::min((int)waypoints.size() - 1, ego_closest_wp_idx + 50);
                
                for (int i = search_start; i <= search_end; i++) {
                    double dx = waypoints[i].x - final_state.x;
                    double dy = waypoints[i].y - final_state.y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    if (dist < min_dist) {
                        min_dist = dist;
                        close_idx = i;
                    }
                }
                
                double dx = final_state.x - waypoints[close_idx].x;
                double dy = final_state.y - waypoints[close_idx].y;
                terminalCost = mpopi_params.w_goal * (dx*dx + dy*dy);
            }
        }
        
        // <3. 제어 정규화항: λ(1-α)U'Σ⁻¹(ε_k + U' - U)> =======================
        double alpha = 0.3;
        {
            for (int timeStep = 0; timeStep < N; timeStep++) {
                double dv = mpopi_state.U_samples[sampleIdx][timeStep].v - 
                           mpopi_state.U_nominal[timeStep].v;
                double ddelta = mpopi_state.U_samples[sampleIdx][timeStep].delta - 
                               mpopi_state.U_nominal[timeStep].delta;
                
                double sv = std::max(mpopi_state.std_v[timeStep], mpopi_params.sigma_v_min);
                double sd = std::max(mpopi_state.std_delta[timeStep], mpopi_params.sigma_delta_min);

                regularizationCost += (dv*dv)/(sv*sv) + (ddelta*ddelta)/(sd*sd);
            }
            regularizationCost *= mpopi_params.temperature * (1.0 - alpha);
        }
        
        // Algorithm 1 라인 9: s_k = c(X) + φ(X) + λ(1-α)...
        mpopi_state.costs[sampleIdx] = pathCost + terminalCost + regularizationCost;
    }
}


// ========================================
// 라인 12-15: 가중치 계산 (Softmax)
// w_k = exp(-1/λ × (s_k - ρ)) / η
// ========================================
void computeWeight(std::vector<double>& mpopi_costs) {
    const int K = mpopi_params.K;
    
    // Algorithm 1 라인 12: ρ = min(S)
    double minCost = *std::min_element(mpopi_costs.begin(), mpopi_costs.end());
    
    // Algorithm 1 라인 13: η = Σ exp(-1/λ × (s_k - ρ))
    double normalizationConstant = 0.0;
    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        double exponent = -(mpopi_costs[sampleIdx] - minCost) / mpopi_params.temperature;
        exponent = std::max(-700.0, std::min(700.0, exponent));
        normalizationConstant += std::exp(exponent);
    }
    
    // Algorithm 1 라인 15: w_k = 1/η × exp(...)
    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        double exponent = -(mpopi_costs[sampleIdx] - minCost) / mpopi_params.temperature;
        exponent = std::max(-700.0, std::min(700.0, exponent));
        mpopi_state.weights[sampleIdx] = std::exp(exponent) / normalizationConstant;
    }
}

// ========================================
// 라인 10-11: 분포 업데이트 (AIS)
// U', Σ' ← PerformAIS(...)
// ========================================
void updateDistribution(std::vector<double>& mpopi_weights) {
    const int K = mpopi_params.K;
    const int N = mpopi_params.N;
    
    // 1. 새로운 평균 계산 (가중 평균)
    std::vector<ControlInput> new_U_nominal(N);
    
    for (int timeStep = 0; timeStep < N; timeStep++) {
        double weighted_v = 0.0;
        double weighted_delta = 0.0;
        
        for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
            weighted_v += mpopi_weights[sampleIdx] * 
                         mpopi_state.U_samples[sampleIdx][timeStep].v;
            weighted_delta += mpopi_weights[sampleIdx] * 
                             mpopi_state.U_samples[sampleIdx][timeStep].delta;
        }
        
        new_U_nominal[timeStep].v = weighted_v;
        new_U_nominal[timeStep].delta = weighted_delta;
    }
    
    // 2. 새로운 분산 계산 (적응적 분산 축소)
    std::vector<double> new_std_v(N);
    std::vector<double> new_std_delta(N);
    
    for (int timeStep = 0; timeStep < N; timeStep++) {
        double weighted_var_v = 0.0;
        double weighted_var_delta = 0.0;
        
        for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
            double dv = mpopi_state.U_samples[sampleIdx][timeStep].v - new_U_nominal[timeStep].v;
            double ddelta = mpopi_state.U_samples[sampleIdx][timeStep].delta - new_U_nominal[timeStep].delta;
            
            weighted_var_v += mpopi_weights[sampleIdx] * (dv * dv);
            weighted_var_delta += mpopi_weights[sampleIdx] * (ddelta * ddelta);
        }
        
        new_std_v[timeStep] = std::sqrt(std::max(weighted_var_v,
                                mpopi_params.sigma_v_min * mpopi_params.sigma_v_min));
        new_std_delta[timeStep] = std::sqrt(std::max(weighted_var_delta,
                                    mpopi_params.sigma_delta_min * mpopi_params.sigma_delta_min));
    }
    
    // 3. 상태 업데이트
    mpopi_state.U_nominal = new_U_nominal;
    mpopi_state.mean_v = std::vector<double>(N);
    mpopi_state.mean_delta = std::vector<double>(N);
    for (int t = 0; t < N; t++) {
        mpopi_state.mean_v[t] = new_U_nominal[t].v;
        mpopi_state.mean_delta[t] = new_U_nominal[t].delta;
    }
    mpopi_state.std_v = new_std_v;
    mpopi_state.std_delta = new_std_delta;
}

// ========================================
// 라인 17-20: Receding Horizon (시간 시프트)
// Algorithm 1 라인 18-20
// ========================================
void shiftSequence() {
    const int N = mpopi_params.N;
    
    // Algorithm 1 라인 17: SendToActuators(u₀)
    mpopi_cmd.v = mpopi_state.U_nominal[0].v;
    mpopi_cmd.delta = mpopi_state.U_nominal[0].delta;
    
    // MPOPI 최적값을 차량에 직접 전송
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    
    if (mpopi_cmd.v > mpopi_vehicle_state.vel) {
        cmd.accel = std::min(1.0, (mpopi_cmd.v - mpopi_vehicle_state.vel) * 0.3);
        cmd.brake = 0.0;
    } else {
        cmd.accel = 0.0;
        cmd.brake = std::min(1.0, (mpopi_vehicle_state.vel - mpopi_cmd.v) * 0.3);
    }
    
    cmd.steering = mpopi_cmd.delta;
    cmd_pub.publish(cmd);
    
    // Algorithm 1 라인 18-19: 시퀀스를 한 칸 앞으로 당기기
    for (int timeStep = 0; timeStep < N - 1; timeStep++) {
        mpopi_state.U_nominal[timeStep] = mpopi_state.U_nominal[timeStep + 1];
    }
    
    // Algorithm 1 라인 20: 마지막 제어 입력 재초기화
    mpopi_state.U_nominal[N - 1].v = 30.0;
    mpopi_state.U_nominal[N - 1].delta = 0.0;
    
    // 분산도 같이 shift
    for (int timeStep = 0; timeStep < N - 1; timeStep++) {
        mpopi_state.std_v[timeStep] = mpopi_state.std_v[timeStep + 1];
        mpopi_state.std_delta[timeStep] = mpopi_state.std_delta[timeStep + 1];
    }
    mpopi_state.std_v[N - 1] = mpopi_params.sigma_v;
    mpopi_state.std_delta[N - 1] = mpopi_params.sigma_delta;
}

// ========================================
// 경로 시각화 함수
// [좌표계] trajectories는 ENU → frame_id = "map" 으로 올바르게 publish
// ========================================
void visualizeMPOPITrajectories() {
    static ros::Publisher pub_optimal = ros::NodeHandle().advertise<nav_msgs::Path>("/mpopi/optimal_path", 1);
    static ros::Publisher pub_reference = ros::NodeHandle().advertise<nav_msgs::Path>("/mpopi/reference_path", 1);
    static ros::Publisher pub_samples = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("/mpopi/sample_trajectories", 1);
    
    int K = mpopi_params.K;
    int N = mpopi_params.N;
    double DT = mpopi_params.DT;
    
    // 1. 최적 경로 publish
    nav_msgs::Path optimal_path;
    optimal_path.header.frame_id = "map";  // trajectories가 ENU → map 프레임 정확
    optimal_path.header.stamp = ros::Time::now();
    
    // 최소 cost를 가진 궤적 찾기
    int best_idx = 0;
    double min_cost = mpopi_state.costs[0];
    for (int k = 0; k < K; k++) {
        if (mpopi_state.costs[k] < min_cost) {
            min_cost = mpopi_state.costs[k];
            best_idx = k;
        }
    }
    
    for (int t = 0; t <= N; t++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now() + ros::Duration(t * DT);
        pose.pose.position.x = mpopi_state.trajectories[best_idx].states[t].x;
        pose.pose.position.y = mpopi_state.trajectories[best_idx].states[t].y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, mpopi_state.trajectories[best_idx].states[t].yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        optimal_path.poses.push_back(pose);
    }
    pub_optimal.publish(optimal_path);
    
    // 2. 참조 경로 publish (waypoints는 ENU → map 프레임)
    nav_msgs::Path reference_path;
    reference_path.header.frame_id = "map";
    reference_path.header.stamp = ros::Time::now();
    
    for (int i = 0; i < (int)waypoints.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = waypoints[i].x;
        pose.pose.position.y = waypoints[i].y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        reference_path.poses.push_back(pose);
    }
    pub_reference.publish(reference_path);
    
    // 3. 샘플 궤적들을 Marker로 publish (비용 기준 상위 20개)
    visualization_msgs::MarkerArray sample_markers;
    
    int top_n = std::min(20, K);
    std::vector<std::pair<double, int>> cost_indices;
    for (int k = 0; k < K; k++) {
        cost_indices.push_back({mpopi_state.costs[k], k});
    }
    std::sort(cost_indices.begin(), cost_indices.end());
    
    for (int n = 0; n < top_n; n++) {
        int k = cost_indices[n].second;
        double cost = cost_indices[n].first;
        
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";  // ENU → map 프레임 정확
        line_marker.header.stamp = ros::Time::now();
        line_marker.id = n;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        
        // 비용이 낮을수록 녹색, 높을수록 빨강
        double cost_ratio = std::min(1.0, cost / (min_cost + 1e-9));
        line_marker.color.r = cost_ratio;
        line_marker.color.g = 1.0 - cost_ratio;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.5;
        line_marker.scale.x = 0.08;
        
        for (int t = 0; t <= N; t++) {
            geometry_msgs::Point pt;
            pt.x = mpopi_state.trajectories[k].states[t].x;
            pt.y = mpopi_state.trajectories[k].states[t].y;
            pt.z = 0.0;
            line_marker.points.push_back(pt);
        }
        
        sample_markers.markers.push_back(line_marker);
    }
    pub_samples.publish(sample_markers);
    
    ROS_DEBUG("[MPOPI] Trajectories visualization published");
}