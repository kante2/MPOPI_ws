
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
    // LatticePlanning.cpp의 findClosestWaypoint() 방식 적용

    findClosestWaypoint(ego, mpopi_ctrl.close_idx); // ego -> close_idx

    // Algorithm 1 라인 4: for ℓ ← 1 to L do
    // L=5 반복: 이번 타임스텝에서 최적화를 5번 수행
    // 각 반복마다 분포가 개선되어 점진적으로 더 좋은 경로로 수렴
    // iteration은 AIS 수행 횟수로, 0부터 4까지 총 5회 반복
    for (int iteration = 0; iteration < 5; iteration++) {
        mpopi_state.current_iteration = iteration;
        
        // Algorithm 1 라인 6: Sample Ek from N(0, Σ')
        sampleKcontrols(mpopi_state);

        // ** 편향 분포,, **
        
        // Algorithm 1 라인 8: x_t ← F(x_{t-1}, g(u'_{t-1} + ε_k_{t-1}))
        // 한 시퀀스(제어입력) -> 동역학 -> 예측 궤적
        // ego + k개의 제어 시퀀스 -> k개의 궤적을 뽑아낸다 !!!!
        rollout(ego, mpopi_state.U_samples, mpopi_state.trajectories); 
          // &&&  U_samples -> trajectories

        // 신규 | 좌표계 편의성을 위해서 trajectory(ENU)가 base_link로 변환되어야 한다.
        trajectoriesToBaseLink(mpopi_state.trajectories, ego); // ← trajectories (ENU) → trajectories (base_link)
        
        // Algorithm 1 라인 9: s_k ← c(X) + φ(X) + λ(1-α)U'Σ⁻¹(ε_k + U' - U)
        computeCostMPOPI(mpopi_state.trajectories,mpopi_ctrl.close_idx, ego);
        
        // Algorithm 1 라인 15: w_k ← 1/η × exp(...)
        computeWeight(mpopi_state.costs);
        
        // Algorithm 1 라인 10-11: if ℓ < L then U', Σ' ← PerformAIS(...)
        // (마지막 반복 전까지만)
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
            // variance_schedule 제거! (중복 감소 방지)
            double std_v_current = mpopi_state.std_v[timeStep];
            double std_delta_current = mpopi_state.std_delta[timeStep];
            
            // 최소 분산 보장
            // sigma_min 을 통해, 최적화가 너무 진행되어 표준편차가 0 에 가까워지면 새로운 상황 대응 유연성이 떨어짐
            // -> 최소한의 의심(노이즈) 를 남겨놓는 구조이다.
            std_v_current = std::max(std_v_current, mpopi_params.sigma_v_min); // 분산
            std_delta_current = std::max(std_delta_current, mpopi_params.sigma_delta_min); // 표준편차
            
            // 가우시안 분포에서 샘플링 | 
            // normal_distribution은 평균, 표준편차를 받는다/ 
            // std_v는 노이즈의 폭을 직접적으로 결정하는 표준편차 
            // 참고 : normalDistV, normalDistDelta는 각각 속도와 조향에 대한 노이즈 분포를 나타냄 , <random> 라이브러리의 normal_distribution 클래스를 사용하여, 평균이 0이고 표준편차가 std_v_current인 가우시안 분포에서 노이즈를 샘플링한다.
            std::normal_distribution<double> normalDistV(0.0, std_v_current);
            std::normal_distribution<double> normalDistDelta(0.0, std_delta_current);
            
            // 노이즈 샘플: ε_k ~ N(0, Σ')
            // 속도, 조향의 노이즈를 독립적으로 생성 / 비대각 성분은 0으로 처리하여, 두 노이즈를 독립적으로 샘플림함 -> 그렇기에 아래 문장이 성립함. 
            // 실제 코드에서, 속도, 조향을 분리하여 관리하는 이유는 2*2행렬 직접 연산 말고, 실수 두개로 연산하는 것이 훨씬 빠르기에 다음과 같이 구현함. 
            double noise_v = normalDistV(generator);
            double noise_delta = normalDistDelta(generator);
            
            // 실제 제어: u_t = U'_t + ε_t
            double sampled_v = mpopi_state.U_nominal[timeStep].v + noise_v;
            double sampled_delta = mpopi_state.U_nominal[timeStep].delta + noise_delta;
            
            // 제어 입력 범위 제한 (Clipping) : 로봇의 물리적 한계를 강제
            sampled_v = std::max(mpopi_params.v_min, 
                                std::min(mpopi_params.v_max, sampled_v));
            sampled_delta = std::max(mpopi_params.delta_min, 
                                    std::min(mpopi_params.delta_max, sampled_delta));
            
            // 샘플 저장: U_samples[k][t]
            // 참고 :std::vector<std::vector<ControlInput>> U_samples; // U_samples[k][n]  ← k번째 샘플의 n번째 타임스텝 제어 입력
            // mpopi_state.U_samples 에는 k개의 서로 다른 미래 제어 시퀀스가 담긴다. 
            mpopi_state.U_samples[sampleIdx][timeStep].v = sampled_v;
            mpopi_state.U_samples[sampleIdx][timeStep].delta = sampled_delta;
        }
    }
}


// ========================================
// 라인 7-9: Rollout + Cost 계산
// x_t = F(x_{t-1}, g(u'_t + ε_k_t))
// ========================================
// void rollout(const VehicleState& current_state) {
void rollout(const VehicleState& ego,
             const std::vector<std::vector<ControlInput>>& U_samples,
             std::vector<MPOPITrajectory>& trajectories) {
    const int K = mpopi_params.K;
    const int N = mpopi_params.N;
    // 참고 : 
    // int K = 500;              // 샘플 수         "500번의 상상"
    // int N = 20;               // 예측 스텝 수     "한번의 상상 -> 0.1초씩 20번 -> 2초 예측"

    // 이 함수가 끝나면 로봇의 머릿속에는 500개의 서로 다른 미래 경로(궤적)가 그려짐.
    
    // rollout 의 결과  -> 장애물과 겹친다 -> 위험 시나리오 -> 비용이 눞음
    // rollout 의 결과  -> 장애물과 겹치지 않는다 -> 안전 시나리오 -> 비용이 낮음
    
    // Algorithm 1 라인 5: for k = 1 to K(500)
    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        // ================= 각 샘플에 대해서,  ===================
        // u1 = U_samples[0][0], u2 = U_samples[0][1], ..., u20 = U_samples[0][19]  -> trajectory 1 (500개)

        trajectories[sampleIdx].states.clear();
        trajectories[sampleIdx].states.resize(N + 1);  // t=0~t=N
        
        // 현재 상태 x_0 가져오기 (Algorithm 1 라인 1)
        // waypoints 는 loadWaypoints() 함수를 통해 파일에서 읽어온 경로점들의 리스트이다. main 부에 정의되어있음,
        trajectories[sampleIdx].states[0] = ego;

        
        // Algorithm 1 라인 7: for t = 1 to T
        // N=20 , 0.1초마다 20번 반복 -> 2초 예측

        // rollout() 실행 후

        // trajectories[sampleIdx].states[0]   // ← 현재 위치 (0초)
        // trajectories[sampleIdx].states[1]   // ← 0.1초 뒤
        // trajectories[sampleIdx].states[2]   // ← 0.2초 뒤
        // trajectories[sampleIdx].states[3]   // ← 0.3초 뒤
        // ...
        // trajectories[sampleIdx].states[19]  // ← 1.9초 뒤
        // trajectories[sampleIdx].states[20]  // ← 2.0초 뒤 (최종 위치!) ← next_state

        // ====================================================
        // for문 내부 
        // timeStep = 0 : 초기 상태 (현재 위치)
        // timeStep = 1 : 0.1초 뒤
        // timeStep = 2 : 0.2초 뒤

        // timeStep=0:
        // prev_state → states[0]  (현재 위치, 초기값으로 이미 설정됨)
        // next_state → states[1]
        // 동역학 계산 후 states[1]에 값 대입 

        // timeStep=1:
        // prev_state → states[1]  ← 이전 루프에서 계산된 값
        // next_state → states[2]
        // 동역학 계산 후 states[2]에 값 대입 

        // timeStep=2:
        // prev_state → states[2]
        // next_state → states[3]
        // 동역학 계산 후 states[3]에 값 대입 

        // ...

        // timeStep=19 (마지막):
        // prev_state → states[19]
        // next_state → states[20]  ← 최종 위치!
        // 동역학 계산 후 states[20]에 값 대입 

        for (int timeStep = 0; timeStep < N; timeStep++) {
            // 이전 상태
            VehicleState& prev_state = trajectories[sampleIdx].states[timeStep];
            VehicleState& next_state = trajectories[sampleIdx].states[timeStep + 1];
            
            // 제어 입력 가져오기
            const ControlInput& control = U_samples[sampleIdx][timeStep];
            
            // Bicycle Model (동역학 함수 F)
            // x_t = F(x_{t-1}, g(u_{t-1}))
            // 조향각 포화 (g 함수)
            double steered_angle = std::max(mpopi_params.delta_min,
                                           std::min(mpopi_params.delta_max, control.delta));
            
            // 동역학 전파 (Bicycle Model)
            // 좌표계: ENU (map frame)
            // 현재 상태에서 물리 법칙을 적용해 다음 상태를 도출
            double dt = mpopi_params.DT;      // 0.1초
            double wheelbase = mpopi_params.L; // 2.7m
            
            // ===== 차량 좌표계(base_link) → ENU 변환 =====
            // 차량의 전진 방향(base_link X축) 이동량
            double rel_dx = control.v * dt;  // base_link 기준 전진
            // 차량의 측면 방향(base_link Y축) 이동량 (조향으로 인한 횡이동)
            double rel_dy = control.v * std::tan(steered_angle) * dt / wheelbase;
            
            // base_link → ENU 좌표 변환 (회전 행렬 적용)
            double cos_yaw = std::cos(prev_state.yaw); // 현재 상태의 방향 cos 
            double sin_yaw = std::sin(prev_state.yaw); // 현재 상태의 방향 sin

            // 회전 행렬 적용 
            double enu_dx = rel_dx * cos_yaw - rel_dy * sin_yaw;
            double enu_dy = rel_dx * sin_yaw + rel_dy * cos_yaw;
            
            // ENU 좌표 업데이트
            next_state.x = prev_state.x + enu_dx;
            next_state.y = prev_state.y + enu_dy;
            
            // 방향(yaw) 업데이트
            next_state.yaw = prev_state.yaw + (control.v / wheelbase) * std::tan(steered_angle) * dt;
            next_state.vel = control.v;
        }
        // 이 마지막 결과 : trajectories[sampleIdx].states[N] 가 2초 뒤의 예측 위치가 된다. (최종 위치)
    }
}


// ** 제거 예정 ----------------------------------------------------------------------------------------
// ========================================
// 좌표계 변환 함수들은 Util_Coordinates.cpp에 정의되어 있음
// ========================================

// ========================================
// 라인 9: 비용 함수 계산
// s_k = c(X) + φ(X) + λ(1-α)U'Σ⁻¹(ε_k + U' - U)
//  computeCost() 에서는 500 개의 시퀀스 각각에 대한 점수를 매긴다.
// ========================================

// 참고로 이제 trajectories부분은 baseLink 기준 좌표로 되어있다. (costmap과 같은 프레임이므로, 비용 계산이 편해짐)

void computeCostMPOPI(const std::vector<MPOPITrajectory>& trajectories, int ego_closest_wp_idx, const VehicleState& ego) {
    const int K = mpopi_params.K;
    const int N = mpopi_params.N;

    // 참고 : 
    // int K = 500;              // 샘플 수         "500번의 상상"
    // int N = 20;               // 예측 스텝 수     "한번의 상상 -> 0.1초씩 20번 -> 2초 예측"

    // 500개의 가상 시나리오에 대해서 성적표를 매기는 과정,.

    // Algorithm 1 라인 5: for k = 1 to K
    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        double pathCost = 0.0;        // c(X): 경로 비용
        double terminalCost = 0.0;    // φ(X): 최종 상태 비용
        double regularizationCost = 0.0; // 제어 정규화항
        
        // <1.  경로 비용 c(X) > ========================================================================
        for (int timeStep = 0; timeStep < N; timeStep++)
        {
            VehicleState& state = mpopi_state.trajectories[sampleIdx].states[timeStep];
            
            // (1) lateral_error 구현 (횡방향 오차)
            // 경로이탈의 제곱을 주어, 많이 벗어나면 많은 벌점을 위해 제곱으로 표현
            // rollout의 결과 :trajectories[k].states[t] (rollout 의 결과)
            // state.x, state.y 는 k번째 샘플의 t번째 타임스텝에서 예측된 위치를 나타냄
            // state는 base_link 좌표이므로, ENU 좌표로 역변환 후 경로와 비교
            double cos_yaw = std::cos(ego.yaw);
            double sin_yaw = std::sin(ego.yaw);
            double enu_x = ego.x + state.x * cos_yaw - state.y * sin_yaw;
            double enu_y = ego.y + state.x * sin_yaw + state.y * cos_yaw;
            
            // ego_closest_wp_idx -> 
            double lateral_error = computeLateralError(enu_x, enu_y, ego_closest_wp_idx);
            pathCost += mpopi_params.w_path * (lateral_error * lateral_error);
            
            // (2) obstacle_cost 구현 (Costmap 활용)
            // ========================================================
            // trajectoriesToBaseLink()로 이미 base_link 좌표로 변환됨
            // state.x, state.y는 이미 base_link 좌표이고, costmap도 base_link 기준
            // ========================================================
            double obstacle_cost = getMPOPICostmapCost(state.x, state.y) / 100.0;
            pathCost += mpopi_params.w_obstacle * obstacle_cost;
            
            // 충돌 즉시 패널티
            // BREAK 를 통해 해당 샘플을 사실상 탈락하게 됨
            // 이제 라이다로 동적 장애물을 COSTMAP에 반영하면 해당 부분을 통해 연산이 크게 줄어들 가능성 <- 논문 포인트 라고 생각하고 있음, 
            // 남은 연산 자원을 유효한 경로 후보들에 집중하게 만드는 전략적 최적화 포인트
            if (obstacle_cost >= 0.99) {
                pathCost += 1e6;
                break;
            }
            
            // (3) drivable area cost 구현 (Camera 정보 활용) -> 카메라 정보는 사용하지 않을 계획. 
            // double drivable_cost = 0.0;  // getCameraCost(state.x, state.y) / 100.0;
            // pathCost += mpopi_params.w_drivable * drivable_cost;
            
            // (4) 속도 추종 cost
            // target_vel은 전역 변수 또는 이전 계획값 사용
            double vel_error = state.vel - target_vel;
            pathCost += mpopi_params.w_velocity * (vel_error * vel_error);
        }
        
        // <2 . 최종 상태 비용 φ(X)> ================================================================
        // 2초뒤에 예측의 마지막 지점인 N=20 이 어디에 있는지를 본다. 
        // states[N] 은 , 20번째 타임스텝의 상태를 나타냄 (즉, 2초 뒤의 예측된 위치 - 즉 예측 최종 위치라고 보면 됨.,)
        {
            VehicleState& final_state = mpopi_state.trajectories[sampleIdx].states[N]; // base link 기준
            
            // base_link → ENU 역변환 (waypoints는 ENU 좌표)
            double cos_yaw = std::cos(ego.yaw);
            double sin_yaw = std::sin(ego.yaw);
            double final_enu_x = ego.x + final_state.x * cos_yaw - final_state.y * sin_yaw;
            double final_enu_y = ego.y + final_state.x * sin_yaw + final_state.y * cos_yaw;
            
            // 가장 가까운 waypoint 찾기
            if (!waypoints.empty()) {
                int close_idx = ego_closest_wp_idx;
                double min_dist = 1e10;
                
                // ego 기준 범위 제한 (전체 순회 대신 인근 범위만 검색)
                int search_start = std::max(0, ego_closest_wp_idx - 5);
                int search_end   = std::min((int)waypoints.size()-1, ego_closest_wp_idx + 50);
                
                for (int i = search_start; i <= search_end; i++) {
                    double dx = waypoints[i].x - final_enu_x;
                    double dy = waypoints[i].y - final_enu_y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    if (dist < min_dist) {
                        min_dist = dist;
                        close_idx = i;
                    }
                }
                
                // <목표점까지 거리>
                // 종착역이 우리가 가야 할 기준 경로(Waypoints)와 얼마나 가까운지를 측정
                double dx = final_enu_x - waypoints[close_idx].x;
                double dy = final_enu_y - waypoints[close_idx].y;
                terminalCost = mpopi_params.w_goal * (dx*dx + dy*dy);
            }
        }
        
        // <3 . 제어 정규화항: λ(1-α)U'Σ⁻¹(ε_k + U' - U) > =========================================================
        // 개선: Σ⁻¹ 반영 (기존: dv² + dd², 개선: dv²/σ_v² + dd²/σ_δ²)
        // 제어 비용 계산하는 부분인데, KL-divergence 에서 나온 개념이다.
        // 확신의 정도에 따라 벌점의 크기를 조절하는 것이 핵심이다./

        // U' - U => 현재 샘플의 제어값과, 기준제어값의 차이이다,, (dv, ddelta 모두 제어 입력의 차이)
        // 공분산의 역행렬 : 1.0 / sv^2 의 형태로 구현함. 
        // lamda(1 - a) : mpopi_params.temperature * (1.0 - alpha); 
        //   이떄 alpha 의 값은, 기존 경혐(mppi) 와 새로운 적응(AIS) 를 얼마나 섞을지 결정한다.
        double alpha = 0.3; // 경험과 적응의 조화 비율 (0.3은 경험에 30%, 적응에 70% 가중치) 
        {
            for (int timeStep = 0; timeStep < N; timeStep++) {
                // 물리적 변화량
                // 1. 속도 변화량 
                double dv = mpopi_state.U_samples[sampleIdx][timeStep].v - 
                           mpopi_state.U_nominal[timeStep].v;
                // 2. 조향 변화량
                double ddelta = mpopi_state.U_samples[sampleIdx][timeStep].delta - 
                               mpopi_state.U_nominal[timeStep].delta;
                
                // 공분산의 역행렬(regularizationCost) : Σ⁻¹ 반영 (분산으로 정규화)
                double sv = std::max(mpopi_state.std_v[timeStep], mpopi_params.sigma_v_min);
                double sd = std::max(mpopi_state.std_delta[timeStep], mpopi_params.sigma_delta_min);

                // mppi, mpopi의 차이점
                // 분산으로 나누어 주는것이 차이점 (sv, sd는 각각 속도와 조향의 노이즈 분산을 나타냄)
                // case1 .확신이 크다 -> sv, sd가 작다 -> 정규화항이 커진다 -> 벌점이 커진다 -> "이미 길을 잘 아니 딴짓을 강하게 처벌".
                // case2 .확신이 작다 -> sv, sd가 크다 -> 정규화항이 작아진다 -> 벌점이 작아진다 -> "아직 길을 잘 모르니, 이것저것 시도".
                regularizationCost += (dv*dv)/(sv*sv) + (ddelta*ddelta)/(sd*sd);
            }
            // 마지막 기존경험 + 새로운 적응을 섞는 방법. 
            regularizationCost *= mpopi_params.temperature * (1.0 - alpha);
        }
        
        // Algorithm 1 라인 9: s_k = c(X) + φ(X) + λ(1-α)...
        // costs[0] = 샘플 0의 총 점수
        // costs[1] = 샘플 1의 총 점수
        // ...
        // costs[499] = 샘플 499의 총 점수
        mpopi_state.costs[sampleIdx] = pathCost + terminalCost + regularizationCost;
    }
}


// ========================================
// 라인 12-15: 가중치 계산 (Softmax)
// w_k = exp(-1/λ × (s_k - ρ)) / η

// 비용을 sofmax함수를 통해 비용(cost)을 확률적(weights)으로 변환한다. 
// ========================================
void computeWeight(std::vector<double>& mpopi_costs) {
    const int K = mpopi_params.K;
    
    // Algorithm 1 라인 12: ρ = min(S)
    //  가장 낮은 비용으로 잡는 이유는 뒤에서 EXP지수함수를 사용하기에 안전하게 계산하기 위한 테크닉
    double minCost = *std::min_element(mpopi_costs.begin(), mpopi_costs.end());
    
    // Algorithm 1 라인 13: η = Σ exp(-1/λ × (s_k - ρ))
    // 참고 : K는 샘플수, 500개로 놓음, 
    double normalizationConstant = 0.0;
    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        double exponent = -(mpopi_costs[sampleIdx] - minCost) / mpopi_params.temperature;
        exponent = std::max(-700.0, std::min(700.0, exponent));  // 수치 안정성
        // 정규화 상수 누적 : 모든 샘플의 지수 함수값을 다 더한다.
        normalizationConstant += std::exp(exponent);
    }
    
    // Algorithm 1 라인 15: w_k = 1/η × exp(...)
    for (int sampleIdx = 0; sampleIdx < K; sampleIdx++) {
        double exponent = -(mpopi_costs[sampleIdx] - minCost) / mpopi_params.temperature;
        exponent = std::max(-700.0, std::min(700.0, exponent));
        // normalizationConstant, 이 값으로 각 샘플의 점수를 나누면 전체 합이 1이되는 확률 분포가 만들어진다.
        // 각 샘플은, 0~1 사이의 가중치를 가지게 되고, 모든 샘플의 가중치 합은 1이 된다.
        mpopi_state.weights[sampleIdx] = std::exp(exponent) / normalizationConstant;
    }

    // 앞으로, 위 가중치를 가지고 점수가 높은 의견으로 결정후, 다음 반복에서는 분산을 좁히는 updatedistribution 과정을 거치게 된다. 
}

// ========================================
// 라인 10-11: 분포 업데이트 (AIS)
// U', Σ' ← PerformAIS(...)
// ** 이것이 mppi 와 다른 moppi 의 특징이자 강점임 /  샘플을 무작위로하는게 아니라,
//  성공 높은 영역을 찾아서, 그 주변을 더 집중적으로 샘플링 ㅎ사도록 공분산/ 평균을 실시간으로 최적화 하는 과정이다.

// mppi와 다르게, ais(adaptive importance sampling) 과정을 통해, 
//  샘플링 분포의 평균과 분산을 업데이트하여, 다음 iteration에서 더 좋은 샘플링이 이루어지도록 학습한다. 
// ========================================
void updateDistribution(std::vector<double>& mpopi_weights) {
    const int K = mpopi_params.K;
    const int N = mpopi_params.N;
    
    //  Algorithm 1 라인 10: if ℓ < L (마지막 iteration 전까지만)
    // (프로세스 초반부에서만 분포 업데이트)
    
    // 1️1.새로운 평균 계산 (가중 평균)
    // Algorithm 1 라인 16: U += Σ w_k × (ε_k + U' - U)
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
    
    // 2️2. 새로운 분산 계산 (적응적 분산 축소)
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
        
        new_std_v[timeStep] = std::sqrt(std::max(weighted_var_v, mpopi_params.sigma_v_min * mpopi_params.sigma_v_min));
        new_std_delta[timeStep] = std::sqrt(std::max(weighted_var_delta, mpopi_params.sigma_delta_min * mpopi_params.sigma_delta_min));
    }
    
    // 3️3. 상태 업데이트
    mpopi_state.U_nominal = new_U_nominal; // 1. 새로운 기준 제어값
    mpopi_state.mean_v = std::vector<double>(N); // 2. 속도 평균
    mpopi_state.mean_delta = std::vector<double>(N); // 3. 조향각 평균 -- 여기까지는 초기화 + 만들어준 벡터에 값을 채워주는 형태로 구현함.
    for (int t = 0; t < N; t++) { 
        mpopi_state.mean_v[t] = new_U_nominal[t].v;
        mpopi_state.mean_delta[t] = new_U_nominal[t].delta;
    }
    mpopi_state.std_v = new_std_v;          // 4.속도의 표준편차 업데이트
    mpopi_state.std_delta = new_std_delta;  // 5.조향각의 표준편차 업데이트

    // 이 함수의 결과 : mpopi_state 의 U_nominal, mean_v, mean_delta, std_v, std_delta 가 모두 업데이트 된다.!
}

// ========================================
// 라인 17-20: Receding Horizon (시간 시프트)
// Algorithm 1 라인 18-20
// ========================================
void shiftSequence() {
    const int N = mpopi_params.N;
    
    // Algorithm 1 라인 17: SendToActuators(u₀)
    // U[0]의 첫 번째 제어를 차량에 전송
    // 가장 완벽한 첫번째 행동을 실제 차량의 모터, 조향에 적용하는 단계이다.

    // 미래 2초분(N=20)을 모두 계획하였으나ㅓ, 실제로는 지금 당장의 0.1초만 실행 후, 다시 전체를 계산하는 전략
    mpopi_cmd.v = mpopi_state.U_nominal[0].v;
    mpopi_cmd.delta = mpopi_state.U_nominal[0].delta;
    
    // MPOPI 최적값을 차량에 직접 전송 (PID 제어 제외, MPOPI 결과 직접 사용)
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    
    // 속도 제어 (간단한 ON/OFF 제어)
    static double prev_vel = 0.0;
    if (mpopi_cmd.v > mpopi_vehicle_state.vel) {
        cmd.accel = std::min(1.0, (mpopi_cmd.v - mpopi_vehicle_state.vel) * 0.3);
        cmd.brake = 0.0;
    } else {
        cmd.accel = 0.0;
        cmd.brake = std::min(1.0, (mpopi_vehicle_state.vel - mpopi_cmd.v) * 0.3);
    }
    
    // 조향각 (MPOPI 최적값)
    cmd.steering = mpopi_cmd.delta;
    cmd_pub.publish(cmd);
    
    // Algorithm 1 라인 18-19: for t = 1 to T-1, u_{t-1} ← u_t
    // 시퀀스를 한 칸 앞으로 당기기
    for (int timeStep = 0; timeStep < N - 1; timeStep++) {
        mpopi_state.U_nominal[timeStep] = mpopi_state.U_nominal[timeStep + 1];
    }
    
    // Algorithm 1 라인 20: u_{T-1} ← Initialize(u_{T-1})
    // 마지막 제어 입력 재초기화
    mpopi_state.U_nominal[N - 1].v = 30.0;      // 초기 속도
    mpopi_state.U_nominal[N - 1].delta = 0.0;  // 초기 조향각
    
    // 분산도 같이 shift (필수)
    // 예측 지평이 앞으로 이동하므로 분산도 함께 이동해야 함
    for (int timeStep = 0; timeStep < N - 1; timeStep++) {
        mpopi_state.std_v[timeStep] = mpopi_state.std_v[timeStep + 1];
        mpopi_state.std_delta[timeStep] = mpopi_state.std_delta[timeStep + 1];
    }
    // 새로 추가되는 마지막 시간스텝의 분산은 초기값으로
    mpopi_state.std_v[N - 1] = mpopi_params.sigma_v;
    mpopi_state.std_delta[N - 1] = mpopi_params.sigma_delta;
}

// ========================================
// 경로 시각화 함수
// RViz에서 MPOPI 경로들을 시각화
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
    optimal_path.header.frame_id = "map";  // 전역 좌표계
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
    
    // 최적 궤적의 모든 점을 경로로 추가
    for (int t = 0; t < N; t++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";  // 전역 좌표계
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
    
    // 2. 참조 경로 publish
    nav_msgs::Path reference_path;
    reference_path.header.frame_id = "map";  // 전역 좌표계 (map 또는 odom)
    reference_path.header.stamp = ros::Time::now();
    
    for (int i = 0; i < waypoints.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";  // 전역 좌표계
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = waypoints[i].x;
        pose.pose.position.y = waypoints[i].y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        pose.pose.orientation.w = 1.0;
        
        reference_path.poses.push_back(pose);
    }
    pub_reference.publish(reference_path);
    
    // 3. 샘플 궤적들을 Marker로 publish (비용 기준 상위 20개만)
    visualization_msgs::MarkerArray sample_markers;
    
    // 비용 기준으로 상위 20개 찾기
    int top_n = std::min(20, K);
    std::vector<std::pair<double, int>> cost_indices;
    for (int k = 0; k < K; k++) {
        cost_indices.push_back({mpopi_state.costs[k], k});
    }
    // 비용 오름차순 정렬
    std::sort(cost_indices.begin(), cost_indices.end());
    
    for (int n = 0; n < top_n; n++) {
        int k = cost_indices[n].second;
        double cost = cost_indices[n].first;
        
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";  // map 기준
        line_marker.header.stamp = ros::Time::now();
        line_marker.id = n;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        // Orientation 초기화 (identity quaternion)
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        
        // 비용이 낮을수록 녹색, 높을수록 빨강
        double cost_ratio = min(1.0, cost / min_cost);
        line_marker.color.r = cost_ratio;
        line_marker.color.g = 1.0 - cost_ratio;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.5;  // 투명도 증가
        
        line_marker.scale.x = 0.08;  // 선의 두께
        
        // 샘플 궤적의 모든 점을 추가
        for (int t = 0; t < N; t++) {
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

// ========================================
// Helper: Compute Lateral Error
// ========================================
// double computeLateralError(double x, double y, int ego_closest_wp_idx) {
//     // 경로 상의 가장 가까운 두 웨이포인트 찾기
//     if (ego_closest_wp_idx < 0 || ego_closest_wp_idx >= (int)waypoints.size() - 1) {
//         return 10.0;  // 유효하지 않은 인덱스면 큰 오차 반환
//     }
    
//     const Waypoint& wp1 = waypoints[ego_closest_wp_idx];
//     const Waypoint& wp2 = waypoints[ego_closest_wp_idx + 1];
    
//     // 두 웨이포인트 사이의 거리
//     double dx = wp2.x - wp1.x;
//     double dy = wp2.y - wp1.y;
//     double line_len = std::sqrt(dx * dx + dy * dy);
    
//     if (line_len < 1e-6) {
//         return std::sqrt((x - wp1.x) * (x - wp1.x) + (y - wp1.y) * (y - wp1.y));
//     }
    
//     // 점(x, y)에서 직선(wp1, wp2)까지의 최단거리 (부호 있는 거리)
//     // 공식: |ax + by + c| / sqrt(a^2 + b^2)
//     // 직선: (wp2.y - wp1.y)*x - (wp2.x - wp1.x)*y + wp2.x*wp1.y - wp2.y*wp1.x = 0
//     double a = wp2.y - wp1.y;
//     double b = -(wp2.x - wp1.x);
//     double c = wp2.x * wp1.y - wp2.y * wp1.x;
    
//     double lateral_error = std::fabs(a * x + b * y + c) / line_len;
    
//     return lateral_error;
// }