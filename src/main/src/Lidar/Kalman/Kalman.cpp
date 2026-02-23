#include "Kalman.hpp"

// 입력
// 1. st_Lidar.vec_clusters => centroid_x, _y
// 2. Timestamp => current_time 으로 들어오는 현재 프레임의 시간 정보

// ===========================================================================================
// EKFTracker - 예측값과 센서 측정값(= LShapeFitting 결과)만 사용
// ===========================================================================================

EKFTracker::EKFTracker(int id, float init_x, float init_y, const Eigen::MatrixXd& shared_Q, const Eigen::MatrixXd& shared_R) {
    this->id = id;
    this->px = init_x;
    this->py = init_y;
    this->theta = 0.0;
    this->v = 0.0;
    this->a = 0.0;

    this->miss_count = 0;
    this->real_cluster_count = 1; // 튀는 노이즈 포인트에 ID 가 부여되는 걸 막으려고

    this->x = Eigen::VectorXd::Zero(5);
    this->x << px, py, theta, v, a;

    // 오차 공분산 P 행렬 : 예측값이 측정값과 얼마나 차이가 날 수 있는가. 불신(?)의 정도. (값이 클수록 측정값z를 더 많이 반영)
    this->P = Eigen::MatrixXd::Identity(5, 5) * 100.0; 
    // this->P(2, 2) = 10.0;   // theta(방향): 아직 모르므로 불확실성 높임
    // this->P(3, 3) = 100.0;  // v(속도): 처음엔 0으로 시작하지만 실제론 빠를 수 있으니 불확실성 극대화
    // this->P(4, 4) = 1.0;  // a(가속도): 위와 동일

    // ----------- Q, R 은 Multi EKFTracker 에서 초기화 --------------

    // 노이즈 공분산 Q : 내 예측값 계산 모델이 얼마나 부정확한가. Q는 매 단계마다 P를 더 키우는 역할. 
    this->Q = shared_Q;

    // 측정 노이즈 공분산 R : 측정값이 얼마나 부정확한가. 
    this->R = shared_R;
}

// ===========================================================================================
// 마할라노비스 거리 계산 함수
// ===========================================================================================

double EKFTracker::getMahalanobisDistance(double measured_x, double measured_y, double measured_theta, double measured_v) {
    Eigen::VectorXd z(4);
    z << measured_x, measured_y, measured_theta, measured_v;

    Eigen::VectorXd h_x(4);
    h_x << px, py, theta, v; // 예측 위치

    Eigen::MatrixXd H = calculateJacobianH();
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    
    Eigen::VectorXd y = z - h_x; // 잔차 (Innovation)

    while (y(2) >  M_PI) y(2) -= 2.0 * M_PI;
    while (y(2) < -M_PI) y(2) += 2.0 * M_PI;

    double m_dist = std::sqrt(y.transpose() * S.inverse() * y);
    return m_dist;
}

// ===========================================================================================
// predict()
// ===========================================================================================

void EKFTracker::predict(double dt) {
    // 1. 상태 예측 (비선형 함수 f 직접 계산)

    // 등가속도 운동 거리
    double dist = v * dt + 0.5 * a * dt * dt;

    // 이동 거리 분해해서 업데이트 중
    px = px + dist * cos(theta); 
    py = py + dist * sin(theta); 
    v  = v + a * dt;             

    // 계산된 변수들을 다시 벡터 x에 동기화 (행렬 연산을 위해)
    x  << px, py, theta, v, a;

    // 2. 자코비안 F_t 생성 (식 32)
    Eigen::MatrixXd F = calculateJacobianF(dt);

    // 3. 공분산 P 업데이트 (식 40)
    P = F * P * F.transpose() + Q;

    // << 오차 공분산 P 행렬 >>
    // : 예측값이 측정값과 얼마나 차이가 날 수 있는가. 불신(?)의 정도. (값이 클수록 측정값z를 더 많이 반영)
    // : 현재 내 상태 (벡터x)가 얼마나 흔들리고 있는지 보여주는 오차의 영역. ex 타원

    // << 자코비안 행렬 F >>
    // : 현재 상태의 변화가 다음 상태에 미치는 영향력 의미
    // : 지금 v 나 theta 를 살짝 바꿨을 때,  다음 위치(x, y)가 얼마나 크게 변하는가?

    // -> 현재 내 상태x에 있는 오차 P가 자코비안 F를 거쳐서 다음 단계 위치오차로 얼마나 번져나가는가를 계산

    // -> update는, F를 거쳐서 커진 오차 P를 실제 측정값R로 다시 좁히는 과정
}

// 자코비안 행렬 : dt만큼 시간이 흐를 떄, 현재 상태(x)의 변화가 다음 상태에 미치는 영향력을 나타낸 행렬
Eigen::MatrixXd EKFTracker::calculateJacobianF(double dt) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5); 
    double dist = v * dt + 0.5 * a * dt * dt;

    F(0, 2) = -dist * sin(theta);         
    F(0, 3) = dt * cos(theta);           
    F(0, 4) = 0.5 * dt * dt * cos(theta); 
    F(1, 2) = dist * cos(theta);          
    F(1, 3) = dt * sin(theta);           
    F(1, 4) = 0.5 * dt * dt * sin(theta); 
    F(3, 4) = dt;                       

    return F;
}


// ===========================================================================================
// update() -> 센서 측정값 (z)(= LShapeFitting 결과값)을 받아 predict 값을 수정하는 과정
// ===========================================================================================

void EKFTracker::update(double measured_x, double measured_y, double measured_theta, double dt) {
    // 1. 측정값 벡터 구성 (z_t) -> 현재 위치 함수

    double dx = measured_x - px;
    double dy = measured_y - py;
    double dist = std::sqrt(dx*dx + dy*dy);
    double calculated_v = dist / dt;

    // // 부작용 방지: 최소 0.2m 이상 움직였을 때만 방향을 새로 세팅
    // if (this->real_cluster_count == 1 && dist > 0.2) {
    //     this->theta = std::atan2(dy, dx);
    //     this->x(2) = this->theta;
    // }

    // ---------------------------------------------------------
    // [핵심 추가] 각도 모호성(180도 플립) 해결 로직
    // ---------------------------------------------------------
    // L-Shape Fitting은 앞뒤 구분이 안 되어 180도 반대 각도를 줄 때가 많음.
    // 현재 필터가 추정하는 theta와 측정값 measured_theta의 차이를 계산.
    
    double angle_diff = measured_theta - this->theta;

    // 각도 차이를 -PI ~ PI 범위로 정규화
    while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    // 만약 차이가 90도(PI/2)보다 크다면, 센서가 물체의 뒤쪽을 앞쪽으로 착각한 것임.
    if (std::abs(angle_diff) > M_PI / 2.0) {
        if (angle_diff > 0) measured_theta -= M_PI;
        else               measured_theta += M_PI;
    }
    
    // 보정된 measured_theta를 다시 한번 정규화
    while (measured_theta > M_PI)  measured_theta -= 2.0 * M_PI;
    while (measured_theta < -M_PI) measured_theta += 2.0 * M_PI;
    // ---------------------------------------------------------

    // 초기 상태에서 움직임이 감지되었을 때 방향 초기화 (기존 로직 유지)
    if (this->real_cluster_count == 1 && dist > 0.2) {
        this->theta = std::atan2(dy, dx);
        this->x(2) = this->theta;
    }

    Eigen::VectorXd z(4);
    z << measured_x, measured_y, measured_theta, calculated_v;

    // 2. 예측값
    Eigen::VectorXd h_x(4);
    h_x << px, py, theta, v;

    // 3. 자코비안 H 
    Eigen::MatrixXd H = calculateJacobianH();

    // 4. 칼만 게인(Kalman Gain)
    Eigen::MatrixXd S = H * P * H.transpose() + R; // 예측 오차 P와 측정 오차 R을 합친 전체 오차의 합
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // << 칼만 게인 >>
    // : 전체 오차 S 중에서 예측 오차 P 가 차지하는 비중
    // : 예측값 불확실성 P와 센서 노이즈 R을 비교해서, 누구 말을 더 믿을지 결정하는 가중치

    // 5. 측정값과 예측값이 얼마나 틀리냐
    Eigen::VectorXd y = z - h_x; 
    // z : 측정값
    // h_x : 예측값
    // => y가 0에 가깝다면 - 예측 정확. 

    while (y(2) >  M_PI) y(2) -= 2.0 * M_PI;
    while (y(2) < -M_PI) y(2) += 2.0 * M_PI;

    Eigen::VectorXd delta_x = K * y;
    // 측정/예측 오차에 칼만게인을 곱한 값을 delta_x : 현재 상태는 어느 방향(측정 or 예측)으로 수정해야 하는가?를 판단하는 값

    // ex) K가 높을 때 = 센서 측정값을 더 신뢰
    // : 측정,예측 오차가 크면 예측이 완전 빗나갔다. -> 센서 방향으로 크게 움직임
    // :         오차가 작으면 예측한 곳에 물체가 있다. -> 살짝만 센서 방향으로 움직임 


    // << 센서 노이즈(측정 오차) R vs 측정 - 예측  y >>
    // R : 센서 자체의 오차 정도
    // y : 이번 프레임 내에서의 측정, 예측 위치 거리 차이
    x += delta_x;

    px = x(0);
    py = x(1);
    theta = x(2);
    v = x(3);
    a = x(4);

    // 각도 theta 를 -PI ~ PI 사이로 정규화
    while (theta >  M_PI) theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;

    x(2) = theta;

    // 6. 오차 공분산 업데이트 (Update Covariance) 
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);

    P = (I - K * H) * P;
    // 센서 관측값 계산 후, 예측값 불확실성 P 업데이트.
    // K=1(센서100% 믿음) : 예측 오차 P = 0이 됨. 

    // => predice 에서 P 커짐 -> update 에서 다시 P 줄임
}


Eigen::MatrixXd EKFTracker::calculateJacobianH() {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 5);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0; 
    H(3, 3) = 1.0; 
    return H;
}

// ===========================================================================================
// EKFTracker - 여러 클러스터가 들어왔을 때 [ 기존에 트래킹하던 물체 / 새 물체 ] 판별 클래스 
// ===========================================================================================

// 1. prediction : 리스트에 있는 모든 EKFTracker 를 한 스텝 미래로 이동시킴
// 2. association : 현재 라이다 클러스터들과 예측된 트랙들 사이의 거리를 계산해 짝 지음
// 3. update : 짝이 맞은 트랙은 update 하고, 짝이 없는 클러스터는 새로 트랙 생성
// 4. management : 오랫동안 안 보이는 트랙은 삭제

MultiObjectTracker::MultiObjectTracker() : next_id(0), last_timestamp(0.0) {

    // 노이즈 공분산 Q : 내 예측값 계산 모델이 얼마나 부정확한가. Q는 매 단계마다 P를 더 키우는 역할. 
    this->Q = Eigen::MatrixXd::Identity(5, 5)* 0.1;
    this->Q(0, 0) = 0.01;   // x 위치 예측 유연성
    this->Q(1, 1) = 0.01;   // y 위치 예측 유연성
    this->Q(2, 2) = 0.05;   // 세타의 안정성을 더 높이려면 (**0.05->0.01) 으로 줄여야. 
    this->Q(3, 3) = 0.01;  // 속도 변화 허용 ** 0.1 -> 0.01
    this->Q(4, 4) = 0.0001; // 가속도는 거의 변하지 않음. 

    // 측정 노이즈 공분산 R : 측정값이 얼마나 부정확한가. 
    this->R = Eigen::MatrixXd::Identity(4, 4);
    this->R(0, 0) = 2.0; // x 오차
    this->R(1, 1) = 2.0; // y 오차
    this->R(2, 2) = 1.0; // LShapeFitting 에서 theta가 무조건 차량 전방방향으로 나오지 않기 때문에 2.0-5.0 정도로 높게 잡아야
    this->R(3, 3) = 30.0; // v 오차 (위치 기반 계산이라 오차가 큼)

}

// ===========================================================================================
// 매 프레임 데이터가 들어올 때마다 호출되는 메인 루프
// ===========================================================================================

void MultiObjectTracker::updateTracks(Lidar& st_Lidar, double current_time) 
{
    st_Lidar.vec_kalman_clusters.clear();

    if (last_timestamp <= 0.0) {
        dt = 0.1; 
    }
    else {
        dt = current_time - last_timestamp;
        // [방어 로직 추가] 
        // 시간이 거꾸로 가거나(<=0), 1초 이상 끊겼을 때(>1.0) 0.1초로 고정
        if (dt <= 0.0 || dt > 1.0) 
        {
            dt = 0.1;
        }
    }

    // ===================================================
    // 1. Prediction
    // ===================================================

    for (EKFTracker& track : vec_EKFtracks) 
    {
        track.predict(dt);
    }

    // -> 예측할 모든 장애물의 위치를 dt 시간만큼 미리 이동시켜 본다. 

    // ===================================================
    // 2. Association & Update
    // ===================================================

    vector<bool> track_updated(vec_EKFtracks.size(), false);
    vector<bool> cluster_matched(st_Lidar.vec_clusters.size(), false);
    vector<int> assignment; // 헝가리안 결과 저장용

    // 새로운 프레임의 여러 클러스터가 들어왔을 때 어떤 클러스터가 내가 추적하던 트랙인지 결정

    int num_tracks = vec_EKFtracks.size();
    int num_clusters = st_Lidar.vec_clusters.size();

    if (num_tracks > 0 && num_clusters > 0) 
    {
        // ---------------------------------------------------
        // 1. Cost Matrix 생성 (모든 트랙 vs 모든 클러스터 거리 계산)
        // ---------------------------------------------------

        vector<vector<double>> cost_matrix(num_tracks, vector<double>(num_clusters));

        for (int i = 0; i < num_tracks; ++i) 
        {
            for (int j = 0; j < num_clusters; ++j) 
            {
                // 마할라노비스 거리 적용
                cost_matrix[i][j] = vec_EKFtracks[i].getMahalanobisDistance(st_Lidar.vec_clusters[j].centroid_x, 
                    st_Lidar.vec_clusters[j].centroid_y, st_Lidar.vec_clusters[j].heading_theta, st_Lidar.vec_clusters[j].velocity);
            }
        }

        // ---------------------------------------------------
        // 2. 헝가리안 알고리즘 호출 
        // ---------------------------------------------------

        HungarianAlgorithm hungarian_solver;
        hungarian_solver.Solve(cost_matrix, assignment); 

        // 1. 위에서 계산된 cost matrix에서 트랙과 클러스터 사이의 거리를 이용
        // 2. 모든 트랙에 중복없이 클러스터를 하나씩 배정했을 때, 선택된 거리값들의 총합이 가장 작아지는 조합을 찾는다.
        // 3. 출력 = assignment (배정 결과)

        // **assignment 구조**
        // (1) 인덱스 : 트랙 i
        // (2) 값 : 매칭된 클러스터 j

        // [2, 0, -1]
        // 0번 트랙 : 2번 클러스터, 1번 트랙 : 0번 클러스터 (update 진행 중)
        // 2번 트랙 : -1 (짝 못 찾음 = miss_count 증가 대상)

        // ---------------------------------------------------
        // 3. 매칭 결과 적용 (Update 단계)
        // ---------------------------------------------------

        for (int i = 0; i < num_tracks; ++i) 
        {
            int matched_cluster_idx = assignment[i];

            // 헝가리안으로 매칭 시켰더라도 거리가 너무 멀면 같은 물체가 아닐 수 있기 때문에
            // 같은 물체라고 판단하는 마할라노비스 거리 threshold 설정

            // 매칭 번호 -1 아니고, 거리 50 이내면 ㄹㅇ (트랙 - 클러스터) 짝으로 ㅇㅈ
            if (matched_cluster_idx != -1 && cost_matrix[i][matched_cluster_idx] < 1000.0) 
            {
                // // ---- 마할라노비스 거리가 실제로 몇이 나오는지 측정 ----
                // cout << "[ID: " << vec_EKFtracks[i].id << "] Matched Cluster: " << matched_cluster_idx 
                //     << " | Mahalanobis Dist: " << cost_matrix[i][matched_cluster_idx] << endl;

                vec_EKFtracks[i].update(st_Lidar.vec_clusters[matched_cluster_idx].centroid_x, 
                                    st_Lidar.vec_clusters[matched_cluster_idx].centroid_y,
                                    st_Lidar.vec_clusters[matched_cluster_idx].heading_theta,
                                    this->dt);
                cout << vec_EKFtracks[i].id << " / " << vec_EKFtracks[i].x(3) << ", " << vec_EKFtracks[i].x(4) << endl;

                vec_EKFtracks[i].real_cluster_count++;

                vec_EKFtracks[i].miss_count = 0;

                cluster_matched[matched_cluster_idx] = true;
                track_updated[i] = true;

                // 1) kalman gain 만큼 위치, 속도 수정
                // 2) 매칭되었으므로 miss_count = 0 으로 초기화 
                // 3) 해당 클러스터와 트랙이 사용되었으므로 matched, updated true로 변환
            }
        }
    }

    // ===================================================
    // 3. 미매칭 트랙 관리 및 삭제
    // ===================================================

    // update 성공하면 miss_count = 0

    // (A) 이전 프레임까진 매칭이 됐던 트랙인데, 이번엔 매칭이 안 된 경우
    // : 바로 삭제x, miss_count 만 증가
    for (int j = 0; j < num_tracks; ++j) 
    {
        if (!track_updated[j]) vec_EKFtracks[j].miss_count++;
    }

    // (B) 매칭 안 된 클러스터로 새 트랙 추가 (새 EKFTracker 객체 생성)
    for (int i = 0; i < num_clusters; ++i) 
    {
        if (!cluster_matched[i]) {
            vec_EKFtracks.emplace_back(next_id++, st_Lidar.vec_clusters[i].centroid_x, st_Lidar.vec_clusters[i].centroid_y, this->Q, this->R);
        }
    }

    // (C) miss_count 가 threshold 이상 넘어가면 삭제
    vec_EKFtracks.erase(remove_if(vec_EKFtracks.begin(), vec_EKFtracks.end(),
        [](const EKFTracker& t){ return t.miss_count > 10; }),
        vec_EKFtracks.end());

    // ===================================================
    // 4. 최종 결과 -> vec_KalmanDetections 구조체
    // ===================================================

    for (const EKFTracker& t : vec_EKFtracks) {
        if (t.miss_count > 0 || t.real_cluster_count < 2) continue;

        KalmanDetection res;
        res.id = t.id;
        res.x = t.x(0); 
        res.y = t.x(1); 
        res.yaw = t.x(2);
        res.v = t.x(3); 
        res.a = t.x(4); 
        
        res.is_confirmed = true;

        res.yaw_x = cos(t.x(2));
        res.yaw_y = sin(t.x(2));

        st_Lidar.vec_kalman_clusters.push_back(res);
        // cout << res.yaw_x << ", " << res.yaw_y << endl;
    }
    last_timestamp = current_time;
}