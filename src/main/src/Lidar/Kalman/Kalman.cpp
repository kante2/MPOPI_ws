#include <Kalman/Kalman.hpp>

// ===========================================================================================
// EKFTracker - 예측값과 센서 측정값(= LShapeFitting 결과)만 사용
// ===========================================================================================

EKFTracker::EKFTracker(int id, float init_x, float init_y) {
    this->id = id;
    this->px = init_x;
    this->py = init_y;
    this->theta = 0.0;
    this->v = 0.0;
    this->a = 0.0;
    this->miss_count = 0;

    // Eigen 벡터 x 초기화
    this->x = Eigen::VectorXd::Zero(5);
    this->x << px, py, theta, v, a;

    // 공분산 행렬 P 초기화 (처음엔 불확실하므로 약간 큰 값)
    this->P = Eigen::MatrixXd::Identity(5, 5);
    this->Q = Eigen::MatrixXd::Identity(5, 5) * 0.05; 
    this->R = Eigen::MatrixXd::Identity(2, 2) * 0.1;
}

// ===========================================================================================
// predict()
// ===========================================================================================

void EKFTracker::predict(double dt) {
    // 1. 상태 예측 (비선형 함수 f 직접 계산)
    // 등가속도 운동 거리: s = v*t + 0.5*a*t^2
    double dist = v * dt + 0.5 * a * dt * dt;

    px = px + dist * cos(theta); // x 위치 업데이트
    py = py + dist * sin(theta); // y 위치 업데이트
    v  = v + a * dt;             // 속도 업데이트

    // 계산된 변수들을 다시 벡터 x에 동기화 (행렬 연산을 위해)
    x << px, py, theta, v, a;

    // 2. 자코비안 F_t 생성 (식 32)
    Eigen::MatrixXd F = calculateJacobianF(dt);

    // 3. 공분산 P 업데이트 (식 40)
    P = F * P * F.transpose() + Q;
}

Eigen::MatrixXd EKFTracker::calculateJacobianF(double dt) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5); // 단위행렬
    double dist = v * dt + 0.5 * a * dt * dt;

    // px를 각 변수들로 편미분한 결과
    F(0, 2) = -dist * sin(theta);         // d_px / d_theta
    F(0, 3) = dt * cos(theta);            // d_px / d_v
    F(0, 4) = 0.5 * dt * dt * cos(theta); // d_px / d_a

    // py를 각 변수들로 편미분한 결과
    F(1, 2) = dist * cos(theta);          // d_py / d_theta
    F(1, 3) = dt * sin(theta);            // d_py / d_v
    F(1, 4) = 0.5 * dt * dt * sin(theta); // d_py / d_a

    // v를 a로 편미분한 결과
    F(3, 4) = dt;                         // d_v / d_a

    return F;
}


// ===========================================================================================
// update() -> 센서 측정값 (z)(= LShapeFitting 결과값)을 받아 predict 값을 수정하는 과정
// ===========================================================================================

void EKFTracker::update(double measured_x, double measured_y) {
    // 1. 관측값 벡터 구성 (z_t)
    Eigen::VectorXd z(2);
    z << measured_x, measured_y;

    // 2. 비선형 관측 모델 적용 (h(x_pred, 0)) - 식 (41)
    Eigen::VectorXd h_x(2);
    h_x << px, py; // 현재 상태에서 예측되는 측정값

    // 3. 자코비안 H_t 계산 - 식 (33)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    H(0, 0) = 1.0; // d_h1 / d_px
    H(1, 1) = 1.0; // d_h2 / d_py

    // 4. 칼만 게인(Kalman Gain) 계산 - 식 (41, 42)
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // 5. 상태 업데이트 (Update State) - 식 (41, 42)
    Eigen::VectorXd y = z - h_x; // 잔차(Innovation)
    // z : 센서 측정값
    // h_x : 예측값
    // => y가 0에 가깝다면 - 예측 정확. 

    Eigen::VectorXd delta_x = K * y;
    // 측정/예측 오차에 칼만게인을 곱한 값을 delta_x : 현재 상태는 어느 방향(측정 or 예측)으로 수정해야 하는가?를 판단하는 값

    px += delta_x(0);
    py += delta_x(1);
    theta += delta_x(2);
    v += delta_x(3);
    a += delta_x(4);

    // 행렬 x 동기화
    x << px, py, theta, v, a;

    // 6. 오차 공분산 업데이트 (Update Covariance) - 식 (41, 42)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    P = (I - K * H) * P;
}


Eigen::MatrixXd EKFTracker::calculateJacobianH() {
    // x, y 위치만 측정하는 경우의 야코비안 (2x5 행렬)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    return H;
}

// ===========================================================================================
// EKFTracker - 여러 클러스터가 들어왔을 때 [ 기존에 트래킹하던 물체 / 새 물체 ] 판별 클래스 
// ===========================================================================================

// 1. prediction : 리스트에 있는 모든 EKFTracker 를 한 스텝 미래로 이동시킴
// 2. association : 현재 라이다 클러스터들과 예측된 트랙들 사이의 거리를 계산해 짝 지음
// 3. update : 짝이 맞은 트랙은 update 하고, 짝이 없는 클러스터는 새로 트랙 생성
// 4. management : 오랫동안 안 보이는 트랙은 삭제

// 트래킹에 필요한 행렬 초기값 설정
MultiObjectTracker::MultiObjectTracker() : next_id(0), last_timestamp(0.0) {
    // 시스템에 맞는 노이즈 값 설정 (튜닝 포인트)
    Q = Eigen::MatrixXd::Identity(5, 5) * 0.05;
    R = Eigen::MatrixXd::Identity(2, 2) * 0.1;
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
    }

    // --------------------------------------------------
    // 1. Prediction
    // --------------------------------------------------

    for (EKFTracker& track : vec_EKFtracks) 
    {
        track.predict(dt);
    }

    // std::vector<EKFTracker> vec_EKFtracks;
    // -> 예측할 모든 장애물의 위치를 dt 시간만큼 미리 이동시켜 본다. 


    // --------------------------------------------------
    // 2. Association & Update
    // --------------------------------------------------

    vector<bool> track_updated(vec_EKFtracks.size(), false);
    vector<bool> cluster_matched(st_Lidar.vec_clusters.size(), false);

    for (int cluster_idx = 0; cluster_idx < st_Lidar.vec_clusters.size(); ++cluster_idx) 
    {
        const LidarCluster& current_cluster = st_Lidar.vec_clusters[cluster_idx];

        double min_dist = 1.5; 
        int best_idx = -1;

        // 센서 측정값 (LShapeFitting 결과값) 과 예측값 매칭 

        for (size_t j = 0; j < vec_EKFtracks.size(); ++j) 
        {
            if (track_updated[j]) continue;

            double dx = current_cluster.centroid_x - vec_EKFtracks[j].x(0);
            double dy = current_cluster.centroid_y - vec_EKFtracks[j].x(1);
            double d = std::sqrt(dx*dx + dy*dy);
            // 1) 모든 클러스터와 트래킹 대상 물체 사이의 유클리드 거리를 계산 -> 마할라노비스거리

            if (d < min_dist) 
            {
                min_dist = d;
                best_idx = j;
            }
            // 2) 설정한 min_dist 보다 가까운 것 중 가장 인접한 쌍을 매칭 (가장 인접한 쌍 = 관측하던 물체)
        }

        // < LShapeFitting 결과값과 예측값 매칭 결과를 바탕으로, 예측값을 센서 측정값으로 수정하는 과정 >
        // 매칭 성공 시 : 해당 트랙의 update()를 호출하여 필터값을 보정

        // 해당 트랙 - 매칭 결과로 연결된 특정 트래킹 객체
        // 필터값(상태 변수 x) - 해당 객체가 가지고 있는 현재 위치, 속도, 각도 등의 수치 데이터

        if (best_idx != -1) {
        vec_EKFtracks[best_idx].update(current_cluster.centroid_x, current_cluster.centroid_y);
        track_updated[best_idx] = true;
        vec_EKFtracks[best_idx].miss_count = 0;
        cluster_matched[cluster_idx] = true;
        }

        // // 매칭 실패 시 : 새로운 장애물로 판단하여 vec_EKFtracks.emplace_back으로 새 트랙 생성
        // else {
        //     // 매칭 안 된 경우 새로운 트랙 생성
        //     vec_EKFtracks.emplace_back(next_id++, cluster);
        //     track_used.push_back(true); // 새로 추가된 트랙 표시
        // }
    }

    // --------------------------------------------------
    // 3. 미매칭 트랙 관리 및 삭제
    // --------------------------------------------------
    // 4. New Track: 매칭되지 않은 새로운 클러스터는 새 트랙으로 생성
    // 2) Association & Update 끝난 직후

    // (A) 기존 트랙들 miss_count 증가 (추가 전에!)
    for (size_t j = 0; j < vec_EKFtracks.size(); ++j) {
        if (!track_updated[j]) vec_EKFtracks[j].miss_count++;
    }

    // (B) 매칭 안 된 클러스터로 새 트랙 추가
    for (size_t i = 0; i < st_Lidar.vec_clusters.size(); ++i) {
        if (!cluster_matched[i]) {
            vec_EKFtracks.emplace_back(next_id++, st_Lidar.vec_clusters[i].centroid_x,
                                                st_Lidar.vec_clusters[i].centroid_y);
        }
    }

    // (C) 삭제
    vec_EKFtracks.erase(remove_if(vec_EKFtracks.begin(), vec_EKFtracks.end(),
        [](const EKFTracker& t){ return t.miss_count > 10; }),
        vec_EKFtracks.end());
    // ***** ))) 여기 너무 더럽


    // --------------------------------------------------
    // 4. 최종 결과를 vec_KalmanDetections 구조체에 복사
    // --------------------------------------------------

    for (const EKFTracker& t : vec_EKFtracks) {
        KalmanDetection res;
        res.id = t.id;
        res.x = t.x(0); 
        res.y = t.x(1); 
        res.yaw = t.x(2);
        res.v = t.x(3); 
        res.a = t.x(4); 
        // res.l = t.x(5);
        
        res.is_confirmed = (t.miss_count == 0);
        st_Lidar.vec_kalman_clusters.push_back(res);
    }
    last_timestamp = current_time;
}