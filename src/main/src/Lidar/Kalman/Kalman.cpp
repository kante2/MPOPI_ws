// #include <Lidar/Kalman/Kalman.hpp>

// // ===========================================================================================
// // ===========================================================================================
// // EKFTracker - 예측값과 센서 측정값(= LShapeFitting 결과)만 사용
// // ===========================================================================================
// // ===========================================================================================

// EKFTracker::EKFTracker(int id_val, const LidarCluster& cluster) : id(id_val), miss_count(0) 
// {
//     x = Eigen::VectorXd::Zero(6);
//     // LidarCluster의 OBB 정보로 초기화
//     x << cluster.centroid_x, cluster.centroid_y, cluster.theta, 0.0, cluster.width, cluster.length;
//     P = Eigen::MatrixXd::Identity(6, 6) * 1.0;
// }

// // ===========================================================================================
// // predict()
// // ===========================================================================================

// void EKFTracker::predict(double dt, const Eigen::MatrixXd& Q_mat) 
// {
//     double theta = x(2);
//     double v = x(3);

//     // 상태 예측
//     // : 물체는 현재 방향으로 일정하게 움직일 것이다
//     x(0) += v * cos(theta) * dt;
//     x(1) += v * sin(theta) * dt;

//     Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
//     F(0, 2) = -v * sin(theta) * dt;
//     F(0, 3) = cos(theta) * dt;
//     F(1, 2) = v * cos(theta) * dt;
//     F(1, 3) = sin(theta) * dt;

//     // 오차 공분산 업데이트 
//     // :  시간이 지남에 따라 불확실성(P)이 커지는 것을 계산
//     P = F * P * F.transpose() + Q_mat;
//     // F : 야코비안 행렬
// }


// // ===========================================================================================
// // update() -> 센서 측정값 (z)(= LShapeFitting 결과값)을 받아 predict 값을 수정하는 과정
// // ===========================================================================================

// void EKFTracker::update(const LidarCluster& cluster, const Eigen::MatrixXd& H_mat, const Eigen::MatrixXd& R_mat) 
// {
//     Eigen::VectorXd z(5);
//     z << cluster.centroid_x, cluster.centroid_y, cluster.theta, cluster.width, cluster.length;

//     // L-shape Symmetry (90도/180도 대응) 
//     // -> OBB 박스가 가로, 세로 바뀌면서 튀는 형상 방지

//     double angle_diff = z(2) - x(2);
//     while (angle_diff > M_PI/4.0) {
//         angle_diff -= M_PI/2.0;
//         std::swap(z(3), z(4)); 
//     }
//     while (angle_diff < -M_PI/4.0) {
//         angle_diff += M_PI/2.0;
//         std::swap(z(3), z(4));
//     }
//     z(2) = x(2) + angle_diff;

//     // 센서 측정값 (z) 과 예측값 (Hx) 차이 계산
//     Eigen::VectorXd y = z - H_mat * x;
//     Eigen::MatrixXd S = H_mat * P * H_mat.transpose() + R_mat;

//     // kalman gain (K) 계산
//     Eigen::MatrixXd K = P * H_mat.transpose() * S.inverse();

//     // 보정된 최종 위치, 속도, 크기
//     x = x + K * y;

//     P = (Eigen::MatrixXd::Identity(6, 6) - K * H_mat) * P;
// }

// // ===========================================================================================
// // ===========================================================================================
// // EKFTracker - 여러 클러스터가 들어왔을 때 기존에 트래킹하던 물체 / 새 물체 판별 클래스 
// // ===========================================================================================
// // ===========================================================================================

// // 트래킹에 필요한 행렬 초기값 설정
// MultiObjectTracker::MultiObjectTracker() : next_id(0), last_timestamp(0.0) 
// {
//     Q = Eigen::MatrixXd::Identity(6, 6) * 0.1;
//     R = Eigen::MatrixXd::Identity(5, 5) * 0.5;
//     H = Eigen::MatrixXd::Zero(5, 6);
//     H(0,0)=1; H(1,1)=1; H(2,2)=1; H(3,4)=1; H(4,5)=1;

//     // -> 각 행렬이 뭘 뜻하는지
// }

// // ===========================================================================================
// // 매 프레임 데이터가 들어올 때마다 호출되는 메인 루프
// // ===========================================================================================

// void MultiObjectTracker::updateTracks(Lidar& st_Lidar, double current_time) 
// {
//     double dt = (last_timestamp <= 0.0) ? 0.1 : (current_time - last_timestamp);
//     last_timestamp = current_time;

//     // --------------------------------------------------
//     // 1. Prediction
//     // --------------------------------------------------

//     for (auto& t : tracks) t.predict(dt, Q);
//     // std::vector<EKFTracker> tracks;
//     // -> 예측할 모든 장애물의 위치를 dt 시간만큼 미리 이동시켜 본다. 

//     // --------------------------------------------------
//     // 2. Association & Update
//     // --------------------------------------------------

//     std::vector<bool> track_used(tracks.size(), false);
//     vec_KalmanDetections.clear(); // 매 프레임 결과 리셋

//     for (auto& cluster : st_Lidar.vec_clusters) {
//         double min_dist = 1.5; 
//         int best_idx = -1;

//         // < 센서 측정값 (LShapeFitting 결과값) 과 예측값 매칭 >

//         for (size_t j = 0; j < tracks.size(); ++j) {
//             if (track_used[j]) continue;
//             double d = sqrt(pow(cluster.centroid_x - tracks[j].x(0), 2) + 
//                             pow(cluster.centroid_y - tracks[j].x(1), 2));
//             // 1) 모든 클러스터와 트래킹 대상 물체 사이의 유클리드 거리를 계산

//             if (d < min_dist) {
//                 min_dist = d;
//                 best_idx = j;
//             }
//             // 2) 설정한 min_dist 보다 가까운 것 중 가장 인접한 쌍을 매칭 (가장 인접한 쌍 = 관측하던 물체)
//         }

//         // < LShapeFitting 결과값과 예측값 매칭 결과를 바탕으로, 예측값을 센서 측정값으로 수정하는 과정 >
//         // 매칭 성공 시 : 해당 트랙의 update()를 호출하여 필터값을 보정

//         // 해당 트랙 - 매칭 결과로 연결된 특정 트래킹 객체
//         // 필터값(상태 변수 x) - 해당 객체가 가지고 있는 현재 위치, 속도, 각도 등의 수치 데이터

//         if (best_idx != -1) {
//             tracks[best_idx].update(cluster, H, R);
//             track_used[best_idx] = true;
//             tracks[best_idx].miss_count = 0;
//         } 

//         // 매칭 실패 시 : 새로운 장애물로 판단하여 tracks.emplace_back으로 새 트랙 생성
//         else {
//             // 매칭 안 된 경우 새로운 트랙 생성
//             tracks.emplace_back(next_id++, cluster);
//             track_used.push_back(true); // 새로 추가된 트랙 표시
//         }
//     }

//     // --------------------------------------------------
//     // 3. 미매칭 트랙 관리 및 삭제
//     // --------------------------------------------------

//     for (size_t j = 0; j < tracks.size(); ++j) {
//         if (!track_used[j]) tracks[j].miss_count++;
//     }

//     // 센서에 포착되지 않은 = 예측, 매칭이 이루어지지 않는 장애물 = 사라진 장애물은 정리
//     tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [](const EKFTracker& t){
//         return t.miss_count > 10;
//     }), tracks.end());


//     // --------------------------------------------------
//     // 4. 최종 결과를 vec_KalmanDetections 구조체에 복사
//     // --------------------------------------------------

//     for (const auto& t : tracks) {
//         KalmanDetection res;
//         res.id = t.id;
//         res.x = t.x(0); 
//         res.y = t.x(1); 
//         res.yaw = t.x(2);
//         res.v = t.x(3); 
//         res.w = t.x(4); 
//         res.l = t.x(5);
        
//         res.is_confirmed = (t.miss_count == 0);
//         vec_KalmanDetections.push_back(res);
//     }
// }