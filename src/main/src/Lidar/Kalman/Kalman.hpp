#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Global/Global.hpp>
#include <Hungarian/Hungarian.hpp>

class EKFTracker {
public:

    int id;
    int miss_count;
    int real_cluster_count;
    
    // 상태 변수 (이미지 식 31의 x_t)
    double px, py, theta, v, a;

    Eigen::VectorXd x; // 상태 벡터
    Eigen::MatrixXd P; // 오차 공분산

    // =================== 0. 초기화 ======================
    // x0 (상태 벡터): 물체의 처음 위치와 속도 설정 = 라이다 측정값
    // P0 (오차 공분산 행렬): 초기 예측이 얼마나 정확할지 설정 (단위행렬 I 에 큰 값을 곱하고 시작)

    EKFTracker(int id, float init_x, float init_y, const Eigen::MatrixXd& shared_Q, const Eigen::MatrixXd& shared_R);


    // =================== 마할라노비스 거리 ======================
    double getMahalanobisDistance(double measured_x, double measured_y, double measured_theta, double measured_v);


    // =============== 1. 예측 단계: 식 (40) ===============
    void predict(double dt);
    // 1. 상태 x 예측 : (비선형 모션 모델로) 다음 위치 계산
    // 2. 자코비안 F 계산 : 비선형 식 -> 선형화해서 기울기 계산
    // 3. 오차 공분산 P 예측 : 예측값대로 이동한 만큼 불확실성이 얼마나 커졌는지 계산

    // => predict() : 무한 반복하면서 x, P 갱신

    // Q))) 비선형 모션 모델이 뭔데
    // Q))) 기울기를 왜 계산하는데


    // =============== 2. 보정 단계: 식 (41) =================
    void update(double measured_x, double measured_y, double measured_theta, double measured_v);
    // 1. 센서 측정값(관측값) 벡터 구성 (z_t) 
    // 2. 비선형 관측 함수 h 적용 
    //   : 라이다 센서가 물체의 중심 좌표 (x, y)를 알려준다고 할 때, 상태 변수 x 에서 px, py만 추출
    // 3. 자코비안 H 계산 
    //   : h 를 각 상태 변수로 미분한 행렬
    //   : 측정 방식에 따른 선형화 기울기 구함
    // 4. 칼만 게인 K 계산 
    //  : 예측값과 센서 측정값 중 어떤 것을 더 믿을지 결정
    // 5. 상태 업데이트 
    //  : K로 예측값을 센서값 쪽으로 당겨서 수정
    // 6. 오차 공분산 업데이트 
    //  : 위치 정확해졌으니깐 불확실성 줄임

    // Q))) 예측값을 센서값 쪽으로 당긴다는 게 뭔데


    Eigen::MatrixXd Q; // 프로세스 노이즈
    Eigen::MatrixXd R; // 측정 노이즈
    Eigen::MatrixXd I; // 단위 행렬

    // 자코비안 F_t 계산 (식 32)
    Eigen::MatrixXd calculateJacobianF(double dt);
    
    // 자코비안 H_t 계산 (식 33)
    Eigen::MatrixXd calculateJacobianH();
};

class MultiObjectTracker {
public:
    MultiObjectTracker();
    vector<EKFTracker> vec_EKFtracks;
    void updateTracks(Lidar& st_Lidar, double current_time);
    
private:
    int next_id;
    double last_timestamp;
    double dt;
    
    // 모든 트랙이 공유하는 노이즈 설정
    Eigen::MatrixXd Q; // 프로세스 노이즈
    Eigen::MatrixXd R; // 측정 노이즈
};

#endif