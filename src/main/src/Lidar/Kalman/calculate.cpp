// #include <cmath>
// #include <iostream>
// #include "kalman_filter.h"

// using Eigen::MatrixXd;
// using Eigen::VectorXd;

// KalmanFilter::KalmanFilter() {}

// KalmanFilter::~KalmanFilter() {}

// void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
//                         MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
//   x_ = x_in;
//   P_ = P_in;
//   F_ = F_in;
//   H_ = H_in;
//   R_ = R_in;
//   Q_ = Q_in;
// }

// // ================================================================
// // 다음 위치 예측
// // ================================================================

// // 등속도 모델 바탕으로 가정
// // -> 1. 모라이 동적 장애물의 속도가 등속도인지 확인해야 함
// // -> 2. 등속도가 아니라면 어떤 모델을 써야하는지 서치 
// //       => EKF는 속도 변화를 불확실성 (noise) 로 처리하여 등속도가 아닌 장애물에 대한 문제를 해결함. 
// // ( 해결 방법 )
// // - 가속도가 심한 물체라면 Q 행렬 안의 가속도 노이즈 값 (noise_ax,y) 를 높이기. (예측값보다 센서 측정값을 더 믿게 됨)
// // - dt 짧게 유지 (센서 데이터를 0.1초마다 받아서 업데이트 하면 가속은 미미하게 보이기 때문에)

// void KalmanFilter::Predict() {
//   // 현재 위치와 속도에 시간 dt 를 반영해서 "이만큼 움직였겠지?" 라고 예측하는 식
//   x_ = F_ * x_;
//   // 예측을 했으니 오차 범위 (공분산 P) 도 업데이트 + 시간이 흐를수록 불확실성이 커지므로 프로세스 노이즈 Q 더해주기
//   P_ = F_ * P_ * F_.transpose() + Q_;
// }


// // ================================================================
// // 예측값과 실제값의 차이를 보정하여 최종 위치 산출
// // ================================================================

// void KalmanFilter::Update(const VectorXd &z) {
  
//   // 1. 오차 계산
//   VectorXd z_prediction = H_ * x_;
//   // 방금 Predict에서 예측한 값
//   VectorXd y = z - z_prediction;
//   // 실제 센서 측정값과 방금 Predict 에서 예측한 값의 차이 구하기 

//   // 2. Kalman Gain 계산
//   // : 내가 예측한 값을 더 믿을까, 아니면 방금 들어온 센서 데이터를 더 믿을까? 를 결정하는 가중치 
//   MatrixXd S = H_ * P_ * H_.transpose() + R_;
//   MatrixXd K = P_ * H_.transpose() * S.inverse();

//   // 3. 새로운 상태 추정값 및 공분산 업데이트
//   x_ = x_ + K * y;
//   // 예측값 (x_) 에 오차 y를 적절한 비중 (K) 으로 곱한 후 더해줌 -> 최종 물체 위치 산출 ( x_ )
//   MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
//   P_ = (I - K * H_) * P_;
// }

// // ================================================================
// // 라이다 사용 x
// // ================================================================

// void KalmanFilter::UpdateEKF(const VectorXd &z) {
//   //predict what next measurement z should be
//   VectorXd z_prediction = MapToPolar(x_);

//   //calculate the difference between
//   //predicted and actual measurement
//   VectorXd y = z - z_prediction;
//   y(1) = Tools::NormalizeAngle(y(1));

//   // Calculate Kalman Game
//   MatrixXd S = H_ * P_ * H_.transpose() + R_;
//   MatrixXd K = P_ * H_.transpose() * S.inverse();

//   // update the state based on measurement
//   x_ = x_ + K * y;
//   // update the state covariance/uncertainty based on measurement
//   MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
//   P_ = (I - K * H_) * P_;
// }

// Eigen::VectorXd KalmanFilter::MapToPolar(const Eigen::VectorXd& x) {
//   Eigen::VectorXd z_predicted(3);

//   float px = x(0);
//   float py = x(1);
//   float vx = x(2);
//   float vy = x(3);

//   float px2_py2_sum = px * px + py * py;
//   float px2_py2_sum_sqrt = sqrt(px2_py2_sum);

//   z_predicted << px2_py2_sum_sqrt, atan2(py, px), ((px * vx + py * vy) / px2_py2_sum_sqrt);
//   return z_predicted;
// }