#include <iostream>
#include "Eigen/Dense"
#include "FusionEKF.h"

#include "division_by_zero_exception.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  //assumed noise values for prediction step
  //used to update process covariance matrix
  noise_ax_ = 9;
  noise_ay_ = 9;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;


  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = Eigen::VectorXd(4);

  //state covariance matrix P
  ekf_.P_ = Eigen::MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  //the state transition vector F in initial form
  ekf_.F_ = Eigen::MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1 , 0,
      0, 0, 0, 1;

  //process covariance matrix Q, we don't know values of deltaT yet
  ekf_.Q_ = Eigen::MatrixXd(4, 4);
  ekf_.Q_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      1, 0, 1, 0,
      0, 1, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization (물체 초기 위치 설정, 속도 ~ 0 으로 초기화)
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    //first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //Convert radar from polar to cartesian coordinates and initialize state.
      ekf_.x_ = MapToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    this->previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  UpdatePredictionMatrices(measurement_pack.timestamp_);
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    try {
      Hj_ = Tools::CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } catch(DivisionByZeroException & error) {

    }
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

// ===========================================================
// 현재 칼만 필터가 추측하고 있는 물체의 최종 결과값 (상태 벡터 x : 물체 위치 (px, py), 속도 (vx, vy)) 를 가져오는 함수
// ===========================================================

Eigen::VectorXd FusionEKF::GetEstimations() const {
  return ekf_.x_;
}

// ===========================================================
// 라이다는 사용x
// ===========================================================

Eigen::VectorXd FusionEKF::MapToCartesian(const Eigen::VectorXd& z) {
  float rho = z(0);
  float phi = z(1);
  float rho_dot = z(2);

  //calculate px and py which is basically project of rho
  //on x and y axis.
  float px = rho * cos(phi);
  float py = rho * sin(phi);
  //calculate vx and vy by projecting rho_dot onto x and y axis
  float vx = rho_dot * cos(phi);
  float vy = rho_dot * sin(phi);

  Eigen::VectorXd cartesian(4);
  cartesian << px, py, vx, vy;

  return cartesian;
}


// ===========================================================
// kalman filter의 predict 단계 직전에 행렬들을 실시간으로 업데이트
// ===========================================================

void FusionEKF::UpdatePredictionMatrices(long current_timestamp) {
  // 이전 측정 시간과 지금 측정 시간의 차이 (dt) 계산
  // -> 칼만 필터는 시간이 얼마나 흘렀느냐에 따라 움직임을 예측하기 때문에 dt가 매우 중요
  float dt = (current_timestamp - previous_timestamp_) / 1000000.0;

  // 상태 전이 행렬 F 업데이트
  // -> 위치는 속도 x  시간만큼 변한다는 공식을 행렬에 적어주는 것
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // 노이즈 행렬 Q 업데이트
  // -> 시간 (dt) 이 길어질수록 예측의 불확실성은 커지기 때문에
  //    가속도 노이즈 (noise_ax, noise_ay)를 사용해서 시간이 흐름에 따라 예측이 얼마나 부정확해질 수 있는지를 Q 행렬에 채워넣는 과정
  ekf_.Q_(0, 0) = (pow(dt, 4) / 4) * noise_ax_;
  ekf_.Q_(0, 2) = (pow(dt, 3) / 2) * noise_ax_;
  ekf_.Q_(1, 1) = (pow(dt, 4) / 4) * noise_ay_;
  ekf_.Q_(1, 3) = (pow(dt, 3) / 2) * noise_ay_;
  ekf_.Q_(2, 0) = (pow(dt, 3) / 2) * noise_ax_;
  ekf_.Q_(2, 2) = (pow(dt, 2) * noise_ax_);
  ekf_.Q_(3, 1) = (pow(dt, 3) / 2) * noise_ay_;
  ekf_.Q_(3, 3) = (pow(dt, 2) * noise_ay_);
}
// 1. 라이다 데이터가 들어올 때마다 current_timestamp 를 받아 dt를 구하고, 
// 2. dt 를 이용해 UpdatePredictionMatrices() 으로 F, Q 행렬을 매번 새로 계산
// 3. ekf_. Predict() 호출