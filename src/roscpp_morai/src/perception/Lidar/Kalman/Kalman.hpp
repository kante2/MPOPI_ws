#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Global/Global.hpp>

class EKFTracker {
public:
    Eigen::VectorXd x; 
    Eigen::MatrixXd P; 
    int id;
    int miss_count;

    // 생성자와 업데이트 시 LidarCluster를 직접 참조
    EKFTracker(int id_val, const LidarCluster& cluster);
    void predict(double dt, const Eigen::MatrixXd& Q);
    void update(const LidarCluster& cluster, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);
};

class MultiObjectTracker {
public:
    std::vector<EKFTracker> tracks;
    // 추적 결과를 저장할 컨테이너 (출력용)
    std::vector<KalmanDetection> vec_KalmanDetections; 
    
    int next_id;
    Eigen::MatrixXd Q, R, H;
    double last_timestamp;

    MultiObjectTracker();
    void updateTracks(Lidar& st_Lidar, double current_time);
};

#endif