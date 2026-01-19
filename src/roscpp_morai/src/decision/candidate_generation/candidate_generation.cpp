#include "candidate.hpp"
#include "../coordinate/coordinate.hpp"
#include <cmath>
#include <algorithm>

// ========================================
// 1. 방향 벡터 계산
// ========================================

void calculatePathDirection(int goal_idx,
                           DirectionVector& out_dir) {
    out_dir.x = 0.0;
    out_dir.y = 0.0;
    
    if (goal_idx >= (int)waypoints.size() - 1) return;
    
    double dx = waypoints[goal_idx + 1].x - waypoints[goal_idx].x;
    double dy = waypoints[goal_idx + 1].y - waypoints[goal_idx].y;
    double len = sqrt(dx*dx + dy*dy);
    
    if (len > 1e-6) {
        out_dir.x = dx / len;
        out_dir.y = dy / len;
    }
}

void calculateNormalVector(const DirectionVector& dir,
                          DirectionVector& out_norm) {
    out_norm.x = -dir.y;
    out_norm.y = dir.x;
}

// ========================================
// 2. 목표점 생성 (Global)
// ========================================

void calculateOffsetGoalGlobal(double goal_ref_x, double goal_ref_y,
                              double offset,
                              const DirectionVector& norm,
                              Point2D& out_goal_global) {
    out_goal_global.x = goal_ref_x + offset * norm.x;
    out_goal_global.y = goal_ref_y + offset * norm.y;
}

void calculateGoalYawGlobal(const DirectionVector& dir,
                           double& out_yaw_global) {
    out_yaw_global = atan2(dir.y, dir.x);
}

// ========================================
// 3. 좌표 변환 (Global → Base_link)
// ========================================

void transformGoalToBaselink(const Point2D& goal_global,
                            double yaw_global,
                            const VehicleState& ego,
                            Point2D& out_goal_baselink,
                            double& out_yaw_baselink) {
    // 위치 변환
    mapToBaseLink(goal_global, ego, out_goal_baselink);
    
    // Yaw 변환
    globalYawToBaselink(yaw_global, ego, out_yaw_baselink);
}

// ========================================
// 4. 5차 다항식 계수 계산
// ========================================

void computePolynomialCoefficients(double X, double Y, double yaw_prime,
                                  PolynomialCoefficients& out_coeff) {
    // a0, a1, a2는 경계조건에 의해 0
    out_coeff.a0 = 0.0;
    out_coeff.a1 = 0.0;
    out_coeff.a2 = 0.0;
    
    // X가 너무 작으면 계산 불가
    if (fabs(X) < 1e-3) {
        out_coeff.a3 = 0.0;
        out_coeff.a4 = 0.0;
        out_coeff.a5 = 0.0;
        return;
    }
    
    double X2 = X*X, X3 = X2*X, X4 = X3*X, X5 = X4*X;
    
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    
    // y(X) = a3 X^3 + a4 X^4 + a5 X^5 = Y
    A(0,0) = X3;     A(0,1) = X4;      A(0,2) = X5;
    b(0)   = Y;
    
    // y'(X) = 3a3 X^2 + 4a4 X^3 + 5a5 X^4 = yaw_prime
    A(1,0) = 3.0*X2; A(1,1) = 4.0*X3;  A(1,2) = 5.0*X4;
    b(1)   = yaw_prime;
    
    // y''(X) = 6a3 X + 12a4 X^2 + 20a5 X^3 = 0
    A(2,0) = 6.0*X;  A(2,1) = 12.0*X2; A(2,2) = 20.0*X3;
    b(2)   = 0.0;
    
    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    out_coeff.a3 = sol(0);
    out_coeff.a4 = sol(1);
    out_coeff.a5 = sol(2);
}

// ========================================
// 5. 경로 샘플링
// ========================================

void samplePolynomialPath(const PolynomialCoefficients& coeff,
                         double X,
                         double sample_spacing,
                         std::vector<Point2D>& out_points) {
    out_points.clear();
    
    if (fabs(X) < 1e-6) {
        out_points.push_back({0.0, 0.0});
        return;
    }
    
    int num_points = (int)(fabs(X) / sample_spacing) + 1;
    if (num_points < 1) num_points = 1;
    
    for (int i = 0; i <= num_points; i++) {
        double x = X * ((double)i / (double)num_points);
        
        double y = coeff.a0 + 
                   coeff.a1 * x + 
                   coeff.a2 * x*x + 
                   coeff.a3 * x*x*x + 
                   coeff.a4 * x*x*x*x + 
                   coeff.a5 * x*x*x*x*x;
        
        out_points.push_back({x, y});
    }
}

// ========================================
// 6. 후보 경로 생성 (통합)
// ========================================

void generateAllCandidatePaths(const VehicleState& ego,
                              int goal_idx,
                              double goal_ref_x, double goal_ref_y,
                              std::vector<CandidatePath>& out_candidates) {
    out_candidates.clear();
    
    // ========================================
    // Step 1: 방향 벡터 계산
    // ========================================
    DirectionVector dir;
    calculatePathDirection(goal_idx, dir);
    
    // ========================================
    // Step 2: 법선 벡터 계산 (offset 방향)
    // ========================================
    DirectionVector norm;
    calculateNormalVector(dir, norm);
    
    // ========================================
    // Step 3: Global yaw 계산 (모든 offset 공통)
    // ========================================
    double yaw_global;
    calculateGoalYawGlobal(dir, yaw_global);
    
    // ========================================
    // Step 4: 여러 offset으로 후보 생성
    // ========================================
    for (int i = 0; i < planner_params.num_offsets; i++) {
        // 4-1. Offset 계산
        double offset = -planner_params.lateral_offset_step * 
                        (planner_params.num_offsets - 1) / 2.0 + 
                        planner_params.lateral_offset_step * i;
        
        // 4-2. Global 좌표에서 offset 적용한 목표점
        Point2D goal_global;
        calculateOffsetGoalGlobal(goal_ref_x, goal_ref_y, offset, 
                                 norm, goal_global);
        
        // 4-3. Base_link 좌표로 변환
        Point2D goal_baselink;
        double yaw_baselink;
        transformGoalToBaselink(goal_global, yaw_global, ego, 
                               goal_baselink, yaw_baselink);
        
        // 4-4. 5차 다항식 계수 계산
        double yaw_prime = tan(yaw_baselink);
        PolynomialCoefficients coeff;
        computePolynomialCoefficients(goal_baselink.x, goal_baselink.y, 
                                     yaw_prime, coeff);
        
        // 4-5. 경로 샘플링
        CandidatePath candidate;
        samplePolynomialPath(coeff, goal_baselink.x, 
                            planner_params.sample_spacing, 
                            candidate.points);
        
        // 4-6. Offset 저장
        candidate.offset = offset;
        
        // 4-7. 후보 추가
        out_candidates.push_back(candidate);
    }
}