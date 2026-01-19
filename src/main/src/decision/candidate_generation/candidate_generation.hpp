#ifndef CANDIDATE_HPP
#define CANDIDATE_HPP

#include "../global.hpp"

// ========================================
// 1. 방향 벡터 계산
// ========================================

void calculatePathDirection(int goal_idx,
                           DirectionVector& out_dir);

void calculateNormalVector(const DirectionVector& dir,
                          DirectionVector& out_norm);

// ========================================
// 2. 목표점 생성 (Global)
// ========================================

void calculateOffsetGoalGlobal(double goal_ref_x, double goal_ref_y,
                              double offset,
                              const DirectionVector& norm,
                              Point2D& out_goal_global);

void calculateGoalYawGlobal(const DirectionVector& dir,
                           double& out_yaw_global);

// ========================================
// 3. 좌표 변환 (Global → Base_link)
// ========================================

void transformGoalToBaselink(const Point2D& goal_global,
                            double yaw_global,
                            const VehicleState& ego,
                            Point2D& out_goal_baselink,
                            double& out_yaw_baselink);

// ========================================
// 4. 5차 다항식 계수 계산
// ========================================

void computePolynomialCoefficients(double X, double Y, double yaw_prime,
                                  PolynomialCoefficients& out_coeff);

// ========================================
// 5. 경로 샘플링
// ========================================

void samplePolynomialPath(const PolynomialCoefficients& coeff,
                         double X,
                         double sample_spacing,
                         std::vector<Point2D>& out_points);

// ========================================
// 6. 후보 경로 생성 (통합)
// ========================================

void generateAllCandidatePaths(const VehicleState& ego,
                              int goal_idx,
                              double goal_ref_x, double goal_ref_y,
                              std::vector<CandidatePath>& out_candidates);

#endif // CANDIDATE_HPP