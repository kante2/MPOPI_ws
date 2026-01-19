#ifndef COST_HPP
#define COST_HPP

#include "../global.hpp"

// ========================================
// 비용 계산
// ========================================

void evaluatePathCollision(CandidatePath& path);

void calculatePathCurvature(const std::vector<Point2D>& points,
                           double& out_max_curvature);

void calculateOffsetCost(double offset, double& out_offset_cost);

void calculateOffsetChangeCost(double offset, double& out_change_cost);

void calculateTotalCost(CandidatePath& path);

void evaluateAllCandidates(std::vector<CandidatePath>& candidates);

void selectBestPath(const std::vector<CandidatePath>& candidates,
                   CandidatePath& out_best_path);

#endif // COST_HPP