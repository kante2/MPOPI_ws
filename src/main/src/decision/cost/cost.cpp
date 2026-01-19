#include "cost.hpp"
#include "../collision/collision.hpp"
#include <cmath>
#include <algorithm>

void evaluatePathCollision(CandidatePath& path) {
    path.obstacle_cost = 0.0;
    path.valid = true;
    
    int lethal_count = 0;
    int valid_point_count = 0;
    
    for (const auto& pt : path.points) {
        int grid_x, grid_y;
        
        if (!worldToCostmapCoord(pt.x, pt.y, grid_x, grid_y)) {
            continue;
        }
        
        valid_point_count++;
        
        int cost = getCostmapCost(pt.x, pt.y);
        
        if (cost >= (int)planner_params.lethal_cost_threshold) {
            lethal_count++;
        }
        
        path.obstacle_cost += cost / 100.0;
    }
    
    if (lethal_count > 0) {
        path.valid = false;
        path.cost = 1e10;
        return;
    }

    if (valid_point_count > 0) {
        path.obstacle_cost /= valid_point_count;
    }
}

void calculatePathCurvature(const std::vector<Point2D>& points,
                           double& out_max_curvature) {
    out_max_curvature = 0.0;
    
    if (points.size() < 3) return;
    
    for (size_t i = 1; i < points.size() - 1; i++) {
        double x0 = points[i-1].x, y0 = points[i-1].y;
        double x1 = points[i].x, y1 = points[i].y;
        double x2 = points[i+1].x, y2 = points[i+1].y;
        
        double dx1 = x1 - x0, dy1 = y1 - y0;
        double dx2 = x2 - x1, dy2 = y2 - y1;
        
        double curvature = fabs((dx1*dy2 - dy1*dx2) / 
                          (sqrt(dx1*dx1 + dy1*dy1) * sqrt(dx2*dx2 + dy2*dy2) + 1e-6));
        
        out_max_curvature = std::max(out_max_curvature, curvature);
    }
}

void calculateOffsetCost(double offset, double& out_offset_cost) {
    out_offset_cost = fabs(offset) * 0.5;
}

void calculateOffsetChangeCost(double offset, double& out_change_cost) {
    out_change_cost = fabs(offset - last_selected_offset) * 0.3;
}

void calculateTotalCost(CandidatePath& path) {
    if (!path.valid) {
        path.cost = 1e10;
        return;
    }
    
    double offset_change_cost;
    calculateOffsetChangeCost(path.offset, offset_change_cost);
    
    path.cost = path.obstacle_cost * 100.0 +
                path.offset_cost +
                path.curvature_cost * 10.0 +
                offset_change_cost;
}

void evaluateAllCandidates(std::vector<CandidatePath>& candidates) {
    for (auto& candidate : candidates) {
        // 충돌 평가
        evaluatePathCollision(candidate);
        
        if (candidate.valid) {
            // 곡률 비용
            calculatePathCurvature(candidate.points, candidate.curvature_cost);
            
            // Offset 비용
            calculateOffsetCost(candidate.offset, candidate.offset_cost);
            
            // 총 비용
            calculateTotalCost(candidate);
        }
    }
}

void selectBestPath(const std::vector<CandidatePath>& candidates,
                   CandidatePath& out_best_path) {
    if (candidates.empty()) {
        out_best_path.valid = true;
        out_best_path.offset = 0.0;
        return;
    }
    
    double best_cost = 1e10;
    int best_idx = 0;
    
    for (int i = 0; i < (int)candidates.size(); i++) {
        if (candidates[i].cost < best_cost) {
            best_cost = candidates[i].cost;
            best_idx = i;
        }
    }
    
    out_best_path = candidates[best_idx];
    last_selected_offset = out_best_path.offset;
}