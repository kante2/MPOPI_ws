void LatticePlanningProcess() {
    
    if (!checkCostmapReady()) {
        ROS_WARN_THROTTLE(1.0, "[Lattice] Waiting for costmap...");
        return;
    }
    
    findClosestWaypoint(ego, ctrl.close_idx);
    findLookaheadGoal(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
    generateOffsetGoals(ctrl.target_idx, ctrl);
    transformOffsetGoalsToBaselink(ctrl, ego);
    computeAllPolynomialPaths(ctrl);
    sampleAllCandidatePaths(ctrl);
    evaluateAllCandidates(ctrl);
    selectBestPath(ctrl);
}

// ========================================
// 코스트맵 잘 들어왔는지 체크
// ========================================

bool checkCostmapAvailable() {
    return (costmap_info.data != nullptr);
}
// ========================================
// 오프셋 후보 생성
// ========================================
void generateOffsetGoals(int goal_idx, Control& ctrl) {
    ctrl.offset_goals.clear();
    
    double goal_ref_x = waypoints[goal_idx].x;
    double goal_ref_y = waypoints[goal_idx].y;
    
    // 방향 벡터 계산
    double dx = 0.0, dy = 0.0;
    if (goal_idx < (int)waypoints.size() - 1) {
        dx = waypoints[goal_idx + 1].x - waypoints[goal_idx].x;
        dy = waypoints[goal_idx + 1].y - waypoints[goal_idx].y;
    }
    
    double len = sqrt(dx*dx + dy*dy);
    if (len < 1e-6) return;
    
    // 단위 방향 벡터
    double dir_x = dx / len;
    double dir_y = dy / len;
    
    // 법선 벡터 (좌측이 양수)
    double norm_x = -dir_y;
    double norm_y = dir_x;
    
    // Global yaw 계산
    double yaw_global = atan2(dir_y, dir_x);
    
    // 여러 offset으로 목표점 생성
    for (int i = 0; i < planner_params.num_offsets; i++) {
        double offset = -planner_params.lateral_offset_step * 
                        (planner_params.num_offsets - 1) / 2.0 + 
                        planner_params.lateral_offset_step * i;
        
        OffsetGoal goal;
        goal.global_x = goal_ref_x + offset * norm_x;
        goal.global_y = goal_ref_y + offset * norm_y;
        goal.global_yaw = yaw_global;
        goal.offset = offset;
        
        ctrl.offset_goals.push_back(goal);
    }
}

// ========================================
// Baselink 변환
// ========================================
void transformOffsetGoalsToBaselink(Control& ctrl, const VehicleState& ego) {
    ctrl.baselink_goals.clear();
    
    for (const auto& goal : ctrl.offset_goals) {
        Point2D global_pt = {goal.global_x, goal.global_y};
        
        BaselinkGoal bl_goal;
        mapToBaseLink(global_pt, ego, bl_goal.point);
        globalYawToBaselink(goal.global_yaw, ego, bl_goal.yaw);
        bl_goal.offset = goal.offset;
        
        ctrl.baselink_goals.push_back(bl_goal);
    }
}

// ========================================
// 다항식 계수 계산 (모든 목표점에 대해)
// ========================================
void computeAllPolynomialPaths(Control& ctrl) {
    ctrl.coefficients.clear();
    
    for (const auto& bl_goal : ctrl.baselink_goals) {
        PolynomialCoefficients coeff;
        
        // 경계조건: y(0)=0, y'(0)=0, y''(0)=0
        coeff.a0 = 0.0;
        coeff.a1 = 0.0;
        coeff.a2 = 0.0;
        
        double X = bl_goal.point.x;
        double Y = bl_goal.point.y;
        
        if (fabs(X) < 1e-3) {
            coeff.a3 = 0.0;
            coeff.a4 = 0.0;
            coeff.a5 = 0.0;
        } else {
            double yaw_prime = tan(bl_goal.yaw);
            double X2 = X*X, X3 = X2*X, X4 = X3*X, X5 = X4*X;
            
            Eigen::Matrix3d A;
            Eigen::Vector3d b;
            
            A(0,0) = X3;     A(0,1) = X4;      A(0,2) = X5;
            b(0)   = Y;
            
            A(1,0) = 3.0*X2; A(1,1) = 4.0*X3;  A(1,2) = 5.0*X4;
            b(1)   = yaw_prime;
            
            A(2,0) = 6.0*X;  A(2,1) = 12.0*X2; A(2,2) = 20.0*X3;
            b(2)   = 0.0;
            
            Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
            coeff.a3 = sol(0);
            coeff.a4 = sol(1);
            coeff.a5 = sol(2);
        }
        
        ctrl.coefficients.push_back(coeff);
    }
}

// ========================================
// 경로 샘플링 (모든 다항식에 대해)
// ========================================
void sampleAllCandidatePaths(Control& ctrl) {
    ctrl.candidates.clear();
    
    for (size_t i = 0; i < ctrl.coefficients.size(); i++) {
        const auto& coeff = ctrl.coefficients[i];
        const auto& bl_goal = ctrl.baselink_goals[i];
        
        CandidatePath candidate;
        candidate.offset = bl_goal.offset;
        
        double X = bl_goal.point.x;
        
        if (fabs(X) < 1e-6) {
            candidate.points.push_back({0.0, 0.0});
        } else {
            int num_points = (int)(fabs(X) / planner_params.sample_spacing) + 1;
            if (num_points < 1) num_points = 1;
            
            for (int j = 0; j <= num_points; j++) {
                double x = X * ((double)j / (double)num_points);
                
                double y = coeff.a0 + 
                           coeff.a1 * x + 
                           coeff.a2 * x*x + 
                           coeff.a3 * x*x*x + 
                           coeff.a4 * x*x*x*x + 
                           coeff.a5 * x*x*x*x*x;
                
                candidate.points.push_back({x, y});
            }
        }
        
        ctrl.candidates.push_back(candidate);
    }
}

// ========================================
// 경로 평가
// ========================================
void evaluateAllCandidates(Control& ctrl) {
    for (auto& path : ctrl.candidates) {
        // 초기화
        path.obstacle_cost = 0.0;
        path.curvature_cost = 0.0;
        path.offset_cost = 0.0;
        path.valid = true;
        
        // 1. 충돌 평가
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
            continue;
        }
        
        if (valid_point_count > 0) {
            path.obstacle_cost /= valid_point_count;
        }

        // 2. 곡률 비용
        if (path.points.size() >= 3) {
            for (size_t i = 1; i < path.points.size() - 1; i++) {
                double x0 = path.points[i-1].x, y0 = path.points[i-1].y;
                double x1 = path.points[i].x, y1 = path.points[i].y;
                double x2 = path.points[i+1].x, y2 = path.points[i+1].y;
                
                double dx1 = x1 - x0, dy1 = y1 - y0;
                double dx2 = x2 - x1, dy2 = y2 - y1;
                
                double curvature = fabs((dx1*dy2 - dy1*dx2) / 
                                  (sqrt(dx1*dx1 + dy1*dy1) * sqrt(dx2*dx2 + dy2*dy2) + 1e-6));
                
                path.curvature_cost = std::max(path.curvature_cost, curvature);
            }
        }

        // 3. Offset 비용
        path.offset_cost = fabs(path.offset) * 0.5;

        // 4. Offset 변화 비용
        double offset_change_cost = fabs(path.offset - last_selected_offset) * 0.3;

        // 5. 총 비용
        path.cost = path.obstacle_cost * 100.0 +
                    path.offset_cost +
                    path.curvature_cost * 10.0 +
                    offset_change_cost;
    }
}

// ========================================
// 최적 경로 선택
// ========================================
void selectBestPath(Control& ctrl) {
    if (ctrl.candidates.empty()) {
        ctrl.best_path.valid = true;
        ctrl.best_path.offset = 0.0;
        return;
    }
    
    double best_cost = 1e10;
    int best_idx = 0;
    
    for (int i = 0; i < (int)ctrl.candidates.size(); i++) {
        if (ctrl.candidates[i].cost < best_cost) {
            best_cost = ctrl.candidates[i].cost;
            best_idx = i;
        }
    }
    
    ctrl.best_path = ctrl.candidates[best_idx];
    last_selected_offset = ctrl.best_path.offset;
}