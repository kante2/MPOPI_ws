
void LatticePlanningProcess() {

    if (!checkCostmapReady()) {
        ROS_WARN_THROTTLE(1.0, "[Lattice] Waiting for costmap...");
        return;
    }

    findClosestWaypoint(ego, ctrl.close_idx);
    findLookaheadGoal(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
    
    double goal_ref_x = waypoints[ctrl.target_idx].x;
    double goal_ref_y = waypoints[ctrl.target_idx].y;

    std::vector<CandidatePath> candidates;
    generateAllCandidatePaths(ego, ctrl.target_idx, 
                             goal_ref_x, goal_ref_y, 
                             candidates);
    evaluateAllCandidates(candidates);

    CandidatePath best_path;
    selectBestPath(candidates, best_path);

    if (best_path.valid && !best_path.points.empty()) {
        nav_msgs::Path local_path;
        convertToNavPath(best_path, local_path);
        local_path_pub.publish(local_path);
        
        visualization_msgs::MarkerArray best_markers;
        visualizeBestPath(best_path, best_markers);
        candidate_paths_pub.publish(best_markers);
        
        ROS_INFO("[Lattice] Best: offset=%.2f, cost=%.4f", 
                 best_path.offset, best_path.cost);
    } else {
        ROS_WARN("[Lattice] No valid path!");
    }
    
    // Visualize all candidates
    visualization_msgs::MarkerArray all_markers;
    visualizeCandidatePaths(candidates, all_markers);
    candidate_paths_pub.publish(all_markers);
}
