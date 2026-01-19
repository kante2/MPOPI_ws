#include "collision.hpp"
#include <cmath>

bool checkCostmapAvailable() {
    return (costmap_info.data != nullptr);
}

bool worldToCostmapCoord(double world_x, double world_y,
                        int& grid_x, int& grid_y) {
    if (!checkCostmapAvailable()) return false;
    
    grid_x = (int)std::floor((world_x - costmap_info.origin_x) / 
                             costmap_info.resolution);
    grid_y = (int)std::floor((world_y - costmap_info.origin_y) / 
                             costmap_info.resolution);
    
    if (grid_x < 0 || grid_x >= (int)costmap_info.data->info.width ||
        grid_y < 0 || grid_y >= (int)costmap_info.data->info.height) {
        return false;
    }
    
    return true;
}

int getCostmapCost(double world_x, double world_y) {
    if (!checkCostmapAvailable()) return 0;
    
    int grid_x, grid_y;
    if (!worldToCostmapCoord(world_x, world_y, grid_x, grid_y)) {
        return (int)planner_params.lethal_cost_threshold;
    }
    
    int idx = grid_y * (int)costmap_info.data->info.width + grid_x;
    if (idx < 0 || idx >= (int)costmap_info.data->data.size()) {
        return (int)planner_params.lethal_cost_threshold;
    }
    
    int8_t raw = costmap_info.data->data[idx];
    
    if (raw < 0) return 30; // Unknown -> medium cost
    
    int cost = (int)raw;
    if (cost < 0) cost = 0;
    if (cost > 100) cost = 100;
    
    return cost;
}