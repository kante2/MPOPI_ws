#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "../global.hpp"

// ========================================
// Costmap 접근
// ========================================

bool checkCostmapAvailable();

bool worldToCostmapCoord(double world_x, double world_y,
                        int& grid_x, int& grid_y);

int getCostmapCost(double world_x, double world_y);

#endif // COLLISION_HPP