#pragma once
#include "global.hpp"

// ========================================
// Lattice Planning Functions
// ========================================

// Entry point
void LatticePlanningProcess();

// Costmap ready check
bool checkCostmapAvailable();

// Lattice steps
void findClosestWaypoint(LatticeControl& lattice_ctrl, const VehicleState& ego); 
void findLookaheadGoal(LatticeControl& lattice_ctrl, const VehicleState& ego);   
void generateOffsetGoals(LatticeControl& lattice_ctrl, const VehicleState& ego);  
void computeAllPolynomialPaths(LatticeControl& ctrl);
void sampleAllCandidatePaths(LatticeControl& ctrl);
void evaluateAllCandidates(LatticeControl& ctrl);
void selectBestPath(LatticeControl& ctrl);

// ========================================
// Coordinate Transform Functions
// ========================================

double quaternionToYaw(double x, double y, double z, double w);

void wgs84ToECEF(double lat, double lon, double h,
                 double& x_ecef, double& y_ecef, double& z_ecef);

void wgs84ToENU(double lat, double lon, double h,
                const CoordinateReference& ref,
                double& x_enu, double& y_enu, double& z_enu);

void mapToBaseLink(const Point2D& map_pt, const VehicleState& ego, Point2D& base_pt);
void BaseLinkToMap(const Point2D& bl_pt, Point2D& map_pt); 
void globalYawToBaselink(double yaw_global, const VehicleState& ego, double& yaw_base);

// ========================================
// Costmap Helper Functions
// ========================================

bool BaseLinkToCostmap(const Point2D& pt_bl, int& grid_x, int& grid_y);
int getCostmapCostFromGrid(int grid_x, int grid_y);