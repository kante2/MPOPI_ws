#pragma once
#include "global.hpp"

// ========================================
// Lattice Planning Functions
// ========================================

void LatticePlanningProcess();
bool checkCostmapAvailable();
void findClosestWaypoint(const VehicleState& ego, int& out_idx);
void findLookaheadGoal(const VehicleState& ego, int close_idx, int& out_target_idx, double& ld);
void generateOffsetGoals(int goal_idx, LatticeControl& lattice_ctrl);
void transformOffsetGoalsToBaselink(LatticeControl& lattice_ctrl, const VehicleState& ego);
void computeAllPolynomialPaths(LatticeControl& lattice_ctrl);
void sampleAllCandidatePaths(LatticeControl& lattice_ctrl);
void evaluateAllCandidates(LatticeControl& lattice_ctrl);
void selectBestPath(LatticeControl& lattice_ctrl);
void getTargetLocalPathIdx(LatticeControl& lattice_ctrl, double ld, int& out_idx);
void getMaxCurvature(int close_idx, int lookahead_idx, double& out_curvature);
void getTargetSpeed(double max_curvature, double& out_speed);


// ========================================
// Control Functions
// ========================================
void ControlProcess();
void getsteering(const VehicleState& ego, ControlData& ctrl);
void computePID(double vel, double target_vel, double& out_accel, double& out_brake);
void pubCmd(const ControlData& data);
double lateralPathError(int target_idx, double x, double y);
double headingError(double yaw, int target_idx);


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

bool worldToCostmapCoord(double world_x, double world_y, int& grid_x, int& grid_y);
int getCostmapCost(double world_x, double world_y);