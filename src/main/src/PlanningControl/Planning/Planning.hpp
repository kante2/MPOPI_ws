#pragma once
#include "Global.hpp"

// ========================================
// Lattice Planning Functions
// ========================================

void LatticePlanningProcess();
bool checkCostmapAvailable();
void findClosestWaypoint(const VehicleState& ego, int& out_idx);
void findLookaheadGoal(const VehicleState& ego, int close_idx, LatticeControl& lattice_ctrl);
void generateOffsetGoals(LatticeControl& lattice_ctrl);
void transformOffsetGoalsToBaselink(LatticeControl& lattice_ctrl, const VehicleState& ego);
void computeAllPolynomialPaths(LatticeControl& lattice_ctrl);
void sampleAllCandidatePaths(LatticeControl& lattice_ctrl);
void evaluateAllCandidates(LatticeControl& lattice_ctrl);
void selectBestPath(LatticeControl& lattice_ctrl);
void getTargetLocalPathIdx(LatticeControl& lattice_ctrl, double ld, int& out_idx);
void getMaxCurvature(int close_idx, int lookahead_idx, double& out_curvature);
void getTargetSpeed(double max_curvature, double& out_speed);


// ========================================
// Jamming Planning Functions
// ========================================

void JammingPlanningProcess();
void filterOffset(const LaneData& lane, Jamming_offset& offset, const JammingParams& jamming_params);
void computePurePursuitSteering(const LaneData& lane, Jamming_offset& offset, const JammingParams& jamming_params);
void computeOffsetPD(Jamming_offset& offset, const JammingParams& jamming_params);
// void computePID(const VehicleState& ego, const JammingParams& jamming_params);
void computePID(double& accel, double& brake, const VehicleState& ego,const JammingParams& jamming_params);
void publishCtrlCmd(double& accel, double& brake,const VehicleState& ego);



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