#pragma once
#include "global.hpp"
///////////////테스트용 임시로 넣어둠 (라이다한테 토픽받고 그걸로 경로생성하는거 테스트하려고)///////////////
// entry
void LatticePlanningProcess();

// costmap ready
bool checkCostmapAvailable();

// lattice steps
void findClosestWaypoint(const VehicleState& ego, int& out_idx);
void findLookaheadGoal(const VehicleState& ego, int close_idx, int& out_target_idx, double& ld);

void generateOffsetGoals(int goal_idx, LatticeControl& ctrl);
void transformOffsetGoalsToBaselink(LatticeControl& ctrl, const VehicleState& ego);
void computeAllPolynomialPaths(LatticeControl& ctrl);
void sampleAllCandidatePaths(LatticeControl& ctrl);
void evaluateAllCandidates(LatticeControl& ctrl);
void selectBestPath(LatticeControl& ctrl);

// transforms & utils (어딘가에 구현돼 있어야 링크됨)
double quaternionToYaw(double x, double y, double z, double w);

void wgs84ToECEF(double lat, double lon, double h,
                 double& x_ecef, double& y_ecef, double& z_ecef);

void wgs84ToENU(double lat, double lon, double h,
                const CoordinateReference& ref,
                double& x_enu, double& y_enu, double& z_enu);

void baselinkToMap(const Point2D& base_pt, const VehicleState& ego, Point2D& map_pt);
void mapToBaseLink(const Point2D& map_pt, const VehicleState& ego, Point2D& base_pt);
void globalYawToBaselink(double yaw_global, const VehicleState& ego, double& yaw_base);

// costmap helpers (어딘가에 구현돼 있어야 링크됨)
bool worldToCostmapCoord(double wx, double wy, int& gx, int& gy);
int getCostmapCost(double wx, double wy);
