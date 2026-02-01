#pragma once
#include "Global.hpp"

// ========================================
// Control Functions
// ========================================
void ControlProcess();
void getsteering(const VehicleState& ego, ControlData& ctrl);
void computePID(double vel, double target_vel, double& out_accel, double& out_brake);
void pubCmd(const ControlData& data);
double lateralPathError(int target_idx, double x, double y);
double headingError(double yaw, int target_idx);

