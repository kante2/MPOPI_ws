#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include "../global.hpp"

// ========================================
// GPS/IMU 좌표 변환
// ========================================

void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z);

void wgs84ToENU(double lat, double lon, double h,
                const CoordinateReference& ref,
                double& x, double& y, double& z);

double quaternionToYaw(double x, double y, double z, double w);

// ========================================
// Map ↔ Base_link 변환
// ========================================

void mapToBaseLink(const Point2D& map_point, 
                   const VehicleState& ego,
                   Point2D& out_baselink);

void baselinkToMap(const Point2D& baselink_point,
                   const VehicleState& ego,
                   Point2D& out_map);

// ========================================
// 각도 유틸
// ========================================

double normalizeAngle(double angle);

void globalYawToBaselink(double yaw_global,
                        const VehicleState& ego,
                        double& out_yaw_baselink);

void baselinkYawToGlobal(double yaw_baselink,
                        const VehicleState& ego,
                        double& out_yaw_global);

#endif // COORDINATE_HPP