#include "coordinate.hpp"
#include <cmath>

// ========================================
// GPS 변환
// ========================================

void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z) {
    const double a = 6378137.0;
    const double e2 = 6.69437999014e-3;

    double rad_lat = lat * M_PI / 180.0;
    double rad_lon = lon * M_PI / 180.0;
    double N = a / sqrt(1.0 - e2 * sin(rad_lat) * sin(rad_lat));

    x = (N + h) * cos(rad_lat) * cos(rad_lon);
    y = (N + h) * cos(rad_lat) * sin(rad_lon);
    z = (N * (1.0 - e2) + h) * sin(rad_lat);
}

void wgs84ToENU(double lat, double lon, double h,
                const CoordinateReference& ref,
                double& x, double& y, double& z) {
    double x_ecef, y_ecef, z_ecef;
    wgs84ToECEF(lat, lon, h, x_ecef, y_ecef, z_ecef);

    double dx = x_ecef - ref.x0_ecef;
    double dy = y_ecef - ref.y0_ecef;
    double dz = z_ecef - ref.z0_ecef;

    double rad_lat = ref.lat0 * M_PI / 180.0;
    double rad_lon = ref.lon0 * M_PI / 180.0;

    double t[3][3] = {
        {-sin(rad_lon), cos(rad_lon), 0},
        {-sin(rad_lat) * cos(rad_lon), -sin(rad_lat) * sin(rad_lon), cos(rad_lat)},
        {cos(rad_lat) * cos(rad_lon), cos(rad_lat) * sin(rad_lon), sin(rad_lat)}
    };

    x = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz;
    y = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz;
    z = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz;
}

double quaternionToYaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return atan2(siny_cosp, cosy_cosp);
}

// ========================================
// Map ↔ Base_link 변환
// ========================================

void mapToBaseLink(const Point2D& map_point, 
                   const VehicleState& ego,
                   Point2D& out_baselink) {
    double dx = map_point.x - ego.x;
    double dy = map_point.y - ego.y;

    double c = cos(ego.yaw);
    double s = sin(ego.yaw);

    out_baselink.x =  c * dx + s * dy;
    out_baselink.y = -s * dx + c * dy;
}

void baselinkToMap(const Point2D& baselink_point,
                   const VehicleState& ego,
                   Point2D& out_map) {
    double c = cos(ego.yaw);
    double s = sin(ego.yaw);

    out_map.x = ego.x + c * baselink_point.x - s * baselink_point.y;
    out_map.y = ego.y + s * baselink_point.x + c * baselink_point.y;
}

// ========================================
// 각도 유틸
// ========================================

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void globalYawToBaselink(double yaw_global,
                        const VehicleState& ego,
                        double& out_yaw_baselink) {
    out_yaw_baselink = normalizeAngle(yaw_global - ego.yaw);
}

void baselinkYawToGlobal(double yaw_baselink,
                        const VehicleState& ego,
                        double& out_yaw_global) {
    out_yaw_global = normalizeAngle(ego.yaw + yaw_baselink);
}