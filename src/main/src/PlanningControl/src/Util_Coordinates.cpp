#include "global.hpp"
#include <cmath>
#include <algorithm>

using namespace std;

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
// Map -> Base_link 변환
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

// ========================================
// Base_link -> costmap 변환
// ========================================

bool BaseLinkToCostmap(const Point2D& pt_bl,
                         int& grid_x, int& grid_y)
{
    if (!checkCostmapAvailable()) return false;

    const auto& cm = *costmap_info.msg;

    // resolution 0 보호
    if (cm.info.resolution <= 1e-9) return false;

    grid_x = (int)std::floor((pt_bl.x - cm.info.origin.position.x) / cm.info.resolution);
    grid_y = (int)std::floor((pt_bl.y - cm.info.origin.position.y) / cm.info.resolution);

    if (grid_x < 0 || grid_x >= (int)cm.info.width ||
        grid_y < 0 || grid_y >= (int)cm.info.height) {
        return false;
    }

    return true;
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

// ========================================
// 거리 계산
// ========================================

double distance2D(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx*dx + dy*dy);
}

double distance2D(const Point2D& p1, const Point2D& p2) {
    return distance2D(p1.x, p1.y, p2.x, p2.y);
}

// ========================================
// 각도 변환
// ========================================

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// ========================================
// Clamp
// ========================================

double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

int clamp(int value, int min_val, int max_val) {
    return std::max(min_val, std::min(max_val, value));
}