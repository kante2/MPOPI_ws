#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>

#include <fstream>
#include <cmath>

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

// ✅ 필요 없으면 주석 처리해도 됨 (헤더 경로 문제로 빌드 깨질 수 있음)
// #include "pure_pursuit/pure_pursuit_function.hpp"

// =====================================================
// Reference struct (WGS84 원점 + ECEF 원점)
// =====================================================
struct CoordinateReference {
    double lat0 = 0.0;
    double lon0 = 0.0;
    double h0   = 0.0;

    double x0_ecef = 0.0;
    double y0_ecef = 0.0;
    double z0_ecef = 0.0;
};

// =====================================================
// WGS84 -> ECEF
// =====================================================
static void wgs84ToECEF(double lat, double lon, double h,
                        double& x, double& y, double& z)
{
    const double a  = 6378137.0;
    const double e2 = 6.69437999014e-3;

    const double rad_lat = lat * M_PI / 180.0;
    const double rad_lon = lon * M_PI / 180.0;

    const double sin_lat = sin(rad_lat);
    const double cos_lat = cos(rad_lat);
    const double sin_lon = sin(rad_lon);
    const double cos_lon = cos(rad_lon);

    const double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);

    x = (N + h) * cos_lat * cos_lon;
    y = (N + h) * cos_lat * sin_lon;
    z = (N * (1.0 - e2) + h) * sin_lat;
}

// =====================================================
// WGS84 -> ENU (ref 기준)
// =====================================================
static void wgs84ToENU(double lat, double lon, double h,
                       const CoordinateReference& ref,
                       double& x, double& y, double& z)
{
    double x_ecef, y_ecef, z_ecef;
    wgs84ToECEF(lat, lon, h, x_ecef, y_ecef, z_ecef);

    const double dx = x_ecef - ref.x0_ecef;
    const double dy = y_ecef - ref.y0_ecef;
    const double dz = z_ecef - ref.z0_ecef;

    const double rad_lat0 = ref.lat0 * M_PI / 180.0;
    const double rad_lon0 = ref.lon0 * M_PI / 180.0;

    const double sin_lat0 = sin(rad_lat0);
    const double cos_lat0 = cos(rad_lat0);
    const double sin_lon0 = sin(rad_lon0);
    const double cos_lon0 = cos(rad_lon0);

    // ECEF -> ENU rotation matrix
    // [e]   [-sin(lon)           cos(lon)          0] [dx]
    // [n] = [-sin(lat)cos(lon)  -sin(lat)sin(lon)  cos(lat)] [dy]
    // [u]   [ cos(lat)cos(lon)   cos(lat)sin(lon)  sin(lat)] [dz]
    x = (-sin_lon0) * dx + (cos_lon0) * dy + 0.0 * dz;
    y = (-sin_lat0 * cos_lon0) * dx + (-sin_lat0 * sin_lon0) * dy + (cos_lat0) * dz;
    z = ( cos_lat0 * cos_lon0) * dx + ( cos_lat0 * sin_lon0) * dy + (sin_lat0) * dz;
}

// =====================================================
// Globals
// =====================================================
static std::ofstream record_file;
static double last_x = 0.0, last_y = 0.0;
static bool first_record = true;

static CoordinateReference g_ref;

// MORAI 오프셋 (환경에 맞게 수정)
static const double eastOffset  = 302459.942;
static const double northOffset = 4122635.537;

// UTM zone/band (네 환경이 52S 라고 적어서 그대로)
static const int  UTM_ZONE = 52;
static const char UTM_BAND = 'S';

static void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
    // 1) MORAI local -> UTM(+offset)
    const double utm_x = msg->position.x + eastOffset;
    const double utm_y = msg->position.y + northOffset;
    const double utm_z = msg->position.z;

    // 2) UTM -> WGS84 (GeoPoint)
    geodesy::UTMPoint utm_point;
    utm_point.easting  = utm_x;
    utm_point.northing = utm_y;
    utm_point.altitude = utm_z;
    utm_point.zone     = UTM_ZONE;
    utm_point.band     = UTM_BAND;

    const geographic_msgs::GeoPoint geo_point = geodesy::toMsg(utm_point);

    // 3) WGS84 -> ENU (ref 기준)
    double enu_x, enu_y, enu_z;
    wgs84ToENU(geo_point.latitude, geo_point.longitude, geo_point.altitude,
               g_ref, enu_x, enu_y, enu_z);

    // 4) 0.1m 간격 기록
    if (first_record) {
        record_file << enu_x << "," << enu_y << "," << enu_z << "\n";
        last_x = enu_x;
        last_y = enu_y;
        first_record = false;
        return;
    }

    const double dx = enu_x - last_x;
    const double dy = enu_y - last_y;
    const double dist = std::hypot(dx, dy);

    if (dist >= 0.1) {
        record_file << enu_x << "," << enu_y << "," << enu_z << "\n";
        last_x = enu_x;
        last_y = enu_y;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_recorder");
    ros::NodeHandle nh("~");

    // ===== params (필요하면 rosparam으로도 바꿀 수 있음) =====
    std::string ref_path, out_path, ego_topic;
    nh.param<std::string>("ref_path",  ref_path,  std::string("/home/autonav/MPOPI_ws/src/main/config/ref.txt"));
    nh.param<std::string>("out_path",  out_path,  std::string("/home/autonav/MPOPI_ws/src/main/config/path_new.csv"));
    nh.param<std::string>("ego_topic", ego_topic, std::string("/Ego_topic"));

    // ===== ref.txt load: lat0 lon0 h0 =====
    {
        std::ifstream ref_file(ref_path.c_str());
        if (!ref_file.is_open()) {
            ROS_ERROR("[path_recorder] Failed to open ref file: %s", ref_path.c_str());
            return 1;
        }
        ref_file >> g_ref.lat0 >> g_ref.lon0 >> g_ref.h0;
        ref_file.close();
    }

    // ===== ref ECEF origin =====
    wgs84ToECEF(g_ref.lat0, g_ref.lon0, g_ref.h0, g_ref.x0_ecef, g_ref.y0_ecef, g_ref.z0_ecef);

    // ===== output open =====
    record_file.open(out_path.c_str(), std::ios::out);
    if (!record_file.is_open()) {
        ROS_ERROR("[path_recorder] Failed to open output file: %s", out_path.c_str());
        return 1;
    }

    ROS_INFO("[path_recorder] ref loaded: lat0=%.9f lon0=%.9f h0=%.3f", g_ref.lat0, g_ref.lon0, g_ref.h0);
    ROS_INFO("[path_recorder] recording to: %s", out_path.c_str());
    ROS_INFO("[path_recorder] subscribe: %s", ego_topic.c_str());

    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 50, egoCallback);

    ros::spin();

    record_file.close();
    return 0;
}