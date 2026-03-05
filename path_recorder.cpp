#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <fstream>
#include <cmath>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include "PlanningControl/Global/Global.hpp"

// 외부 함수 선언 (Util_Coordinates.cpp에서 구현)
extern void wgs84ToECEF(double lat, double lon, double h, double& x, double& y, double& z);
extern void wgs84ToENU(double lat, double lon, double h, const CoordinateReference& ref, 
                       double& x, double& y, double& z);

std::ofstream record_file;
double last_x = 0.0, last_y = 0.0;
bool first_record = true;

// 기준점 좌표는 Global.cpp의 외부 coord_ref 사용
extern CoordinateReference coord_ref;

// MORAI 오프셋 (환경에 맞게 수정)
const double eastOffset  = 302459.942;
const double northOffset = 4122635.537;

void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    // 1. UTM + offset
    double utm_x = msg->position.x + eastOffset;
    double utm_y = msg->position.y + northOffset;
    double utm_z = msg->position.z;

    // 2. UTM → WGS84
    geodesy::UTMPoint utm_point;
    utm_point.easting  = utm_x;
    utm_point.northing = utm_y;
    utm_point.altitude = utm_z;
    utm_point.zone     = 52;
    utm_point.band     = 'S';

    geographic_msgs::GeoPoint geo_point = geodesy::toMsg(utm_point);

    // 3. WGS84 → ENU
    double enu_x, enu_y, enu_z;
    wgs84ToENU(geo_point.latitude, geo_point.longitude, geo_point.altitude,
               coord_ref, enu_x, enu_y, enu_z);

    // 4. 0.1m 간격 기록
    if (first_record) {
        record_file << enu_x << "," << enu_y << "," << enu_z << "\n";
        last_x = enu_x;
        last_y = enu_y;
        first_record = false;
        return;
    }

    double dx = enu_x - last_x;
    double dy = enu_y - last_y;
    double dist = std::hypot(dx, dy);

    if (dist >= 0.1) {
        record_file << enu_x << "," << enu_y << "," << enu_z << "\n";
        last_x = enu_x;
        last_y = enu_y;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_recorder");
    ros::NodeHandle nh;

    std::ifstream ref_file("/root/MPOPI_ws/src/main/config/ref.txt");
    ref_file >> coord_ref.lat0 >> coord_ref.lon0 >> coord_ref.h0;
    ref_file.close();
    wgs84ToECEF(coord_ref.lat0, coord_ref.lon0, coord_ref.h0, 
                coord_ref.x0_ecef, coord_ref.y0_ecef, coord_ref.z0_ecef);

    record_file.open("/root/MPOPI_ws/src/main/config/ref_0305.txt");

    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 50, egoCallback);

    ros::spin();

    record_file.close();
    return 0;
}
