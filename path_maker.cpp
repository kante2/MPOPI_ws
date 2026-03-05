#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <morai_msgs/GPSMessage.h>
#define _USE_MATH_DEFINES

using namespace std;

struct EgoPose {
    double current_e;
    double current_n;
    double current_u;
};

//=================== 전역 변수 선언 ===================//
constexpr double pi = M_PI;
const double a = 6378137.0;           // WGS-84 타원체 장축 반경 (단위: m)
const double e = 0.006694379991;      // WGS-84 타원체 이심률 제곱

const double ref_lat = 37.238838359501933;
const double ref_lon = 126.772902206454901;
const double ref_alt = 0.0;

static double current_e = 0.0;
static double current_n = 0.0;

EgoPose egoPose = {0.0, 0.0};

// CSV 파일 저장용 전역 변수
ofstream csv_file;
bool csv_header_written = false;
int data_count = 0;

void Gps2EnuCB (const morai_msgs::GPSMessage::ConstPtr& msg) {
    double wgs_lat = msg->latitude;
    double wgs_lon = msg->longitude;
    double wgs_alt = msg->altitude;
    
    // WGS값 radian으로 변경(계산 목적)
    double rad_lat = wgs_lat * pi / 180;
    double rad_lon = wgs_lon * pi / 180;
    double rad_alt = wgs_alt * pi / 180;
    double k = a / sqrt(1-e*pow(sin(rad_lat), 2));

    // WGS->ECEF
    double ecef_x = k * cos(rad_lat) * cos(rad_lon);
    double ecef_y = k * cos(rad_lat) * sin(rad_lon);
    double ecef_z = k * (1-e) * sin(rad_lat);

    double ref_rad_lat = ref_lat * pi / 180;
    double ref_rad_lon = ref_lon * pi / 180;
    double ref_rad_alt = ref_alt * pi / 180;
    double ref_k = a / sqrt(1-e*pow(sin(ref_rad_lat), 2));

    double ref_ecef_x = ref_k * cos(ref_rad_lat) * cos(ref_rad_lon);
    double ref_ecef_y = ref_k * cos(ref_rad_lat) * sin(ref_rad_lon);
    double ref_ecef_z = ref_k * (1-e) * sin(ref_rad_lat);

    // ecef 좌표 enu 좌표로 변환
    egoPose.current_e = (-sin(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lon)*(ecef_y - ref_ecef_y));
    egoPose.current_n = (-sin(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) - sin(rad_lat)*sin(rad_lon)*(ecef_y - ref_ecef_y) + cos(rad_lat)*(ecef_z - ref_ecef_z));
    egoPose.current_u = (cos(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lat)*sin(rad_lon)*(ecef_y-ref_ecef_y) + sin(rad_lat)*(ecef_z-ref_ecef_z));

    // ✅ 터미널 출력 (기존)
    cout << fixed;
    cout.precision(4);
    cout << egoPose.current_e << " " 
         << egoPose.current_n << " " 
         << egoPose.current_u << endl;

    // ✅ CSV 파일에 저장 (신규)
    if (csv_file.is_open()) {
        // 헤더 작성 (첫 번째 데이터일 때만)
        if (!csv_header_written) {
            csv_file << "timestamp,east,north,up" << endl;
            csv_header_written = true;
            ROS_INFO("CSV header written!");
        }

        // 데이터 작성
        csv_file << fixed;
        csv_file.precision(4);
        csv_file << ros::Time::now().toSec() << ","
                 << egoPose.current_e << ","
                 << egoPose.current_n << ","
                 << egoPose.current_u << endl;
        
        csv_file.flush();  // 즉시 저장
        data_count++;
        
        // 100개마다 로그 출력
        if (data_count % 100 == 0) {
            ROS_INFO("[CSV] %d data points saved", data_count);
        }
    } else {
        ROS_WARN("CSV file is not open!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_maker");
    ros::NodeHandle nh;

    // ✅ CSV 파일 열기
    string csv_path = "/root/MPOPI_ws/src/main/config/track_log_MPOPI_ver.csv";
    csv_file.open(csv_path, ios::app);  // append 모드 (기존 데이터 뒤에 추가)
    
    if (!csv_file.is_open()) {
        ROS_ERROR("Failed to open CSV file: %s", csv_path.c_str());
        return -1;
    }
    
    ROS_INFO("=========================================");
    ROS_INFO("GPS to ENU Converter (with CSV logging)");
    ROS_INFO("Saving to: %s", csv_path.c_str());
    ROS_INFO("=========================================");

    ros::Subscriber gps_sub = nh.subscribe("/gps", 1, Gps2EnuCB);

    ros::spin();

    // ✅ 종료 시 파일 닫기
    if (csv_file.is_open()) {
        csv_file.close();
        ROS_INFO("=========================================");
        ROS_INFO("CSV file closed!");
        ROS_INFO("Total data points saved: %d", data_count);
        ROS_INFO("File path: %s", csv_path.c_str());
        ROS_INFO("=========================================");
    }

    return 0;
}