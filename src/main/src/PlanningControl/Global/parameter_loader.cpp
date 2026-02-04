#include "parameter_loader.hpp"
#include "Global.hpp"
#include "Planning.hpp"

#include <ros/ros.h>
#include <fstream>   
#include <sstream>     
#include <string>       

void initializePlannerParameters() {
    planner_params.num_offsets = 13; // ** 9 -> 13
    planner_params.lateral_offset_step = 1.0; // ** 
    planner_params.sample_spacing = 0.1;
    planner_params.lethal_cost_threshold = 90.0;
    planner_params.vehicle_front_offset = 4.0;
    
    ROS_INFO("[Planner] Initialized with %d offsets, step: %.2fm", 
             planner_params.num_offsets, 
             planner_params.lateral_offset_step);
}

void initializeControlParameters() {
    Kp = 0.2; // 
    Ki = 0.0;
    Kd = 0.0;
    k_gain = 1.0;
    curve_standard = 0.0025;
    curve_vel = 30.0 / 3.6;
    target_vel = 70.0 / 3.6;
    
    ROS_INFO("[Control] Target vel: %.1f km/h", target_vel * 3.6);
}

bool loadWaypoints() {
    waypoints.clear();

    const std::string ref_file_name  = "src/main/config/ref.txt";
    const std::string path_file_name = "src/main/config/track_log_recorded_final.csv";

    // -------------------------
    // 1) 기준점(ref.txt) 읽기
    // -------------------------
    ROS_INFO("Opening ref file: %s", ref_file_name.c_str());
    std::ifstream ref_file(ref_file_name);
    if (!ref_file.is_open()) {
        ROS_ERROR("Failed to open ref file: %s", ref_file_name.c_str());
        return false;
    }

    // ref.txt: lat0 lon0 h0 형태라고 가정
    ref_file >> coord_ref.lat0 >> coord_ref.lon0 >> coord_ref.h0;
    ref_file.close();

    wgs84ToECEF(coord_ref.lat0, coord_ref.lon0, coord_ref.h0,
                coord_ref.x0_ecef, coord_ref.y0_ecef, coord_ref.z0_ecef);

    coord_ref_initialized = true;
    ROS_INFO("[Ref] lat0=%.8f lon0=%.8f h0=%.3f", coord_ref.lat0, coord_ref.lon0, coord_ref.h0);

    // -------------------------
    // 2) 경로(csv) 읽기
    // -------------------------
    ROS_INFO("Opening path file: %s", path_file_name.c_str());
    std::ifstream path_file(path_file_name);
    if (!path_file.is_open()) {
        ROS_ERROR("Failed to open path file: %s", path_file_name.c_str());
        return false;
    }

    std::string line;
    while (std::getline(path_file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);

        // 케이스 A: 공백 구분 "x y (z)" 형태
        double x, y, z;
        if (ss >> x >> y) {
            Waypoint wp;
            wp.x = x;
            wp.y = y;
            wp.curvature = 0.0;
            waypoints.push_back(wp);
            continue;
        }

        // 케이스 B: 콤마 구분 "x,y,..." 형태
        ss.clear();
        ss.str(line);

        std::string val;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) {
            // trim
            val.erase(0, val.find_first_not_of(" \t\r\n"));
            val.erase(val.find_last_not_of(" \t\r\n") + 1);

            try {
                row.push_back(std::stod(val));
            } catch (...) {
                // 헤더나 문자열이면 스킵
            }
        }

        if (row.size() >= 2) {
            Waypoint wp;
            wp.x = row[0];
            wp.y = row[1];
            wp.curvature = 0.0;
            waypoints.push_back(wp);
        }
    }

    path_file.close();

    if (waypoints.size() < 3) {
        ROS_ERROR("Not enough waypoints! (size: %zu)", waypoints.size());
        return false;
    }

    ROS_INFO("[Waypoints] Loaded %zu waypoints from %s", waypoints.size(), path_file_name.c_str());
    return true;
}

bool load_overtakingZone() {
    overtaking_zone.clear();
    // 파일 경로는 실제 환경에 맞춰 수정해주세요 (예: src/main/config/...)
    const std::string path_file_name = "src/main/config/overtaking_zone.csv";

    ROS_INFO("Opening overtaking zone file: %s", path_file_name.c_str());
    std::ifstream path_file(path_file_name);
    
    if (!path_file.is_open()) {
        ROS_ERROR("Failed to open overtaking zone file!");
        return false;
    }

    std::string line;
    while (std::getline(path_file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string val;
        std::vector<double> row;

        // CSV 파싱 (콤마 구분)
        while (std::getline(ss, val, ',')) {
            try { row.push_back(std::stod(val)); } catch (...) {}
        }

        if (row.size() >= 2) {
            Point2D wp;
            wp.x = row[0];
            wp.y = row[1];
            overtaking_zone.push_back(wp);
        }
    }
    path_file.close();
    ROS_INFO("[OvertakingZone] Loaded %zu points.", overtaking_zone.size());
    return true;
}