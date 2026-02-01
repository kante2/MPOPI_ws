#include "Visualizer.hpp"
#include "Global.hpp"
#include <geometry_msgs/Point.h>

void publishCandidatePaths() {
    visualization_msgs::MarkerArray marker_array;
    
    double best_offset = lattice_ctrl.best_path.offset;

    for (size_t i = 0; i < lattice_ctrl.candidates.size(); i++) {
        const auto& candidate = lattice_ctrl.candidates[i];
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "candidate_paths";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.orientation.w = 1.0; 
        
        if (!candidate.valid) {
            marker.scale.x = 0.03; 
            marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5; marker.color.a = 0.5;
        } 
        else if (fabs(candidate.offset - best_offset) < 0.01) {
            marker.scale.x = 0.15; 
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
        } 
        else {
            marker.scale.x = 0.05; 
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 0.6;
        }
        
        for (size_t j = 0; j < candidate.points.size(); j++) {
            geometry_msgs::Point p_visual;
            p_visual.x = candidate.points[j].x;
            p_visual.y = candidate.points[j].y;
            p_visual.z = 0.0; 
            marker.points.push_back(p_visual);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub.publish(marker_array);
}

void publishVehicleFootprint() { // 내 차(노란 박스) 그리기
    visualization_msgs::Marker car_marker;
    car_marker.header.frame_id = "base_link";
    car_marker.header.stamp = ros::Time::now();
    car_marker.ns = "ego_shape";
    car_marker.id = 0;
    car_marker.type = visualization_msgs::Marker::CUBE;
    car_marker.action = visualization_msgs::Marker::ADD;

    double front_len = planner_params.vehicle_front_offset; 
    double rear_len  = 1.0; 
    double width     = 2.0; 

    car_marker.pose.position.x = (front_len - rear_len) / 2.0;
    car_marker.pose.position.y = 0.0;
    car_marker.pose.position.z = 0.5; 
    car_marker.pose.orientation.w = 1.0;

    car_marker.scale.x = front_len + rear_len; 
    car_marker.scale.y = width;
    car_marker.scale.z = 1.5; 

    car_marker.color.r = 1.0;
    car_marker.color.g = 1.0;
    car_marker.color.b = 0.0;
    car_marker.color.a = 0.5; 

    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(car_marker);
    marker_pub.publish(arr);
}

void publishLocalPath() { // 로컬 경로를 파란색 MarkerArray로 발행
    if (lattice_ctrl.best_path.points.empty()) {
        return;
    }
    
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "base_link";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "local_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.3;
    
    path_marker.color.r = 0.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 1.0;
    path_marker.color.a = 1.0;
    
    for (const auto& point : lattice_ctrl.best_path.points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        path_marker.points.push_back(p);
    }
    
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(path_marker);
    local_path_pub.publish(arr);
}