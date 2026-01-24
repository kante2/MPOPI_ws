#include <LidarProcess/LidarProcess.hpp>

// MultiObjectTracker g_tracker;

void LidarProcess(Lidar& st_Lidar, LidarCluster& st_LidarCluster, const std_msgs::Header& header) 
{
    // FilterHeight(st_Lidar);
    FilterRange(st_Lidar);
    CropBox(st_Lidar);
    Voxel(st_Lidar);
    Ransac(st_Lidar);
    Euclidean(st_Lidar);
    LShapeFitting(st_Lidar);

    for (LidarCluster& cluster : st_Lidar.vec_clusters)
    {
        cluster.header = header;
    }
    Costmap(st_Lidar, header.stamp);

    // =============================================================================
    // kalman
    // =============================================================================
    
    // double current_time = ros::Time::now().toSec();
    // g_tracker.updateTracks(st_Lidar, current_time);

//     // =============================================================================
//     // costmap
//     // =============================================================================
    
//     // LShapeFitting()의 결과를 Detection 스트럭쳐에 저장
//     // -> costmap 에서 Detection 구조체에 접근해서 사용

//     // 라이다좌표계-> 베이스링크좌표계 TF 변환
//     transformLidarToBaselink(st_LidarCluster);   
//     // 지금은 sensormsgs를 받아오는데, Lshapefitting 결과를 costmap으로 옮겨야한다. 
    
//     if (!state.tf_ok) return;
// // 2. 코스트맵 초기화 및 원점 클리어
//     initializeCostmap(st_LidarCluster.header.stamp); 
//     clearOriginArea();  

//     // 3. L-Shape Fitting 결과(vec_Corners)를 사용하여 코스트맵 채우기
//     // 변환된 결과가 담긴 state.baselink_cloud를 사용합니다.
//     const LidarCluster& cluster = state.baselink_cloud;

//     if (!cluster.vec_Corners.empty()) {
//         // A. 그릴 범위(Bounding Box) 결정을 위해 corners에서 min/max 추출
//         double min_x = std::numeric_limits<double>::max();
//         double min_y = std::numeric_limits<double>::max();
//         double max_x = std::numeric_limits<double>::lowest();
//         double max_y = std::numeric_limits<double>::lowest();

//         for (const Point2D& corner : cluster.vec_Corners) {
//             min_x = std::min(min_x, corner.x);
//             min_y = std::min(min_y, corner.y);
//             max_x = std::max(max_x, corner.x);
//             max_y = std::max(max_y, corner.y);
//         }

//         // B. 기존 7번 함수로 그리드 탐색 범위 계산 (인플레이션 포함)
//         int gx0, gy0, gx1, gy1;
//         projectAABBToGrid(state.costmap, min_x, min_y, max_x, max_y, 
//                           gx0, gy0, gx1, gy1, params.inflation_radius);

//         // C. 그리드 영역 탐색 및 다각형 내부 채우기
//         // (그리드 좌표로 변환된 꼭짓점 리스트 준비)
//         std::vector<std::pair<int, int>> grid_corners;
//         for (const auto& c : cluster.vec_Corners) {
//             int gx, gy;
//             convertBaselinkToGrid(state.costmap, c.x, c.y, gx, gy);
//             grid_corners.push_back({gx, gy});
//         }

//         for (int y = gy0; y <= gy1; ++y) {
//             for (int x = gx0; x <= gx1; ++x) {
//                 // 다각형 내부 판별 알고리즘 사용
//                 if (isPointInPolygon(x, y, grid_corners)) {
//                     int idx = convertGridToIndex(state.costmap, x, y);
//                     fillCostAtIndex(state.costmap, idx, params.obstacle_cost);
//                 }
//             }
//         }
//     }

//     // 4. 최종 퍼블리시
//     pub_costmap.publish(state.costmap);
}