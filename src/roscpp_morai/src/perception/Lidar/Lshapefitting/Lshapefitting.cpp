#include <Lidar/Lshapefitting/Lshapefitting.hpp>

class LshapefittingProcess
{
    public:
        LshapefittingProcess() {};
        
        static bool SortingPoints (const Point2D& p1, const Point2D& p2);

        double CrossProduct (const Point2D& p1, const Point2D& p2, const Point2D& p3);
        
        vector <Point2D> Calculate_ConvexHull (vector<Point2D>& vec_Points);

        double Calculate_Closeness (const vector<Point2D>& vec_Points,
                                    const vector<Point2D>& vec_Convexhull_Points,
                                    float& BestTheta,
                                    float& BestAobb,
                                    float& BestMinX, float& BestMaxX, float& BestMinY, float& BestMaxY); 

        void Calculate_Process (LidarCluster& st_LidarCluster);
};

// ========================================================================
// 클러스터 1개 안에서 진행되는 중간 계산 과정
// ========================================================================

// -----------------------   x, y 오름차순 정렬 함수  --------------------------
bool LshapefittingProcess::SortingPoints(const Point2D& p1, const Point2D& p2) 
{
    if (p1.x == p2.x) {
        return p1.y < p2.y;
    }
    else {
        return p1.x < p2.x;
    }
}

// ----------------------------- 외적 계산 함수 --------------------------------
double LshapefittingProcess::CrossProduct (const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

// ------ 오름차순 정렬된 Point2D 에 대해 외적 계산한 결과 이용해서 convex hull 정점구하는 함수 -------
vector <Point2D> LshapefittingProcess::Calculate_ConvexHull (vector<Point2D>& vec_Points)
{
    if ((int)vec_Points.size() < 3) return vec_Points;

    // 1. 오름차순 정렬
    sort(vec_Points.begin(), vec_Points.end(), LshapefittingProcess::SortingPoints);

    vector<Point2D> vec_Convexhull_Points;

    // 2. lower Hull 계산
    for (int i = 0; i < (int)vec_Points.size(); i++)
    {
        while (vec_Convexhull_Points.size() >= 2)
        {
            if (CrossProduct(vec_Convexhull_Points[vec_Convexhull_Points.size() - 2], vec_Convexhull_Points.back(), vec_Points[i]) <= 0) 
                vec_Convexhull_Points.pop_back();
            else break;
        }
        vec_Convexhull_Points.push_back(vec_Points[i]);
    }

    // 3. upper Hull 계산
    int lower_size = vec_Convexhull_Points.size();

    for (int i = (int)vec_Points.size() - 2; i >= 0; --i) {
        while (vec_Convexhull_Points.size() > lower_size) {
            if (CrossProduct(vec_Convexhull_Points[vec_Convexhull_Points.size() - 2], vec_Convexhull_Points.back(), vec_Points[i]) <= 0) 
                vec_Convexhull_Points.pop_back();
            else break;
        }
        vec_Convexhull_Points.push_back(vec_Points[i]);
    }
    vec_Convexhull_Points.pop_back();

    return vec_Convexhull_Points; // 외곽선 정점 집합 반환
}

// ------------------ 가장 핏한 직사각형을 찾는 closeness ------------------
double LshapefittingProcess::Calculate_Closeness (
    const vector<Point2D>& vec_Points,
    const vector<Point2D>& vec_Convexhull_Points, // (vec) convexhull 1개에 들어있는 points
    float& BestTheta,
    float& BestAobb,
    float& BestMinX, float& BestMaxX, float& BestMinY, float& BestMaxY)
{
    //============ to do ===============
    // 1) min_x = + infinity, max_x = - infinity ~~ 기본 설정. 
    
    // ---------------------------------------------------------------------------
    // st_LidarCluster 에 있는 p = (x, y) 에 대해 
    // d = min (|x - rotate_rect_min_x|, |rotate_rect_max_x - x|, |y - miny|, |maxy - y|)
    // score += 1/d
    
    // 1. 외적으로 후보 theta 구하고
    // 2. theta 하나 당 rotate (축정렬) 
    // (for 정점 순회 1번)
    // 3. 모든 정점을 포함하는 직사각형 생성 (min,max 구해짐) -> return Aobb
    // (for 정점 순회 1번 더)
    // 4. convex_hull 을 이루는 모든 정점들에 대해 정점 하나씩 순회하면서  min,max (직사각형 꼭짓점) 까지의 
    //    dist 를 구해서 가장 작은 dist를 저장. 
    // 5. 이걸 모든 점에 대해 반복. 1/d 을 더해서 각 theta 에 대한 score로 저장  

    // 입력 : convexhull 정점 집합 vec_Convexhull_Points


    float MaxBeta = -1e9;
    const float d0 = 0.1f;

    int HullSize = (int) vec_Convexhull_Points.size();
    int PointSize = (int)vec_Points.size();

    for (int i = 0; i < HullSize; i++)
    // convexhull 의 한 엣지에 대해
    {
        // 1. 후보 각도 구하기 (Convex Hull의 변 방향)
        Point2D p1 = vec_Convexhull_Points[i];
        Point2D p2 = vec_Convexhull_Points[(i + 1) % HullSize]; // p1 바로 다음 점
        float theta = atan2(p2.y - p1.y, p2.x - p1.x);

        float Cos = cos(-theta);
        float Sin = sin(-theta);

        float min_x = 1e9, max_x = -1e9;
        float min_y = 1e9, max_y = -1e9;

        vector<Point2D> vec_Rotated_Points; // convexhull 정점이 아닌 클러스터 전체 포인트를 축정렬해야. 

        // 2. 클러스터의 모든 점을 edge의 theta로 회전시켜 경계값(min/max) 찾기
        for (int j = 0; j < PointSize; j++)
        {
            float rotated_x = vec_Points[j].x * Cos - vec_Points[j].y * Sin;
            float rotated_y = vec_Points[j].x * Sin + vec_Points[j].y * Cos;
            vec_Rotated_Points.push_back({rotated_x, rotated_y});

            min_x = min(min_x, rotated_x);
            max_x = max(max_x, rotated_x);
            min_y = min(min_y, rotated_y);
            max_y = max(max_y, rotated_y);
        }

        // 3. Closeness 점수(Beta) 계산 
        float beta = 0.0;

        for (int j = 0; j < (int)vec_Rotated_Points.size(); j++)
        {
            const Point2D& r_point = vec_Rotated_Points[j];

            float min_dst_x = min(max_x - r_point.x, r_point.x - min_x);
            float min_dst_y = min(max_y - r_point.y, r_point.y - min_y);

            float d = max(min(min_dst_x, min_dst_y), d0); 
            beta += (1.0 / d);               
        }

        // 4. 최대 점수 갱신
        if (beta > MaxBeta)
        {
            MaxBeta = beta;
            BestTheta = theta;
            BestAobb = (max_x - min_x) * (max_y - min_y); // obb 넓이도 구함

            // 나중에 꼭짓점 계산을 위해 저장
            BestMinX = min_x; 
            BestMaxX = max_x;
            BestMinY = min_y; 
            BestMaxY = max_y;
        }
    }
    return MaxBeta;
}

// ===============================================================
// struct에 있는 중간 계산 함수들 포함해서 클러스터 1개에서 진행되는 계산 총괄
// ===============================================================

void LshapefittingProcess::Calculate_Process (LidarCluster& st_LidarCluster)
{
    // 1. cluster 점 -> z 자르고 (x, y) 로만 저장 -> vector<Point2D> 생성
    // 2. Calculate_ConvexHull 실행
    // -> convexhull 이루는 정점 도출

    // 3. 정점 이용해서 hull area 계산 (함수x) = Ahull 에 저장. ====> process 함수에서 진행해야 하는 것

    // 4. Calculate_Closeness 실행
    // -> theta 에 대한 score = 베타(theta), Aobb 반환

    // 5. Ahull/Aobb+abs = 감마(theta)

    // 6. 클러스터 1개에 대한 score = 베타 * 감마


    // ===== LShapeFitting 함수에서 작성 =====

    // 7. max score -> best theta. 
    //    -> struct LidarCluster 에 저장. (직사각형의 꼭짓점 => costmap 에서의 위치)

    // ------------------------------------------------------------------------------------

    // if (st_LidarCluster.pcl_cluster_point == nullptr) {
    // ROS_ERROR("Cluster point cloud is null!");
    // return;
    // }

    // =========================
    // 1. Point2D 변환 
    // =========================
    vector<Point2D> vec_Points; // 클러스터 1개를 이루고 있는 포인트 클라우드를 vector 에 담아서 표현
    vec_Points.reserve(st_LidarCluster.pcl_cluster_point->size());

    float min_z =  1e9f, max_z = -1e9f;

    for (int idx = 0; idx < (int)st_LidarCluster.pcl_cluster_point->size(); idx++) 
    {
        // ====== 지금은 클러스터 1개 안에 들어와서 포인트들을 3D -> 2D 로 변환 ing ======= 
        const pcl::PointXYZI& st_Point = st_LidarCluster.pcl_cluster_point->at(idx);

        vec_Points.push_back({st_Point.x, st_Point.y});
        min_z = min(min_z, st_Point.z);
        max_z = max(max_z, st_Point.z);
    }

    st_LidarCluster.min_z = min_z;
    st_LidarCluster.max_z = max_z;

    if (vec_Points.size() < 3) return;

    // =========================
    // 2. Convex Hull 정점 반환
    // =========================
    vector<Point2D> vec_Convexhull_P = Calculate_ConvexHull(vec_Points); // convex hull 정점이 vector에 저장

    // =========================
    // 3. Convex Hull Area (Ahull)
    // =========================
    float Ahull = 0.0;
    
    for(int i = 0; i < (int)vec_Convexhull_P.size(); i++)
    {
        Ahull += (vec_Convexhull_P[i].x * vec_Convexhull_P[(i+1)%(int)vec_Convexhull_P.size()].y) - 
                     (vec_Convexhull_P[(i+1)%(int)vec_Convexhull_P.size()].x * vec_Convexhull_P[i].y);
    }
    Ahull = abs(Ahull) * 0.5;

    // =========================
    // 4. Convex Hull closeness
    // =========================
    float BestTheta = 0.0;
    float BestAobb = 0.0;
    float BestMinX = 0.0, BestMaxX = 0.0, BestMinY = 0.0, BestMaxY = 0.0;
    float Beta = Calculate_Closeness(vec_Points, vec_Convexhull_P, BestTheta, BestAobb, BestMinX, BestMaxX, BestMinY, BestMaxY);

    // =========================
    // 5. Convex Hull closeness
    // =========================
    // Ahull/Aobb+abs = 감마(theta)
    const float d0 = 0.1f;
    float Gamma = Ahull / (BestAobb + d0); 

    float score = Beta * Gamma; 

    st_LidarCluster.vec_Corners.resize(4);
    
    // 축정렬한 점을 - theta 만큼 다시 원상복귀하기 위해
    st_LidarCluster.theta = BestTheta;
    st_LidarCluster.rotate_rect_min_x = BestMinX;
    st_LidarCluster.rotate_rect_max_x = BestMaxX;
    st_LidarCluster.rotate_rect_min_y = BestMinY;
    st_LidarCluster.rotate_rect_max_y = BestMaxY;

    float Cos = cos(BestTheta);
    float Sin = sin(BestTheta);

    // OBB 네 꼭짓점 위치 계산
    float local_corners [4][2] = 
    {
        {BestMinX, BestMinY},
        {BestMinX, BestMaxY},
        {BestMaxX, BestMaxY},
        {BestMaxX, BestMinY}
    };

    for (int i = 0; i < 4; i++) 
    {
        float local_x = local_corners[i][0]; 
        float local_y = local_corners[i][1];
        float global_x = local_x * Cos - local_y * Sin;
        float global_y = local_x * Sin + local_y * Cos;

        st_LidarCluster.vec_Corners[i].x = global_x;
        st_LidarCluster.vec_Corners[i].y = global_y;
    }

    // OBB 중심점, width, length 계산
    float local_centroid_x = (BestMinX + BestMaxX) / 2.0f;
    float local_centroid_y = (BestMinY + BestMaxY) / 2.0f;

    st_LidarCluster.centroid_x = local_centroid_x * Cos - local_centroid_y * Sin;
    st_LidarCluster.centroid_y = local_centroid_x * Sin + local_centroid_y * Cos;

    st_LidarCluster.width = BestMaxX - BestMinX;
    st_LidarCluster.length = BestMaxY - BestMinY;
}


// ===============================================================================
// 여러 클러스터 돌면서 convex hull의 전체 후보 각도에 대해 직사각형 계산하고, 최적의 직사각형 피팅
// ===============================================================================

void LShapeFitting (Lidar& st_Lidar)
{
    vector<LidarCluster>& vec_Clusters = st_Lidar.vec_clusters;
    LshapefittingProcess func_Process;

    for (int i = 0; i < vec_Clusters.size(); i++) 
    {
        if (vec_Clusters[i].pcl_cluster_point->size() < 3) continue;
        func_Process.Calculate_Process(vec_Clusters[i]);
    }

    // // 2. 칼만 필터 업데이트를 위한 데이터 변환 (LidarCluster -> ForKalman)
    // std::vector<ForKalman> detections;
    // for (int i = 0; i < vec_Clusters.size(); i++)
    // {
    //     ForKalman det;
    //     det.x = vec_Clusters[i].center_x;
    //     det.y = vec_Clusters[i].center_y;
    //     det.yaw = vec_Clusters[i].theta;
    //     det.w = vec_Clusters[i].width;
    //     det.l = vec_Clusters[i].length;
    //     det.cluster_idx = i; // 원본 인덱스 저장
    //     detections.push_back(det);
    // }

    // // 3. 칼만 필터 트래킹 실행 (현재 시간 timestamp 필요)
    // double current_time = /* 현재 시스템 시간 또는 센서 시간 */;
    // g_tracker.updateTracks(detections, current_time);

    // // 4. 트래커의 결과를 다시 LidarCluster 구조체에 반영
    // // 트래커 내부에 저장된 ID와 속도 정보를 원본 클러스터에 매칭
    // for (auto& track : g_tracker.tracks)
    // {
    //     // track.last_cluster_idx 등을 활용하거나 
    //     // 가장 가까운 클러스터를 찾아 ID와 속도를 부여
    //     for (auto& cluster : vec_Clusters)
    //     {
    //         float dist = sqrt(pow(cluster.center_x - track.x(0), 2) + 
    //                           pow(cluster.center_y - track.x(1), 2));
            
    //         if (dist < 0.5) { // 50cm 이내면 동일 객체로 판단
    //             cluster.id = track.id;
    //             cluster.velocity = track.x(3); // EKF로 추정된 선속도
    //             cluster.is_updated = true;
                
    //             // 필터링된 값으로 좌표 보정 (선택 사항)
    //             // cluster.center_x = track.x(0); 
    //             // cluster.center_y = track.x(1);
    //         }
    //     }
    // }

}