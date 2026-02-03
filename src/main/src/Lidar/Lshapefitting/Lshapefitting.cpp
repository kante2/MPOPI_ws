#include <Lshapefitting/Lshapefitting.hpp>

class LshapefittingProcess
{
    public:
        LshapefittingProcess() {};
        
        static bool SortingPoints (const Point2D& p1, const Point2D& p2);

        double CrossProduct (const Point2D& p1, const Point2D& p2, const Point2D& p3);
        
        vector <Point2D> Calculate_ConvexHull (vector<Point2D>& vec_Points);

        void Calculate_Closeness (const vector<Point2D>& vec_Points,
                                    const vector<Point2D>& vec_Convexhull_Points,
                                    float& BestTheta,
                                    float& BestAreaOBB,
                                    float& BestMinX, float& BestMaxX, float& BestMinY, float& BestMaxY,
                                    float& AreaHull); 

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
    // 1. 오름차순 정렬
    int PointSize = (int)vec_Points.size();
    if (PointSize < 3) return vec_Points;

    sort(vec_Points.begin(), vec_Points.end(), LshapefittingProcess::SortingPoints);

    // 2. lower Hull 계산
    vector<Point2D> vec_Convexhull_Points;
    int HullSize = vec_Convexhull_Points.size();

    for (int i = 0; i < PointSize; i++) 
    // 왼 -> 오 (진행방향 기준 반시계 방향으로 꺾이는 점만 선택)
    {
        while (HullSize >= 2)
        {
            if (CrossProduct(vec_Convexhull_Points[HullSize - 2], vec_Convexhull_Points.back(), vec_Points[i]) <= 0) 
            // (CrossProduct (p1(마지막 바로 전 점), p2(현재 마지막 점), p3(새로 추가하려는 점)) <= 0) 
            //  : 현재 마지막 점인 p2가 껍질 안쪽으로 들어가서 움푹 파인 모양이 되어버림 -> 제거해야

                vec_Convexhull_Points.pop_back();
                // p2 제거하고 새로 추가하려는 점인 p3 가 외적 계산 결과 >0 를 만족할 때까지 기존 껍질에 있던 점을 역순으로 검사하는 과정

            else break;
        }
        vec_Convexhull_Points.push_back(vec_Points[i]);
    }

    // 3. upper Hull 계산
    int lower_size = HullSize;

    for (int i = PointSize - 2; i >= 0; --i) 
    // 오 -> 왼 (반시계 방향)
    {
        while (HullSize > lower_size) 
        {
            if (CrossProduct(vec_Convexhull_Points[HullSize - 2], vec_Convexhull_Points.back(), vec_Points[i]) <= 0) 
                vec_Convexhull_Points.pop_back();
            else break;
        }
        vec_Convexhull_Points.push_back(vec_Points[i]);
    }

    vec_Convexhull_Points.pop_back(); // 마지막에 중복되는 정점 pop

    return vec_Convexhull_Points; // 외곽선 정점 집합 반환
}

// ------------------ 가장 핏한 직사각형을 찾는 closeness ------------------
void LshapefittingProcess::Calculate_Closeness (
    const vector<Point2D>& vec_Points,
    const vector<Point2D>& vec_Convexhull_Points, // (vec) convexhull 1개에 들어있는 points
    float& BestTheta,
    float& BestAreaOBB,
    float& BestMinX, float& BestMaxX, float& BestMinY, float& BestMaxY,
    float& AreaHull)
{
    float MaxScore = -1e9;
    const float d0 = 0.1f;

    int HullSize = (int) vec_Convexhull_Points.size();

    for (int i = 0; i < HullSize; i++)
    // convexhull 의 한 엣지에 대해 score를 매겨서 가장 높은 score를
    {
        // 1. 후보 각도 구하기 (Convex Hull의 변 방향)
        Point2D p1 = vec_Convexhull_Points[i];
        Point2D p2 = vec_Convexhull_Points[(i + 1) % HullSize]; // p1 바로 다음 점

        float theta = atan2(p2.y - p1.y, p2.x - p1.x);

        // 2. 축정렬 : 클러스터의 모든 점을 edge의 theta로 회전시켜 경계값(min/max) 찾기
        float Cos = cos(-theta);
        float Sin = sin(-theta);

        float min_x = 1e9;
        float max_x = -1e9;
        float min_y = 1e9;
        float max_y = -1e9;

        vector<Point2D> vec_Rotated_Points; // convexhull 정점이 아닌 클러스터 전체 포인트를 축정렬해야. 
    
        int PointSize = (int)vec_Points.size();

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

        float AreaOBB = (max_x - min_x) * (max_y - min_y);

        // 3. 점들과 가장 가까운 거리에 있는 직사각형(=클러스터에 가장 핏한 직사각형) 찾기 -> Closeness 점수(Beta) 계산 
        float beta = 0.0;
        int RotatedPointSize = (int)vec_Rotated_Points.size();

        for (int j = 0; j < RotatedPointSize; j++)
        {
            const Point2D& r_point = vec_Rotated_Points[j];

            float min_dst_x = min(max_x - r_point.x, r_point.x - min_x);
            float min_dst_y = min(max_y - r_point.y, r_point.y - min_y);

            float d = max(min(min_dst_x, min_dst_y), d0); 
            beta += (1.0 / d);               
        }

        float gamma = AreaHull / (AreaOBB + d0);
        float score = beta * gamma;

        // 4. 최대 점수 갱신
        if (score > MaxScore)
        {
            MaxScore = score;
            BestTheta = theta;
            BestAreaOBB = AreaOBB; 

            // OBB 꼭짓점 계산에 쓰이도록
            BestMinX = min_x; 
            BestMaxX = max_x;
            BestMinY = min_y; 
            BestMaxY = max_y;
        }
    }
}

// ===============================================================
// struct에 있는 중간 계산 함수들 포함해서 클러스터 1개에서 진행되는 계산 총괄
// ===============================================================

// =========================
// 직전 프레임의 OBB 정보 저장 구조체
// =========================
struct PrevClusterInfo 
{
    float x;
    float y;
    float heading_theta;
};

static std::vector <PrevClusterInfo> vec_prev_clusters_info;


void LshapefittingProcess::Calculate_Process (LidarCluster& st_LidarCluster)
{
    
    // =========================
    // 1. Point2D 변환 
    // =========================

    vector<Point2D> vec_Points; // 클러스터 1개를 이루고 있는 포인트 클라우드를 vector 에 담아서 표현
    vec_Points.reserve(st_LidarCluster.pcl_cluster_point->size());

    float min_z =  1e9f;
    float max_z = -1e9f;

    for (int idx = 0; idx < (int)st_LidarCluster.pcl_cluster_point->size(); idx++) 
    {
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
    // 3. Convex Hull Area (AreaHull) 계산
    // =========================

    float AreaHull = 0.0;
    int HullSize = (int)vec_Convexhull_P.size();
    
    for(int i = 0; i < HullSize; i++)
    {
        AreaHull += (vec_Convexhull_P[i].x * vec_Convexhull_P[(i+1) % HullSize].y) 
                    - (vec_Convexhull_P[(i+1) % HullSize].x * vec_Convexhull_P[i].y);
    }

    AreaHull = abs(AreaHull) * 0.5;

    // =========================
    // 4. Convex Hull closeness
    // =========================

    float BestTheta = 0.0;
    float BestAreaOBB = 0.0;

    float BestMinX = 0.0;
    float BestMaxX = 0.0;
    float BestMinY = 0.0;
    float BestMaxY = 0.0;

    LshapefittingProcess Closeness_Process;

    Closeness_Process.Calculate_Closeness(vec_Points, vec_Convexhull_P, BestTheta, BestAreaOBB, BestMinX, BestMaxX, BestMinY, BestMaxY, AreaHull);

    // =========================
    // 5. OBB pose, centroid etc 계산
    // =========================
    st_LidarCluster.vec_Corners.resize(4);
    
    // 축정렬한 점을 - theta 만큼 다시 원상복귀하기 위해
    st_LidarCluster.theta = BestTheta;

    float padding_value = 0.5f;

    BestMinX = BestMinX - padding_value; 
    BestMaxX = BestMaxX + padding_value;
    BestMinY = BestMinY - padding_value;
    BestMaxY = BestMaxY + padding_value;
    
    // st_LidarCluster.rotate_rect_min_x = BestMinX - padding_value; // 단순히 변수에 저장
    // st_LidarCluster.rotate_rect_max_x = BestMaxX + padding_value;
    // st_LidarCluster.rotate_rect_min_y = BestMinY - padding_value;
    // st_LidarCluster.rotate_rect_max_y = BestMaxY + padding_value;

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

    float l = BestMaxX - BestMinX;
    float w = BestMaxY - BestMinY;

    if (w < l) {
        st_LidarCluster.heading_theta = BestTheta; // rviz의 x축 (length) 이 길면 그대로
    }
    else {
        st_LidarCluster.heading_theta = BestTheta + M_PI_2; // y축(width) 이 길면 90도 회전
    }

    st_LidarCluster.width = min( w, l);
    st_LidarCluster.length = max( w, l);

    
    if (!((st_LidarCluster.width > 0.01f && st_LidarCluster.width < 1.0f ) && st_LidarCluster.length > 3.0f)) 
    {
        for (int i = 0; i < 4; i++) 
        {
            float local_x = local_corners[i][0]; 
            float local_y = local_corners[i][1];

            float global_x = local_x * Cos - local_y * Sin;
            float global_y = local_x * Sin + local_y * Cos;

            st_LidarCluster.vec_Corners[i].x = global_x;
            st_LidarCluster.vec_Corners[i].y = global_y;
        }
    }

    // OBB 중심점, width, length 계산
    float local_centroid_x = (BestMinX + BestMaxX) / 2.0f;
    float local_centroid_y = (BestMinY + BestMaxY) / 2.0f;

    st_LidarCluster.centroid_x = local_centroid_x * Cos - local_centroid_y * Sin;
    st_LidarCluster.centroid_y = local_centroid_x * Sin + local_centroid_y * Cos;


    // ===================================================================
    // 방향 벡터를 차량 전방방향으로 고정하기 위한 이전/현재 프레임 중심점 위치 변화 이용 
    // ===================================================================

    // 1. 현재 OBB 정보
    float curr_centroid_x = st_LidarCluster.centroid_x;
    float curr_centroid_y = st_LidarCluster.centroid_y;
    float curr_heading_theta = st_LidarCluster.heading_theta;

    float final_heading_theta = curr_heading_theta; // 최종 결정될 각도
    float min_dist = 1.0f; // 1m 이내만 같은 물체로 간주
    int matched_idx = -1;

    // 2. 이전 프레임 기록들 중 가장 가까운 OBB 찾기
    for (int i = 0; i < vec_prev_clusters_info.size(); ++i) 
    {
        float dx = curr_centroid_x - vec_prev_clusters_info[i].x;
        float dy = curr_centroid_y - vec_prev_clusters_info[i].y;
        float dist = sqrt(dx*dx + dy*dy);

        if (dist < min_dist) 
        {
            min_dist = dist;
            matched_idx = i;
        }
    }

    // 3. 매칭된 이전 OBB 정보가 없다면
    if (matched_idx != -1)
    {
        float move_dx = curr_centroid_x - vec_prev_clusters_info[matched_idx].x;
        float move_dy = curr_centroid_y - vec_prev_clusters_info[matched_idx].y;
        float move_dist = sqrt(move_dx*move_dx + move_dy*move_dy); 

        // (A) 움직임이 어느 정도 있을 때 (0.2m 이상): 이동 방향(Vector) 우선
        if (move_dist > 0.2f) 
        {
            float dot = move_dx * cos(curr_heading_theta) + move_dy * sin(curr_heading_theta);
            if (dot > 0) 
            {
                final_heading_theta += M_PI; // 이동 방향과 반대면 180도 반전
            }
        } 
        // (B) 정지 상태일 때: 이전 프레임의 각도 유지
        else 
        {
            float prev_heading_theta = vec_prev_clusters_info[matched_idx].heading_theta;
            float dot = cos(prev_heading_theta) * cos(curr_heading_theta) + sin(prev_heading_theta) * sin(curr_heading_theta);
            if (dot < 0) 
            {
                final_heading_theta += M_PI; // 이전 각도와 너무 차이나면 반전
            }
        }
    }

    // 각도를 [-PI, PI] 범위로 정규화
    while (final_heading_theta > M_PI)  final_heading_theta -= 2.0f * M_PI;
    while (final_heading_theta < -M_PI) final_heading_theta += 2.0f * M_PI;

    // 보정된 각도를 클러스터에 최종 저장 (LShapeFitting에서 백업할 때 이 값을 사용함)
    st_LidarCluster.heading_theta = final_heading_theta;

    // 차량 전방 방향으로 고정된 헤딩 벡터 계산
    st_LidarCluster.heading_direction_x = cos(final_heading_theta);
    st_LidarCluster.heading_direction_y = sin(final_heading_theta);

    // -------- data 확인 -----------
    cout << st_LidarCluster.heading_direction_x << endl;
    cout << st_LidarCluster.heading_direction_y << endl;
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

    vec_prev_clusters_info.clear(); 
    for (int i = 0; i < vec_Clusters.size(); i++)
    {
        if (vec_Clusters[i].pcl_cluster_point->size() < 3) continue;

        PrevClusterInfo prev_cluster_info; 
        prev_cluster_info.x = vec_Clusters[i].centroid_x;
        prev_cluster_info.y = vec_Clusters[i].centroid_y;
        prev_cluster_info.heading_theta = vec_Clusters[i].heading_theta;
        vec_prev_clusters_info.push_back(prev_cluster_info);
    }
}