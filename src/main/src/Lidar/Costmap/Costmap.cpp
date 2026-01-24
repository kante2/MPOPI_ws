#include <Costmap/Costmap.hpp>

CostmapState state;
CostmapParams params;
unique_ptr<tf2_ros::Buffer> tfBuffer;
ros::Publisher pub_costmap;
string costmap_frame = "base_link";

unique_ptr<tf2_ros::TransformListener> tfListener;

// ========================================
// 초기화
// ========================================

void initCostmapModule(ros::NodeHandle &nh) 
{
    tfBuffer.reset(new tf2_ros::Buffer());
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

    pub_costmap = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
    
    // ---- global.hpp에서 파라미터 값 수정 ------

    // params.resolution = 0.1;
    // params.width = 30.0;   
    // params.height = 30.0;
    // params.obstacle_cost = 100;
    // params.free_cost = 0;
    // params.unknown_cost = -1;
    // params.inflation_radius = 1.0;
}

// ========================================
// 0. 헬퍼 함수
// ========================================

string normalizeFrameId(string frame_id) 
{
  while (!frame_id.empty() && frame_id.front() == '/') 
  {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
}

int clampToRange(int value, int min_val, int max_val) 
{
  return max(min_val, min(value, max_val));
}

// ========================================
// 1. TF 변환 (Lidar link -> base link)
// ========================================

void transformLidarToBaselink(const LidarCluster& st_LidarCluster) 
{
    state.tf_ok = false;

    string raw_src = st_LidarCluster.header.frame_id;
    string src = normalizeFrameId(raw_src);
    string tgt = costmap_frame;

    ROS_INFO("TF CHECK -> src: '%s' (raw: '%s'), tgt: '%s'", src.c_str(), raw_src.c_str(), tgt.c_str());

    if (src.empty()) 
    {
        ROS_ERROR("TF ABORT: Source frame is EMPTY! Did you fill the header?");
        return; 
    }

    if (src == tgt) 
    {
      state.baselink_cloud = st_LidarCluster; //state.baselink_cloud 타입 : LidarCluster
      state.tf_ok = true;
      return;
    }

    try 
    {
        if (!tfBuffer) 
        {
        ROS_ERROR("TF ABORT: tfBuffer is NULL! Check initialization.");
        return;
        }

        // --------------------- 1. 변환 행렬 가져오기 ----------------------------

        geometry_msgs::TransformStamped tf =
            tfBuffer->lookupTransform(tgt, src, ros::Time(0), ros::Duration(0.05));

        // 결과 저장용 객체 초기화 (기존 데이터 복사)
        state.baselink_cloud = st_LidarCluster;
        state.baselink_cloud.header.frame_id = tgt;
        state.baselink_cloud.vec_Corners.clear(); // 변환된 점들을 넣기 위해 비움

        // -------------------- 2. 중심점(Centroid) 변환 -------------------------------

        geometry_msgs::Point pt_in, pt_out; // tf에서 사용할 수 있는 자료형
        
        // lidar_link
        pt_in.x = st_LidarCluster.centroid_x;
        pt_in.y = st_LidarCluster.centroid_y;
        pt_in.z = 0; 

        tf2::doTransform(pt_in, pt_out, tf); //tf 변환 행렬 사용해서 좌표계 변환
        
        // base_link
        state.baselink_cloud.centroid_x = pt_out.x;
        state.baselink_cloud.centroid_y = pt_out.y;

        // ------------------- 3. OBB 꼭짓점(vec_Corners) 변환 -------------------------

        for (const Point2D& corner : st_LidarCluster.vec_Corners) 
        {
            geometry_msgs::Point c_in, c_out;
            
            // lidar_link 
            c_in.x = corner.x;
            c_in.y = corner.y;
            c_in.z = 0;

            tf2::doTransform(c_in, c_out, tf);

            // base_link
            Point2D transformed_corner;
            transformed_corner.x = c_out.x;
            transformed_corner.y = c_out.y;
            state.baselink_cloud.vec_Corners.push_back(transformed_corner);
        }

        // ------------------------- 4. Theta (각도) 변환 ---------------------------------

        // tf에서 yaw(회전량)만 추출
        double tf_yaw = tf2::getYaw(tf.transform.rotation);
        
        // 기존 theta에 좌표계 회전량을 더함
        float transformed_theta = st_LidarCluster.theta + static_cast<float>(tf_yaw);

        // 각도 범위를 -PI ~ PI 사이로 정규화 
        while (transformed_theta > M_PI) transformed_theta -= 2.0 * M_PI;
        while (transformed_theta < -M_PI) transformed_theta += 2.0 * M_PI;

        // base_link
        state.baselink_cloud.theta = transformed_theta;

        state.tf_ok = true;
    }

    catch (const exception &e) 
    {
        ROS_ERROR("Costmap TF Error: %s", e.what());
        state.tf_ok = false;
    }
}

// ========================================
// 2. Costmap 초기화
// ========================================

void initializeCostmap(const ros::Time &stamp) 
{
  state.costmap.header.frame_id = costmap_frame;
  state.costmap.header.stamp = stamp;

  state.costmap.info.resolution = params.resolution;

  // 1. 가로 세로 픽셀 수 계산
  const uint32_t h = static_cast<uint32_t>(round(params.height / params.resolution));
  const uint32_t w = static_cast<uint32_t>(round(params.width/ params.resolution));

  // 크기가 바뀔 때만 메모리를 재할당하거나 정보를 갱신 (선택 사항)
  state.costmap.info.width = w;
  state.costmap.info.height = h;

  // 2. Origin 설정 (차량이 중앙에 오도록 하되, 해상도 단위로 정렬)
  // 정교한 정렬을 위해 아래와 같이 작성하기도 합니다.
  state.costmap.info.origin.position.x = -(static_cast<double>(w) * params.resolution) * 0.5;
  state.costmap.info.origin.position.y = -(static_cast<double>(h) * params.resolution) * 0.5;
  state.costmap.info.origin.position.z = 0.0;

  state.costmap.info.origin.orientation.w = 1.0;
  // 나머지 orientation은 0.0이 기본값이므로 꼭 필요한 경우만 명시

  // 3. 데이터 초기화
  const size_t total = static_cast<size_t>(w) * h;
  
  // 데이터 크기가 이미 맞다면 fill, 아니면 assign
  if (state.costmap.data.size() != total) 
  {
      state.costmap.data.assign(total, params.unknown_cost);
  } 
  else 
  {
      fill(state.costmap.data.begin(), state.costmap.data.end(), params.unknown_cost);
  }
}


// ========================================
// 3. 원점(차량 바로 아래) 주변 장애물 삭제
// ========================================

void clearOriginArea() 
{
  int ox, oy;
  if (!convertBaselinkToGrid(state.costmap, 0.0f, 0.0f, ox, oy)) return;

  const int radius = 3; // 차량이 더 크거나 노이즈가 넓게 찍힌다면  숫자 더 크게

  for (int y = oy - radius; y <= oy + radius; ++y) 
  {
    for (int x = ox - radius; x <= ox + radius; ++x) 
    {
      int idx = convertGridToIndex(state.costmap, x, y);
      if (idx >= 0 && idx < (int)state.costmap.data.size()) 
      {
        state.costmap.data[idx] = params.free_cost;
      }
    }
  }
}


// ========================================
// 4-1. 2D 점이 다각형 내부에 있는지 판별 (Cross Product 활용)
// ========================================

bool isPointInPolygon(int x, int y, const vector<pair<int, int>>& polygon) 
{
    int n = polygon.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) 
    {
        if (((polygon[i].second > y) != (polygon[j].second > y)) &&
            (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
            (polygon[j].second - polygon[i].second) + polygon[i].first)) 
        {
            inside = !inside;
        }
    }
    return inside;
}

// ========================================
// 4-2. L-Shape(4개 꼭짓점) 내부를 코스트맵에 채우는 함수 예시
// ========================================

void fillLShapePolygon(nav_msgs::OccupancyGrid &costmap, 
                       const vector<Point2D>& corners, 
                       int8_t cost_value,
                       float inflation = 0.0f)
{ 
    if (corners.empty()) return;

    // 1-1. AABB 계산 (Inflation 반영) -> 여기서 AABB 는 range
    double min_x = numeric_limits<double>::max();
    double min_y = numeric_limits<double>::max();
    double max_x = numeric_limits<double>::lowest(); 
    double max_y = numeric_limits<double>::lowest();

    for (const Point2D& c : corners) 
    {
        min_x = min(min_x, (double)c.x); 
        min_y = min(min_y, (double)c.y);
        max_x = max(max_x, (double)c.x); 
        max_y = max(max_y, (double)c.y);
    }

    // 1-2. AABB 범위 변환 : base_link 좌표(미터) → costmap 그리드 좌표(셀)
    int gx0, gy0, gx1, gy1;
    convertBaselinkToGrid(costmap, min_x - inflation, min_y - inflation, gx0, gy0);
    convertBaselinkToGrid(costmap, max_x + inflation, max_y + inflation, gx1, gy1);

    // 경계 클램핑
    gx0 = max(0, gx0); 
    gy0 = max(0, gy0);
    gx1 = min((int)costmap.info.width - 1, gx1);
    gy1 = min((int)costmap.info.height - 1, gy1);

    // 2-1. OBB 꼭짓점 변환 : base_link 좌표(미터) -> costmap 그리드 좌표(셀)
    vector<pair<int, int>> grid_corners;
    for (const Point2D& c : corners) 
    {
        int gx, gy;
        convertBaselinkToGrid(costmap, c.x, c.y, gx, gy);
        grid_corners.push_back({gx, gy});
    }

    // 3. AABB 범위 내에서 OBB 만 색칠
    for (int y = gy0; y <= gy1; ++y) 
    {
        for (int x = gx0; x <= gx1; ++x) 
        {
            if (isPointInPolygon(x, y, grid_corners)) 
            {
                int idx = convertGridToIndex(costmap, x, y);
                fillCostAtIndex(costmap, idx, cost_value);
            }
        }
    }
}


// ========================================
// 5. base_link 좌표(미터) → costmap 그리드 좌표(셀) // 한점을 변환 그래서 7번안 함수에서 호출됨
// ========================================

bool convertBaselinkToGrid(const nav_msgs::OccupancyGrid &costmap,
                           float baselink_x, float baselink_y,
                           int &grid_x, int &grid_y) 
{
  grid_x = (int)floor((baselink_x - costmap.info.origin.position.x) / costmap.info.resolution);
  grid_y = (int)floor((baselink_y - costmap.info.origin.position.y) / costmap.info.resolution);

  if (grid_x < 0 || grid_x >= (int)costmap.info.width) return false;
  if (grid_y < 0 || grid_y >= (int)costmap.info.height) return false;
  return true;
}


// ========================================
// 6. 2D grid 좌표 → 1D 배열 인덱스
// ========================================

int convertGridToIndex(const nav_msgs::OccupancyGrid &costmap,
                       int grid_x, int grid_y) 
{
  if (grid_x < 0 || grid_x >= (int)costmap.info.width) return -1;
  if (grid_y < 0 || grid_y >= (int)costmap.info.height) return -1;
  return grid_y * (int)costmap.info.width + grid_x;
}


// ========================================
// 7. costmap 셀 하나에 장애물 cost 기록
// ========================================

void fillCostAtIndex(nav_msgs::OccupancyGrid &costmap,
                     int idx,
                     int8_t cost_value) 
{
  if (idx < 0 || idx >= (int)costmap.data.size()) return;
  costmap.data[idx] = max(costmap.data[idx], cost_value);
}


// ========================================
// final. costmap process 실행
// ========================================

void Costmap(const Lidar& st_Lidar, const ros::Time& stamp) 
{
    initializeCostmap(stamp); 
    clearOriginArea();  

    for (const LidarCluster& cluster : st_Lidar.vec_clusters)
    {
      transformLidarToBaselink(cluster);   
      if (!state.tf_ok)
      {
        continue;
      }

      if (!state.baselink_cloud.vec_Corners.empty()) 
      {
        fillLShapePolygon(state.costmap, 
                          state.baselink_cloud.vec_Corners, 
                          params.obstacle_cost, 
                          params.inflation_radius);
      }
    }
    pub_costmap.publish(state.costmap);
}