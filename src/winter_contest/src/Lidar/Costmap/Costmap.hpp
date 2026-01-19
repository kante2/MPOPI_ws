#ifndef COSTMAP_HPP
#define COSTMAP_HPP
#include <Global/Global.hpp>

using namespace std;

extern unique_ptr<tf2_ros::Buffer> tfBuffer;
extern ros::Publisher pub_costmap;
extern string costmap_frame;

extern CostmapState state;
extern CostmapParams params;

void initCostmapModule(ros::NodeHandle &nh);
string normalizeFrameId(string frame_id);
void transformLidarToBaselink(const LidarCluster& st_LidarCluster);

void initializeCostmap(const ros::Time &stamp);
void clearOriginArea();
int clampToRange(int value, int min_val, int max_val);

// LShapeFitting 결과 - 이미 2D (이 함수 필요 x)
// void projectAABB3DTo2D(const Eigen::Vector3f &min_3d,
//                        const Eigen::Vector3f &max_3d,
//                        float &min_2d_x, float &min_2d_y,
//                        float &max_2d_x, float &max_2d_y);

bool isPointInPolygon(int x, int y, const vector<std::pair<int, int>>& polygon);

void fillLShapePolygon(nav_msgs::OccupancyGrid &costmap, 
                       const std::vector<Point2D>& corners, 
                       int8_t cost_value);

bool convertBaselinkToGrid(const nav_msgs::OccupancyGrid &costmap,
                           float baselink_x, float baselink_y,
                           int &grid_x, int &grid_y);

int convertGridToIndex(const nav_msgs::OccupancyGrid &costmap,
                       int grid_x, int grid_y);

void projectAABBToGrid(const nav_msgs::OccupancyGrid &costmap,
                       float min_x, float min_y,
                       float max_x, float max_y,
                       int &grid_x0, int &grid_y0,
                       int &grid_x1, int &grid_y1,
                       float inflation_radius);

// 인덱스로 1칸 채우기 (콜백에서 흐름 노출용)
void fillCostAtIndex(nav_msgs::OccupancyGrid &costmap,
                     int idx,
                     int8_t cost_value);

void Costmap(const LidarCluster& st_LidarCluster);

#endif