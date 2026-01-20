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
// void broadcastDynamicEgoTF(double x, double y, double yaw);

string normalizeFrameId(string frame_id);
int clampToRange(int value, int min_val, int max_val);

void transformLidarToBaselink(const LidarCluster& st_LidarCluster);

void initializeCostmap(const ros::Time &stamp);
void clearOriginArea();

bool isPointInPolygon(int x, int y, const vector<std::pair<int, int>>& polygon);

void fillLShapePolygon(nav_msgs::OccupancyGrid &costmap, 
                       const std::vector<Point2D>& corners, 
                       int8_t cost_value);

bool convertBaselinkToGrid(const nav_msgs::OccupancyGrid &costmap,
                           float baselink_x, float baselink_y,
                           int &grid_x, int &grid_y);

int convertGridToIndex(const nav_msgs::OccupancyGrid &costmap,
                       int grid_x, int grid_y);

void fillCostAtIndex(nav_msgs::OccupancyGrid &costmap,
                     int idx,
                     int8_t cost_value);

void Costmap(const Lidar& st_Lidar, const ros::Time& stamp);

#endif
