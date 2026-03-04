#ifndef PARAMETER_LOADER_HPP
#define PARAMETER_LOADER_HPP


void initializeMPOPIState();

// 파라미터 초기화 함수 선언
void initializePlannerParameters();
void initializeControlParameters();

// Waypoint 로드 함수 선언
bool loadWaypoints();
bool load_overtakingZone();
void loadNoCameraZones();
void loadNoLidarZones();
#endif // PARAMETER_LOADER_HPP