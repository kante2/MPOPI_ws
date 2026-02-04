cmake_minimum_required(VERSION 3.0.2)
project(Camera)

## 컴파일에 필요한 의존성 패키지를 찾습니다.
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp       ## c++ 코드 추가
  sensor_msgs
  std_msgs
  cv_bridge
  nav_msgs     ## Costmap용
  image_transport ## 이미지 전송용
)

## 시스템 의존성 (OpenCV 등) - 파이썬 전용 노드라 필수는 아니지만 명시
find_package(OpenCV REQUIRED)

## 파이썬 모듈 설정 (setup.py가 있을 경우 사용, 현재는 스크립트만 있으므로 생략 가능)
# catkin_python_setup()

## 런타임 의존성 선언
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Camera
  CATKIN_DEPENDS rospy roscpp sensor_msgs std_msgs nav_msgs cv_bridge image_transport
#  DEPENDS system_lib
)

## [중요] 파이썬 스크립트 설치 설정
catkin_install_python(PROGRAMS
  Camera.py
  bev_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# [추가] C++ 노드 빌드 설정
# 1. 헤더 파일 경로 (OpenCV 포함)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 2. 실행 파일 생성 (camera_costmap이라는 이름으로 만듦)
# 주의: src/camera_costmap.cpp 파일이 해당 경로에 있어야 합니다.
add_executable(camera_costmap camera_costmap.cpp)

# 3. 라이브러리 링크 (OpenCV 및 ROS 라이브러리 연결)
target_link_libraries(camera_costmap ${catkin_LIBRARIES} ${OpenCV_LIBS})
