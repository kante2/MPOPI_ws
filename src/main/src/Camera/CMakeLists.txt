cmake_minimum_required(VERSION 3.0.2)
project(Camera)

## 컴파일에 필요한 의존성 패키지를 찾습니다.
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  # image_transport # 필요하다면 추가
)

## 시스템 의존성 (OpenCV 등) - 파이썬 전용 노드라 필수는 아니지만 명시
find_package(OpenCV REQUIRED)

## 파이썬 모듈 설정 (setup.py가 있을 경우 사용, 현재는 스크립트만 있으므로 생략 가능)
# catkin_python_setup()

## 런타임 의존성 선언
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Camera
  CATKIN_DEPENDS rospy sensor_msgs std_msgs cv_bridge
#  DEPENDS system_lib
)

## [중요] 파이썬 스크립트 설치 설정
## src 폴더 안에 있는 lane_drive.py를 실행 가능한 프로그램으로 설치합니다.
catkin_install_python(PROGRAMS
  Camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)