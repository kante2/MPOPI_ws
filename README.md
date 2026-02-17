# AIM Autonomous Vehicle Planning & Control System

자율주행 차량의 경로 계획 및 제어를 위한 ROS 기반 통합 시스템입니다. 카메라, LiDAR 센서 입력을 활용하여 격자 기반 경로 계획(Lattice Planning)을 수행하고, 제어 신호를 생성합니다.

---

## 📋 프로젝트 개요

### 주요 기능

- **격자 기반 경로 계획 (Lattice Planning)**: 다중 경로 후보 생성 및 최적 경로 선택
- **다중 센서 융합**: 카메라 및 LiDAR 데이터 처리
- **Costmap 기반 충돌 회피**: 동적 장애물 회피
- **추월 구간 인식**: 추월 금지/가능 구간에 따른 모드 전환
- **속도 제어**: 곡률 기반 적응형 속도 조절
- **RViz 시각화**: 경로, costmap, 차량 상태 실시간 시각화
- **UI 기반 모드 선택**: 표준/매드맥스 모드 전환

---

## 📁 프로젝트 구조

```
aim_ws/
├── src/
│   ├── main/                          # 메인 패키지
│   │   ├── src/
│   │   │   ├── Camera/                # 카메라 센서 처리
│   │   │   ├── Lidar/                 # LiDAR 센서 처리
│   │   │   └── PlanningControl/       # 경로 계획 및 제어
│   │   │       ├── Planning/          # 격자 경로 계획 (LatticePlanning.cpp)
│   │   │       ├── Control/           # 제어 로직
│   │   │       ├── Global/            # 전역 데이터 구조
│   │   │       └── Visualizer/        # 시각화 모듈
│   │   ├── scripts/
│   │   │   └── driving_mode_ui.py     # UI 모드 선택 인터페이스
│   │   └── config/
│   │       ├── lattice_planner.rviz   # RViz 설정
│   │       ├── yaml_hybrid.yaml       # 경로 계획 설정
│   │       ├── Path.txt               # 기준 경로
│   │       └── *.csv                  # 추월 구간, 비용맵 설정
│   └── MORAI-ROS_morai_msgs/          # 메시지 정의
├── devel/                              # 빌드 산출물
├── build/                              # CMake 빌드 디렉토리
├── launch scripts                      # 실행 스크립트들
└── visualization_*.rviz                # RViz 설정 파일들
```

---

## 🚀 시작 가이드

### 사전 요구사항

- **OS**: Ubuntu 20.04 LTS
- **ROS**: ROS Noetic
- **Python**: Python 3.8+
- **필수 패키지**:
  - ros-noetic-tf2-sensor-msgs
  - ros-noetic-rosbridge-server
  - ultralytics (YOLOv8)

### 1️⃣ 초기 설정

```bash
cd ~/aim_ws

# 필수 의존성 설치
./init_setting.sh
```

**설치 내용:**
- ROS 센서 메시지 라이브러리
- ROS Bridge 서버 (웹 통신용)
- YOLOv8 (객체 감지)

### 2️⃣ 빌드

```bash
./build.sh
```

또는 수동 빌드:
```bash
cd ~/aim_ws
source devel/setup.bash
catkin_make
```

### 3️⃣ 시스템 실행

프로젝트는 여러 모듈로 구성되어 있으며, 각 모듈을 개별 터미널에서 실행합니다.

#### **터미널 1: 데이터 브리지 시작**
```bash
./bridge.sh
```
- ROS Bridge WebSocket 서버 실행
- 웹/외부 통신 담당

#### **터미널 2: 카메라 센서**
```bash
./camera.sh
```
- 카메라 입력 수집
- 프레임 퍼블리시

#### **터미널 3: LiDAR 센서**
```bash
./lidar.sh
```
- LiDAR 포인트 클라우드 수집
- 3D 데이터 퍼블리시

#### **터미널 4: 경로 계획 및 제어**
```bash
./lattice_planner.sh
```
- 격자 기반 경로 계획 실행
- Costmap 업데이트
- 제어 신호 생성

#### **터미널 5: RViz 시각화**
```bash
./rviz_visualization.sh
```
- 경로, 센서, 차량 상태 시각화
- 실시간 모니터링

#### **터미널 6: UI 모드 선택 (선택사항)**
```bash
./driving_mode_ui.sh
```
- 표준 / 매드맥스 모드 선택
- 주행 전략 변경

---

## 🎛️ 주행 모드

### 1. **표준 모드 (Standard)**
- 안정적인 경로 추종
- 일반적인 속도 제어
- 보수적인 회피 전략

### 2. **매드맥스 모드 (Mad Max)**
- 공격적인 경로 계획
- 고속 주행
- 적극적인 추월 시도

---

## 📊 주요 설정 파일

### `config/yaml_hybrid.yaml`
- 격자 경로 계획 파라미터
- 다항식 차수, 샘플링 간격 등

### `config/Path.txt`
- 기준 경로점 (위도, 경도)
- 글로벌 네비게이션 기준

### `config/*.csv`
- **No_CameraCostmap_zone.csv**: 카메라 비용맵 제외 구간
- **No_LidarCostmap_zone.csv**: LiDAR 비용맵 제외 구간
- **overtaking_zone.csv**: 추월 가능/불가 구간 정의

### `config/lattice_planner.rviz`
- RViz 시각화 설정
- 표시 레이어, 카메라 각도 등

---

## 🔍 핵심 모듈

### Planning (경로 계획)
**파일**: `src/main/src/PlanningControl/Planning/LatticePlanning.cpp`

**주요 함수:**
- `LatticePlanningProcess()`: 메인 경로 계획 루프
- `generateOffsetGoals()`: 다중 경로 목표 생성
- `computeAllPolynomialPaths()`: 다항식 경로 계산
- `evaluateAllCandidates()`: 경로 비용 평가
- `selectBestPath()`: 최적 경로 선택
- `checkOvertakingZone()`: 추월 구간 인식

**특징:**
- 5차 다항식 경로 생성
- 차량 역학 제약 고려
- 동적 costmap 기반 충돌 회피

### Camera (카메라 센서)
**위치**: `src/main/src/Camera/`

**기능:**
- 이미지 입력 수집
- YOLOv8 기반 객체 감지
- 가능한 영역 맵 생성

### Lidar (LiDAR 센서)
**위치**: `src/main/src/Lidar/`

**기능:**
- 포인트 클라우드 처리
- 장애물 탐지
- 3D 비용맵 생성

### Control (제어)
**위치**: `src/main/src/PlanningControl/Control/`

**기능:**
- Stanley 제어기 / Pure Pursuit 등
- 속도 제어
- 스티어링 명령 생성

---

## 📝 로그 및 데이터

### 경로 기록
- `config/track_log.csv`: 실시간 주행 기록
- `config/track_log_recorded.csv`: 기록된 전체 경로
- `config/track_log_recorded_final.csv`: 최종 경로

### ROS Topics
주요 퍼블리시 토픽:
- `/lattice/path`: 계획된 경로
- `/lattice/costmap`: 충돌 회피용 비용맵
- `/odom`: 차량 위치/자세
- `/cmd_vel`: 속도 명령
- `/imu`: IMU 데이터

---

## 🛠️ 트러블슈팅

### 문제: RViz가 실행되지 않음
```bash
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
```

### 문제: Costmap 수신 실패
- 카메라/LiDAR 노드가 실행 중인지 확인
- ROS 마스터 연결 상태 확인
```bash
rosnode list
rostopic list
```

### 문제: 경로 계획이 느림
- `yaml_hybrid.yaml`에서 샘플링 포인트 감소
- 평가 함수 최적화

---

## 📦 빌드 및 설치

### Docker 환경 (권장)
```bash
cd docker-noetic
docker-compose -f docker-compose.pc.yaml up -d
```

### 직접 설치
1. ROS Noetic 설치: http://wiki.ros.org/noetic/Installation
2. 의존성 설치: `./init_setting.sh`
3. 빌드: `./build.sh`

---

## 📄 라이선스 및 참고

- **플랫폼**: ROS (Robot Operating System)
- **기반 기술**: Lattice Planner (경로 계획)
- **센서**: 카메라 + LiDAR 다중 센서 융합
- **시뮬레이터**: MORAI 시뮬레이터 연동 (메시지 정의)

---

## 👥 개발 및 기여

이 프로젝트는 자율주행 차량의 경로 계획 및 제어 연구를 위해 개발되었습니다.

개선 사항 및 버그 보고는 환영합니다.

---

## 📞 주요 실행 명령 요약

| 작업 | 명령어 |
|-----|--------|
| **초기 설정** | `./init_setting.sh` |
| **빌드** | `./build.sh` |
| **데이터 브리지** | `./bridge.sh` |
| **카메라 실행** | `./camera.sh` |
| **LiDAR 실행** | `./lidar.sh` |
| **경로 계획** | `./lattice_planner.sh` |
| **시각화** | `./rviz_visualization.sh` |
| **UI 모드 선택** | `./driving_mode_ui.sh` |

---

**마지막 업데이트**: 2026년 2월
