#!/bin/bash

# RViz 시각화 실행 스크립트
# visualization_rviz.rviz 설정파일을 사용하여 RViz 실행

echo "Starting RViz with visualization_rviz.rviz..."

# X11 포워딩 설정 확인
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY is not set. Setting to :0"
    export DISPLAY=:0
fi

# OpenGL 관련 환경 변수 설정
export QT_X11_NO_MITSHM=1
export LIBGL_DEBUG=verbose

# 현재 스크립트 디렉토리 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/visualization_2_rviz.rviz"

# RViz 설정 파일 존재 확인
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "Error: RViz config file not found: $RVIZ_CONFIG"
    exit 1
fi

echo "DISPLAY=$DISPLAY"
echo "Running RViz..."

# RViz 실행
rviz -d "$RVIZ_CONFIG"
