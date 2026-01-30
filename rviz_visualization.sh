#!/bin/bash

# RViz 시각화 실행 스크립트
# visualization_rviz.rviz 설정파일을 사용하여 RViz 실행

echo "Starting RViz with visualization_rviz.rviz..."

# 현재 스크립트 디렉토리 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/visualization_rviz.rviz"

# RViz 설정 파일 존재 확인
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "Error: RViz config file not found: $RVIZ_CONFIG"
    exit 1
fi

# RViz 실행
rviz -d "$RVIZ_CONFIG"
