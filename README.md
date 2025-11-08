# Charging Robot Operation

듀얼 카메라 통합 충전 로봇 제어 시스템

## 개요

이 프로젝트는 충전 로봇을 위한 GUI 기반 제어 시스템을 제공합니다:
- Intel RealSense D435 카메라 (원거리 시야)
- ArduCam-40 카메라 (근거리 시야)
- Modbus를 통한 로봇 TCP (Tool Center Point) 제어
- ArUco 마커 및 ChArUco 보드 감지

## 프로젝트 구조

```
Charging_Robot_Operation/
├── config/                      # 카메라 캘리브레이션 파일
│   ├── ds435_calibration.yaml
│   └── arducam40_calibration.yaml
├── lib/                         # 라이브러리 모듈
│   ├── aruco.py                # ArUco/ChArUco 포즈 추정
│   ├── arducam.py              # ArduCam 컨트롤러
│   ├── robot_controller.py     # 로봇 TCP 컨트롤러
│   └── __init__.py
├── scripts/                     # 독립 실행 스크립트
│   ├── ds435_ar_tag_detect.py
│   └── ds435_ar_tag_detect_with_csv.py
├── ui/                          # UI 정의 파일
│   └── robot_camera_ui.ui
├── robot_camera_app.py          # 메인 애플리케이션
└── README.md
```

## 주요 기능

### 듀얼 카메라 시스템
- D435 카메라 (원거리 시야)
- ArduCam (근거리 시야)
- 실시간 비디오 스트리밍
- ArUco 마커 감지
- ChArUco 보드 감지

### 로봇 제어
- TCP 위치 읽기
- TCP 위치 제어
- Modbus TCP 통신

### GUI 애플리케이션
- PyQt5 기반 사용자 인터페이스
- 독립적인 카메라 제어
- 스냅샷 캡처
- 실시간 감지 시각화

## 필요 라이브러리

- Python 3.8+
- Intel RealSense SDK 2.0
- OpenCV with ArUco module
- PyQt5
- PyModbus
- NumPy
- PyYAML

## 설치

```bash
pip install pyrealsense2 opencv-contrib-python PyQt5 pymodbus numpy pyyaml
```

## 사용 방법

### 메인 애플리케이션 실행

```bash
cd /home/amap/Project/KAIST/Charging_Robot_Operation
python3 robot_camera_app.py
```

### 독립 스크립트 실행

ArUco 마커 감지:
```bash
cd scripts
python3 ds435_ar_tag_detect.py
```

CSV 기록 포함:
```bash
cd scripts
python3 ds435_ar_tag_detect_with_csv.py
```

## 설정

### ArUco 감지 설정
- Dictionary: DICT_4X4_50
- 마커 크기: 2cm (설정 가능)

### ChArUco 보드 설정
- 그리드 크기: 8x6
- 사각형 크기: 5cm
- 마커 크기: 3.5cm

### 로봇 연결
- 프로토콜: Modbus TCP
- 기본 IP: 192.168.0.29
- 기본 포트: 1502

## 컴포넌트

### D435CameraController
캘리브레이션된 파라미터와 ArUco/ChArUco 감지 기능을 가진 Intel RealSense D435 카메라 컨트롤러

### ArduCamController
캘리브레이션된 파라미터와 마커 감지 기능을 가진 ArduCam-40 카메라 컨트롤러

### RobotTCPController
Modbus TCP 프로토콜을 통한 로봇 이동 제어 및 TCP 위치 읽기/제어

### ArucoCameraPoseEstimator
포즈 추정 기능을 가진 ArUco 마커 및 ChArUco 보드 감지

## 캘리브레이션 데이터

### D435 카메라
- 시리얼 번호: 207522071359
- 해상도: 640x480
- 재투영 오차: 0.208 pixels

### ArduCam-40
- 해상도: 1280x720
- 재투영 오차: 0.0

## 라이선스

KAIST Research Project
