# 마커 타입 선택 가이드

Robot Camera Application에서 AprilTag와 ArUco 마커를 선택하여 사용할 수 있습니다.

## UI에서 마커 타입 선택

### 위치
D435 Camera 섹션의 **"Marker Type:"** 드롭다운 메뉴

### 사용 가능한 마커 타입

#### AprilTag (권장)
1. **AprilTag 36h11** ⭐ - 가장 안정적, 현재 사용 중
2. **AprilTag 16h5** - 빠른 감지
3. **AprilTag 25h9** - 중간 성능
4. **AprilTag 36h10** - 높은 정확도

#### ArUco
5. **ArUco 6x6 (DICT_6X6_250)** - 6x6 내부 패턴, 250개 ID
6. **ArUco 4x4 (DICT_4X4_50)** - 4x4 내부 패턴, 50개 ID
7. **ArUco 5x5 (DICT_5X5_100)** - 5x5 내부 패턴, 100개 ID
8. **ArUco 7x7 (DICT_7X7_50)** - 7x7 내부 패턴, 50개 ID

## 사용 방법

### 1. 프로그램 시작 전
프로그램을 시작하면 기본적으로 **AprilTag 36h11**이 선택되어 있습니다.

### 2. 카메라 시작 전에 선택
```
1. 프로그램 실행
2. "Marker Type:" 드롭다운에서 원하는 마커 타입 선택
3. "Start D435" 버튼 클릭
```

### 3. 카메라 실행 중에도 변경 가능
```
1. 카메라가 이미 실행 중인 상태
2. "Marker Type:" 드롭다운에서 다른 마커 타입 선택
3. 즉시 적용됨 (카메라 재시작 불필요)
```

## 현재 설정

### 하드웨어
- **마커 타입**: AprilTag 36h11
- **마커 ID**: 10
- **마커 크기**: 20mm

### 카메라
- Intel RealSense D435
- 해상도: 640x480

## 마커 타입 선택 가이드

### AprilTag를 사용해야 하는 경우:
- ✅ 현재 사용 중인 마커가 AprilTag인 경우
- ✅ 높은 정확도가 필요한 경우
- ✅ 어려운 조명 조건에서 사용하는 경우
- ✅ 로봇 정밀 제어가 필요한 경우

### ArUco를 사용해야 하는 경우:
- ✅ 기존 ArUco 마커가 이미 있는 경우
- ✅ 다른 시스템과의 호환성이 필요한 경우
- ✅ 빠른 감지 속도가 중요한 경우

## 문제 해결

### 마커가 감지되지 않는 경우
1. **올바른 마커 타입 선택 확인**
   - AprilTag 마커 → AprilTag 선택
   - ArUco 마커 → ArUco 선택

2. **"ArUco Detection" 체크박스 활성화**
   - D435 Camera 섹션의 "ArUco Detection" 체크박스를 체크

3. **조명 개선**
   - 균일한 조명
   - 반사 최소화
   - 그림자 제거

4. **마커 크기 확인**
   - 현재 설정: 20mm
   - 실제 마커 크기와 일치하는지 확인

## 로그 확인

프로그램 실행 중 콘솔에서 다음 메시지를 확인할 수 있습니다:

```
✅ Marker type changed to: AprilTag 36h11
🔄 Marker type updated to: AprilTag
```

## 마커 생성

새로운 마커가 필요한 경우:

### AprilTag 생성
웹사이트에서 다운로드:
- https://github.com/AprilRobotics/apriltag-imgs

### ArUco 생성
D435_calibration 폴더의 스크립트 사용:
```bash
cd /home/amap/Project/KAIST/D435_calibration
python3 generate_aruco_markers.py
```

## 기술 세부사항

### 코드 위치
- **UI 파일**: `ui/robot_camera_ui.ui`
  - ComboBox 위젯: `markerTypeComboBox`

- **Python 파일**: `robot_camera_app.py`
  - 마커 타입 변경 핸들러: `on_marker_type_changed()`
  - 카메라 컨트롤러 업데이트: `D435CameraController.update_marker_type()`

### 지원되는 OpenCV ArUco 딕셔너리
```python
cv2.aruco.DICT_APRILTAG_36h11  # AprilTag 36h11
cv2.aruco.DICT_APRILTAG_16h5   # AprilTag 16h5
cv2.aruco.DICT_APRILTAG_25h9   # AprilTag 25h9
cv2.aruco.DICT_APRILTAG_36h10  # AprilTag 36h10
cv2.aruco.DICT_6X6_250         # ArUco 6x6
cv2.aruco.DICT_4X4_50          # ArUco 4x4
cv2.aruco.DICT_5X5_100         # ArUco 5x5
cv2.aruco.DICT_7X7_50          # ArUco 7x7
```

## 참고 자료

- AprilTag 공식 문서: https://april.eecs.umich.edu/software/apriltag
- OpenCV ArUco 문서: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
