# 실행 명령어 정리

모든 명령어는 `~` 경로 사용 — 어느 위치에서든 실행 가능.

---

## 가상환경

```bash
# 가상환경 활성화
source ~/Project/Charging_Robot_Operation/charging_robot/bin/activate

# 비활성화
deactivate
```

- 경로: `~/Project/Charging_Robot_Operation/charging_robot/` (Python 3.10.12)
- 모든 스크립트 실행 전 가상환경 활성화 필수

---

## 단축 변수 (선택)

```bash
P=~/Project/Charging_Robot_Operation
PY=$P/charging_robot/bin/python
```

아래 명령어는 전체 경로로 기재. 위 변수 설정 시 `$PY $P/scripts/main.py`로 축약 가능.

---

## 메인 애플리케이션

```bash
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/main.py
```

GUI 메인 애플리케이션 (Charging Robot Task Manager)

---

## 카메라 / ArUco 마커

```bash
# ArduCam 듀얼 ArUco 마커 인식 + 로봇 TCP 자세 표시/저장
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test_arducam_dual_aruco.py

# RealSense D435 ArUco 감지 테스트 (모든 사전 타입)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test_aruco_detect.py

# 이미지 파일에서 ArUco 감지 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test_aruco_image.py

# IPPE 2해 비교 분석 (5초 통계)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/analyze_ippe_ambiguity.py
```

---

## 캘리브레이션

```bash
# BoofCV 카메라 캘리브레이션
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/calibrate_camera_boofcv.py

# Hand-Eye 캘리브레이션 실행
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/hand_eye_cli.py calibrate --data-dir <dir>

# Hand-Eye 캘리브레이션 결과 비교
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/hand_eye_cli.py compare --data-dir <dir>
```

---

## 로봇 상태 확인

```bash
# 카메라 TCP 위치 읽기 (레지스터 158~169)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/read_robot_position.py

# 로봇 상태 및 주요 레지스터 확인
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/check_robot_status.py

# 레지스터 값 디버깅 (301-306, 351-352)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/debug_registers.py
```

---

## 로봇 이동 명령

```bash
# 회전 절대값 설정 (XYZ 유지, 회전만 변경)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/set_rz.py --rx 90 --ry 0 --rz 90
```

---

## 로봇 테스트 (scripts/test/)

### 이동 테스트

```bash
# 절대 좌표 이동 테스트 (Base 좌표계, CMD 20)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_absolute_move.py

# 상대 이동 테스트 (현재 위치 + 상대값 → CMD 20)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_relative_move.py

# tool.trans 상대 이동 테스트 (CMD 10-13)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_tool_trans.py

# Base 좌표계 절대 이동 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_base_coordinate.py
```

### 회전 테스트

```bash
# TF4 tool.rotx/roty/rotz 개별 회전 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_tf4_tool_rotate.py

# TF1 Rz 회전 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_tool1_rz_rotate.py

# 베이스 좌표계 회전 테스트 (movel 방식)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_base_rotate.py
```

### 프레임 테스트

```bash
# 툴프레임별(TF0-TF5) 위치 읽기 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_toolframe.py

# 베이스프레임 전환 및 절대 이동 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_baseframe.py
```

### 기타 테스트

```bash
# Main_task.prs 통합 테스트 (레지스터 맵 검증)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_main_task.py

# TCP Position 레지스터(158~169) 자동 업데이트 확인
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_read_tcp_position.py

# plane_extractor 단위 테스트 (pytest)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test/test_plane_extractor.py
```

---

## 비전 + 로봇 통합 테스트

```bash
# AR Tag 좌표 테스트 (1/Q Rx, 2/W Ry, 3/E Rz, R 홈등록, H 홈이동)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test_ar_coordinate.py

# AR Tag 법선 정렬 테스트 (로봇만 이동)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test_ar_align.py

# CSV 저장 테스트 (TF4 좌표 저장)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test_csv_save.py

# TF4 TCP 좌표 읽기 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/test_tf4_switching.py
```

---

## 유틸리티 / 분석 도구

```bash
# 3D 좌표계 실시간 시각화 (ArUco + Vision TCP + 로봇 베이스)
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/utils/coordinate_visualizer.py

# CSV 기반 3D 좌표계 시각화
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/utils/coordinate_visualizer.py --csv <file>

# 로봇 Tool Frame 정보 읽기 및 관계 분석
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/utils/robot_tf_investigation.py
```

---

## 센서 단독 테스트

```bash
# ArduCam USB 카메라 단독 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/Sensor/arducam/arducam_controller.py

# RealSense D435 카메라 단독 테스트
~/Project/Charging_Robot_Operation/charging_robot/bin/python ~/Project/Charging_Robot_Operation/scripts/Sensor/d435/d435_controller.py
```

---

## 네트워크 설정

| 항목 | 값 |
|------|-----|
| 로봇 IP | `192.168.0.29` |
| Modbus 포트 | `1502` (Main_task.prs) |
| ArduCam 장치 | `/dev/video6` (device_index=6) |
