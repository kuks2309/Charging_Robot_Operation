# Issues and Fixes Log

---

## 2026-02-07 | ArUco 마커 크기(marker_size) 오설정으로 위치 추정 부정확

**증상:** ArUco 마커가 검출은 되지만 위치(특히 Z축 깊이)가 부정확함

**원인:** `VisionManager` 기본 `marker_size_meters=0.03` (30mm)으로 설정되어 있었으나, 실제 마커 물리적 크기는 15mm. 2배 차이로 solvePnP의 Z축 추정이 약 2배 과대 계산됨.

**수정 파일:** `scripts/services/vision_manager.py`

**수정 내용:**
- `marker_size_meters` 기본값: `0.03` → `0.015` (15mm)
- `update_marker_size()` 내 하드코딩된 `dictionary_type` → `self._dictionary_type`으로 변경 (인스턴스 설정 재사용)

**추가 확인:**
- ArUco 딕셔너리: `DICT_5X5_50` (7x7 전체 셀, 내부 5x5) — 기존 설정 정확
- 캘리브레이션 파일(`config/arducam_calibration.yaml`)이 D435용 데이터를 사용 중 — ArduCam 전용 캘리브레이션 필요 여부 검토

---

## 2026-02-07 | TF5 툴프레임 설정 오류 수정

**증상:** 모션 테스트 탭에서 TF5 라디오 버튼 선택 시 "툴프레임 설정 실패: 잘못된 툴프레임 번호: 5 (0-4만 가능)" 오류 발생

**원인:** `scripts/Robot/communication/modbus_client.py`의 `send_set_toolframe()` 메서드에서 frame=5 케이스가 누락됨. `CMD_TOOLFRAME_5 = 46`은 이미 정의되어 있었으나 validation 로직에 포함되지 않음.

**수정 파일:** `scripts/Robot/communication/modbus_client.py` (line 512-515)

**수정 내용:**
- `elif frame == 5: cmd = self.CMD_TOOLFRAME_5` 분기 추가
- 에러 메시지 범위 "0-4만 가능" → "0-5만 가능" 변경
- 독스트링 업데이트: 유효 범위 0-5, TF5는 Hand-Eye Calibration 용도 명시

**확인:** 로봇 티치펜던트에서 WO T5 (X=29.23, Y=80.24, Z=23.40) 존재 확인됨

---

## 2026-02-07 | 분포 그래프 서브탭 추가 (Raw Data / Outlier 제거)

**요청:** 분포 그래프 영역에서 원본 데이터와 이상치 제거 데이터를 비교할 수 있도록 서브탭 분리

**수정 파일:** `scripts/tabs/tab_aruco_reliability.py`

**수정 내용:**
- `__init__`: 단일 Figure/Canvas를 QTabWidget + 2개 Figure/Canvas로 교체 ("Raw Data", "Outlier 제거" 탭)
- `_draw_distribution()`: 2x3 scatter+histogram 공통 드로잉 헬퍼 신규 추가
- `_plot_graphs()`: Raw Data 탭은 전체 원본 데이터, Outlier 제거 탭은 2σ 단일 패스 필터링 후 데이터 표시. 탭 이름에 제거된 샘플 수 표시
- `_clear_graphs()`: 양쪽 Figure 초기화 + 탭 이름 리셋
- `_on_export_graph()`: 현재 활성 탭의 figure 기준으로 이미지 내보내기

**확인:** Python 구문 검증 통과, Architect 검증 8개 항목 전부 PASS

---

## 2026-02-07 | 스냅샷 저장 시 TypeError 수정 (Qt 시그널-슬롯 인자 불일치)

**증상:** 스냅샷 버튼 클릭 시 `TypeError: TabVision._on_snapshot() takes 1 positional argument but 2 were given` 에러 발생 및 core dump

**원인:** `@require_camera_running` / `@require_robot_connection` 데코레이터의 `wrapper(self, *args, **kwargs)`가 Qt `QPushButton.clicked(bool)` 시그널의 `checked` 인자를 그대로 원래 메서드에 전달. 데코레이터 없이는 Qt가 슬롯의 인자 수를 자동 매칭하지만, `*args` wrapper가 이 메커니즘을 우회하여 추가 인자가 전달됨.

**수정 파일:** `scripts/utils/common.py`

**수정 내용:**
- `import inspect` 추가
- 두 데코레이터 모두 `_n_params = len(inspect.signature(method).parameters) - 1`로 원래 메서드의 매개변수 수를 데코레이션 시점에 계산
- `method(self, *args, **kwargs)` → `method(self, *args[:_n_params], **kwargs)`로 변경하여 필요한 인자만 전달

**영향 범위:** `@require_camera_running`, `@require_robot_connection`을 사용하는 모든 탭 (tab_vision, tab_eye_in_hand, tab_calibration, tab_motion_test, calibration_mixin)

---

## 2026-02-07 | 연속 명령 실행 시 레이스 컨디션으로 로봇 미작동

**증상:** TF 설정 후 tool.rot 회전 명령을 보내면 로봇이 움직이지 않음. 레지스터에 값은 정상 기록되나 실제 동작 없음.

**원인:** PRS 클린업 시퀀스(task_done=2 → task_number=0 → task_done=0)에서 Python이 task_done=2를 감지하여 즉시 다음 명령을 쓰지만, PRS가 아직 task_number=0 쓰기를 완료하지 않아 Python이 쓴 값이 덮어씌워짐.

**수정 내용:**
- 연속 명령 사이에 `time.sleep(0.2)` 삽입 (핸들러 레벨, `main_window.py`)
- `_wait_for_cleanup()`을 `wait_for_done()`에 넣으면 **모든 조그 명령이 깨짐** → 전역 수정 불가, 핸들러 레벨에서만 처리
- `int(angle)` → `int(round(angle))` 변경으로 소수점 각도 반올림 개선 (`modbus_client.py`)

**교훈:** 전역 wait 함수 수정은 사이드이펙트 위험. 연속 명령이 필요한 곳에서만 로컬로 딜레이 삽입.

---

## 2026-02-07 | TF5 → TF4 변경 및 TF 확인 로직 추가

**증상:** TF5(Hand-Eye Cal) 사용 시 큰 TCP 오프셋으로 3축보호장치 작동. tool.rot 명령이 TF4에서는 정상 동작 확인.

**수정 파일:** `scripts/main_window.py`, `scripts/tabs/tab_aruco_reliability.py`

**수정 내용:**
- 정렬 핸들러 (`_on_ar_tag_align_single_axis`, `_on_ar_tag_align_parallel`): `send_set_toolframe(5)` → `send_set_toolframe(4)`
- GroupBox 타이틀: "마커 평행 정렬 (TF5)" → "마커 평행 정렬 (TF4)"
- `_ensure_toolframe(tf)` 헬퍼 메서드 추가
- 조그 이동/회전 핸들러에서 실행 전 `_ensure_toolframe(3)` 호출 (베이스 좌표계 기본 TF3 보장)

**확인:** TF4 + tool.rotx/roty/rotz(1) ×1 스케일링 테스트 통과, 3축보호장치 미작동

**영향 범위:** 모든 조그 및 정렬 명령

---

## 2026-02-07 | tool.rot 스케일링 ×10 오류 및 TF4 테스트 검증

**증상:** `test_tf4_tool_rotate.py`에서 ×10 스케일(`tool.rotx(10)` = 10° 회전) 전송 시 로봇이 과도한 회전 시도 → 타임아웃/3축보호장치 작동. ×1 스케일에서는 정상 동작.

**원인:** PRS `Main_task.prs`의 CMD 14-17은 레지스터 값을 **그대로 도 단위로** 사용 (`tool.rotx(Rx)`). ×10 스케일링은 CMD 20 (movel)에만 적용 (`x/10, y/10...`). 기존 `test_tool1_rz_rotate.py`가 ×10을 사용한 것은 CMD 20 방식과 혼동.

**테스트 파일:** `scripts/test/test_tf4_tool_rotate.py`

**테스트 결과 (TF4, ×1 스케일):**

- `tool.rotx(1)`: dRx=+1.00° (정확)
- `tool.roty(1)`: dRy=-0.76°, dRz=+0.65° (크로스축 커플링 - 툴프레임 회전 특성상 정상)
- `tool.rotz(1)`: dRy=-0.65°, dRz=-0.76° (크로스축 커플링 - 정상)
- 3축보호장치 미작동

**결론:** tool.rot 명령은 반드시 **×1 스케일링** 사용. TF4에서 안전하게 동작 확인됨.

---

## 2026-02-12 | GUI ArUco 신뢰성 탭 시각화 이상 — test_arducam_dual_aruco.py와 코드 정밀 비교

**증상:** ArUco 신뢰성 탭(GUI)에서 ID:1 마커 주변에 비정상적인 큰 빨간 사각형 표시. 동일 카메라·동일 마커에서 `test_arducam_dual_aruco.py`는 정상 동작.

### 정밀 코드 비교

#### 1. DetectorParameters — 기본값 vs 커스텀

| 파라미터 | test script (기본값) | GUI aruco_detector.py |
|----------|---------------------|----------------------|
| `cornerRefinementMethod` | `CORNER_REFINE_NONE` | `CORNER_REFINE_SUBPIX` |
| `cornerRefinementWinSize` | 5 | 7 |
| `cornerRefinementMaxIterations` | 30 | 50 |
| `cornerRefinementMinAccuracy` | 0.1 | 0.01 |
| `minMarkerPerimeterRate` | 0.03 | **0.01** |
| `minCornerDistanceRate` | 0.05 | **0.01** |
| `minDistanceToBorder` | 3 | **1** |
| `minMarkerDistanceRate` | 0.05 | **0.01** |

- test: `test_arducam_dual_aruco.py:108` — `cv2.aruco.DetectorParameters()`
- GUI: `Sensor/aruco/aruco_detector.py:32-60` — 커스텀

#### 2. 좌표계 불일치 버그 (Critical)

**test script (정상):**
```
frame → gray → detectMarkers(gray) → corners
동일 frame에 drawDetectedMarkers(frame, corners) → 좌표 일치 ✓
```
- `test_arducam_dual_aruco.py:194-195,243`

**GUI 탭 (버그):**
```
frame(distorted) → cv2.undistort() → undistorted_frame
undistorted_frame에서 detectMarkers → corners (undistorted 좌표계)
distorted frame에 cv2.line(frame, corners) → 좌표 불일치 ✗
```
- `tab_aruco_reliability.py:481` : undistort 적용
- `tab_aruco_reliability.py:490` : undistorted에서 검출
- `tab_aruco_reliability.py:507` : **distorted frame에 그리기**
- `tab_aruco_reliability.py:513` : distorted frame 표시

#### 3. 시각화 파이프라인 차이

**test script:** 검출·시각화 모두 동일 frame.

**GUI 탭:** `visualize_markers` 결과(`vis_frame`)를 버리고(`_`), undistorted 좌표를 distorted frame에 직접 그림.
- `tab_aruco_reliability.py:490` : `_, markers = detect_markers(undistorted_frame)`
- `tab_aruco_reliability.py:507` : `cv2.line(frame, corners)` — 좌표계 불일치

#### 4. Disambiguation 로직 — 동일

양쪽 모두 Z축 법선 기준(`R[2,2] < 0`) + LM refinement. 차이 없음.

#### 5. 빨간 박스 출처 조사

전체 파이프라인 코드 추적 결과:
- `tab_aruco_reliability.py`: `(0,0,255)` 또는 `cv2.rectangle` 호출 없음
- 탭 색상(`line 493`): tag_id1→`(0,255,0)` 초록, tag_id2→`(255,0,0)` 파랑(BGR)
- `visualize_markers`: `image.copy()` 기반, 원본 미수정
- `detect_and_estimate_pose`: 입력 이미지 그리기 없음
- 전체 `scripts/` 검색: 해당 파이프라인에서 빨간 사각형 코드 미발견

**현재 코드에서 빨간 박스 출처 특정 불가.** `git status`에서 `aruco_detector.py` 수정 상태 — 스크린샷 시점 코드와 다를 가능성.

### 수정 방향

1. **좌표계 통일**: undistorted frame에서 검출 → undistorted frame에 시각화 → undistorted frame 표시
2. **DetectorParameters**: `minMarkerPerimeterRate` 등 기본값 수준 복원

---

## 2026-02-12 | ArUco Euler rx 방향 불일치 + ±180° 정규화 누락 (Critical)

**증상:** `test_arducam_dual_aruco.py`는 ArUco rx가 카메라 TCP rx와 일치하지만, 메인 앱 여러 모듈에서 rx 방향이 반전되거나 ±180° 경계에서 부호 불일치.

### 문제 1: R vs R.T — Euler 추출 방향 불일치

`solvePnP` → `R = cv2.Rodrigues(rvec)` → R은 **마커→카메라** 변환.
`aruco_detector.py`에서 `camera_rotation = R.T` (카메라→마커)도 저장.

| 파일 | 사용 값 | 좌표계 | 비고 |
|------|---------|--------|------|
| `test_arducam_dual_aruco.py:231` | `R` | 마커→카메라 | **기준 (정상)** |
| `tab_aruco_reliability.py:1809` | `R` (rvec→R) | 마커→카메라 | 정상 |
| `main_window.py:1142` | `rotation_matrix` = `R` | 마커→카메라 | 정상 |
| `aruco_detector.py:764` (시각화) | `camera_rotation` = **R.T** | 카메라→마커 | **반전** |
| `data_collector.py:130` | `camera_rotation` = **R.T** | 카메라→마커 | **반전** |
| `alignment_service.py:204,295,383` | `camera_rotation` = **R.T** | 카메라→마커 | **반전** |

### 문제 2: rx ±180° 정규화 누락

마커가 카메라를 향할 때 rx ≈ ±180°. 정규화 없이 -172°와 +194°가 혼재:
- 통계(mean/std) 무의미 (평균≈+10°, 표준편차≈180°)
- outlier 제거에서 wrapping을 이상치로 오판
- 정렬 보정량 부호 오류 가능

### 반영 안 된 근본 원인

`aruco_detector.py`가 `camera_rotation`(R.T)과 `rotation_matrix`(R) **두 가지를 모두 반환**하는데, 소비측이 어떤 것을 써야 하는지 규칙이 없었음. 시각화는 R.T를, 정렬도 R.T를 그대로 가져다 씀. 실제 로봇 보정에 필요한 것은 R → **용도별 좌표계 규칙 부재**가 근본 원인.

### 수정 방향

1. **R.T → R 통일**: euler 추출 시 `rotation_matrix`(R) 사용으로 통일
2. **rx 정규화**: euler 추출 후 `rx = rx % 360` (양수 방향 보장)
3. **규칙 명문화**: IPPE Z축 disambiguation + rx 양수 정규화 = 카메라 TCP rx 일치

---

## 2026-02-12 | IPPE Bimodal 문제 — Z축 disambiguation만으로 불충분 (Critical)

**증상:** `test_arducam_dual_aruco.py` 50회 연속 측정에서 ID1 마커의 rx가 두 값 사이를 프레임마다 점프.

### 실측 데이터 (로봇 고정, 마커 고정)

| 마커 | 축 | 안정성 | 값 |
|------|-----|--------|-----|
| ID0 | rx | 안정 | ~189° (184.9~192.3°) |
| ID0 | ry | 약간 산포 | ~5° (2.3~8.3°) |
| ID1 | rx | **bimodal** | 모드A: ~166° / 모드B: ~193° |
| ID1 | ry | **bimodal** | 모드A: ~13° / 모드B: ~6° |

- ID0는 안정, ID1만 28° 점프 (166° ↔ 193°)
- 물리적으로 동일 평면 → ID1도 ID0처럼 ~190°가 정답
- 모드A(166°)는 **틀린 IPPE 해**, 모드B(193°)가 **맞는 해**

### 원인

현재 disambiguation: `R[2,2] < 0` (Z축이 카메라를 향하는 해 선택).
ID1의 두 IPPE 해 **모두** `R[2,2] < 0` 조건을 만족 → 구별 불가 → solution[0]을 그대로 사용 → 프레임마다 sol0/sol1이 뒤바뀌며 점프.

ID0는 카메라 중앙에 가까워 disambiguation 성공, ID1은 가장자리에 위치하여 실패.

**참고:** 이 문제는 `rx % 360` 정규화와 별개. 두 값 모두 양수(166°, 193°)이므로 정규화로는 해결 불가.

### 다중 알고리즘 비교 검증 (50프레임)

3개 알고리즘(IPPE_SQUARE, SQPNP, ITERATIVE)을 동일 프레임에서 비교한 결과:

| 알고리즘 | 솔루션 수 | ID1 rx 값 | 비고 |
|----------|-----------|-----------|------|
| IPPE_SQUARE | 2 | 166° / 193° | 기본 |
| SQPNP | 2 | 166° / 193° | 동일 bimodal |
| ITERATIVE | 1 (가끔 2) | 166° 또는 193° | 가끔 3번째 로컬 미니멈(170.5°) |

**결론**: Bimodal은 IPPE 고유 문제가 아님. **solvePnP 공통 문제**.

### 듀얼 마커 거리 제약 검증

IPPE 2해 × 2마커 = 4가지 조합의 마커 간 거리 비교:

| 조합 [ID0솔, ID1솔] | 거리 | 비고 |
|---------------------|------|------|
| [0,0] | ~59mm | 동일 |
| [0,1] | ~59mm | 동일 |
| [1,0] | ~59mm | 동일 |
| [1,1] | ~59mm | 동일 |

**결론**: IPPE 두 해는 **tvec이 거의 동일**하고 rvec만 다름. 거리 제약으로 disambiguation **불가능**.

### KNOWN_MARKER_DISTANCE 오류 발견

- 기존 `KNOWN_MARKER_DISTANCE = 0.055` (55mm) → 실측 **59mm**
- Raw IPPE tvec로 계산한 거리 ~59mm가 정확, LM refinement 후 ~55mm는 **과보정**
- `solvePnPRefineLM()`이 tvec을 ~4mm 이동시킴 → LM이 오히려 정확도 저하

### reproj_error 검증

- "틀린" 해가 "맞는" 해보다 낮은 reproj_error를 보이는 프레임 다수 존재
- reproj_error 기반 disambiguation **신뢰 불가**

### 해결 방향 (업데이트)

1. ~~reproj_error 비교~~ → **신뢰 불가 확인**
2. ~~듀얼 마커 거리 제약~~ → **tvec 동일하여 불가능**
3. **Coplanarity 제약**: 두 마커 동일 평면 → rx 차이 최소인 조합 선택
4. **Temporal consistency**: 이전 프레임 해와 가까운 해 선택 (rx 점프 방지)

---

## 2026-02-13 | ArUco 탭 Set Detection Pose 구현 및 TF5 보호정지 재발

**증상:** ArUco 신뢰성 검증 탭에 Set Detection Pose 버튼 기능 추가 시, TF5 설정으로 3축 보호정지 재발. movel 절대 이동 시 4축 오류 추가 발생.

**경위:**
1. 사용자 요청으로 ArUco 탭에 TF5 설정 → 조그(tool.rot)에서 3축 보호정지
2. `send_move_to_pose`로 절대 이동 시도 → 4축 한계 오류
3. 증분 `send_base_rotate` 시도 → 사용자 의도와 불일치 (절대 이동 원함)
4. `set_rz.py` 참조 후 직접 레지스터 쓰기 방식으로 해결

**근본 원인 (반복 실수):**
- 작업 시작 전 `issues_and_fixes.md` 미참조 (2026-02-07 TF5→TF4 이력 무시)
- MEMORY.md 경고 사항 미적용 ("TF5 → 3축 보호정지, TF4 사용")
- 기존 검증 코드(`set_rz.py`) 미분석, 시행착오 접근

**수정 파일:** `scripts/tabs/tab_aruco_reliability.py`, `scripts/main_window.py`, `scripts/Robot/communication/modbus_client.py`

**수정 내용:**

1. **Set Detection Pose 구현** (`tab_aruco_reliability.py`):
   - `btnSetDetectionPose` 시그널 연결 + `_on_set_detection_pose()` 핸들러
   - Detection Pose UI: Rx/Ry/Rz 스핀박스 (기본값 90/0/90°, 편집 가능)
   - `set_rz.py` 방식: TF3 전환 → `read_current_pose()` → 레지스터 301~306 직접 쓰기 → CMD 20 → TF4 복귀

2. **ArUco 탭 TF: TF5 → TF4** (`main_window.py`):
   - `_on_tab_changed()`: ArUco 탭 선택 시 TF4
   - `_setup_robot_connection()`: 연결 시 초기 TF4

3. **로봇 연결 검증 개선** (`modbus_client.py`):
   - `_verify_communication()`: TCP 포즈 float32 파싱 + NaN/Inf 검증
   - 대체: 상태 레지스터(352) 읽기 폴백

4. **캘리브레이션 파일 교체**: `config/arducam_calibration.yaml` → 2026-02-13 신규 캘리브레이션

**교훈 (재발 방지):**
- **작업 시작 전 `issues_and_fixes.md` 필수 참조**
- **MEMORY.md 경고와 사용자 요청 교차 검증** (TF5 요청 시 즉시 경고)
- **기존 작동 코드 먼저 분석** (`set_rz.py` 등 레퍼런스 확인 후 동일 패턴 적용)
- **API 메서드 존재 여부 사전 확인** (`read_tcp_pose` 미존재 → `read_current_pose` 사용)

---

## 2026-02-14 | main.py에서 오른쪽 ArUco 마커 인식 실패 (DetectorParameters 원인)

**증상:** `python main.py` (ArduCam 선택)에서 오른쪽 ArUco 마커(ID:1) 인식 불가. `python scripts/test_arducam_dual_aruco.py`에서는 양쪽 모두 정상 인식. 02-12부터 반복 보고된 이슈.

**근본 원인:** `ArucoCameraPoseEstimator`의 커스텀 `DetectorParameters`가 이미지 가장자리 마커 검출 실패 유발. 특히 `CORNER_REFINE_SUBPIX`가 핵심 원인.

### 관련 이력 (반복 이슈)

| 날짜 | 이슈 | 관련 원인 |
|------|------|-----------|
| 02-07 | marker_size 오설정 → Z축 부정확 | ArUco 설정 문제 시작 |
| 02-12 | GUI에서 ID:1 비정상 큰 사각형 | undistort 좌표 ↔ distorted frame 불일치 |
| 02-12 | DetectorParameters 차이 최초 발견 | CORNER_REFINE_SUBPIX 지목했으나 미수정 |
| 02-14 | 오른쪽 마커 인식 실패 (본 이슈) | DetectorParameters 근본 수정 |

### 테스트 스크립트 vs 메인 앱 파이프라인 비교

| 파라미터 | test_arducam (정상) | ArucoCameraPoseEstimator (실패) | 영향 |
|----------|--------------------|---------------------------------|------|
| cornerRefinementMethod | CORNER_REFINE_NONE (기본) | **CORNER_REFINE_SUBPIX** | **가장자리 코너 검출 실패** |
| polygonalApproxAccuracyRate | 0.03 (기본) | **0.05** | 마커 윤곽 근사 느슨 |
| minMarkerPerimeterRate | 0.03 (기본) | **0.01** | 너무 작은 후보 허용 |
| minCornerDistanceRate | 0.05 (기본) | **0.01** | 코너 간 거리 제약 약화 |
| minDistanceToBorder | 3 (기본) | **1** | 가장자리 허용하지만 SUBPIX가 실패 |
| solvePnP 알고리즘 | IPPE_SQUARE + Z축 disambig + LM | **동일** | 차이 없음 |

### 디버깅 과정

#### 시도 1: undistort 제거 — 실패

```
가설: cv2.undistort()가 극단적 왜곡 계수(k2=9.89, k3=-218.6)로
      가장자리 이미지를 파괴 → 오른쪽 마커 검출 실패
수정: tab_aruco_reliability.py에서 undistorted_frame → frame (원본 프레임으로 검출)
결과: 실패. 여전히 오른쪽 마커 미검출
실패 원인: undistort는 부차적 문제. 근본 원인은 DetectorParameters
```

#### 시도 2: tab에 직접 검출기 구현 (vision_manager 우회) — 검출 성공, 설계 불량

```
가설: ArucoCameraPoseEstimator의 커스텀 DetectorParameters가 원인
수정: tab_aruco_reliability.py에 DEFAULT DetectorParameters 검출기를 직접 구현
      (_init_direct_detector, _detect_markers_direct 메서드 추가)
결과: 검출 성공! 양쪽 마커 모두 인식됨
문제: vision_manager를 우회하는 구조 → 코드 중복, 유지보수 불량
      같은 검출기가 ArucoCameraPoseEstimator + tab 두 곳에 존재
```

#### 시도 3: ArucoCameraPoseEstimator 근본 수정 — 최종 성공

```
수정: ArucoCameraPoseEstimator.__init__에서 커스텀 파라미터 20줄 전체 제거
      → cv2.aruco.DetectorParameters() DEFAULT 사용
      → tab의 우회 코드(_init_direct_detector, _detect_markers_direct) 제거
      → vision_manager 정상 경로 복원
결과: 성공. ArucoCameraPoseEstimator를 사용하는 모든 곳에서 자동 적용
```

### 수정 파일

1. `scripts/Sensor/aruco/aruco_detector.py` — 근본 수정
2. `scripts/tabs/tab_aruco_reliability.py` — vision_manager 복원

### 수정 내용

1. **`ArucoCameraPoseEstimator.__init__`**: 커스텀 DetectorParameters 전체 제거 → `cv2.aruco.DetectorParameters()` DEFAULT 사용
2. **`tab_aruco_reliability.py`**: 우회 코드 제거, `vision_manager.detect_markers()` 정상 사용 복원

**핵심 결론:** `CORNER_REFINE_SUBPIX`가 이미지 가장자리 마커의 서브픽셀 코너 보정 시 실패. DEFAULT 파라미터(`CORNER_REFINE_NONE`)를 사용하면 해결됨. solvePnP 알고리즘(IPPE_SQUARE + Z축 disambiguation + LM refinement)은 이미 테스트 스크립트와 동일하므로 변경 불필요.

**교훈:** 02-12에 DetectorParameters 차이를 이미 발견했지만, undistort 문제에만 집중하여 근본 원인 수정이 지연됨. 증상이 아닌 원인을 추적할 것.

---

## 2026-02-14 | DetectorParameters 수정 복원 누락 (복원 후 재발)

**증상:** 02-14 수정 이후 코드 복원(git checkout 등) 과정에서 `aruco_detector.py`의 커스텀 DetectorParameters가 되돌려짐. 메인 앱에서 오른쪽 마커(ID:1) 검출 실패 재발. `test_arducam_dual_aruco.py`는 정상.

**원인:** 복원 과정에서 02-14 수정(커스텀 DetectorParameters 제거)이 반영되지 않아 `CORNER_REFINE_SUBPIX` 등 커스텀 파라미터가 다시 적용됨.

**수정 파일:** `scripts/Sensor/aruco/aruco_detector.py`

**수정 내용:**
- `ArucoCameraPoseEstimator.__init__`: 커스텀 DetectorParameters 28줄 제거 → `cv2.aruco.DetectorParameters()` DEFAULT 사용 (02-14 수정 재적용)
- AprilTag 분기(`CORNER_REFINE_APRILTAG`)만 유지

**검증:** Python import 성공

**기존 수정 적용 상태 확인:**
- `marker_size_meters=0.015`: 적용됨 (vision_manager.py, aruco_detector.py)
- `self._dictionary_type` in `update_marker_size()`: 적용됨 (vision_manager.py)
- `arducam_calibration.yaml` 신규 캘리브레이션: 적용됨 (staged)

**교훈:** 코드 복원/체크아웃 후 반드시 `issues_and_fixes.md`의 최근 수정 사항 전체를 재검증할 것.

---

## 2026-02-14 | ArUco 정렬 탭 기능 확장 (2D/3D 각도, Depth, 보정 버튼)

**구현 내용:**

1. **마커 중심 연결선 + 중점 표시**: 두 마커 중심을 빨간 라인으로 연결, 중점에 녹색 십자 표시
2. **이미지 중심 십자선**: 회색 가로/세로 축으로 정렬 기준선 표시
3. **Ry 각도 (2D/3D)**: 픽셀 기반 2D + solvePnP tvec 기반 3D 기울기 표시 (CCW+ 관례)
4. **Rz 각도 (3D)**: tvec Z 차이 기반 깊이 방향 기울기 표시
5. **Depth/ArUco Z 비교**: 각 마커 중심의 RealSense depth (Z_d) 와 solvePnP tvec Z (Z_ar) 표시
6. **TCP ry 보정 버튼**: `send_base_rotate('ry', angle)` 호출 → base movel로 Ry 보정
7. **TCP rz 보정 버튼**: Rz 보정 + Y 보정 (`ΔY = D × tan(ΔRz)`) 적용 → `send_move_to_pose`
8. **dY/dZ 오프셋 표시**: 이미지 중심 대비 마커 중점의 픽셀 오프셋 (가로=dY, 세로=dZ)

**수정 파일:**
- `scripts/tabs/tab_aruco_reliability.py` — UI, 프레임 처리, 보정 핸들러
- `scripts/Sensor/aruco/aruco_detector.py` — `detect_marker_centers`에 `estimate_pose` 옵션 추가
- `scripts/services/vision_manager.py` — `estimate_pose` 파라미터 전달
- `scripts/services/camera_manager.py` — `rs.align(rs.stream.color)` 추가, depth 5x5 median 필터
- `scripts/main_window.py` — `align_base_ry_requested`, `align_base_rz_requested` 시그널 핸들러

**해결된 이슈:**
- **Depth 100mm 오차**: `rs.align` 미적용 → color/depth 센서 물리적 오프셋 미보정. `rs.align(rs.stream.color)` 적용 후 ArUco Z와 2-4mm 이내 일치
- **QPushButton UnboundLocalError**: 로컬 import → 탑레벨 import로 이동

---

## 2026-02-14 | 베이스 좌표계 정밀 이동 (0.1mm) 명령 추가

**구현 내용:** 기존 1mm 단위 base translate(CMD 50-53)에 추가로 0.1mm 해상도 정밀 이동 명령 구현

**설계:**
| 항목 | 기존 (mm) | 신규 (0.1mm) |
|---|---|---|
| CMD X/Y/Z | 50/51/52 | 60/61/62 |
| CMD XYZ | 53 | 63 |
| 레지스터 | 301-303 (x,y,z) | 313-315 (fx,fy,fz) |
| PRS | `transx(x)` | `transx(fx/10)` |
| Python | `int(distance)` | `int(round(distance*10))` |

**수정 파일:**
- `Robot_scripts/robot_scripts/Main_task.prs` — CMD 60-63 추가 (레지스터 fx/fy/fz 읽기, ÷10 적용)
- `scripts/Robot/communication/modbus_client.py` — 상수 + `send_base_fine_translate()` 메서드

**주의:** 로봇 컨트롤러 Modbus 서버에 레지스터 `fx(313)`, `fy(314)`, `fz(315)` 등록 필요

---
