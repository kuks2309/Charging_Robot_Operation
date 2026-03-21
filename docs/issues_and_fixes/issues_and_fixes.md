# Issues and Fixes Log

---

## 2026-03-21 | ±180° Euler 각도 정규화 미적용 → PRS 165번 movel 중단

### 문제

- **PRS line 165 `movel(p1)` 중단**: `send_base_rotate`에서 현재 Euler 각도에 delta를 더한 결과가 ±180° 범위를 초과하면 로봇 컨트롤러가 에러/중단
- 예: 현재 Rz=175°, +10° 회전 → 목표 Rz=185° → ±180° 범위 초과 → PRS movel 에러
- 예: 현재 Rz=-175°, -10° 회전 → 목표 Rz=-185° → 동일 문제
- **Python, PRS 양쪽 모두 각도 정규화 로직 부재**: 값을 그대로 전달하여 로봇 컨트롤러가 거부

### 수정 내용

1. **`normalize_angle()` 정적 메서드 추가** (`modbus_client.py`)
   - 각도를 [-180°, 180°] 범위로 래핑: `angle % 360`, `> 180 → -360`
2. **`send_base_rotate` 정규화 적용** (`modbus_client.py`)
   - 목표 Euler 각도(Rx/Ry/Rz) 계산 후 `normalize_angle()` 적용
3. **`send_move_to_pose` 정규화 적용** (`modbus_client.py`)
   - 절대 좌표 이동 시에도 Rx/Ry/Rz 정규화 (외부 호출자 보호)
4. **PRS CMD 20 이중 보호** (`Main_task.prs`)
   - `movel` 직전 while 루프로 Rx/Ry/Rz를 ±180° 범위로 정규화
   - PRS 호환성을 위해 `%` 연산자 대신 `while > 180 do -360` 패턴 사용
5. **장경로 회전 방지** (`modbus_client.py`)
   - `send_base_rotate`에서 정규화 후 movel delta > 180° 감지
   - 경계 교차 시 CMD 54-56 (증분 회전)으로 자동 전환
6. **NaN/Inf 입력 검증** (`modbus_client.py`)
   - `normalize_angle()`에 `math.isfinite()` 가드 추가
   - 통신 이상 시 부분 레지스터 쓰기 방지
7. **독립 테스트 스크립트 정규화 적용**
   - `test_base_rotate.py`, `test_absolute_move.py`, `test_relative_move.py`,
     `test_baseframe.py`, `test_main_task.py`, `set_rz.py`
   - `int()` → `int(round())` 절삭 오류도 수정
8. **단위 테스트 추가** (`tests/test_normalize_angle.py`)
   - 경계값, 래핑, NaN/Inf, 멱등성, int16 범위 테스트

### 교훈

- **모든 movel 경로에서 Euler 각도 정규화 필수** — 현재값 + delta가 ±180° 경계를 넘을 수 있음
- **Python + PRS 이중 보호**: Python이 1차 정규화, PRS가 2차 안전장치
- **-180°와 180°는 동일 각도** — 정규화 후 부호 차이는 무해
- **movel은 Euler 각도를 선형 보간** — 정규화로 부호 반전 시 장경로(340°) 회전 위험. 경계 교차 감지 필수
- **독립 테스트 스크립트도 정규화 필요** — ModbusClient를 우회하는 직접 레지스터 쓰기 주의

---

## 2026-03-21 | ArUco 신뢰성 탭 TF5 전환 + 순수 회전 모션 감지 실패

### 문제

1. **ArUco 신뢰성 탭에서 TF5 미사용**: 탭 전환 시 TF4로 설정, movel/align 함수들도 TF3→TF4 사용
2. **movel로 RxRyRz 동시 변경 시 보호정지**: TF5의 큰 TCP 오프셋으로 회전+이동 경로가 축 한계 초과
3. **순수 회전 시 타임아웃**: `wait_for_done_motion_aware`가 XYZ만 모션 감지 → 순수 회전(Ry -5° 등) 시 변화=0으로 판단 → idle_timeout 발생
4. **±180° Rz 특이점**: 회전 후 Rz=-180→+180 부호 반전 시 dRz=360°로 오감지

### 수정 내용

1. **탭 전체 TF5 적용** (`tab_aruco_reliability.py`, `main_window.py`)
   - 탭 전환 시 TF5 설정 (`_on_tab_changed`)
   - align 함수: `_ensure_toolframe(5)` 확인 후 실행, 불필요한 TF 복귀 제거
   - detection_pose/set_robot_position: TF5 기준 movel
2. **Set Robot Position 2단계 분리** (`tab_aruco_reliability.py`)
   - 1단계: 현재 XYZ 유지 + 목표 RxRyRz (순수 회전)
   - 2단계: 목표 XYZ + RxRyRz 유지 (순수 병진)
3. **Set Detection Pose 개별 축 회전** (`tab_aruco_reliability.py`)
   - Rx, Ry, Rz 각각 순차 movel (delta > 0.5° 축만)
4. **모션 감지에 회전 포함 + ±180° 래핑** (`modbus_client.py`)
   - `wait_for_done_motion_aware`: XYZ(>0.05mm) + RxRyRz(>0.1°) 모두 감지
   - ±180° 래핑: `d > 180 → d = 360 - d`
5. **UI 라벨 수정** (`tab_aruco_reliability.ui`)
   - "마커 평행 정렬 (TF4)" → "마커 평행 정렬 (TF5)"
6. **Config 좌표 업데이트** (`charging_gun_coupling.json`)
   - detection_pose: X=386.0, Y=550.6, Z=236.1, Rx=90.0, Ry=0.0, Rz=180.0 (TF5 기준)

### 교훈

- TF5는 TCP 오프셋이 크므로 movel 시 회전+이동 동시 변경은 경로 분리 필수
- 모션 감지는 XYZ뿐 아니라 회전도 포함해야 순수 회전 명령에서 타임아웃 방지
- Euler 각도 ±180° 경계는 항상 래핑 처리 필요

---

## 2026-03-21 | Task 편집 탭 TCP Position 그룹박스에 현재 TF 번호 미표시

### 문제

- Task 편집 탭의 TCP Position 그룹박스 타이틀이 항상 "TCP Position"으로 고정
- 현재 어떤 Tool Frame(TF) 기준인지 알 수 없어 혼동 발생 (TF3/TF4 간 TCP 좌표 150mm 이상 차이)
- `update_current_toolframe()` 메서드가 placeholder(`pass`)로 비어있었음

### 수정 내용

- `scripts/tabs/tab_task_edit.py`: `update_current_toolframe()` 구현 — `self.groupTCPPosition.setTitle(f"TCP Position (TF{toolframe})")`
- `main_window.py:1884`에서 이미 호출하고 있으므로 추가 연결 불필요

### 교훈

- TF에 따라 TCP 좌표가 크게 달라지므로, 현재 TF를 항상 명시적으로 표시해야 혼동 방지

---

## 2026-03-21 | Modbus 포트/IP 기본값 불일치 + Detection Pose config 외부화 + 테스트 폴더 통합

### 문제

1. **Modbus 포트 기본값 불일치**: `modbus_client.py` 기본 포트 1502인데, UI(`main_window.ui`, `tab_task_edit.ui`) 기본값도 1502지만, 로봇 IP가 `192.168.0.29`(구)로 하드코딩 → 실제 로봇 IP `192.168.0.39`와 불일치
2. **UI 포트 변경 미반영**: `tab_task_edit.ui`의 기본 IP/포트가 구버전(192.168.0.29:1502)으로 하드코딩되어, 앱 재시작 시 항상 구값으로 리셋
3. **Detection Pose 값 하드코딩**: `tab_aruco_reliability.ui` 스핀박스 기본값(Rx=90, Ry=0, Rz=90)만 사용, 외부 config 없음 → 앱 재시작 시 항상 리셋, XYZ 절대이동 불가
4. **테스트 폴더 이원화**: `scripts/test/`와 `scripts/tests/` 두 폴더에 테스트 분산
5. **btnSetRobotPosition 미존재 시 크래시**: UI 파일 미갱신 상태에서 시그널 연결 시 AttributeError → 탭 전환 불가

### 수정 내용

**Modbus 기본값 통일**
- `modbus_client.py`: 기본 IP `192.168.0.29` → `192.168.0.39`
- `main_window.ui`: 기본 IP `192.168.0.29` → `192.168.0.39`
- `tab_task_edit.ui`: 기본 IP `192.168.0.29` → `192.168.0.39`
- 포트는 1502 유지 (로봇 Modbus 서버 포트 확인 완료, 5001/5002는 TCP 버스용)

**Detection Pose config 외부화**
- `config/charging/charging_gun_coupling.json`에 `detection_pose` 추가 (x, y, z, rx, ry, rz, use_xyz)
- `tab_aruco_reliability.py`: `_load_detection_pose_config()` 메서드 추가, 앱 시작 시 + 버튼 클릭 시 config 재로드
- **Set Detection Pose** 버튼: Rx/Ry/Rz만 변경 (XYZ는 현재 로봇 위치 유지)
- **Set Robot Position** 버튼 신규 추가: config의 XYZ + RxRyRz로 절대 이동 (확인 팝업 포함)
- `hasattr` 가드로 UI 파일 미갱신 시에도 크래시 방지

**테스트 폴더 통합**
- `scripts/test/` → `scripts/tests/`로 12개 파일 이동, 빈 폴더 삭제
- `test_modbus_connection.py` 신규 추가 (포트별 연결 테스트)

### 교훈

- **로봇 하드웨어 포트 확인 필수**: 티치펜던트 설정(5001/5002)과 Modbus 프로토콜 포트(1502)는 별개. 실제 테스트로 검증
- **UI 기본값은 반드시 config와 동기화**: `.ui` 파일 하드코딩 기본값이 있으면, config에서 로드하여 덮어쓰기 필요
- **신규 UI 위젯 참조 시 hasattr 가드 필수**: `.ui` 파일과 `.py` 파일 동기화 불일치 방지

---

## 2026-03-21 | 레이저 캘리브레이션 ROI 로직 전면 수정 + 자동 스캔 재설계

### 문제

1. **ROI config 키 무단 변경**: `center_y_offset_px` → `center_y_px`, `mode: single` → `symmetric` 등 config 파일이 임의로 변경됨 (약 5시간 낭비)
2. **rects[0] 하드코딩**: `tab_laser_calibration.py` 4곳, `laser_scan_service.py` 1곳, `tab_laser_scan.py` 2곳에서 symmetric 모드 시 IndexError 또는 오른쪽 ROI 무시
3. **`_roi_params`, `compute_calib_roi_rects`에서 `center_x_px` 미지원**: `compute_roi_rects`와 draw 함수 간 좌표 불일치
4. **자동 스캔 이동축 오류**: Z축이어야 하는데 X축(`send_base_linear('x', ...)`)으로 구현됨
5. **자동 스캔 범위/스텝 하드코딩**: `x_offsets_mm: [0, -40, -80]` 고정, UI 조정 불가
6. **ROI config 키 하드코딩**: `roi`/`roi_preview` 고정 → config 키 변경 시 코드 수정 필요
7. **z_tcp_mm 기록 버그**: X좌표(pose[0]) 기록 → Z좌표(pose[2])여야 함
8. **Nelder-Mead 파라미터 무제한**: h, alpha, delta_z bounds 없음 → 발산 (h=2e14, RMSE=85px)

### 수정 내용

**ROI 로직 (image_processing.py)**
- `_roi_params`: `center_x_px` 지원 추가
- `compute_calib_roi_rects`: `center_x_px` 지원 추가
- docstring 3개 함수 업데이트 (`center_x_px`/`center_y_px` 키 문서화)

**tab_laser_calibration.py — rects[0] 버그 4곳 수정**
- `_draw_calib_roi_overlay`: `rects[0]` → 전체 rects 반복
- `_filter_by_calib_roi`: `rects[0]` → union 필터 (`np.zeros + |=`)
- `_draw_rgb_overlay`: `rects[0]` → 전체 rects 마스크 병합
- `_run_vert_scan`: `rects[0]` → union bounding box

**tab_laser_calibration.py — ROI config 동적 로딩**
- config 키 동적 읽기 (dict인 키 자동 감지, `_note` 제외)
- `self._all_rois = [(name, config), ...]` 구조로 저장
- overlay에 ROI 이름 표시 (`roi_laser`, `roi_laser_base(L)/(R)`)
- 자동 스캔 시 모든 ROI에서 레이저 검출

**tab_laser_scan.py — IndexError 2곳 수정**
- `_draw_laser_results`: `enumerate(rects)` → `zip(rects, labels)`, 미사용 label 초기화
- `angles[1]` 접근 전 `len(angles) >= 2` guard 추가

**laser_scan_service.py — IndexError + docstring**
- `rects[1]` → `if len(rects) > 1 else None` guard
- docstring: `center_y_px` → `center_y_offset_px (or center_y_px)`, `mode` 키 추가

**자동 스캔 (main_window.py)**
- 이동축: `send_base_linear('x', ...)` → `send_base_linear('z', ...)`
- 스텝 생성: `x_offsets_mm` 리스트 → `scan_range_mm / step_mm`로 자동 생성
- UI `spinScanRange` 추가 (config 저장/로드)
- z_tcp 기록: `pose[0]` → `pose[2]`
- CSV 저장: 모든 ROI의 픽셀 단위 레이저 좌표 (col_px, center_y_px) 기록
- Nelder-Mead bounds 추가: h=[1,500], alpha=[0,30], delta_z=[-2000,2000]

### 삭제된 잘못된 데이터
- `data/laser_scan/calibration/vertical/scan_raw_20260315_102600.csv` (X축 이동 데이터)
- `data/laser_scan/calibration/vertical/calib_vert_20260315_102955.json` (발산 결과)
- `data/laser_scan/calibration/vertical/calib_vert_20260315_103205.json` (발산 결과)
- `config/laser/vertical/laser_vertical_triangulation_calib.json` (발산 캘리브 결과)

### 교훈
- **사용자가 수정한 config 파일을 임의로 변경하지 말 것**
- **ROI config 키는 동적으로 읽어 하드코딩 금지**
- **이동축, 좌표축 등 물리적 의미를 반드시 확인 후 구현**

---

## 2026-03-15 | 레이저 캘리브레이션 탭 수동 캡처 버튼 3개 제거

### 변경 내용

수직 스캔 섹션의 수동 버튼 3개(`위치로 이동`, `캡처 추가`, `클리어`)와 관련 핸들러 제거.

자동 스캔(`btnAutoVertScan`)이 동일 기능을 완전히 대체하므로 수동 버튼 불필요.

### 제거 항목

- **UI** (`ui/tab_laser_calibration.ui`): `btnMoveToVertScan`, `btnScanVertical`, `btnClearVertScan` 위젯
- **Python** (`scripts/tabs/tab_laser_calibration.py`):
  - 3개 signal 연결 제거
  - `_on_add_vert_capture()` 메서드 제거
  - `_set_auto_scan_ui_state()` 내 제거된 버튼 참조 3줄 제거

### 유지 항목

- `_on_move_jig_pos_vertical()` — `btnMoveJigPosVertical`(지그 위치 저장 섹션)이 동일 메서드 사용 중이므로 유지
- `_run_vert_scan()`, `_on_clear_vert_scan()` — `main_window.py` 자동 스캔 경로에서 직접 호출
- `labelVertScanStatus`, `labelCaptureCount` — 자동 스캔 결과 표시에 사용

---

## 2026-03-14 | ROI center_y 하위호환 — center_y_px / center_y_offset_px 혼용 대응

### 문제

`laser_vertical_scan_roi.json`을 `center_y_offset_px: 0` (중심 상대)로 변경했으나,
`laser_horizontal_scan_roi.json`과 `laser_vertical_calib_roi.json`은 여전히 `center_y_px` (절대 위치) 사용.

`_roi_params()`, `compute_calib_roi_rects()`, `compute_roi_rects()` 세 함수 모두
`center_y_offset_px`만 읽도록 되어 있어 레거시 파일에서 Y 위치가 이미지 중심(540px)으로
잘못 계산되는 버그 발생.

### 해결

`utils/image_processing.py` 세 함수에 하위호환 처리 추가:

```python
int(roi_cfg['center_y_px']) if 'center_y_px' in roi_cfg \
    else frame_h // 2 + int(roi_cfg.get('center_y_offset_px', 0))
```

- `center_y_px` 키 존재 시 → 절대 위치 사용 (레거시)
- 없으면 → `frame_h // 2 + center_y_offset_px` (신규)

**변경 파일:** `scripts/utils/image_processing.py` (`_roi_params`, `compute_calib_roi_rects`, `compute_roi_rects`)

---

## 2026-03-14 | config/laser ROI 스키마 통일 — compute_roi_rects 신설

### 문제

두 가지 ROI 스키마 혼재 + 절대 위치와 offset 혼용:

- **Scan ROI** (`laser_vertical_scan_roi.json`, `laser_horizontal_scan_roi.json`): `offset_x`(중심 상대) + `y_offset`(절대 위치) 혼용
- **Calib ROI** (`laser_vertical_calib_roi.json`): `center_x_offset_px`, `width_px`, `center_y_px`, `height_px` 사용
- `tab_laser_calibration.py`가 잘못된 파일(`laser_vertical_calib_roi.json`) 읽음 → `laser_vertical_scan_roi.json` 읽어야 함

### 해결

**통일 스키마** (`center_x` 컨벤션):

```json
{ "roi": { "mode": "symmetric", "center_x_offset_px": 0, "width_px": 280, "center_y_px": 450, "height_px": 900, "color": [0,0,255], "thickness": 6 } }
```

**`compute_roi_rects(frame_w, frame_h, roi_cfg)`** 신설 (`utils/image_processing.py`):

- 통일 스키마 사용, `height_px=0` → 전체 프레임 높이
- `mode="symmetric"`: 좌/우 2개, `mode="single"`: 1개
- `compute_scan_roi_rects`, `compute_calib_roi_rects` → deprecated alias 유지

**변경 파일:**

- `config/laser/vertical/laser_vertical_scan_roi.json`: 새 스키마, `center_y_px=450`
- `config/laser/horizontal/laser_horizontal_scan_roi.json`: 새 스키마, `center_y_px=530`
- `tab_laser_calibration.py`: `_roi_cfg` dict 기반으로 리팩터링, `_calib_roi_bounds()` 제거, `LASER_VERT_SCAN_ROI_FILE` 사용
- `tab_laser_calibration.ui`: `btnCreateROI` → `btnCreateROIVertical` + `btnCreateROIHorizontal`
- `tab_laser_scan.py`, `tab_ai_detection.py`, `laser_scan_service.py`: `compute_roi_rects` 사용
- `test_image_processing.py`: `_STD_CFG` 새 키, `height_px=0` 테스트 추가

---

## 2026-03-14 | 레이저 피팅 직선 그리기 공통화 — draw_laser_fit_line 신설

### 문제

동일한 `np.polyval + cv2.line` 5줄 패턴이 3곳에 중복:

- `tab_laser_calibration._draw_conv_overlay` L582 (PINK)
- `tab_laser_calibration._draw_jig_laser_detect` L900 (CYAN)
- `tab_laser_scan._draw_laser_results` L163 (가변색)

### 해결

`utils/image_processing.py`에 `draw_laser_fit_line` 공용 함수 신설.

```python
def draw_laser_fit_line(frame, coeffs, inlier_cols, color, thickness=2):
    if len(coeffs) < 2 or len(inlier_cols) == 0:
        return
    x0, x1 = int(inlier_cols.min()), int(inlier_cols.max())
    y0 = int(round(np.polyval(coeffs, x0)))
    y1 = int(round(np.polyval(coeffs, x1)))
    cv2.line(frame, (x0, y0), (x1, y1), color, thickness)
```

- `len(coeffs) < 2` 가드를 함수 내부로 통합 (호출부 조건문 제거)
- `tab_laser_calibration._draw_line_overlay` L739 세그먼트 루프는 엔드포인트 원 표시와 결합되어 **제외**

### 수정 파일

- `scripts/utils/image_processing.py` — `draw_laser_fit_line` 추가
- `scripts/tabs/tab_laser_calibration.py` — 2곳 교체, import 추가
- `scripts/tabs/tab_laser_scan.py` — 1곳 교체, import 추가
- `scripts/tests/test_image_processing.py` — 5개 테스트 추가 (총 23개)

---

## 2026-03-14 | AI Detection 탭 — ROI 내 레이저 검출 구현

### 배경
- 기존 `btnLaserAlign`은 Port_T bbox 좌표 기반 동적 노란 ROI를 생성했으나, 이는 요청한 적 없는 코드였음
- 목표: 레이저 정렬 버튼 클릭 시 `laser_vertical_calib_roi.json` ROI 안에서만 레이저를 검출하고, YOLOv8 결과 위에 해당 ROI와 검출 결과를 오버레이

### 구현 내용

#### 1. `LaserState(IntEnum)` — bool 대체 3-상태 열거형
```python
class LaserState(IntEnum):
    OFF = 0           # 레이저 모드 비활성
    ROI_VISIBLE = 1   # ROI 표시만 (정렬 대기)
    ALIGNING = 2      # 정렬 진행 중
```
- 기존 `_laser_mode_active: bool` 완전 제거 → `_laser_state: LaserState`로 교체
- 버튼 토글: OFF→ROI_VISIBLE, ROI_VISIBLE→OFF (정렬 중 클릭은 무시)

#### 2. `compute_calib_roi_rects(frame_w, frame_h, roi_cfg)` — `image_processing.py` 신규
- calib 스키마(`center_x_offset_px/width_px/center_y_px/height_px/mode`) → `[(x0,y0,x1,y1), ...]`
- `mode="symmetric"` (기본): 좌우 대칭 2개 rect 반환
- `mode="single"`: `img_cx + offset` 위치 1개 rect 반환
- scan ROI 스키마(`compute_scan_roi_rects`)와 분리 유지 (config 키 이름 불변)

#### 3. `_last_laser_results` — 쓰레드 안전 검출 결과 공유
```python
self._last_laser_results: dict = {'left': None, 'right': None}
```
- `_run_detection` (백그라운드 스레드)에서 `with self._detect_lock:` 안에 쓰기
- `_measure_laser_dy` (정렬 스레드)에서 동일 락으로 읽기 (shallow copy)

#### 4. `_run_detection` — 레이저 검출 루프 추가
- `_laser_state != LaserState.OFF`이면 매 프레임 ROI별 검출 실행
- `LaserDetectionService.detect_in_roi(frame, rect)` 호출 (left/right)
- 검출 성공 시: `cv2.circle`로 inlier 점 표시 + `draw_laser_fit_line`으로 피팅 직선 (초록)
- `draw_laser_fit_line(frame, coeffs, inlier_cols, color)`: `image_processing.py` 공용 함수 사용

#### 5. `_measure_laser_dy` — Port_T 의존성 제거
- 기존: Port_T bbox 좌표에서 ROI 동적 생성 → 포트 미검출 시 실패
- 변경: 3-phase wait 후 `_last_laser_results` 복사 → `np.polyval(coeffs, center_x)` + `np.mean` 집계
- 반환: `float(laser_y - center_y_px)` (roi_cfg 기준 중심 Y 대비 레이저 Y 편차)

### 수정 파일
- `scripts/tabs/tab_ai_detection.py` — LaserState enum, _last_laser_results, _run_detection 레이저 루프, _on_laser_align_clicked 3-상태 로직, _measure_laser_dy 재작성
- `scripts/utils/image_processing.py` — `compute_calib_roi_rects` 추가

---

## 2026-03-14 | ROI Overlay 공통화 — 탭별 독자 구현 제거

### 문제

ROI overlay 함수가 공용 모듈 없이 각 탭에서 중복 구현되어 있었음.

#### 중복 1 — 같은 config, 다른 구현

- `tab_laser_calibration._draw_calib_roi_overlay`: `laser_vertical_calib_roi.json` 사용, cv2 직접 호출(rectangle×2 + addWeighted + putText)
- `tab_ai_detection`: 동일 config에서 공용 `draw_laser_calib_roi` 사용 — 탭 간 불일치

#### 중복 2 — 스키마 불호환으로 인한 독자 구현

- `tab_laser_scan._compute_roi_rects` + `_draw_roi_boxes`: `laser_vertical_scan_roi.json` 스키마(offset_x/roi_width/y_offset)로 좌표 계산
- `laser_scan_service._capture_laser_data`: 동일 좌표 계산 로직을 독자 재구현 → 화면 ROI와 검출 ROI 불일치 위험

### 해결 방안

config 파일 스키마 변경 없이 공용 함수 2개 신설.

1. **`draw_roi_box(frame, x0,y0,x1,y1, color, thickness, *, alpha, min_thickness, label)`**
   - 좌표 직접 지정 ROI 렌더링 프리미티브
   - `alpha>0`: addWeighted 반투명 fill, `min_thickness>0`: 최소 두께, `label`: putText
   - `draw_roi_single`, `draw_roi_symmetric` 내부가 이 함수를 호출하도록 리팩토링

2. **`compute_scan_roi_rects(frame_w, frame_h, roi_cfg)`**
   - scan ROI 스키마(offset_x/roi_width/roi_height/y_offset)로 좌/우 ROI 좌표 계산
   - 반환: `[(lx0,y0,lx1,y1), (rx0,y0,rx1,y1)]`

### 수정 파일

- `scripts/utils/image_processing.py` — `draw_roi_box`, `compute_scan_roi_rects` 추가; `draw_roi_single`, `draw_roi_symmetric` 내부를 `draw_roi_box` 호출로 교체
- `scripts/tabs/tab_laser_calibration.py` — `_draw_calib_roi_overlay` cv2 직접 호출 → `draw_roi_box` 단일 호출
- `scripts/tabs/tab_laser_scan.py` — `_compute_roi_rects` staticmethod 완전 제거, `_draw_roi_boxes`·`_draw_laser_results` → `compute_scan_roi_rects` 사용
- `scripts/services/laser_scan_service.py` — ROI 좌표 인라인 계산 → `compute_scan_roi_rects` 단일 호출
- `scripts/tests/test_image_processing.py` — 신규 8개 테스트 추가 (총 18개)

### 변경 금지 사항 (의도적 유지)

- `laser_vertical_calib_roi.json`, `laser_vertical_scan_roi.json` config 스키마 키 이름 불변
- `draw_laser_calib_roi` 기존 시그니처 불변 (`tab_ai_detection` 기존 호출 보호)
- `_calib_roi_bounds()`, `_filter_by_calib_roi()` 좌표 계산 로직 유지 (렌더링과 분리)

---

## 2026-03-14 | AI Detection — Laser 정렬 버튼 ROI 오버레이 구현

### 내용
- Laser 정렬 버튼 클릭 시 `laser_vertical_calib_roi.json`을 읽어 ArduCam 프레임에 ROI 박스 오버레이
- YOLOv8 추론 결과 위에 ROI를 그려 레이저 검출 영역을 시각적으로 확인 가능

### 변경 사항
- `_on_laser_align_clicked`: 버튼 토글마다 JSON 재로드 → 파일 수정 후 OFF→ON으로 즉시 반영
- 기존 Port_T bbox 기반 동적 노란 ROI 제거 (요청한 적 없는 코드)
- 공용 ROI overlay 함수 3종을 `utils/image_processing.py`에 추가:
  - `draw_roi_single(frame, roi_cfg)`: 단일 박스 (img_cx + offset)
  - `draw_roi_symmetric(frame, roi_cfg)`: 좌우 대칭 2개 (img_cx ± offset)
  - `draw_laser_calib_roi(frame, roi_cfg)`: JSON `mode` 필드로 single/symmetric 선택 (기본: symmetric)
- `view_results.py`에서 위 함수들 re-export (하위 호환)
- JSON에 `"mode": "symmetric"` 필드 추가

### 수정 파일
- `scripts/tabs/tab_ai_detection.py`
- `scripts/utils/image_processing.py`
- `scripts/AI/view_results.py`
- `config/laser/align/laser_vertical_calib_roi.json`

---

## 2026-03-14 | tmux에서 충전 로봇 앱 실행

### 내용
- Qt GUI 앱을 tmux 세션 안에서 실행 가능
- 가상환경(`charging_robot`)과 X11 디스플레이 변수 명시 필요

### 실행 명령
```bash
tmux new-session -d -s charging_robot -c /home/argoon/Project/Charging_Robot_Operation
tmux send-keys -t charging_robot "DISPLAY=:0 /home/argoon/Project/Charging_Robot_Operation/charging_robot/bin/python scripts/main_window.py" Enter
tmux attach -t charging_robot
```

### 환경 정보
- `DISPLAY=:0`, `XDG_SESSION_TYPE=x11`
- 가상환경: `charging_robot/bin/python` (python 3.10)

---

## 2026-03-14 | DS435 3-phase wait 타이밍 버그 — Phase 1→2 사이 sleep 누락

### 증상
- DS435 수직/수평 정렬 측정 함수(`_measure_ds435_dy`, `_measure_ds435_dx`)가 로봇 이동 전 프레임의 OBB 결과를 반환할 수 있었음
- ArduCam 측정 함수와 동일한 3-phase wait를 의도했으나 sleep 위치가 달랐음

### 원인
- 기존 구현이 `for flag in (False, True, False):` 루프 안에서 `if flag is True: time.sleep(0.05)` 처리
- sleep이 Phase 2 감지 **이후**(Phase 2→3 사이)에 삽입됨
- ArduCam 패턴은 sleep이 Phase 1 감지 **이후**(Phase 1→2 사이) — 즉 Phase 2 폴링 시작 전에 위치
- 결과: Phase 1 완료 직후 Phase 2(`running=True`)를 즉시 re-detect할 수 있어 동일 프레임 결과를 반환할 위험

### 수정 내용
- `_measure_ds435_dy`, `_measure_ds435_dx`를 ArduCam의 `_measure_dy`, `_measure_dx`와 동일한 3-phase while 루프 구조로 재작성
- `time.sleep(0.05)` 위치: Phase 1 while 루프 완료 직후, Phase 2 while 루프 시작 전으로 이동

### 수정 파일
- `scripts/tabs/tab_ai_detection.py`

---

## 2026-03-14 | PortAlignmentService — 대형 오프셋 분할 보정 + base frame 지원

### 내용
- `COARSE_MAX_MM` 20.0 → 30.0 상향 (DS435 사용 거리에서 오프셋 범위 확대)
- `align_vertical()`에 `frame='tcp'|'base'` 파라미터 추가: 레이저 정렬 등 base frame 이동 필요 시 `send_base_linear` 경로 선택 가능
- 보정량이 `COARSE_MAX_MM` 초과 시 단순 실패 대신 분할 이동(chunked correction) 수행 (최대 10회)
- `_fine_align()`에 `move_fn` 파라미터 추가: frame-agnostic 이동 함수 주입

### 수정 파일
- `scripts/services/port_alignment_service.py`

---

## 2026-03-14 | AI Detection 탭 — 자동차 충전건 결합 그룹박스 추가

### 내용
- "자동차 충전건 결합" 그룹박스 신규 추가 (DS435→ArduCam 핸드오프 + 레이저 정렬 통합 UI)
- `btnDS435ChargingAlign`: DS435 수직+수평 정렬 자동 순차 실행 (Port detect 미활성 시 자동 활성화)
- `btnHandoff` (ArduCam 충전위치 변환): `config/charging/charging_gun_coupling.json` → `ds435_to_arducam` 오프셋으로 `send_base_linear` 이동
- `btnArducamChargingAlign`: ArduCam 수직→Ry→수평 정렬 자동 순차 실행
- `btnLaserAlign`: Port_T bbox 하단 기준 ROI 내 레이저 라인 검출 → `align_vertical(frame='base', tcp_axis='x')` 보정
- `_measure_laser_dy()`: Port_T bbox 좌우 ROI 분할 검출 + fallback 전체 ROI, 레이저 Y − port_t_ymax 반환
- `LaserDetectionService` 추가 임포트, `_last_raw_frame`, `_last_port_t_ymax/xmin/xmax` 상태 추가

### 수정 파일
- `scripts/tabs/tab_ai_detection.py`

---

## 2026-03-12 | AI Detection 탭 — DS435 Port detect 모델 로드 실패 (TkAgg 충돌)

### 증상
- "Port detect" 버튼 클릭 시 모델 로드 실패
- 로그: `[DS435 Detect] 모델 로드 실패: Cannot load backend 'TkAgg' which requires the 'tk' interactive framework, as 'qt' is currently running`

### 원인
- `view_obb_results.py`가 모듈 최상단에 `matplotlib.use('TkAgg')`를 실행 (standalone 스크립트 설계)
- `_load_ds435_model()`에서 `from view_obb_results import overlay_obb` 시 해당 코드가 즉시 실행됨
- 앱은 이미 Qt5Agg 백엔드로 matplotlib를 초기화한 상태 → 백엔드 전환 불가 → 예외 발생

### 수정 내용
- `scripts/AI/obb_overlay.py` 신규 생성: matplotlib 의존성 없이 `overlay_obb`, `put_text_ko`, `get_font` 유틸 함수만 포함 (앱 import 가능)
- `tab_ai_detection.py`: `from view_obb_results import overlay_obb` → `from obb_overlay import overlay_obb`로 교체
- UI 분리 원칙 준수: 인라인 cv2 drawing 코드를 탭 파일에 두지 않고 `obb_overlay.py` 서비스 레이어로 분리

### 수정 파일
- `scripts/AI/obb_overlay.py` (신규)
- `scripts/tabs/tab_ai_detection.py`

---

## 2026-03-12 | AI Detection 탭 — DS435 AI Detection 그룹박스 기능 추가

### 내용
- 기존 DS435 AI Detection 그룹박스는 UI만 존재하고 내부가 완전히 비어 있었음
- Port detect (OBB 검출 토글), 수직 정렬, 수평 정렬 버튼 추가

**신규 구현:**
- `btnPortDetect` (checkable): DS435 YOLOv8-OBB 추론 토글
  - 모델: `config/AI_weights/DS435/best.pt` (task='obb')
  - 오버레이: `obb_overlay.overlay_obb()` (충전포트 OBB 박스 + 신뢰도)
  - OBB 4 꼭짓점 평균 → `_ds435_last_center (cx, cy)` 저장
- `btnDS435VertAlign`: OBB 중심 Y - 이미지 중심 Y → PortAlignmentService.align_vertical()
- `btnDS435HorizAlign`: OBB 중심 X - 이미지 중심 X → PortAlignmentService.align_horizontal()
- 3-phase wait 측정 함수 (`_measure_ds435_dy`, `_measure_ds435_dx`): 백그라운드 추론 완료 후 결과 반환

### 수정 파일
- `scripts/tabs/tab_ai_detection.py`
- `scripts/AI/obb_overlay.py` (신규)

---

## 2026-03-12 | robot_tf_investigation.py — 하드코딩 절대경로 → 상대경로

### 증상

- 다른 PC에 설치 시 `SAVE_PATH`가 `/home/amap/Project/KAIST/...` 절대경로로 고정되어 파일 저장 실패

### 원인

- `scripts/utils/robot_tf_investigation.py`의 `SAVE_PATH`가 개발 머신 절대경로로 하드코딩
- docstring의 저장 위치 설명도 동일한 절대경로 사용

### 수정 내용

- `SAVE_PATH`: 절대경로 → `os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'tf_config.json')`
- docstring: `저장 위치: config/tf_config.json (프로젝트 루트 기준)` 으로 변경

### 수정 파일

- `scripts/utils/robot_tf_investigation.py`

---

## 2026-03-12 | config 폴더 기능별 하위폴더 구조 리팩토링

### 내용
config 루트에 평탄하게 놓여 있던 yaml/json 파일들을 기능별 하위폴더로 재구성.
레이저는 향후 수평 스캔 구현을 고려해 방향별(vertical/horizontal) 구조로 설계.

### 신규 구조

```
config/
├── AI_weights/ArduCam/          (기존 유지)
├── calibration/
│   ├── arducam/                 ← arducam_calibration.yaml (active)
│   │   └── archive/             ← 타임스탬프 버전 12개
│   ├── ds435/                   ← ds435_calibration.yaml (active)
│   │   └── archive/
│   └── stereo/                  ← sweep_calibration.json, charging_robot_camera_calibration_20260307.yaml
├── camera/
│   ├── arducam/                 ← camera_config.json
│   └── ds435/                   (향후 확장 대비)
├── laser/
│   ├── vertical/                ← laser_vertical_scan_roi.json, laser_vertical_triangulation_calib.json, laser_vertical_calib_roi.json
│   └── horizontal/              (추후 구현 예정)
└── charging/                    ← charging_gun_coupling.json, charging_coupling_config.json
```

### 경로 변경 요약

| 구 경로 | 신 경로 |
| ------- | ------- |
| `config/arducam_calibration.yaml` | `config/calibration/arducam/arducam_calibration.yaml` |
| `config/ds435_calibration.yaml` | `config/calibration/ds435/ds435_calibration.yaml` |
| `config/laser_scan_roi.json` | `config/laser/vertical/laser_vertical_scan_roi.json` |
| `config/laser_triangulation_calib.json` | `config/laser/vertical/laser_vertical_triangulation_calib.json` |
| `config/laser_calib_roi.json` | `config/laser/vertical/laser_vertical_calib_roi.json` |
| `config/camera_config.json` | `config/camera/arducam/camera_config.json` |
| `config/charging_gun_coupling.json` | `config/charging/charging_gun_coupling.json` |
| `config/charging_coupling_config.json` | `config/charging/charging_coupling_config.json` |

### 수정 파일
- `scripts/Sensor/arducam/arducam_controller.py`
- `scripts/Sensor/d435/d435_controller.py`
- `scripts/services/arducam_manager.py`
- `scripts/services/camera_manager.py`
- `scripts/services/laser_scan_service.py` (경로 + docstring)
- `scripts/tabs/tab_aruco_reliability.py`
- `scripts/tabs/tab_eye_in_hand.py`
- `scripts/tabs/tab_laser_calibration.py`
- `scripts/tabs/tab_laser_scan.py`
- `scripts/main_window.py` (3곳)
- `scripts/utils/coordinate_visualizer.py`
- `scripts/analyze_ippe_ambiguity.py`
- `scripts/calibrate_camera_boofcv.py`
- `scripts/test_aruco_detect.py`
- `scripts/utils/robot_tf_investigation.py` (하드코딩 절대경로 → 상대경로)

---

## 2026-03-12 | Laser Calibration 탭 — 캘리브레이션 지그 탭에 Laser ROI 토글 기능 추가

### 내용

- "캘리브레이션 지그" 서브탭에 "캘리브레이션" GroupBox + "Laser ROI 생성" 토글 버튼 추가
- 이미지 수평 중심 기준 ±200px 수직 스트립 ROI

### 구현

**신규 파일:**

- `config/laser/vertical/laser_vertical_calib_roi.json` — `half_width_px: 200`, 초록색

**수정 파일:**

- `ui/tab_laser_calibration.ui`: `tabCalibJig`에 `groupCalibration` GroupBox + `btnCreateROI` (checkable) 추가
- `scripts/tabs/tab_laser_calibration.py`:
  - `_load_calib_roi_config()` — JSON 설정 로드
  - `_draw_calib_roi_overlay(frame)` — 반투명 수직 스트립 + 경계선 오버레이
  - `_filter_by_calib_roi(cols, centers_y)` — x 범위 필터 (`_draw_conv_overlay`, `_draw_line_overlay`에 적용)
  - `_on_create_roi()` — 토글 동작 (on/off)
  - `deactivate()` — 탭 전환 시 ROI 상태 자동 리셋
  - 죽은 `roi_create_requested` 시그널 제거

---

## 2026-03-12 | UI/알고리즘 분리 전체 codebase 재검증

### 내용
리팩토링 후 `scripts/` 전체를 대상으로 UI/알고리즘 분리 원칙 준수 여부 재검증.

### 검증 결과

| 레이어 | 상태 | 비고 |
|--------|------|------|
| `scripts/tabs/` | PASS | 이전 리팩토링 완전 적용 — 알고리즘 직접 호출 0건 |
| `scripts/services/` | PASS | 알고리즘 레이어 — cv2 사용 정상 |
| `scripts/Sensor/` | PASS | 센서 레이어 — cv2 사용 정상 |
| `scripts/AI/view_results.py` | PASS | 순수 드로잉(line/circle)만 사용 |
| `scripts/main_window.py` | PASS | import cv2만 존재, 알고리즘 호출 없음 |
| `scripts/utils/coordinate_visualizer.py` | PARTIAL | solvePnP/Rodrigues 직접 호출 |

### 잔존 항목 — `scripts/utils/coordinate_visualizer.py`

- L331: `cv2.solvePnP(...)` 직접 실행
- L338: `cv2.solvePnPRefineLM(...)` 직접 실행
- L101, L113, L391: `cv2.Rodrigues(...)`

utils/ 시각화 도구이므로 tabs 위반보다 심각도 낮음. 향후 수정 시 `ArucoCameraPoseEstimator` 경유로 전환 권장. 현재는 즉시 수정 대상 아님.

---

## 2026-03-12 | UI/알고리즘 분리 원칙 위반 전면 리팩토링

### 증상
- `scripts/tabs/*.py` 파일들이 cv2 알고리즘(`solvePnP`, `calibrateCamera`, `calibrateHandEye`, `Rodrigues`, `undistort`), 레이저 검출(`extract_laser_center`, `fit_laser_line`), 마스크 처리(`extract_red_mask`, `fit_multiple_lines`)를 탭에서 직접 호출
- 탭이 `camera_matrix`/`dist_coeffs`를 직접 보유하여 왜곡 보정까지 수행
- `tab_stereo_calibration.py`가 `ArucoCameraPoseEstimator`를 탭 내에서 직접 인스턴스화 (VisionManager 우회)
- 서비스 레이어 없이 알고리즘 코드가 탭에 분산 → 테스트 불가, 재사용 불가

### 원인
- 초기 개발 시 빠른 구현을 위해 서비스 분리 없이 탭에 직접 구현

### 수정 내용

**신규 생성 (4개):**
- `scripts/utils/image_processing.py`: `undistort_frame()`, `rvec_to_euler_deg()` 공통 유틸
- `scripts/services/camera_calibration_service.py`: `CameraCalibrationService` — `cv2.calibrateCamera` 래퍼
- `scripts/services/hand_eye_calibration_service.py`: `HandEyeCalibrationService` — `solvePnP`, `Rodrigues`, `calibrateHandEye` 래퍼
- `scripts/services/laser_detection_service.py`: `LaserDetectionService` — 레이저 검출 7개 static method (`detect_laser_center`, `detect_laser_center_conv`, `detect_and_fit`, `fit_laser_centers`, `detect_in_roi`, `get_laser_y_at_center`, `extract_red_mask`, `fit_multiple_lines`)

**수정 (6개 탭):**
- `tab_calibration.py`: `cv2.calibrateCamera` → `CameraCalibrationService`, `cv2.undistort` → `undistort_frame`
- `tab_eye_in_hand.py`: `solvePnP`/`Rodrigues` → `solve_pnp_and_store`, `calibrateHandEye` → `calibrate`, `undistort` → `undistort_frame`, `_euler_to_rotation_matrix` 서비스로 이전
- `tab_laser_scan.py`: `_detect_laser_in_roi` 정적 메서드 삭제 → `LaserDetectionService.detect_in_roi`, `undistort` → `undistort_frame`
- `tab_laser_calibration.py`: `extract_laser_center*`/`fit_laser_line`/`extract_red_mask`/`fit_multiple_lines` 직접 호출 → `LaserDetectionService` 경유, `undistort` → `undistort_frame`
- `tab_stereo_calibration.py`: `ArucoCameraPoseEstimator` 직접 생성 → `__init__(aruco_estimator=None)` DI 패턴 (기본값으로 내부 생성, 하위호환)
- `tab_aruco_reliability.py`: `cv2.Rodrigues` → `rvec_to_euler_deg`, `cv2.undistort` → `undistort_frame`

**단위 테스트 추가:**
- `scripts/tests/test_image_processing.py` (10개)
- `scripts/tests/test_laser_detection_service.py` (10개)
- `scripts/tests/test_calibration_services.py` (15개)
- 실행: `PYTHONPATH=scripts charging_robot/bin/python -m pytest scripts/tests/ -v -p no:launch_testing` → 35/35 PASS

### 검증
```
cv2.solvePnP / Rodrigues / calibrateCamera / calibrateHandEye / undistort in scripts/tabs/: 0건
extract_laser_center / fit_laser_line (직접 import) in scripts/tabs/: 0건
Syntax check 전 파일: PASS
```

### 수정 파일
- `scripts/utils/image_processing.py` (신규)
- `scripts/services/camera_calibration_service.py` (신규)
- `scripts/services/hand_eye_calibration_service.py` (신규)
- `scripts/services/laser_detection_service.py` (신규)
- `scripts/tabs/tab_calibration.py`
- `scripts/tabs/tab_eye_in_hand.py`
- `scripts/tabs/tab_laser_scan.py`
- `scripts/tabs/tab_laser_calibration.py`
- `scripts/tabs/tab_stereo_calibration.py`
- `scripts/tabs/tab_aruco_reliability.py`
- `scripts/tests/` (신규 디렉토리)

---

## 2026-03-11 | AI Detection — Ry 보정 부호 반전 (로봇이 반대 방향으로 움직임)

### 증상
- "Ry 보정" 버튼 클릭 시 기울기가 줄어들지 않고 오히려 커짐
- 예: 측정 -5.00° → 보정 후 -9.80° (반대 방향 이동)

### 원인
- `align_ry`에서 `send_base_rotate('ry', angle)` 호출 시 부호 미반전
- `compute_ry_angle`은 커플러 대칭축의 기울기를 반환 → 기울기를 상쇄하려면 `-angle` 송신 필요
- ArUco 보정과 달리 AI 검출 기반 보정은 "마커 방향을 따라가는" 것이 아니라 "기울기를 취소"하는 방향

### 수정 내용
- `scripts/services/port_alignment_service.py` `align_ry()`:
  - `send_base_rotate('ry', angle)` → `send_base_rotate('ry', -angle)`
  - 로그 메시지도 동일하게 `-angle` 표시

### 수정 파일
- `scripts/services/port_alignment_service.py`

---

## 2026-03-11 | AI Detection — Ry 보정 single-shot → 반복 루프 (미세 조정 없음)

### 증상
- 1회 보정 후 잔여 +1.29° 남아도 재보정 없이 종료
- 로그: `잔여 +1.29° > 1.0° — 버튼 재시도 권장`

### 원인
- `align_ry` 구현이 단순 single-shot (1회 보정 + 잔여 로그만)
- threshold 미달 시 자동 재시도 로직 없음

### 수정 내용
- `align_ry`를 최대 5회 반복 루프로 변경
- 매 반복: 측정 → |angle| < threshold_deg(0.5°)이면 조기 종료 → 보정 → 다음 반복
- 최종 5회 소진 시 잔여 측정 후 메시지 반환

### 수정 파일
- `scripts/services/port_alignment_service.py`

---

## 2026-03-11 | AI Detection — Port Center Align 로봇 이동 비활성화 상태

### 증상
- "Port Center Align" 버튼 클릭 시 로그만 출력되고 로봇이 움직이지 않음

### 원인
- Ry 보정 테스트를 위해 `_on_port_center_align_clicked` 내 로봇 이동 코드를 주석 처리
- `# TODO: re-enable after Ry correction integration`

### 수정 내용
- 주석 해제하여 정상 동작 복원
- 로봇 미연결 시 오버레이만 표시, 연결 시 백그라운드 스레드(`_run_align`)로 수직 정렬 실행

### 수정 파일
- `scripts/tabs/tab_ai_detection.py`

---

## 2026-03-11 | Laser Calibration 탭 — 이미지 중심 오버레이 누락

### 증상
- Laser Calibration 탭 카메라 뷰에 이미지 중심 십자선이 표시되지 않음

### 원인
- `update_frame()`에 중심 오버레이 드로잉 코드 없음
- 프로젝트 표준 유틸리티(`utils/overlay.py`)가 이미 존재했으나 해당 탭에서 미사용

### 수정 내용
- `scripts/tabs/tab_laser_calibration.py`: `from utils.overlay import draw_image_center_crosshair` 임포트 추가
- `update_frame()` 마지막에 `draw_image_center_crosshair(display)` 호출 추가 → 모든 표시 모드(laser/conv/lines/rgb/기본)에서 항상 표시

### 수정 파일
- `scripts/tabs/tab_laser_calibration.py`

---

## 2026-03-11 | 종료 시 core dump — `terminate called without an active exception`

### 증상
- 앱 종료 시 카메라 정지 로그 직후 `terminate called without an active exception` 출력 후 core dump
- 재현 순서: `카메라 정지됨` → `ArduCam 정지됨` → 크래시

### 원인
1. **`closeEvent` 종료 순서 오류**: `status_timer`를 카메라보다 나중에 정지 → 카메라 `release()` 중 타이머 콜백(keepalive 등)이 끼어들 수 있음
2. **`__del__` 소멸자의 위험한 C++ 호출**: `ArduCamManager.__del__`, `ArduCamController.__del__`가 인터프리터 종료 시 GC에 의해 호출됨 → `cv2` 모듈이 이미 정리된 상태에서 `VideoCapture.release()` 실행 → OpenCV V4L2 백엔드 내부 스레드가 C++ 예외 발생 → `std::terminate()` (Python `try/except`로 잡을 수 없음)

### 수정 내용
- `scripts/main_window.py` `closeEvent`: `status_timer.stop()`을 카메라 정지보다 **먼저** 호출하도록 순서 변경
- `scripts/services/arducam_manager.py`: `__del__` 제거 (`closeEvent`에서 명시적 `stop()` 호출로 충분)
- `scripts/Sensor/arducam/arducam_controller.py`: `__del__` 제거 (동일 이유)

### 수정 파일
- `scripts/main_window.py`
- `scripts/services/arducam_manager.py`
- `scripts/Sensor/arducam/arducam_controller.py`

---

## 2026-03-11 | AI Detection 탭 — Port Center Align 로봇 이동 미구현

### 증상
- "Port Center Align" 버튼 클릭 시 로그에 `port_cy`, `img_cy`, `dy` 값은 정상 출력
- 오버레이(중심선, 정렬선)도 표시되지 않음
- 실제 로봇이 전혀 움직이지 않음

### 원인
1. **로봇 이동 코드 없음**: `_on_port_center_align_clicked()`가 `compute_vertical_alignment()`로 `dy_px`만 계산하고 로그/오버레이 저장 후 종료. `send_tcp_linear` 등 이동 명령 없음
2. **robot 참조 없음**: `TabAIDetection`에 `self.robot = None` 및 `set_robot()` 미구현 → `main_window.py`에서 로봇 연결 시 탭에 전달되지 않음

### 수정 내용

- **`scripts/services/port_alignment_service.py`** (신규): UI 독립 정렬 알고리즘 레이어
  - `compute_vertical_alignment(dets, img_h)`: Circle_A 최대 3개 + Port_T 최대 2개 신뢰도 상위 선택, Y 평균 → `dy_px = port_cy - img_h/2`
    - 최소 조건: Circle_A ≥ 1개 AND Port_T ≥ 1개 (하나라도 없으면 None 반환)
  - `PortAlignmentService.align_vertical(measure_fn)`: 2mm 테스트 이동 → px/mm 실측 → coarse 보정 → fine 정렬
    - coarse: d0 측정 → +2mm 이동 → d1 측정 → `px_per_mm=(d1-d0)/2` → `correction=-d1/px_per_mm`
    - 안전 체크: `|delta_px| < 2px` 불안정 중단, `|correction| > 20mm` 과대 중단
    - fine: proportional correction, `FINE_MAX_STEP_MM=0.5`, `FINE_CONVERGE_PX=1.5`, 부호반전 2회 발산 중단
- **`scripts/tabs/tab_ai_detection.py`**:
  - `PortAlignmentService` 주입, `set_robot()` → `_align_svc.set_robot()` 전달
  - `_measure_dy()`: 3-phase wait (현재 추론 완료 → 새 추론 시작 → 새 추론 완료) 후 `dy_px` 반환
  - `_run_align()`: 백그라운드 스레드에서 `_align_svc.align_vertical(_measure_dy)` 호출
- **`scripts/AI/view_results.py`**: `draw_center_crosshair()`, `draw_align_overlay()` 추가 (중심선·dy 오버레이)
- `scripts/main_window.py`: 로봇 연결/해제 시 `tabAIDetection.set_robot()` 호출 추가 (line 978, 1060)

### 수정 파일

- `scripts/services/port_alignment_service.py` (신규)
- `scripts/tabs/tab_ai_detection.py`
- `scripts/AI/view_results.py`
- `scripts/main_window.py`

---

## 2026-03-11 | AI Detection 탭 — 정렬 기준 오류 + dy 오버레이 미표시

### 증상
1. Port Center Align 1회 실행 후에도 잔여 오프셋이 남음 (수렴 불충분)
2. Port hall detect 활성 중에도 이미지에 dy 오버레이(기준선·오프셋 수치)가 표시되지 않음 — 버튼 클릭 후 1회 스냅샷만 표시

### 원인
1. **정렬 기준 오류**: `compute_vertical_alignment`가 Circle_A 3개 Y 평균을 사용 → 커플러 기구학적 중심(하단 원)과 불일치. 이후 Port_T 2개 Y 평균으로 수정했으나, 실제 커플러 기구학적 중심은 Circle_A 하단(최대 Y) 원에 해당
2. **coarse 1회 보정의 잔여 오프셋**: px/mm 실측 오차와 로봇 최소 스텝으로 인해 coarse 1회 보정 후 잔여 오프셋 발생 → fine 정렬 루프 부재
3. **dy 오버레이 스냅샷 방식**: `_run_detection`이 `_align_result`(버튼 클릭 시 1회 저장)를 사용 → 검출 중 항상 표시되지 않음

### 수정 내용
- **`scripts/services/port_alignment_service.py`**
  - `compute_vertical_alignment`: Circle_A 하단(Y 최대) primary, Port_T 2개 Y 평균 fallback
  - `PortAlignmentService._fine_align()` 추가: proportional correction 반복 (`FINE_MAX_STEP_MM=0.5`, `FINE_CONVERGE_PX=1.5`, 부호반전 2회 발산 중단)
- **`scripts/tabs/tab_ai_detection.py`**
  - `_run_detection`: `_align_result` 스냅샷 → 매 프레임 `compute_vertical_alignment(dets, frame.shape[0])` 실시간 계산으로 교체 → dy 오버레이 항상 표시

### 수정 파일
- `scripts/services/port_alignment_service.py`
- `scripts/tabs/tab_ai_detection.py`

---

## 2026-03-11 | Depth 보정 — 불안정한 측정 및 수렴 실패

### 증상
- `_on_test_depth_adjust` 실행 시 10회 반복해도 오차가 줄지 않고 발산 (±30mm 수준)
- 같은 X 위치에서 depth가 371mm ↔ 402mm로 심하게 흔들림
- `FINE_TOLERANCE_MM=0.1`로 설정되어 있어 절대 수렴하지 않음
- 이동 후 로봇이 정지했음에도 depth 값이 불안정

### 원인
1. **측정 픽셀 불일치**: `_test_measure_ds435_marker_depth`가 첫 번째 유효한 샘플 픽셀을 즉시 반환. 로봇 이동 시 마커 위치가 바뀌어 어떤 픽셀이 유효한지 달라짐 → 매 iter마다 다른 물리적 위치의 depth를 읽음 (e.g. sample=(732,487)→402mm, sample=(661,486)→371mm)
2. **비현실적인 tolerance**: `FINE_TOLERANCE_MM=0.1mm` — depth 센서 노이즈가 수mm 수준이므로 절대 달성 불가
3. **카메라 프레임 지연**: 이동 후 `_settle(0.5s)`만으로 DS435 프레임 갱신 불충분 → 이동 전 구식 프레임에서 depth 읽음

### 수정 내용
- `_test_measure_ds435_marker_depth`: 유효한 모든 샘플을 수집 후 **중앙값 반환** (첫 번째 유효값 즉시 반환 방식 제거)
- `FINE_TOLERANCE_MM`: `0.1` → `1.0`
- `POST_MOVE_SETTLE_S = 1.2s`: 이동 후 카메라 프레임 안정화 대기 시간 확보 (`_settle(0.5)` 대체)

### 수정 파일
- `scripts/main_window.py` (`_test_measure_ds435_marker_depth`, `_on_test_depth_adjust`)

---

## 2026-03-11 | Laser Calibration 탭 — Set Detection Pose 버튼 없음

### 증상
- Laser Calibration 탭에서 Detection Pose로 이동하는 버튼이 없어 ArUco 신뢰성 검증 탭으로 직접 이동해야 했음

### 원인
- `tab_laser_calibration.py` 에 robot 연결 및 Detection Pose 기능이 구현되어 있지 않음
- `ui/tab_laser_calibration.ui` 카메라 버튼 영역에 해당 버튼 미포함

### 수정 내용
- `ui/tab_laser_calibration.ui`: 카메라 버튼 영역(ArduCam 전용 레이블 우측)에 `btnSetDetectionPose` 버튼 추가 (빨간 스타일)
- `scripts/tabs/tab_laser_calibration.py`: `self.robot = None`, `set_robot()`, `_on_set_detection_pose()` 추가 (ArUco 신뢰성 탭과 동일 로직: TF3 → 현재 XYZ + Rx=90/Ry=0/Rz=90 movel → TF4 복귀)
- `scripts/main_window.py`: 로봇 연결/해제 시 `tabLaserCalibration.set_robot()` 호출 추가

### 수정 파일
- `ui/tab_laser_calibration.ui`
- `scripts/tabs/tab_laser_calibration.py`
- `scripts/main_window.py`

---

## 2026-03-11 | AI Detection 탭 신규 추가 (듀얼 카메라 뷰)

### 내용
DS435 + ArduCam 듀얼 카메라 피드를 동시에 관찰하는 전용 탭 추가. 현재는 카메라 관찰 전용이며, AI 검출 기능은 플레이스홀더.

**신규 파일:**
- `scripts/tabs/tab_ai_detection.py` — `TabAIDetection(QWidget)` 구현
  - 640×360 고정 크기 DS435 / ArduCam 라벨 2개 (수평 배치)
  - `update_ds435_frame()`, `update_arducam_frame()` — BGR→RGB 변환 후 QLabel 표시

**수정 파일:**
- `scripts/tabs/__init__.py` — `TabAIDetection` import/export 추가
- `scripts/main_window.py`:
  - `_on_ai_ds435_frame()`, `_on_ai_arducam_frame()` — 3프레임마다 1회 업데이트 (프레임 카운트 기반 스로틀링)
  - `_on_tab_changed()`: AI Detection 탭 진입 시 DS435 + ArduCam 카메라 자동 시작 (TF 변경 없음)

---

## 2026-03-11 | modbus_client — write_command() pre-flight PRS IDLE 대기 + wait_for_done() 고속 완료 감지

### 증상
1. 연속 명령 전송 시 이전 명령의 stale DONE/RUNNING 상태가 남아 다음 명령이 덮어씌워지는 레이스 발생
2. 0.1mm 같은 고속 완료 명령에서 `wait_for_done()`이 Running 상태를 놓쳐 5초 타임아웃 발생

### 원인
1. `write_command()` 가 직전 PRS cleanup 완료 여부 확인 없이 즉시 명령 전송
2. `wait_for_done()` 에서 Running 미감지 시 완료 판정 수단이 없었음 (5초 타임아웃만 존재)

### 수정 내용

**1. `write_command()` pre-flight PRS IDLE 대기**
- 명령 레지스터 쓰기 전 최대 2초간 PRS 상태(352)가 IDLE(0)이 될 때까지 폴링
- stale DONE(2)/RUNNING(1) 소진 후 명령 전송 보장

**2. `wait_for_done()` 고속 완료 감지**
- Running 미감지 IDLE 상태에서 명령 레지스터(351) == 0 이면 완료로 판정
- 0.1mm 고속 이동처럼 PRS가 ~50ms 내 완료 시 5초 타임아웃 없이 즉시 반환

**3. `send_go_home`, `send_tcp_linear`, `send_tcp_rotate` — `stop_flag_callback` 파라미터 추가**
- 사용자 중지 콜백을 `wait_for_done_motion_aware()` 체인으로 전달 가능

### 수정 파일
- `scripts/Robot/communication/modbus_client.py`

---

## 2026-03-11 | laser_scan_service — 비선형 삼각측량 코드 구현 + 각도 추정 UI 표시

### 내용
- 2026-03-07 설계 문서에서 실제 코드로 구현 (`laser_scan_service.py`)

**구현 메서드:**
- `_load_triangulation_calib()`: `config/laser_triangulation_calib.json`에서 `h_mm`, `Bx_mm`, `alpha_deg` 로드. 필수 키 미존재 시 None 반환
- `_estimate_tilt_angle()`: 좌/우 Z-Y 데이터로 `y_model(delta, d0, phi_rad)` 피팅. `scipy.optimize.differential_evolution`으로 전역 최적화 (초기값 불필요). 좌/우 개별 추정 후 평균

**결과 구조:**
```python
{'estimated_deg': float, 'left_deg': float, 'right_deg': float}
```

**tab_laser_scan.py 각도 추정 결과 표시:**
- 기존: 좌우 slope 차이만 표시 (`labelTiltEstimate`)
- 변경: `추정 각도: {deg:.2f}° (L={left}° R={right}°)` 표시
- 캘리브레이션 없을 때: "각도 추정: 캘리브레이션 없음" 표시

### 수정 파일
- `scripts/services/laser_scan_service.py`
- `scripts/tabs/tab_laser_scan.py`

---

## 2026-03-11 | ArduCam 1920×1080 — 코드 전체 반영

### 증상
- ArduCam 교체(1280×720 → 1920×1080) 이후 `arducam_controller.py` 기본 해상도와 `StereoOffsetCalculator`의 이미지 중심이 구버전 값으로 남아 계산 오차 발생

### 수정 내용

**1. `arducam_controller.py` 기본 해상도 변경**
- `width=1280, height=720` → `width=1920, height=1080`
- fallback intrinsics: `fx/fy=1920, ppx=960, ppy=540`

**2. `StereoOffsetCalculator` — camera_info 검증 + 이미지 중심 동적 계산**
- `_validate_camera_info()` 추가: sweep JSON의 `camera_info`와 현재 ArduCam intrinsics(`fy`, `width`, `height`) 비교
  - 불일치 시 `self.is_valid = False` + 경고 로그 (sweep 재실행 권고)
  - `camera_info` 없으면 구버전 데이터로 판단하여 경고
- `compute_camera_offset_mm()`: ArduCam 이미지 중심을 하드코딩 제거 → sweep JSON `camera_info.arducam.width/height`에서 동적 계산
- DS435 이미지 중심: `DS435_CENTER_X/Y = 640.0/360.0` 클래스 상수로 분리

### 수정 파일
- `scripts/Sensor/arducam/arducam_controller.py`
- `scripts/services/stereo_offset_calculator.py`

---

## 2026-03-11 | _on_ar_tag_align_base_y — measure_func 파라미터로 ArduCam Y 정렬 지원

### 증상
- `_on_ar_tag_align_base_y()`가 DS435 기반 `_measure_marker_dy_px`만 내부 호출 → ArduCam 통합 정렬에서 Y축 보정 시 DS435 측정 함수를 잘못 사용

### 수정 내용
- `measure_func=None` 파라미터 추가
  - `None`이면 기존대로 `_measure_marker_dy_px` 사용 (DS435, 하위호환)
  - ArduCam 호출 시 `_arducam_measure_dy()` 클로저를 전달하여 ArduCam 기반 측정 수행
- `_stereo_align_y_core()` 내 ArduCam Y 정렬 시 `measure_func=_arducam_measure_dy` 명시 전달

### 수정 파일
- `scripts/main_window.py`

---

## 2026-03-11 | 차량 충전건 결합 GroupBox UI 추가

### 내용
- 테스트 탭에 "차량 충전건 결합 (TF4)" GroupBox 신규 추가
- `btnVehicleStopCameras` 버튼: DS435 + ArduCam 양쪽 카메라 즉시 정지
- `_on_vehicle_stop_cameras()` 핸들러: 카메라 정지 + `btnTestToggleCameras` 상태 동기화

### 수정 파일
- `scripts/main_window.py`

---

## 2026-03-07 | DS435→ArduCam 핸드오프 — Detection Pose 미실행

### 증상
- 핸드오프 실행 시 로봇이 임의 자세에서 바로 DS435 센터링 시작
- DS435 마커 검출 실패 또는 부정확한 정렬

### 원인
- `_on_stereo_calib_handoff_ds435_to_arducam`, `_on_test_handoff_ds435_to_arducam` 모두 Detection Pose (Rx=90, Ry=0, Rz=90) 이동 단계 없음
- DS435 단독 정렬 함수에는 Detection Pose 포함되어 있으나, 핸드오프 함수에서는 누락

### 수정 내용
- 두 핸드오프 함수 시작부에 "0단계: Detection Pose 이동" 추가
- 현재 자세 읽기 → Rx/Ry/Rz 차이 0.5° 초과 시 movel 이동 → 안정화 대기

### 수정 파일
- `scripts/main_window.py`

---

## 2026-03-07 | DS435→ArduCam 핸드오프 — Ry 보정 시 잘못된 camera intrinsics 사용

### 증상
- DS435→ArduCam 핸드오프 4단계(ArduCam 통합 정렬)에서 Ry 보정이 실질적으로 동작하지 않음
- 스테레오 탭, 테스트 탭 양쪽 모두 동일 증상

### 원인
- `_detect_dual_alignment()` (L1253)이 항상 `self.tabArucoReliability.camera_matrix`를 사용
- ArUco 신뢰성 탭의 카메라 선택이 DS435인 경우, ArduCam 프레임을 DS435 intrinsics로 분석
- `undistortPoints` → 잘못된 왜곡 보정 (center 좌표 자체가 틀림)
- `solvePnP` → 잘못된 tvec → `angle_3d` 오류
- `_get_effective_ry()`가 `angle_3d`를 우선 사용하므로 Ry 보정값 자체가 부정확
- DS435 fx≈640 vs ArduCam fx≈5347 (약 8배 차이)

### 영향 범위 (4곳)
| 함수 | 프레임 소스 | 기존 intrinsics | 수정 후 |
|------|-----------|----------------|---------|
| `_calib_detect_aruco_alignment` | ArduCam | ArUco 탭 (불일치 가능) | ArduCam |
| `_stereo_detect_aruco_alignment` | ArduCam | ArUco 탭 (불일치 가능) | ArduCam |
| `_stereo_detect_full_alignment` | ArduCam | ArUco 탭 (불일치 가능) | ArduCam |
| `_test_detect_full_alignment` | ArduCam | ArUco 탭 (불일치 가능) | ArduCam |
| `_aruco_tab_detect_alignment` | ArUco 탭 선택 | ArUco 탭 (일치) | 변경 없음 |

### 수정 내용
1. `_detect_dual_alignment`에 `camera_matrix`, `dist_coeffs` 선택 파라미터 추가 (None이면 기존 ArUco 탭 사용)
2. `_get_arducam_intrinsics()` 헬퍼 추가 (arducam_manager.intrinsics → numpy 변환)
3. ArduCam 프레임을 사용하는 4개 호출부에서 ArduCam intrinsics 명시 전달

### 수정 파일
- `scripts/main_window.py`

---

## 2026-03-07 | Modbus TCP Heartbeat + Keepalive 메커니즘

**증상:** 로봇과 30분간 명령 전송이 없으면 로봇 컨트롤러 측에서 TCP/IP 연결을 단절함. 또한 네트워크 장애 시 연결 끊김을 감지하지 못하고 UI가 무응답 상태에 빠짐.

**원인:** Modbus TCP 연결에 대한 능동적 연결 감시 및 keepalive 메커니즘 부재

**수정 파일:** `scripts/Robot/communication/modbus_client.py`, `scripts/main_window.py`

**수정 내용:**

1. **Heartbeat (연결 끊김 감지)**
   - `_update_robot_status()` (100ms 주기) 내 4회 `read_registers()` 중 전부 실패 = 1사이클 실패
   - 연속 5사이클(~500ms) 실패 시 `_handle_connection_lost()` 호출
   - ModbusClient: `_connection_lost` 플래그 + `mark_connection_lost()` + `wait_for_done()` 조기 탈출
   - 비차단 `QMessageBox` 경고 (QTimer.singleShot 사용)
   - `_on_disconnect(reason)` 분기로 수동 해제/끊김 감지 구분

2. **Keepalive (연결 유지)**
   - `write_command()` 호출 시 `_last_command_time` 타임스탬프 기록
   - 60초간 명령 미전송 + 로봇 Idle 상태 시 현재 위치로 movel(CMD 20) 자동 전송
   - movel 전송 → `write_command()` → 타임스탬프 갱신 → 다음 60초 대기 (자동 반복)

**교훈:** TCP 소켓 레벨 읽기(register read)만으로는 로봇 컨트롤러의 idle 타임아웃을 방지할 수 없음. 로봇 측 명령 레지스터에 주기적으로 쓰기(write_command)가 필요.

---

## 2026-03-07 | ArduCam 카메라 교체 1280x720 → 1920x1080 (렌즈 변경 포함)

**증상:** ArduCam 카메라를 교체하여 해상도가 1280x720에서 1920x1080으로 변경. 렌즈도 변경되어 초점거리가 fy=4003.2 → fy=5347.3으로 변경됨. 기존 코드의 해상도/PIXEL_TO_MM/ROI/sweep 데이터 등이 새 카메라에 맞지 않음.

**핵심 판단:**
- PIXEL_TO_MM 스케일링: 해상도 비율(1920/1280=1.5x)이 아닌 **초점거리 비율**(fy_old/fy_new = 4003.2/5347.3 = 0.7486) 사용. 렌즈가 다르므로 해상도 비율 적용 시 10.9% 체계적 오차 발생.
- Sweep 데이터: 기존 1280x720/fy=4003.2 기준 데이터는 완전 무효. 스케일링 불가 → 검증 게이트로 차단.
- 레이저 ROI: FOV 차이 + ppx=820.3 비대칭으로 산술 스케일링 부정확 → fy 비율 기반 초기값 + 현장 수동 조정.

**수정 파일 (14개):**

| 파일 | 변경 내용 |
|------|-----------|
| `scripts/Sensor/arducam/arducam_controller.py` | CameraIntrinsics 기본값 1920x1080 |
| `scripts/services/arducam_manager.py` | Intrinsics/color_resolution 기본값 1920x1080 |
| `scripts/services/chessboard_alignment_service.py` | PIXEL_TO_MM 0.0625 → 0.0468 (×0.7486) |
| `scripts/tabs/tab_calibration.py` | PIXEL_TO_MM 0.25 → 0.187 (×0.7486) |
| `scripts/tabs/calibration_mixin.py` | PIXEL_TO_MM 0.25 → 0.187 (×0.7486) |
| `scripts/services/stereo_offset_calculator.py` | is_valid 검증 게이트 + 카메라별 IMAGE_CENTER 분리 |
| `scripts/services/sweep_calibration_service.py` | save_data에 camera_info 메타데이터 추가 |
| `scripts/main_window.py` | StereoOffsetCalculator에 intrinsics 전달 + is_valid 체크 |
| `scripts/test_arducam_dual_aruco.py` | cap 해상도 1920x1080 |
| `scripts/utils/coordinate_visualizer.py` | cap 해상도 1920x1080 |
| `scripts/test_aruco_detect.py` | cap 해상도 1920x1080 |
| `scripts/analyze_ippe_ambiguity.py` | cap 해상도 1920x1080 |
| `config/laser_scan_roi.json` | ROI 값 fy 비율 스케일링 + 두께 3 |
| `scripts/tabs/tab_laser_calibration.py`, `tab_stereo_calibration.py` | 주석 해상도 업데이트 |

**StereoOffsetCalculator 검증 게이트 (상세):**
- `sweep_calibration_service.py`: 새 sweep 저장 시 `camera_info` (width, height, fy) 메타데이터 포함
- `stereo_offset_calculator.py`: 생성 시 `current_arducam_intrinsics` 비교하여 `is_valid=False` 설정 (camera_info 없거나 불일치 시)
- `main_window.py`: `is_valid=False`이면 핸드오프 차단 + 사용자에게 sweep 재실행 안내

**Architect 발견 버그 (IMAGE_CENTER):**
- 기존: `IMAGE_CENTER_X/Y = 640/360` 하드코딩 → DS435와 ArduCam 모두 같은 값 사용
- 문제: 새 카메라로 sweep 재실행 시 ArduCam midpoint가 1920x1080 좌표계 → 640/360 사용하면 수 mm 오프셋 오차
- 수정: DS435는 `DS435_CENTER_X/Y = 640/360` 고정, ArduCam은 `camera_info`에서 width/height 추출하여 동적 계산

**현장 작업 필요:**
1. `config/laser_scan_roi.json` — 레이저 ROI 시각적 확인 후 미세 조정 (현재 fy 비율 스케일링 초기값)
2. PIXEL_TO_MM 값 — 이론값이므로 현장 측정 후 확정 권장
3. Sweep 캘리브레이션 — 새 카메라로 재실행 필요 (기존 데이터 자동 차단됨)

**교훈:**
1. 카메라 교체 시 해상도 비율과 초점거리 비율을 구분해야 함. 렌즈가 다르면 해상도 비율은 무의미.
2. 캘리브레이션 데이터에 카메라 메타데이터를 포함시켜야 카메라 교체 시 자동 무효화 가능.
3. 하드코딩된 이미지 중심 좌표는 카메라별로 분리해야 다중 카메라 시스템에서 안전.

---

## 2026-03-07 | Ry 정렬 버튼 미동작 (시그널 미연결 + 신뢰성 검증 필수 의존)

**증상:** ArUco 신뢰성 검증 탭 → ar tag tcp align → "마커 평행 정렬 (TF4)" 섹션의 Ry 정렬 버튼이 카메라 스트리밍 중에도 비활성 상태. 신뢰성 검증 완료 후에만 활성화되며, 클릭 시 TF4 tool.rot(vision ry→robot rz 매핑)으로 동작하여 의도한 base Ry 보정과 불일치.

**원인:**
1. `btnAlignRy`가 `_on_align_single_axis('ry')`에 연결 → `_last_tcp_correction` 필요 (신뢰성 검증 완료 후에만 설정)
2. 라이브 마커 각도 기반 `_on_align_ry_from_angle()` 메서드와 `align_base_ry_requested` 시그널은 존재하나 버튼에 미연결
3. `align_base_ry_requested` 시그널이 `main_window.py`에서 핸들러(`_on_ar_tag_align_base_ry`)에 미연결

**수정 파일:** `scripts/tabs/tab_aruco_reliability.py`, `scripts/main_window.py`

**수정 내용:**
- `tab_aruco_reliability.py:184`: `btnAlignRy` 연결을 `_on_align_single_axis('ry')` → `_on_align_ry_from_angle`으로 변경
- `tab_aruco_reliability.py:365-370`: 프레임 업데이트 시 `active_ry` 유무에 따라 버튼 활성화 + 각도값 표시
- `tab_aruco_reliability.py:1403-1406`: 신뢰성 검증 완료 시 Ry 버튼 중복 활성화 제거 (라이브 경로로 일원화)
- `main_window.py:230`: `align_base_ry_requested` 시그널을 `_on_ar_tag_align_base_ry` 핸들러에 연결

**동작 흐름:** 카메라 스트리밍 → 듀얼 마커 검출 → `_last_marker_angle` 캐시 + 버튼 활성화 → Ry 정렬 클릭 → `send_base_rotate('ry', angle)` 실행

**교훈:** 시그널 정의 + 핸들러 구현만으로는 부족. 시그널 연결(connect)과 버튼 활성화 조건까지 확인 필요.

---

## 2026-03-07 | 레이저 캘리브레이션 ArUco 재검출 실패 (오버레이 프레임 혼용)

**증상:** 레이저 캘리브레이션 탭에서 첫 ArUco 정렬은 성공하나, Z 조정 후 재검출 시 `[Detect] 마커 0/1 미검출` 반복 실패. 카메라 해상도 1280x720→1920x1080 변경 후 발생.

**원인:** `_on_camera_frame`의 레이저 캘리브레이션 프레임 핸들러에서 `draw_dual_marker_overlay()`로 마커 코너 위에 색상 선을 그린 후, 오버레이된 프레임을 `update_frame()`에 전달. `current_frame`에 오버레이가 포함된 상태로 저장됨. 이후 `_calib_detect_aruco_alignment()`가 이 오버레이된 `current_frame`으로 ArUco 검출을 시도하여 마커 흑백 패턴이 훼손되어 검출 실패.

- 첫 정렬 시에는 `_show_aruco_overlay=False`라 오버레이 미적용 → 성공
- 정렬 완료 후 `_show_aruco_overlay=True` → 이후 `current_frame`에 오버레이 포함 → 재검출 실패

**수정 파일:** `scripts/main_window.py`, `scripts/tabs/tab_laser_calibration.py`, `scripts/tabs/tab_stereo_calibration.py`

**수정 내용:**

- `tab_laser_calibration.py`: `self._raw_frame = None` 초기화 추가 (오버레이 없는 순수 원본 프레임 전용)
- `main_window.py` 프레임 핸들러: 오버레이 적용 전 `_raw_frame = frame.copy()` 저장
- `main_window.py` `_calib_detect_aruco_alignment()`: `_raw_frame` 우선 사용, fallback으로 `current_frame`
- `tab_stereo_calibration.py`: `current_frame = frame` → `frame.copy()` (참조→복사, 예방적 수정)

**교훈:** 표시용 프레임과 검출용 프레임은 반드시 분리. 오버레이는 별도 사본(display)에만 적용하고, 원본(raw)은 검출 전용으로 보존할 것.

---

## 2026-03-07 | DS435 정렬 시 TCP 자세 미고정으로 정렬 정확도 저하

**증상:** Stereo Camera 탭에서 DS435 정렬(btnAlignDS435) 실행 시, 로봇 TCP 자세(Rx/Ry/Rz)가 임의 상태에서 정렬이 시작되어 ArUco 검출 및 Y/Z 센터링 정확도가 일관되지 않음

**원인:** 정렬 로직이 현재 TCP 자세를 확인하지 않고 바로 ArUco 검출 및 보정을 시작. Detection Pose(Rx=90°, Ry=0°, Rz=90°)로의 사전 이동 단계가 없었음

**수정 파일:** `scripts/main_window.py` (`_on_stereo_calib_align_ds435`)

**수정 내용:**
- 정렬 로직 시작 전 현재 TCP pose를 읽어 Rx/Ry/Rz 확인
- 목표 자세(Rx=90, Ry=0, Rz=90)와 0.5° 이상 차이 시 현재 TF 유지한 채 movel(CMD 20)로 자동 이동 (TF 변경 없음)
- X/Y/Z 위치는 현재값 유지, 회전값만 고정
- 이미 목표 자세이면 이동 생략 (불필요한 동작 방지)

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

## 2026-02-21 | Base Y 보정 알고리즘 수정 + PRS fine translate 레지스터 오류 + wait_for_done 레이스 컨디션

### 문제 1: `send_base_translate` 메서드 미존재

**증상:** `'ModbusClient' object has no attribute 'send_base_translate'`

**수정:** `main_window.py`에서 `send_base_translate` → `send_base_linear` 전체 치환

### 문제 2: PRS `read_register()` 정수 주소 불가

**증상:** PRS 377줄 `invalid argument[1]:type of string data`. fine translate 실행 불가.

**원인:** Doosan PRS `modserv.read_register()`는 문자열 이름만 허용. 레지스터 313-315는 미등록 이름.

**수정:** 미사용 `'x2'(307)`, `'y2'(308)`, `'z2'(309)` 레지스터로 변경

| 파일 | 변경 |
|------|------|
| `Main_task.prs` CMD 60-63 | `read_register(313/314/315)` → `read_register('x2'/'y2'/'z2')` |
| `modbus_client.py` | `REGISTER_FINE_X/Y/Z = 313/314/315` → `307/308/309` |

### 문제 3: `wait_for_done()` 0.3초 IDLE 조기반환

**증상:** 로봇 이동 전 "완료" 반환. 후속 측정이 이동 전 좌표를 읽음.

**원인:** PRS 루프(50ms) 명령 읽기 전, 0.3초 IDLE을 "빠른 명령 완료"로 오판.

**수정:** `modbus_client.py` — `saw_running` 플래그 추가. RUNNING(1)을 감지한 후에만 IDLE을 완료로 판정.

### 문제 4: Base Y 보정 알고리즘 개선

**증상:** 보정 후 잔여 오차 50px+, 과대보정(-98mm) 산출.

**수정 (`main_window.py` `_on_ar_tag_align_base_y`):**
- 테스트 방향: 항상 +5mm → 오프셋 반대 방향 (`d0>0` → Y-, `d0<0` → Y+)
- 위치 안정화 폴링: 연속 3회 변화 < 0.05mm일 때 이동 완료 판정
- 비율 산출: 명령값 → `read_current_pose()` 실측값(`actual_mm`) 기반
- 보정 이동: `send_base_fine_translate` (0.1mm 해상도)
- 보정 후 재측정 검증 추가

---

## 2026-02-21 | PRS 0.1mm 이송 명령 통일 업데이트

**변경:** 모든 이송(translation) 명령이 0.1mm 해상도로 통일됨

### PRS 변경 (Main_task.prs)

| CMD | 변경 전 | 변경 후 |
|-----|---------|---------|
| 10-12 (tool.trans) | `tool.transx(x)` raw mm | `tool.transx(x/10)` 0.1mm |
| 13 (tool.trans xyz) | `tool.trans(x,y,z)` raw mm | `tool.trans(x/10,y/10,z/10)` 0.1mm |
| 50-52 (base trans) | `transx(x)` raw mm | `transx(x/10)` 0.1mm |
| 53 (base trans xyz) | `trans(x,y,z)` raw mm | `trans(x/10,y/10,z/10)` 0.1mm |
| 60-63 (fine trans) | 변경 없음 | deprecated 주석 추가 (CMD 50-53으로 대체) |

### Python 변경 (modbus_client.py)

1. **`send_tcp_linear`**: `int(distance)` → `int(round(distance * 10))` (4개 분기)
2. **`send_base_linear`**: `int(distance)` → `int(round(distance * 10))` (4개 분기)
3. **`send_base_fine_translate`**: deprecation 래퍼로 교체 → `send_base_linear` 호출
4. **`send_move_to_pose`**: `int(x * 10)` → `int(round(x * 10))` (6개 값, truncation 수정)
5. **`write_pose_main`**: 동일 truncation 수정 (6개 값)
6. **`write_pose_back`**: 동일 truncation 수정 (6개 값)
7. CMD_BASE_FINE_*, REGISTER_FINE_* 상수: DEPRECATED 주석 추가

### Python 변경 (main_window.py)

- `_on_ar_tag_align_base_y`: `int(test_cmd)` 래핑 제거 4곳 (float 그대로 전달)
- `send_base_fine_translate` → `send_base_linear` 교체 1곳

### 로봇 실측 검증 (02-21 10:37)

PRS 배포 전 테스트에서 10배 이동 확인 → PRS 업로드 후 정상 동작 확인:

| 명령 | 기대값 | 레지스터 | 실제 이동 | 오차 | 판정 |
| ------ | -------- | --------- | ---------- | ------ | ------ |
| Base Z +0.1mm | 0.1mm | 1 | 0.02mm | -0.08mm | OK (로봇 분해능 한계) |
| Base Z +0.5mm | 0.5mm | 5 | 0.47mm | -0.03mm | PASS |
| Base Z +1.0mm | 1.0mm | 10 | 0.98mm | -0.02mm | PASS |
| Base Z +5.0mm | 5.0mm | 50 | 4.99mm | -0.01mm | PASS |

**결론:** 0.5mm 이상 오차 ±0.05mm 이내. 0.1mm는 기구부 분해능 한계로 오차 있으나 명령 동작 정상.

### 설계 결정

- **CMD 60-63 PRS에서 유지**: 배포 시차 안전성을 위해 제거하지 않고 deprecated 유지
- **deprecation 래퍼**: 단순 alias 대신 warnings.warn() 래퍼로 점진적 마이그레이션 지원
- **`int(round())` 통일**: `int(x*10)` truncation 문제를 코드베이스 전체에서 수정

---

## 2026-02-21 | 이미지 저장 경로 정리 (images/ 폴더 구조화)

**증상:** 이미지가 `images/` 루트 및 `aruco_analysis/` 등 분산 저장되어 관리 어려움

**수정 파일:**
- `scripts/tabs/tab_aruco_reliability.py`
- `scripts/tabs/tab_vision.py`
- `scripts/utils/common.py`

**수정 내용:**
- ArUco 탭 캡처 이미지 저장 경로: `aruco_analysis/` → `images/aruco_mark/`
- ArUco 탭 CSV 기본 저장 경로: `aruco_analysis/` → `images/aruco_mark/`
- Vision 탭 스냅샷 기본 저장 경로: `images/vision/` 설정
- `save_snapshot()` 유틸에 `default_dir` 파라미터 추가 (기존 호출 호환)
- 기존 `images/` 루트 이미지 17장을 `vision/`으로, ArUco 마커 사진 1장을 `aruco_mark/`으로 이동
- 빈 날짜 폴더(`20260117/`, `20260124/`) 제거

**최종 폴더 구조:**
```
images/
├── aruco_mark/   # ArUco 탭 관련 이미지
├── vision/       # Vision 탭 관련 이미지, 로봇 참고 스크린샷
└── laser/        # 레이저 캘리브레이션 이미지
```

---

## 2026-02-21 | tab_aruco_reliability 프로그래밍 UI → .ui 파일 전환

**증상:** `tab_aruco_reliability.py`에서 3개 서브탭(aruco 정렬, 신뢰성 검증 설정, ar tag tcp align)의 UI를 Python 코드로 동적 생성하고 있어 Qt Designer로 편집 불가. 92개+ 위젯이 `_setup_right_panel_tabs()`, `_setup_disambiguation_ui()`, `_setup_plane_result_labels()` 3개 메서드에서 프로그래밍으로 생성됨.

**원인:** 초기 개발 시 `.ui` 파일에는 좌측 패널(카메라+통계)과 우측 `groupControl`만 정의하고, 나머지 탭 UI는 Python 코드로 추가하는 방식으로 구현됨. Qt Designer 활용 불가 및 UI 유지보수 어려움.

**수정 파일:**
- `ui/tab_aruco_reliability.ui` — 전면 재작성
- `scripts/tabs/tab_aruco_reliability.py` — 3개 setup 메서드 제거, signal 연결 통합

**수정 내용:**

1. **`.ui` 파일 전면 재작성** — QTabWidget(`rightTabWidget`) 포함 전체 레이아웃:
   ```
   HBoxLayout
   ├── Left: widgetLeft
   │   ├── groupCameraStream (카메라 뷰 + 버튼)
   │   └── groupStatistics (통계 + 평면결과 + 로봇포즈)
   └── Right: rightTabWidget (QTabWidget)
       ├── Tab 0: "aruco 정렬" — 마커 정보, 각도, 보정 버튼
       ├── Tab 1: "신뢰성 검증 설정" — 캡처 설정, disambiguation, 그래프, 로그
       └── Tab 2: "ar tag tcp align" — 6축 조그, 평행정렬, 디버그
   ```

2. **Python 코드 정리** (~360줄 삭제):
   - `_setup_right_panel_tabs()` (155-369줄) 전체 삭제
   - `_setup_disambiguation_ui()` (371-429줄) 전체 삭제
   - `_setup_plane_result_labels()` (431-513줄) 전체 삭제
   - `__init__`에서 삭제된 메서드 호출 제거
   - `_connect_signals()`로 시그널 연결 통합 (기존 setup 메서드 내 연결 이동)
   - 불필요 import 정리 (`QDoubleSpinBox`, `QSpinBox`, `QCheckBox`, `QGroupBox`, `QGridLayout`, `QPushButton`, `QHBoxLayout`, `QLabel`)

3. **JogMixin 호환성 유지**: `.ui`에서 정확한 위젯 이름 사용
   - 버튼: `btnJog{X,Y,Z,Rx,Ry,Rz}{Plus,Minus}`
   - 스핀: `spinJogStep{X,Y,Z,Rx,Ry,Rz}`

**검증:**
- Python 구문 검증 통과
- 89개 위젯 이름 매칭 확인 (Python self.* 참조 ↔ .ui name 속성)
- 런타임 로드 테스트 성공 (3개 탭, JogMixin, matplotlib 캔버스 정상)

**교훈:** UI 위젯이 30개 이상이면 처음부터 `.ui` 파일로 정의할 것. 프로그래밍 UI 생성은 동적 위젯(런타임 개수 변동)에만 사용.

---

## 2026-02-21 | 레이저 캘리브레이션 탭 — undistort 미적용 + 이미지 저장 경로 오류

### 문제 1: 카메라 보정(undistort) 미적용

**증상:** 레이저 캘리브레이션 탭에서 원본(distorted) 이미지가 그대로 표시됨. 레이저 중심선 추출도 왜곡된 이미지에서 수행되어 정확도 저하.

**원인:** ArUco 탭과 달리 캘리브레이션 로드 및 `cv2.undistort()` 로직이 누락되어 있었음.

**수정 파일:** `scripts/tabs/tab_laser_calibration.py`, `ui/tab_laser_calibration.ui`

**수정 내용:**
1. **UI**: 우측 패널에 "이미지 모드" 그룹박스 추가 — `radioOriginal` / `radioUndistorted` (기본 선택)
2. **Python**:
   - `config/arducam_calibration.yaml`에서 `camera_matrix`, `dist_coeffs` 로드 (`_load_calibration()`)
   - `update_frame()`: `radioUndistorted` 선택 시 `cv2.undistort()` 적용 후 표시
   - 레이저 추출은 항상 표시 중인 프레임(보정된 이미지)에서 수행
   - `display_frame` 별도 저장 — 이미지 저장 시 보정된 프레임 기반

### 문제 2: 이미지 저장 경로 오류

**증상:** 로그에 "이미지 저장: /home/argoon/images/laser/..." 표시되나, 프로젝트 폴더(`images/laser/`)에는 파일 없음.

**원인:** `os.path.expanduser("~/images/laser")` → `/home/argoon/images/laser/`로 해석. 프로젝트 내 `images/laser/`가 아닌 홈 디렉토리에 저장됨.

**수정:** `_get_save_dir()` — 프로젝트 루트 기준 `images/laser/` 경로로 변경
```python
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
save_dir = os.path.join(os.path.abspath(project_root), 'images', 'laser')
```

### 문제 3: 이미지 저장 검증 강화

**수정:** `.jpg` → `.png` (무손실), `os.path.exists()` + `os.path.getsize() > 0` 검증 추가

---

## 2026-02-21 | 레이저 색상 분리 HSV → RGB 기반으로 변경

**증상:** HSV 마스크가 레이저 과포화(blooming) 영역을 누락

**원인:** HSV 색공간의 S(채도) 채널 특성. 레이저 중심부는 R,G,B 모두 포화 → S→0이 되어 S≥50 조건에서 탈락. 레이저가 가장 강한 지점에서 마스크가 빠지는 구조적 문제.

**이론적 근거:** 레이저는 단색광(~650nm)이므로 카메라 센서에서 항상 R≥G, R≥B. 과포화 시에도 R이 먼저 포화되어 이 관계 유지. HSV는 범용 색상 분류에 적합하나, 단색광 레이저 특성을 직접 활용하는 RGB 판별이 이론적으로 올바른 접근.

**수정 파일:** `scripts/Sensor/laser/extract_laser_center.py`

**수정 내용:**
- `extract_red_mask()` — HSV `inRange` 방식 → RGB 채널 차이 방식으로 변경
- 판별 조건: `(R - G) > 30 AND (R - B) > 30 AND R ≥ 80`
- dilation(7x7 ellipse) 추가: 레이저 경계 확장 후 line mask 필터 ROI로 사용

**검증 결과 (저장 이미지 3장):**

| 이미지 | HSV 검출 | RGB 검출 | RGB 추가분 |
|--------|----------|----------|-----------|
| 102549.jpg | 18,627 px | 21,086 px | +13% |
| 103942.jpg | 33,555 px | 43,795 px | +31% |
| 104350.png | 35,533 px | 40,851 px | +15% |

- RGB가 모든 이미지에서 더 많은 레이저 픽셀 검출
- 속도: RGB 2.0ms vs HSV 2.2ms (색공간 변환 불필요)

---

## 2026-02-21 | 레이저 캘리브레이션 탭 display_type 리팩터링 및 버튼 재구성

**수정 파일:** `scripts/tabs/tab_laser_calibration.py`, `ui/tab_laser_calibration.ui`

**수정 내용:**
- HSV 마스크 버튼 → `RGB 레이저 추출`로 이름 변경 (btnHsvMask → btnRgbMask)
- 버튼 순서: RGB 레이저 추출(맨 위) → 중심 추출 → 직선 추출 → 레이저 표시(맨 아래)
- display_type 키: `'hsv'` → `'rgb'`

---

## 2026-02-21 | 로봇 이동 중 카메라 이미지 업데이트 정지

**증상:** 조그 이동 중 카메라 프레임이 멈추고, 이동 완료 후에야 갱신

**원인:** `_on_jog_move_from_tab()` / `_on_jog_rotate_from_tab()`에서 `send_base_linear()` / `send_base_rotate()` 호출 시 `process_events_callback`을 전달하지 않음. `wait_for_done()` 내부의 `time.sleep(0.1)` 루프가 메인 Qt 스레드를 블로킹 → `frame_ready` 시그널 처리 불가.

**수정 파일:** `scripts/main_window.py`

**수정 내용:**
- `QApplication`을 상단 import에 추가
- 두 핸들러 모두 `process_events_callback=QApplication.processEvents` 전달
- `wait_for_done()` 루프에서 100ms마다 Qt 이벤트 처리 → 카메라 프레임 정상 갱신

**참고:** Thread 방식도 가능하나, Modbus 통신 스레드 안전성 및 동시 명령 방지 로직 필요. 현재 `wait_for_done()`에 이미 설계된 `process_events_callback` 패턴이 더 적합.

---

## 2026-02-21 | Hough 직선 검출 — 밀도 필터 제거 및 갭 기반 라인 분리

**증상:** 레이저 중심점(conv center)은 정상 검출되지만 Hough 직선 피팅 결과가 표시되지 않거나, 충전포트 양쪽의 물리적으로 분리된 레이저를 하나의 라인으로 묶음

**원인:**
1. `min_density` 필터가 희소 영역의 유효 직선을 제거
2. Hough 변환이 동일 직선(collinear) 위의 점을 하나의 라인으로 그룹화 — 충전포트 갭(빈 영역)을 무시하고 양쪽 점을 하나로 합침

**수정 파일:**
- `scripts/Sensor/laser/extract_laser_center.py`
- `scripts/tabs/tab_laser_calibration.py`

**수정 내용:**
1. **밀도 필터 완전 제거**: `fit_multiple_lines_hough`, `fit_multiple_lines_ransac`, `fit_multiple_lines` 래퍼에서 `min_density` 파라미터 및 필터 블록 삭제
2. **갭 기반 라인 분리**: `fit_multiple_lines_hough`에 `gap_threshold=10.0` 파라미터 추가. Hough inlier 수집 후 x좌표 정렬 → 10px 이상 갭 발생 시 별도 라인으로 분리 → 각 세그먼트별 독립 least-squares refit
3. **세그먼트 기반 라인 그리기**: `_draw_line_overlay`에서 전체 직선 대신 inlier 연속 구간에서만 라인 표시, 시작/끝점에 원 마커(반지름 6px) 추가
4. **라벨 위치**: 가장 긴 세그먼트 중심에 배치 (갭 위에 라벨 표시 방지)

---

## 2026-02-21 레이저 삼각법 캘리브레이션 프로세스 문서

### 개요

레이저 삼각법 캘리브레이션은 **로봇 TCP의 X축 이동에 따른 Z축 보정량(ΔX/ΔZ 비율)**을 실험적으로 결정하는 과정이다.

**원리**: 레이저 라인이 카메라 이미지의 동일한 Y 위치(target_y)에 표시되도록 Z축을 조정한 뒤, X축을 이동하고 다시 Z축을 재조정한다. 두 위치의 TCP 좌표(X1,Z1)과 (X2,Z2) 차이(ΔX, ΔZ)가 해당 위치에서의 레이저 삼각법 보정 계수가 된다.

### 아키텍처 (Signal/Slot 패턴)

```
tab_laser_calibration.py (UI + 시그널)     main_window.py (핸들러 + 로봇 제어)
┌──────────────────────────┐              ┌──────────────────────────┐
│ btnAlignAruco 클릭        │──시그널──▶   │ _on_calib_align_aruco()  │
│ btnAdjustZ 클릭           │──시그널──▶   │ _on_calib_adjust_z()     │
│ btnSavePos 클릭           │──시그널──▶   │ _on_calib_save_pos()     │
│ btnMoveXAdjustZ 클릭      │──시그널──▶   │ _on_calib_move_x_adjust_z() │
│ btnSaveCompare 클릭       │──시그널──▶   │ _on_calib_save_compare() │
│ btnAutoCalib 클릭         │──시그널──▶   │ _on_calib_auto()         │
│ btnCancelCalib 클릭       │──시그널──▶   │ _on_calib_cancel()       │
│                          │              │                          │
│ get_current_laser_y()    │◀──호출──     │ _calib_measure_laser_y() │
│ _update_calib_step()     │◀──호출──     │ (각 핸들러)               │
│ _add_calib_row()         │◀──호출──     │ _on_calib_save_compare() │
└──────────────────────────┘              └──────────────────────────┘
```

### 수동 캘리브레이션 (5단계)

#### 1단계: ArUco 정렬 (`_on_calib_align_aruco`)

**목적**: 카메라 시야에서 충전포트(ArUco 마커)가 수평·중앙 정렬되도록 로봇 위치 보정

**순서**:
1. `_calib_detect_aruco_alignment()` 호출 → 듀얼 마커 검출 + `compute_dual_alignment()` → `(angle_ry, offset_y)` 반환
2. **Ry 보정** (임계값 ≥ 0.5°): `_on_ar_tag_align_base_ry(angle_ry)` → base frame Rz 회전
3. 재검출 후 **Y 보정** (임계값 ≥ 5px): `_on_ar_tag_align_base_y(offset_y)` → base frame Y 이동
4. ArUco 오버레이 플래그 ON (`tab._show_aruco_overlay = True`) → 카메라 콜백에서 지속 표시
5. 최종 검출 결과(Ry, offset_y) 상태 표시

**참고**: Ry, Y 보정은 각각 1회만 수행. 반복 보정이 필요하면 수동으로 다시 실행.

#### 2단계: Z 조정 (`_on_calib_adjust_z`)

**목적**: 레이저 라인이 목표 Y 위치(target_y)에 오도록 Z축 이동

**현재 구현 (측정만)**: 버튼 클릭 시 `_calib_measure_laser_y()` 호출 → laser_y, target_y, error, TCP 포즈 표시. **실제 Z축 이동은 수행하지 않음.**

**실제 Z 조정 알고리즘** (`_calib_adjust_z_to_target`): 자동 캘리브레이션 및 `_on_calib_move_x_adjust_z`에서 내부적으로 호출됨.

```
┌─ 1) 레이저 검출 확인 (미검출 → 즉시 중단)
│
├─ 2) 1mm 테스트 이동으로 방향/비율 자동 판별
│     Z를 -1mm 이동 → px 변화 측정 → px_per_mm 계산
│     error 증가 시: 방향 반전 (원위치 + px_per_mm 부호 반전)
│
├─ 3) 거친 접근 (error > 10px)
│     move_z = -error / px_per_mm (비율 기반 1회 이동)
│     안전 제한: 잔여 이동량(50mm - 누적) 내로 클램프
│
└─ 4) 미세 조정 (단계별 수렴)
      step 1.0mm: 최대 15회 반복, error ≤ 1px → 완료
      step 0.5mm: 최대 15회 반복, error ≤ 2px → 다음 단계
      step 0.1mm: 최대 15회 반복, error ≤ 1px → 완료
      각 반복: move_z = -error/px_per_mm, |move_z| > step이면 step으로 제한
```

**안전 제한**: 총 Z 이동 ≤ 50mm (`max_total_z`). 초과 시 즉시 중단.

**위치 안정화** (`_calib_wait_for_position_stable`): 매 Z 이동 후 0.2초 PRS 클린업 대기 + 연속 3회 TCP 변화 < 0.05mm 확인 (최대 10초 타임아웃).

#### 3단계: 위치 저장 (`_on_calib_save_pos`)

**목적**: 현재 TCP 위치의 X, Z를 pos1으로 저장

- `robot.read_current_pose()` → `tab._calib_pos1 = (pose[0], pose[2])` (X, Z)
- `tab.set_current_pose(*pose[:6])` → UI에 TCP 좌표 표시

#### 4단계: X 이동 + Z 재조정 (`_on_calib_move_x_adjust_z`)

**목적**: X축으로 설정값만큼 이동한 후 레이저가 다시 target_y에 오도록 Z 재조정

1. `robot.send_base_linear('x', x_step)` → X축 이동 (설정: `spinXMoveStep`, 기본값 UI 참조)
2. `_calib_wait_for_position_stable('x')` → X축 위치 안정화
3. `_calib_adjust_z_to_target()` → Z축 자동 조정 (위 알고리즘)

#### 5단계: 비교 저장 (`_on_calib_save_compare`)

**목적**: 현재 TCP 위치를 pos2로 읽고, pos1과의 차이(ΔX, ΔZ)를 테이블에 추가

- pos1(3단계) vs pos2(현재): `ΔX = X2 - X1`, `ΔZ = Z2 - Z1`
- `tab._add_calib_row(x1, z1, x2, z2)` → 테이블에 행 추가 + `_calib_data` 리스트에 dict 저장
- pos1 초기화 (`tab._calib_pos1 = None`) → 다음 반복 준비

### 자동 캘리브레이션 (상태 머신)

`_on_calib_auto()` → `_auto_calib_step()` (QTimer.singleShot 기반 비동기 상태 머신)

```
                    ┌─────────────────────────┐
                    │       ALIGNING          │ ArUco Ry+Y 보정
                    └───────────┬─────────────┘
                                │ 500ms
                    ┌───────────▼─────────────┐
                    │     Z_ADJUSTING         │ 레이저 → target_y 맞춤
                    └───────────┬─────────────┘
                                │ 100ms
                    ┌───────────▼─────────────┐
                    │     SAVING_POS1         │ TCP (X1, Z1) 저장
                    └───────────┬─────────────┘
                                │ 100ms
                    ┌───────────▼─────────────┐
                    │      MOVING_X           │ X축 spinXMoveStep 이동
                    └───────────┬─────────────┘
                                │ 100ms
                    ┌───────────▼─────────────┐
                    │    Z_READJUSTING        │ 레이저 재조정
                    └───────────┬─────────────┘
                                │ 100ms
                    ┌───────────▼─────────────┐
                    │     SAVING_POS2         │ TCP (X2, Z2) 저장 + ΔX/ΔZ 계산
                    └───────────┬─────────────┘
                                │
                     반복 < total │ 반복 ≥ total
                    ┌────────┐  │  ┌──────────┐
                    │ALIGNING│◀─┘  │ COMPLETE │
                    └────────┘     └──────────┘
```

**반복 횟수**: `spinRepeatCount` UI 설정값 (프로그레스바로 진행률 표시)
**취소**: `tab._z_adjust_cancel = True` + `tab._auto_calib_running = False` → 각 상태 전이 시 체크

### 레이저 Y 측정 (`get_current_laser_y`)

```python
# tab_laser_calibration.py
extract_laser_center_conv(frame, min_half_width, max_half_width)  # 컨볼루션 기반 중심점 추출
fit_laser_line(cols, centers_y, mad_scale)                         # MAD 기반 RANSAC 직선 피팅
laser_y = np.polyval(coeffs, center_x)                             # 이미지 중심 x에서의 y값
```

- `min_half_width`, `max_half_width`: UI spinbox (`spinMinStripe`, `spinMaxStripe`)에서 설정
- `mad_scale`: UI spinbox (`spinMadScale`)에서 설정
- 반환값: 이미지 중심 열(center_x)에서의 레이저 Y 좌표 (px), 또는 None

### 결과 저장 (`_save_calib_result`)

JSON 파일 (`config/laser_calibration.json`)에 저장:
```json
{
  "timestamp": "2026-02-21T...",
  "target_laser_y_px": 240.0,
  "x_move_step_mm": 10.0,
  "data": [
    {"x1": ..., "z1": ..., "x2": ..., "z2": ..., "dx": ..., "dz": ...}
  ],
  "summary": {
    "count": N,
    "avg_dx": ...,
    "avg_dz": ...
  }
}
```

### 관련 파일

| 파일 | 역할 |
|------|------|
| `scripts/tabs/tab_laser_calibration.py` | UI, 시그널 발행, 레이저 Y 측정, 테이블/JSON 관리 |
| `scripts/main_window.py` (line 1375~1960) | 로봇 제어 핸들러, Z 조정 알고리즘, 자동 상태 머신 |
| `scripts/Sensor/laser/extract_laser_center.py` | 레이저 중심점 추출 (`extract_laser_center_conv`) |
| `scripts/services/vision_manager.py` | ArUco 검출 래퍼 (`detect_marker_centers`) |
| `scripts/Sensor/aruco/aruco_detector.py` | `compute_dual_alignment`, `draw_dual_marker_overlay` |
| `ui/tab_laser_calibration.ui` | Qt Designer UI 파일 |

### 현재 상태 및 제한사항

1. **Z 조정 버튼 (2단계)**: 측정값만 표시하고 실제 이동은 수행하지 않음. 수동 Z 이동은 다른 탭(조그) 사용 필요
2. **ArUco 정렬**: 1회 보정만 수행. 정밀도가 부족하면 수동 반복 필요
3. **Z 안전 제한**: 단일 캘리브레이션 사이클에서 총 50mm까지만 Z 이동 허용
4. **레이저 미검출 시**: 해당 단계에서 즉시 중단. 자동 모드는 전체 중단
5. **px_per_mm 비율**: 매 Z 조정 시작마다 1mm 테스트로 재측정 (환경 변화에 적응)

---

## 2026-02-28 | 탭 전환 시 카메라 자동 정지 및 버튼 초기화

**증상:** 탭을 전환해도 이전 탭에서 시작한 카메라가 계속 실행되어 리소스 낭비 및 혼란 발생. 레이저 캘리브레이션 탭의 표시 모드 토글 버튼(레이저/중심/직선/RGB)도 전환 후에도 활성 상태 유지.

**원인:** `_on_tab_changed()`에서 ToolFrame 설정만 처리하고 카메라 정지/UI 초기화 로직이 없었음.

**수정 파일:**
- `scripts/main_window.py` — `_stop_all_cameras()` 헬퍼 추가, `_on_tab_changed()` 수정
- `scripts/tabs/tab_laser_calibration.py` — `deactivate()` 메서드 추가
- `scripts/tabs/tab_stereo_calibration.py` — `deactivate()` 메서드 추가

**수정 내용:**

1. `_stop_all_cameras()` 신규 메서드:
   - 글로벌 `camera_manager` 정지 (DS435 또는 ArduCam)
   - 비활성 카메라 매니저 독립 실행 확인 후 정지 (스테레오 캘리브레이션에서 양쪽 동시 실행 케이스)
   - `tabStereoCalibration.deactivate()` → 양쪽 카메라 정지 + 시작/정지 버튼 enabled 리셋
   - `tabLaserCalibration.deactivate()` → 토글 버튼 4개 해제 + 결과 라벨 초기화
   - 카메라 실제 정지 시에만 로그 출력

2. `_on_tab_changed()` 첫줄에 `_stop_all_cameras()` 호출 추가 (TF 설정 전에 실행)

3. `tab_laser_calibration.deactivate()`: `display_type=None`, 토글 버튼 `setChecked(False)` + 텍스트 원복, `_clear_result_labels()`

4. `tab_stereo_calibration.deactivate()`: `_on_stop()` 호출로 양쪽 카메라 정지 및 버튼 상태 리셋

---

## 2026-02-28 | ArUco 듀얼 마커 정렬 로직 서비스 레이어 분리

**증상:** `tab_aruco_reliability.py`의 `_update_aruco_tab()` 메서드에 마커 검출, 각도 계산, 시각화 로직이 ~80줄 인라인으로 혼재

**원인:** UI 탭에서 영상처리(cv2 직접 호출)를 수행하는 구조 (UI와 알고리즘 미분리)

**수정 파일:**
- `scripts/Sensor/aruco/aruco_detector.py` — `compute_dual_alignment()`, `draw_dual_marker_overlay()`, `DualMarkerAlignmentResult` 추가
- `scripts/services/vision_manager.py` — `compute_dual_alignment()`, `draw_dual_marker_overlay()` 래퍼 추가
- `scripts/tabs/tab_aruco_reliability.py` — 인라인 로직 제거, 서비스 레이어 함수 호출로 교체 (~80줄 → ~25줄)

**수정 내용:**

1. **`DualMarkerAlignmentResult` 데이터클래스**: 마커 중심, tvec, 중점, 2D/3D 각도, 이미지 중심 오프셋을 구조화
2. **`compute_dual_alignment(markers, tag_id1, tag_id2, w, h)`**: 2개 마커 검색 → 중점/각도/오프셋 계산
3. **`draw_dual_marker_overlay(frame, markers, ...)`**: 코너 라인, 중심점, 연결선, 중점 크로스, 정보 텍스트 오버레이 (표준 시각 스타일)
4. **vision_manager 래퍼**: detect_marker_centers + compute_dual_alignment을 한 번에 호출하는 편의 메서드
5. **tab_aruco_reliability 정리**: 인라인 cv2.line/putText/atan2 코드 제거 → `compute_dual_alignment()` + `draw_dual_marker_overlay()` 호출

**효과:** sweep_calibration_service 등 다른 서비스에서도 동일 정렬 로직 재사용 가능

---

## 2026-02-28 | 스테레오 캘리브레이션 탭 — 로봇-픽셀 관계 분석 (Sweep Calibration) 구현

**구현 내용:** 로봇 X/Y/Z를 스윕하면서 두 카메라(ArduCam + DS435)의 ArUco 마커 픽셀 위치를 기록하고 px/mm 비율을 산출

### 동작 흐름

1. 현재 위치 기록 (원점) + 안전 확인 대화상자
2. Z 스윕: -step_mm × count회 (차트 방향), 원점 복귀
3. X 스윕: +step_mm × count회, 원점 복귀
4. Y 스윕: +step_mm × count회, 원점 복귀
5. 분석: 선형회귀 + Z축 2차회귀
6. 저장: `config/sweep_calibration.json`

### 각 스텝 기록 데이터

- 로봇 포즈 (X,Y,Z,Rx,Ry,Rz)
- ArduCam: 마커1 중심, 마커2 중심, 중간점, 마커 픽셀 크기
- DS435: 마커1 중심, 마커2 중심, 중간점, depth(mm), 마커 픽셀 크기

### 신규/수정 파일

| 파일 | 변경 |
|------|------|
| `scripts/services/sweep_calibration_service.py` | NEW — QTimer.singleShot 상태머신 서비스 |
| `ui/tab_stereo_calibration.ui` | 하단 스윕 UI GroupBox 추가 |
| `scripts/tabs/tab_stereo_calibration.py` | 시그널, 버튼 연결, 데이터 테이블, CSV 저장 |
| `scripts/main_window.py` | 핸들러 (안전 대화상자, 서비스 생성, 결과 요약) |
| `scripts/services/__init__.py` | SweepCalibrationService export |

### 핵심 설계

- **QTimer.singleShot 상태머신**: IDLE → Z_SWEEP → Z_RETURN → X_SWEEP → X_RETURN → Y_SWEEP → Y_RETURN → ANALYZING → DONE
- **서비스 클래스 분리**: main_window.py 비대화 방지
- **get_frame() 직접 사용**: isVisible() 문제 회피
- **위치 안정화 폴링**: 연속 3회 < 0.05mm
- **Z축 2차 회귀**: 배율 변화 비선형성 포착
- **중지 시 원점 복귀**: cancel → _return_to_origin → sweep_finished.emit({})
- **실시간 데이터 테이블**: data_captured 시그널 → QTableWidget 표시
- **CSV 저장**: QFileDialog → config/ 디렉토리 기본

### 수정된 버그 (5건)

| # | 버그 | 수정 |
|---|------|------|
| 1 | 전축 공유 원점 데이터 | Z는 start(), X/Y는 _do_return()에서 fresh origin 캡처 |
| 2 | 회귀 기준점 누락 | `mm_relative = mm_valid - mm_valid[0]` |
| 3 | 마커 크기 회귀 인덱스 불일치 | (mm, size) 페어 동시 수집 |
| 4 | 서비스 재생성 시그널 누수 | disconnect + deleteLater 후 재생성 |
| 5 | 취소 후 재시작 불가 | cancel 경로에서 sweep_finished.emit({}) 추가 |

---

## 2026-02-28 | 스윕 캘리브레이션 — 카메라 미실행 시 데이터 캡처 실패

**증상:** 카메라를 수동으로 시작하지 않은 상태에서 "스윕 시작" 클릭 시, 모든 데이터 포인트에서 마커 미검출 → 분석 실패 (insufficient_data)

**원인:** `_on_sweep_start` 핸들러에서 로봇 연결만 확인하고, 카메라 실행 상태를 확인하지 않았음. `get_frame()`은 카메라 미실행 시 `None` 반환 → `_capture_camera()` → `None` → 데이터 없음

**수정 파일:** `scripts/main_window.py`

**수정 내용:**
1. 안전 대화상자 후 양쪽 카메라 자동 시작 (`ds435_camera_manager.start()`, `arducam_manager.start()`)
2. 카메라 시작 시 1초 안정화 대기 + 탭 버튼 상태 동기화
3. intrinsics 유효성 확인 (DS435 런타임 intrinsics, ArduCam 캘리브레이션 파일 intrinsics)
4. intrinsics 없으면 오류 메시지 출력 후 UI 리셋

---

## 2026-02-28 | 레이저 캘리브레이션 — 목표 Y 위치 기준 변경

**변경:** 레이저 목표 Y 위치 설정을 절대 좌표 기반에서 ArUco 마커 중심 기준 오프셋으로 변경

**수정 파일:** `ui/tab_laser_calibration.ui`

**수정 내용:**
- 라벨: "레이저 목표 Y 위치" → "마커 중심 Y 오프셋"
- 툴팁: ArUco 마커 중심으로부터의 수직 오프셋 (양수=아래) 설명
- spinTargetLaserY: 범위 `-200~200` → `0~500`, 기본값 `30` → `190`
- 의미: `target_y = ArUco_center_y + offset_px`

---

## 2026-02-28 | DS435→ArduCam 핸드오프 — 1단계 Z 보정 과대 안전중단

**증상:** 핸드오프 1단계에서 DS435 Z축 센터링 시 "보정 과대 (-50.9mm > 50.0mm), 안전 중단" 오류. Z 오프셋 157.8px → ~56mm 보정 필요하나 MAX_CORRECTION=50mm 제한에 걸림.

**원인:** `_ds435_adaptive_align()` 메서드에서 보정량이 MAX_CORRECTION_MM(50mm) 초과 시 즉시 중단하여 아무 이동도 하지 않음.

**수정 파일:** `scripts/main_window.py` (`_ds435_adaptive_align` 메서드)

**수정 내용:**
- 보정 과대 시 **안전 중단 → 2단계 분할 이동**으로 변경
- 1차: correction / 2 이동
- 재측정: 마커 재검출하여 잔여 오프셋 파악
- 2차: 잔여 오프셋 기반 보정 (MAX_CORRECTION 클램프)
- 예시: -56mm 필요 → 1차 -28mm → 재측정 → 2차 ~-28mm

---

## 2026-02-28 | DS435→ArduCam 핸드오프 — 3단계 카메라 오프셋 부호 반전

**증상:** 3단계에서 camera_offset_mm 적용 시 로봇이 반대 방향으로 이동. ArduCam FOV에 마커가 들어오지 않음.

**원인:** `StereoOffsetCalculator.camera_offset_mm`의 부호가 이미지 좌표계 기준이었으나, 로봇 이동 방향과 반대.

**수정 파일:** `scripts/main_window.py` (`_on_stereo_calib_handoff_ds435_to_arducam` 메서드)

**수정 내용:**
- `cam_offset_y = -raw_y`, `cam_offset_z = -raw_z` 부호 반전 적용
- raw: dY=-37.31mm, dZ=-57.16mm → 적용: dY=+37.31mm, dZ=+57.16mm

---

## 2026-02-28 | DS435→ArduCam 핸드오프 — 4단계 통합 정렬 인라인 코드 오작동

**증상:** 핸드오프 4단계(ArduCam 통합 정렬)가 정상 동작하지 않음. 기존 통합 정렬 버튼은 정상 작동.

**원인:** 핸드오프 4단계에 `_on_stereo_calib_align_aruco_combined()` 로직을 인라인 복사하면서 미묘한 차이 발생 (타이밍, 에러 핸들링, 로깅 차이).

**수정 파일:** `scripts/main_window.py` (`_on_stereo_calib_handoff_ds435_to_arducam` 메서드)

**수정 내용:**
- 4단계 인라인 정렬 코드 전체 삭제
- 기존 검증된 `_on_stereo_calib_align_aruco_combined()` 메서드 직접 호출로 교체
- 버튼 상태 관리: 호출 전 `_set_stereo_align_buttons_enabled(True)` → combined 자체 버튼 관리

**추가:** TARGET_DEPTH_MM 380mm → 370mm 변경

---

## 2026-02-28 | DS435→ArduCam 핸드오프 — 3단계 후 ArduCam 미검출 게이트가 4단계 차단

**증상:** 핸드오프 3단계(카메라 오프셋 적용) 후 "ArduCam 미검출, 4단계 정밀 정렬 건너뜀"으로 조기 종료. 하지만 직접 통합 정렬 버튼을 누르면 ArduCam에서 마커가 정상 검출됨.

**원인:** `_check_arducam_marker_visible()`이 3단계 직후 단 1회만 검출 시도. ArduCam 프레임 갱신 타이밍 지연으로 미검출 반환 → `return`으로 4단계 전체가 건너뛰어짐. 실제로는 통합 정렬의 `_stereo_detect_full_alignment(max_retries=3)`이 재시도하면 검출 성공.

**수정 파일:** `scripts/main_window.py` (`_on_stereo_calib_handoff_ds435_to_arducam` 메서드)

**수정 내용:**

- 3단계 후 ArduCam 미검출 시 `return` (조기 종료) 삭제
- 미검출이어도 경고 로그만 출력하고 4단계 통합 정렬 무조건 진행
- 4단계 통합 정렬을 최대 3회 반복 호출하여 수렴 확인 (10px 미만)

---

## 2026-02-28 | ArduCam 미세 정렬 — Running 상태 감지 실패 타임아웃

**증상:** `_fine_align_axis` Y축 미세 정렬 중 "이동 실패: Running 상태 감지 실패 (5초 타임아웃)" 발생. 0.1mm 이동이 간헐적으로 실패하여 미세 정렬이 조기 종료.

**원인:** 0.1mm 미세 이동은 PRS가 ~50ms 내 완료하므로, `wait_for_done()`의 0.02s 폴링으로도 Running 상태를 못 잡는 경우 발생. 5초 타임아웃에 도달하여 실패 반환.

**수정 파일:** `scripts/main_window.py` (`_fine_align_axis` 메서드)

**수정 내용:**
- `send_base_linear(wait=True)` → `wait=False` + `time.sleep(0.3)` 고정 대기로 변경
- 0.1mm 미세 이동은 PRS 완료 시간(~50ms)이 짧으므로 0.3s 고정 sleep이 안정적
- `wait_for_done()` 전역 로직은 미수정 (JOG 안전 보장)

---

## 2026-02-28 | main_window.py 중복 코드 리팩터링 — 함수화 및 통합

**증상:** `main_window.py` 3960줄에 중복 함수, 중복 변수, 반복 패턴 다수 존재. 유지보수 어려움 및 버그 수정 시 동기화 누락 위험.

**분석 결과:**
- 마커 검출 함수 4개 ~90% 유사도 반복
- 적응형 정렬 알고리즘 3곳 전체 복제
- 로봇 연결 가드 27곳 반복
- tag_id 획득 10곳, 마커 검색+중심점 5곳 반복
- 함수 내 중복 import 19곳 (time 10회, QApplication 9회)
- combined 메서드가 개별 축 정렬 로직 인라인 복제 (128줄)

**수정 파일:** `scripts/main_window.py`

**수정 내용 (3단계 리팩터링):**

Phase 1 — 안전한 헬퍼 추출 (-73줄):
- 중복 import 19곳 제거
- `_require_robot()` 가드 (21곳 적용)
- `_get_target_tag_ids()` (8곳 적용)
- `_find_dual_marker_centers()` (4곳 적용)
- `_get_effective_ry()` (11곳 적용)
- `_settle()` (30+곳 적용)
- `_set_buttons_enabled()` 범용화

Phase 2 — 검출/안정화 통합 (-73줄):
- `_detect_dual_alignment()` 통합 검출 함수 (4개 함수 → 1개 + 4 thin wrapper)
- `_wait_for_position_stable()` 위치 안정화 통합 (3곳 → 1개)

Phase 3 — combined 재구성 (-35줄):
- `_stereo_align_z_core()`, `_stereo_align_y_core()` 코어 로직 분리
- `_on_stereo_calib_align_aruco_combined()` → 코어 메서드 호출로 재구성
- 적응형 정렬 통합은 분할 이동 차이로 보류 (상호 참조 주석 추가)

**결과:** 3960줄 → 3779줄 (총 -181줄, 4.6% 감소). 헬퍼 함수 10개 추출.

---

## 2026-02-28 | 리팩터링 후 코드 리뷰 — 버그 수정 7건

### 1. `_require_robot()` 무한 재귀 (CRITICAL)

**증상:** 조그 이동 시 `RecursionError: maximum recursion depth exceeded` → 프로그램 크래시

**원인:** `_require_robot()` 헬퍼 본문에서 `self.robot` 체크 대신 `self._require_robot()` 자기 호출

**수정:** `if not self._require_robot()` → `if not self.robot or not self.robot.is_connected`

**수정 파일:** `scripts/main_window.py:298-303`

### 2. `_on_ar_tag_align_parallel` — `axis` NameError (CRITICAL)

**증상:** tool.rot 회전 성공/실패 시 `NameError: name 'axis' is not defined` → 크래시

**원인:** 루프 변수 `vision_axis`를 사용해야 하는데 미정의 `axis` 변수 참조 (4곳)

**수정:** `axis.upper()` → `vision_axis.upper()` (line 1605-1610)

**수정 파일:** `scripts/main_window.py`

### 3. `_fine_align_axis` — 이중 sleep (CRITICAL)

**증상:** 0.1mm 미세 정렬 루프에서 불필요한 450ms 지연 (0.3s + 0.15s). 20회 반복 시 3초 손실

**원인:** `time.sleep(0.3)` 후 `self._settle(0.15)` 추가 호출 (settle 내부에도 sleep 존재)

**수정:** 두 호출을 `self._settle(0.3)` 하나로 통합

**수정 파일:** `scripts/main_window.py`

### 4. TF4→TF3 미복원 — `_on_ar_tag_align_single_axis` / `_parallel` (HIGH)

**증상:** tool.rot 실패 또는 예외 발생 시 로봇이 TF4 상태로 방치 → 후속 조그 이동이 잘못된 좌표계에서 실행

**원인:** `except` 블록과 중간 `return`에서 `_ensure_toolframe(3)` 미호출

**수정:** `try/finally` 패턴 + `tf_changed` 플래그로 TF3 항상 복원 보장

**수정 파일:** `scripts/main_window.py:1523-1571, 1573-1632`

### 5. `_on_jog_move_from_tab` — cmd_map 데드코드 (MEDIUM)

**증상:** `cmd_map` dict를 만들고 `cmd` 검증하지만, 실제 호출에서 `axis` 문자열 직접 전달 → 유지보수 혼란

**수정:** cmd_map 제거, `if axis not in ('x', 'y', 'z')` 단순 가드로 대체

**수정 파일:** `scripts/main_window.py:891`

### 6. spinbox singleStep 불일치 (MEDIUM)

**증상:** 조그 스핀박스 화살표 클릭 시 1.0mm씩 증가. 0.1mm 해상도를 지원하지만 UI에서 미반영

**수정:** `spinJogStepX/Y/Z`의 `singleStep`을 `0.1`로 설정 (두 UI 파일)

**수정 파일:** `ui/tab_task_edit.ui`, `ui/tab_aruco_reliability.ui`

### 7. Eye in Hand 탭 — DEBUG print() 잔존 (LOW)

**증상:** 탭 전환 시 `[DEBUG]` 메시지 5개가 stdout으로 출력

**수정:** print() 제거, 필요한 로그는 `self._log()`로 유지

**수정 파일:** `scripts/main_window.py:3698-3714`

### 0.1mm 파이프라인 검증 결과

End-to-End 스케일링 확인:

```text
UI spinbox(0.1mm) → signal(str, float) → send_base_linear(axis, 0.1)
  → int(round(0.1 × 10)) = 1 → to_uint16() → write_register
  → PRS: transx(1/10) = 0.1mm  ✓
```

- `int(round(val*10))` 사용 (truncation 방지): PASS
- CMD 50-53 사용 (deprecated 60-63 아님): PASS
- `to_uint16()` 음수 처리: PASS
- UI minimum=0.1, singleStep=0.1: PASS

### 향후 개선 권고 (미적용)

| 우선   | 내용                                                         |
| ------ | ------------------------------------------------------------ |
| MEDIUM | `_settle()` 50ms 간격 이벤트 펌핑 (E-stop 반응성 개선) |
| MEDIUM | 조그 거리/각도 상한 바운드 체크 (`MAX_JOG_DISTANCE_MM`) |
| MEDIUM | `_on_ar_tag_align_base_rz` 무제한 보정 클램핑 |
| LOW | `_measure_marker_dy_px`에 `_find_dual_marker_centers` 헬퍼 적용 |
| LOW | `QInputDialog` 인라인 import → top-level 이동 |
| LOW | ModbusClient 레지스터+커맨드 쓰기 Lock 보호 |

---

## 2026-02-28 | Laser Scan 탭 신규 생성 + ROI 시각화

**목적:** 로봇 Z축 스캔으로 대상물 각도 측정을 위한 전용 탭 추가. 레이저 검출 영역(ROI)을 이미지 중심 기준 좌우 대칭으로 표시.

### 1. Laser Scan 탭 생성

**신규 파일:**
- `ui/tab_laser_scan.ui` — 좌측 패널 (Camera Stream + 이미지 모드 + 조그 이동) + 빈 우측 패널
- `scripts/tabs/tab_laser_scan.py` — `QWidget + JogMixin` 구성, ArduCam 전용

**수정 파일:**
- `scripts/tabs/__init__.py` — `TabLaserScan` import/export 추가
- `scripts/main_window.py` — 3곳 수정:
  - import 추가 (line 25)
  - `_load_separated_tabs()`: 인덱스 8에 "Laser Scan" 탭 삽입 (line 185-187)
  - `_connect_tab_signals()`: log, camera, jog, arducam_required 시그널 연결 (line 259-266)
  - `_process_camera_frame()`: 탭 인덱스 8에서 `update_frame()` 호출 (line 3390-3391)
  - `_stop_all_cameras()`: `deactivate()` 호출 추가 (line 3687)

**기능:** 카메라 시작/정지, undistort 지원, 이미지 저장(`images/laser_scan/`), 6축 조그 이동

### 2. ROI 시각화 (이미지 중심 기준 좌우 대칭)

**신규 파일:**
- `config/laser_scan_roi.json` — ROI 설정 파일

```json
{
  "roi": {
    "offset_x": 500,
    "roi_width": 80,
    "roi_height": 600,
    "y_offset": 60,
    "color": [0, 0, 255],
    "thickness": 2
  }
}
```

| Field | Default | Description |
|-------|---------|-------------|
| offset_x | 500 | 이미지 중심에서 ROI 중심까지 X 거리 (px) |
| roi_width | 80 | ROI 폭 (px) |
| roi_height | 600 | ROI 높이 (px) |
| y_offset | 60 | ROI 상단 Y 오프셋 (px) |
| color | [0,0,255] | BGR 색상 (빨강) |
| thickness | 2 | 선 두께 (px) |

**수정 파일:** `scripts/tabs/tab_laser_scan.py`

**추가 메서드:**
- `_load_roi_config()` — JSON config 로드, 실패 시 None fallback
- `_compute_roi_rects(frame_w, frame_h, roi_cfg)` — `@staticmethod` 순수 함수. cx=width//2 기준 좌우 대칭 좌표 계산, 프레임 경계 클리핑
- `_draw_roi_boxes(frame)` — cv2.rectangle로 빨간 사각형 2개 시각화

**ROI 좌표 계산 (1280x720 기준):**
```
cx = 640
Left ROI:  x=100~180, y=60~660
Right ROI: x=1100~1180, y=60~660
```

**3-copy 패턴 적용:** `display_frame`(저장용)과 `display`(표시용) 분리 → 저장 이미지에 ROI 미포함

**교훈:** 정적 ROI는 탭 내 구현이 적절 (4-5줄 산술). `_compute_roi_rects`를 순수 함수로 구현하면 향후 서비스 레이어 추출이 용이.

---

## 2026-03-07 | ArduCam 카메라 교체 — 캘리브레이션 및 해상도 설정 변경

**증상:** ArduCam 카메라가 교체되어 새 카메라에 맞는 설정 필요

**원인:** 새 ArduCam(FHD Camera)은 해상도 1920×1080으로 변경됨. 기존 캘리브레이션 파라미터(1280×720, fx=3988.9)가 새 카메라와 불일치.

**수정 파일:**
- `config/arducam_calibration.yaml` — 새 카메라 캘리브레이션으로 교체
- `scripts/main_window.py` (line 67) — `color_resolution` 변경

**수정 내용:**
- 기존 캘리브레이션 → `arducam_calibration_backup_20260307.yaml`로 백업
- `charging_robot_camera_calibration_20260307.yaml` → `arducam_calibration.yaml`로 복사
- ArduCamManager 생성 시 `color_resolution=(1920, 1080)` 파라미터 추가

**변경 전후 비교:**

| 항목 | 이전 (1280×720) | 이후 (1920×1080) |
|------|-----------------|------------------|
| fx | 3988.9 | 5335.9 |
| fy | 4003.2 | 5347.3 |
| cx | 669.7 | 820.3 |
| cy | 241.6 | 478.2 |

**디바이스:** `FHD Camera` → `/dev/video6` (usb-0000:00:14.0-8), device_index=6 유지

**교훈:** ArduCam 교체 시 캘리브레이션 파일 교체 + 해상도 설정 변경 필수. `ArduCamManager`의 `color_resolution` 기본값이 (1280, 720)이므로 명시적 지정 필요.

---

## 2026-03-07 | 레이저 삼각측량 — 선형→비선형 기하학 모델 전환

**증상:** 선형 sensitivity 방식(`arctan(|slope|/sensitivity)`)으로 각도 추정 시, 스캔 범위(50mm vs 20mm)나 거리가 달라지면 4-5° 오차 발생. 50mm 캘리브레이션 sensitivity=2.59로 20mm 스캔 시 20° 실제 → 15.42° 추정.

**원인:** slope = fy·B·tan(α)/d² 관계에서 d(카메라-표면 거리)가 Z 이동에 따라 변함. slope 자체가 Z 위치의 비선형 함수이므로, 단일 sensitivity 상수로는 거리/범위가 달라지면 정확한 역산 불가.

**분석:** 50mm 스캔을 10mm 구간으로 분할 시 slope가 -0.88~-0.97로 변화 (d² 의존성 확인). 논문(docs/paper/s40430-022-03458-2.pdf) 기반 비선형 기하학 모델 도출.

**비선형 기하학 모델:**
- 레이저 원점: (Bx, 0, h) — 카메라 기준 상대 위치
- 레이저 방향: (cos α, 0, -sin α) — α는 레이저 각도
- 표면 평면: 수직에서 φ° 기울어진 평면, 거리 d0
- 교차점 계산 → 카메라 투영: `y = cy - fy × z_hit / x_hit`
- `scipy.optimize.differential_evolution`으로 (φ, d0) 피팅

```python
def y_model(delta, d0, phi_rad):
    tp = np.tan(phi_rad)
    t = (d0 - Bx + tp * (h - delta)) / (cos_a + sin_a * tp)
    z_hit = h - t * sin_a
    x_hit = Bx + t * cos_a
    return cy - fy * z_hit / x_hit
```

**피팅된 기하학 파라미터:**
- h = 70.0 mm (레이저-카메라 수직 거리)
- Bx = -30.0 mm (수평 오프셋)
- α = 13.278° (레이저 각도, 피팅 결과)

**수정 파일 (2개):**

| 파일 | 변경 내용 |
|------|-----------|
| `config/laser_triangulation_calib.json` | `sensitivity_px_per_mm` → `h_mm=70, Bx_mm=-30, alpha_deg=13.278` |
| `scripts/services/laser_scan_service.py` | `_load_triangulation_calib()` 새 키 검증, `_estimate_tilt_angle()` 비선형 모델 + differential_evolution, scipy import 추가 |

**검증 결과 (기존 스캔 데이터):**

| 실제 각도 | 스캔 범위 | 선형 방식 | 비선형 방식 | 오차 |
|-----------|----------|----------|------------|------|
| 20° | 50mm | 20.0° (캘리브레이션 기준) | 20.48° | +0.48° |
| 20° | 20mm | 15.42° (**4.58° 오차**) | 20.53° | +0.53° |
| 17° | 20mm | 16.65° | 17.72° | +0.72° |

**교훈:**
1. 삼각측량에서 단일 sensitivity 상수는 특정 스캔 범위/거리에서만 유효 — 범용성 부족
2. 비선형 기하학 모델은 스캔 범위/거리에 무관하게 1° 이내 정확도 유지
3. `differential_evolution`은 초기값 불필요한 전역 최적화 — 레이저 삼각측량처럼 파라미터 범위만 알고 정확한 초기값을 모를 때 적합
4. 기하학 파라미터(h, Bx, α)는 물리적 배치가 변하지 않는 한 재캘리브레이션 불필요

---

## 2026-03-07 | 충전건 결합 오프셋 설정 — ArUco 정렬→입구→완전결합 경로 정의

**목적:** ArUco 통합정렬 위치에서 충전건 결합 입구, 완전 결합까지의 변환 관계를 config로 정의하여 자동 결합 경로 생성 기반 마련

**측정 좌표 (2026-03-07):**

| 위치 | X | Y | Z | Rx | Ry | Rz |
|------|---|---|---|----|----|-----|
| ArUco 통합정렬 | 676.90 | -117.90 | 377.50 | 107.30 | -1.10 | 90.00 |
| 충전건 입구 | 761.97 | -113.06 | 508.74 | 113.50 | -0.20 | 87.50 |
| 완전 결합 | 991.20 | -131.30 | 478.00 | 109.40 | -0.90 | 87.90 |

**변환 분석:**

1. **ArUco → 입구 (tool frame 오프셋)**
   - Rx, Ry가 ArUco 보정마다 달라지므로 base frame 델타는 비일정
   - tool frame 오프셋으로 변환: `Δp_tool = R_aruco^T @ Δp_base`
   - **tool frame (mm):** x=7.4, y=150.5, z=42.2 (Rx/Ry 변화에 불변)
   - **회전 델타 (°):** ΔRx=+6.20, ΔRy=+0.90, ΔRz=-2.50

2. **입구 → 완전결합 (tool Z축 리니어)**
   - 입구에서 tool Z축 방향으로 직선 이동만으로 결합 완료
   - **tool Z 이동:** ~223 mm

**결합 절차 (계획):**
1. ArUco 통합정렬 → Rx, Ry 보정값 획득
2. Rx, Ry 회전 보정 적용 (위치 고정, 자세만 변경)
3. 보정된 자세에서 입구까지 위치 이동 (base frame 오프셋 — 미측정, 추후 보완)
4. 입구에서 tool Z축 리니어 ~223mm 이동 → 완전 결합

**수정 파일:**
- `config/charging_gun_coupling.json` — **신규 생성** (기준좌표 3개 + 오프셋 2단계)

---

## 2026-03-07 | 테스트 탭 — DS435→ArduCam 핸드오프 stage 2 depth 보정 미동작

### 증상
- 테스트 탭 핸드오프 2단계(X축 거리 370mm 유지)가 동작하지 않음
- 로그/UI에 아무런 실패 메시지 없이 stage 2가 건너뜀

### 원인 (3건)
1. **경계값 조건 버그**: `abs(depth_error) > DEPTH_TOLERANCE_MM` — depth 365mm, 목표 370mm일 때 오차 5.0mm = 허용치 5.0mm → `5.0 > 5.0 = False` → 이동 안 함
2. **Silent skip**: depth 측정 실패 또는 허용 범위 내일 때 로그/상태 출력 없음
3. **하드코딩**: `TARGET_DEPTH_MM = 370.0` 직접 기입, config 미사용

### 수정 내용
1. `>` → `>=` 비교 (스테레오 탭 + 테스트 탭 양쪽)
2. config 연동: `charging_coupling_config.json`의 `handoff.target_depth_mm` 사용
3. 전구간 로깅 추가 (depth 측정 성공/실패, 이동 결과, 건너뜀 사유)
4. **반복 수렴 제어**: 1회 이동 → 최대 5회 반복 + 3회 평균 측정 + 감쇠 계수 0.7 (DS435 depth ±2mm 노이즈 진동 방지)
5. 수렴 기준 1.5mm (DS435 depth 해상도 고려)

### 수정 파일
- `scripts/main_window.py` — `_on_test_handoff_ds435_to_arducam()`, `_on_stereo_calib_handoff_ds435_to_arducam()`

---

## 2026-03-07 | 테스트 탭 — DS435 depth 다중 샘플링

### 증상
- `_test_measure_ds435_marker_depth()` 마커 중심점에서 depth=None 반환 가능성

### 원인
- DS435 depth raw에서 마커 중심 5x5 패치에 유효 픽셀이 없는 경우 존재 (sweep 데이터에서 6개 중 5개 null 관찰)

### 수정 내용
- 중심 → 마커1 → 마커2 → 주변 ±20px 오프셋 총 7포인트 순차 샘플링
- 각 실패 단계별 로그 출력 (프레임 없음/intrinsics 없음/마커 미검출/타겟 ID 불일치/depth null)

### 수정 파일
- `scripts/main_window.py` — `_test_measure_ds435_marker_depth()`

---

## 2026-03-07 | 테스트 탭 — 레이저 스캔 결과 키 불일치 + disconnect 크래시

### 증상
1. 레이저 스캔 완료 후 결과 slope=0.000, R²=0.0000 표시 (실제 분석은 정상)
2. 스캔 완료 후 `TypeError: disconnect() failed` → 앱 크래시

### 원인
1. `_on_test_laser_scan_finished`에서 `results.get('slope_left')` 사용 — 실제 키는 `results['left']['slope_px_per_mm']`
2. `_cleanup_laser_scan_service()`에서 `step_data_captured.disconnect()` 시도 — 테스트 탭에서 해당 시그널 미연결. `except RuntimeError`로만 캐치하여 `TypeError` 누락

### 수정 내용
1. 결과 키 수정: `results['left']['slope_px_per_mm']`, `results['right']['r_squared']`, `results['angle_estimate']['estimated_deg']`
2. `except (RuntimeError, TypeError)` 로 변경

### 수정 파일
- `scripts/main_window.py` — `_on_test_laser_scan_finished()`, `_cleanup_laser_scan_service()`

---

## 2026-03-07 | 테스트 탭 — Z축 미세 보정 임계값 과대 + Rx 보정 버튼 추가

### 증상
- 4단계 ArduCam 통합 정렬에서 Z축 미세 보정이 실행되지 않음

### 원인
- Z/Y 미세 보정 임계값 1.5px — Z 잔여 오차가 X 정렬 후 1.5px 이하로 감소되어 건너뜀

### 수정 내용
1. 미세 보정 임계값 1.5px → **0.5px** (0.04mm 수준 정밀 보정)
2. 임계값 이하 시 "이미 수렴" 로그 출력 (건너뜀 사유 확인)
3. **Rx 보정 적용 버튼 추가**: 레이저 스캔 각도 추정값을 현재 Rx에 더해 `send_base_rotate('rx', angle)` 실행
4. 스캔 완료 전 버튼 비활성, 스캔 후 각도 유효 시 활성화

### 수정 파일
- `scripts/main_window.py` — `_test_align_aruco_combined()`, `_rebuild_test_tab()`, `_on_test_apply_ry()`

**미완료 사항:**
- Rx, Ry 보정 후 입구까지의 base frame 위치 오프셋 실측 필요 (tool frame 오프셋 검증)
- tool Z 리니어 거리 (~223mm) 정밀 실측 필요

**교훈:**
1. Rx, Ry가 보정마다 변하는 경우, base frame 오프셋이 비일정 → tool frame 오프셋으로 표현해야 일관성 확보
2. 회전 보정을 먼저 적용하고 위치를 이동하면, base frame 오프셋이 일정해져 계산이 단순해짐
3. 입구→결합은 tool Z 리니어 단일 축이므로, 정렬만 정확하면 결합 자체는 단순

---

## 2026-03-07 | 충전건 결합 — TF4 좌표계 불일치로 인한 위치 오차

### 증상
- ArUco 정렬 후 위치가 X=~673, Z=~377로 표시됨 (기대값: X=~522, Z=~451)
- 입구 이동 시 ~150mm X, ~74mm Z 오차 발생
- 같은 물리적 위치에서 좌표가 크게 다르게 읽힘

### 원인
- Tool Frame(TF)에 따라 TCP 좌표 해석이 달라짐
- TF3(기본값)과 TF4에서 동일 물리 위치의 좌표 차이: ΔX≈+150mm, ΔZ≈-74mm
- config 기준 좌표가 다른 TF에서 측정된 값으로 저장되어 있었음
- 탭 전환 시 TF 상태가 보장되지 않아 혼재 발생

### 수정 내용
1. **TF4 강제 설정**: 테스트 탭(Tab4) 진입 시 `send_set_toolframe(4)` 자동 실행
2. **이동 전 TF4 보장**: `_stop_cameras_for_movement()` 헬퍼에서 카메라 정지 + TF4 설정 + 버튼 상태 동기화
3. **config 좌표 전면 재측정**: 모든 reference_positions를 TF4 기준으로 재측정·갱신
4. **GroupBox 명시**: "충전건 결합 (TF4)" 라벨로 TF4 필수 표기

### 수정 파일
- `scripts/main_window.py` — `_on_tab_changed()`, `_stop_cameras_for_movement()`, GroupBox 타이틀
- `config/charging_gun_coupling.json` — 전체 reference_positions 재측정

### 교훈
- **TF는 좌표 해석의 기준** — 동일 물리 위치도 TF에 따라 수십~수백mm 차이 발생
- config에 좌표 저장 시 반드시 TF 기준 명시 필요
- 워크플로우 전체에서 TF 일관성 보장이 최우선

---

## 2026-03-07 | 충전건 결합 — 장거리 이동 시 wait_for_done 타임아웃

### 증상
- 입구 이동(~128mm X, ~155mm Z 병진) 또는 TCP Z 리니어(223mm) 시 30초 타임아웃 발생
- 로봇은 아직 이동 중인데 타임아웃으로 명령 실패 처리

### 원인
- `wait_for_done()` 고정 30초 타임아웃 — 장거리 저속 이동에 부족
- 로봇이 실제로 움직이고 있는 동안에도 시간 기반으로 타임아웃 판정

### 수정 내용
1. **`wait_for_done_motion_aware()` 신규 메서드 추가** (modbus_client.py)
   - Phase 1: Running 상태 감지 (기존 wait_for_done과 동일)
   - Phase 2: 위치 기반 타임아웃 — 현재 TCP pose를 주기적으로 읽어 이전과 비교
   - 위치 변화 > 0.05mm → last_motion_time 갱신 (이동 중 판정)
   - 위치 변화 없이 idle_timeout(기본 10초) 경과 → 타임아웃
   - max_timeout(기본 120초) 안전 제한
2. 충전건 결합 이동 핸들러에서 `wait_for_done_motion_aware()` 사용

### 수정 파일
- `scripts/Robot/communication/modbus_client.py` — `wait_for_done_motion_aware()` 추가
- `scripts/main_window.py` — `_on_test_move_to_entrance()`, `_on_test_release_return()` 등

---

## 2026-03-07 | 충전건 결합 — config 좌표 재측정 (TF4 기준)

### 증상
- config의 entrance 좌표(761.97, -113.06, 508.74)가 실제 입구 위치와 ~100mm 이상 불일치
- delta 오프셋(97.09, -2.01, 111.79)도 부정확

### 원인
- 기존 측정이 TF4가 아닌 다른 TF 기준으로 수행됨
- TF3↔TF4 좌표 차이가 오프셋에도 전파

### 수정 내용 (TF4 기준 재측정 결과)

| 항목 | 이전값 | 수정값 (TF4) |
|------|--------|-------------|
| ds435_detection | (469.0, -120.7, 515.2) | (524.3, -125.9, 491.7) |
| aruco_alignment | (522.00, -117.80, 450.90) | 유지 (TF4 기준 확인) |
| entrance | (761.97, -113.06, 508.74) | (650.50, -120.70, 606.20) |
| corrected_to_entrance delta | (97.09, -2.01, 111.79) | (128.50, -2.90, 155.30) |
| entrance_to_coupling tool_z_step | 50.0mm | 35.0mm |
| release_return tcp_z_retract | 200.0mm | 100.0mm |

### 수정 파일
- `config/charging_gun_coupling.json`

---

## 2026-03-07 | 충전건 결합 — 해제 복귀 기능 구현

### 내용
- "해제 복귀" 버튼 (`btnTestReleaseReturn`) 추가
- **동작 순서**:
  1. 카메라 정지 + TF4 강제
  2. TCP Z 후퇴 (`tcp_z_retract_mm`: 100mm, CMD 12 음수)
  3. `wait_for_done_motion_aware()` 완료 대기
  4. 입구→보정위치 역변환 (corrected_to_entrance delta 부호 반전)
  5. 현재 위치 + 역delta → `send_move_to_pose()` 절대 이동
  6. `wait_for_done_motion_aware()` 완료 대기

### 수정 파일
- `scripts/main_window.py` — `_on_test_release_return()`, 버튼 레이아웃

---

## 2026-03-07 | 충전건 결합 — 카메라 관리 개선

### 내용
1. **이동 전 카메라 자동 정지**: `_stop_cameras_for_movement()` 헬퍼로 통합
2. **탭 진입 시 카메라 자동 시작**: `_on_tab_changed()`에서 테스트 탭 진입 시 ArduCam 자동 구동
3. **토글 버튼 상태 동기화**: 카메라 정지/시작 시 `btnTestToggleCameras` checked/text 동기화
4. **config 핫 리로드**: 매 버튼 클릭마다 `charging_gun_coupling.json` 재로드 (수동 재시작 불필요)

### 수정 파일
- `scripts/main_window.py`

---

## 2026-03-11 | "Running 상태 감지 실패 (5초 타임아웃)" — Phase 1 근본 원인 3가지 수정

**증상:** 로봇 이동 명령 후 "이동 실패: Running 상태 감지 실패 (5초 타임아웃)" 에러 발생. motion-aware timeout 수정 이후에도 간헐적으로 재현.

**원인 분석 (3가지):**

1. **잔여 STATUS_DONE false positive**: 이전 명령의 `task_done=2(DONE)`가 레지스터에 남은 상태에서 다음 명령의 Phase 1이 폴링을 시작하면 즉시 성공 반환(false positive). 실제 명령이 처리되지 않았을 수 있음. 무증상 버그.

2. **PRS cleanup 레이스 — 명령 덮어쓰기**: PRS cleanup 순서 `task_done=2 → task_number=0 → task_done=0`. Python이 `task_done=2`를 읽어 성공 반환 후 즉시 새 명령을 쓰면, PRS cleanup의 `task_number=0`이 새 명령을 덮어씀. 명령 소실 → Phase 1이 5초간 IDLE만 감지 → 타임아웃.

3. **고속 완료 명령 미감지**: `toolframe()`, `workframe()` 등 즉시 완료 명령은 `RUNNING→DONE→IDLE` 전체 사이클이 ~3ms. Python 폴링 간격 20ms 내에 모든 상태 변화가 완료되어 아무것도 감지하지 못함 → 타임아웃.

**수정 내용:**

1. **`write_command()` pre-flight 체크 추가**: 명령 전송 전 PRS가 `STATUS_IDLE(0)` 상태가 될 때까지 최대 2초 대기. 이전 명령의 DONE/RUNNING 소진 후에만 새 명령 전송 → 원인 1, 2 해결.

2. **Phase 1 고속 완료 감지 추가** (`wait_for_done`, `wait_for_done_motion_aware`): `STATUS_IDLE` + 명령 레지스터(351) == 0 이면 "고속 처리 완료" 반환. PRS가 명령을 읽고 즉시 처리한 경우(`task_number=0`으로 클리어됨) 정상 완료로 판단 → 원인 3 해결.

**수정 파일:** `scripts/Robot/communication/modbus_client.py`

---

---

## 2026-03-11 | ArduCam 이미지 미표시 — device_index 오류

### 증상
- Stereo Calibration 탭과 테스트(충전건 결합) 탭에서 ArduCam 이미지가 표시되지 않음
- DS435 이미지는 정상 표시

### 원인
- `main_window.py`에서 `ArduCamManager(device_index=6, ...)`로 하드코딩
- RealSense 카메라가 `/dev/video2`~`/dev/video7`(인덱스 2~7)을 점유하면서 FHD Camera(ArduCam)가 `/dev/video0`(인덱스 0)으로 밀림
- index=6은 RealSense 가상 디바이스를 가리켜 프레임 미출력

### 수정 내용
- `main_window.py:68`: `device_index=6` → `device_index=0`

### 현재 장치 배치 (참고)
| 장치 | /dev/video | OpenCV 인덱스 |
|------|-----------|--------------|
| FHD Camera (ArduCam) | video0, video1 | 0 |
| RealSense Depth Ca | video2~video7 | 2~7 |

---

## 2026-03-11 | ArduCam 포트 자동 감지 기능 추가

### 배경
- USB 장치 연결 순서/재부팅에 따라 ArduCam device_index가 변동
- 하드코딩된 인덱스를 매번 수동 수정해야 하는 문제

### 구현 내용
sysfs 기반 자동 감지:
- `/sys/class/video4linux/videoN/name` — 장치명 키워드 매칭 (`"FHD Camera"`)
- `/sys/class/video4linux/videoN/index == 0` — 캡처 노드만 선택 (메타데이터 노드 제외)
- 감지 실패 시 `fallback_index: 0` 사용 + 로그 출력

### 수정/추가 파일
- `scripts/utils/camera_utils.py` (신규) — `detect_arducam_index(keyword) -> Optional[int]`
- `config/camera_config.json` (신규) — `device_index: "auto"`, `name_keyword: "FHD Camera"`, `fallback_index: 0`
- `scripts/main_window.py` — config 로드 → 자동 감지 → fallback 순으로 인덱스 결정 후 ArduCamManager 생성
- `scripts/services/arducam_manager.py` — `list_available_cameras()` sysfs 실제 장치명 표시
- `scripts/test_arducam_dual_aruco.py`, `scripts/utils/coordinate_visualizer.py`, `scripts/analyze_ippe_ambiguity.py`, `scripts/test_aruco_detect.py` — `DEVICE_INDEX = 6` 하드코딩 → `detect_arducam_index() if is not None else 6`

### 주의
- `detect_arducam_index() or N` 패턴 금지 (index=0일 때 0 or N → N 버그)
- 반드시 `idx if idx is not None else fallback` 패턴 사용

---

## 2026-03-11 | AI Detection 탭 — 카메라 피드 미표시 (`isVisible()` 오작동)

### 증상
- "AI detection" 탭 클릭 시 DS435/ArduCam 카메라 피드가 표시되지 않음 (검은 화면 유지)
- 다른 탭(테스트, Stereo Calibration 등)에서는 동일 카메라가 정상 표시

### 원인
- `_on_ai_ds435_frame()` / `_on_ai_arducam_frame()` 에서 `self.tabAIDetection.isVisible()` 체크 사용
- **`QWidget.isVisible()`은 Qt UI 파일(`uic.loadUi`)로 생성된 위젯에는 정상 동작하지만, Python 코드로 생성 후 `insertTab()`으로 동적 삽입된 QWidget에서는 현재 탭이어도 `False`를 반환할 수 있음**
- `tabTest`(UI 파일 위젯)는 정상 → `tabAIDetection`(동적 생성 위젯)만 실패

### 수정 내용
- `isVisible()` → `self.tabWidget.currentWidget() is self.tabAIDetection` 로 교체
- `currentWidget()`은 탭위젯에 직접 현재 페이지를 조회하므로 동적 삽입 탭에서도 확실히 동작

### 교훈
- **동적으로 `insertTab()`된 QWidget은 `isVisible()` 대신 `tabWidget.currentWidget() is widget` 패턴 사용**
- `uic.loadUi()`로 로드된 원본 탭 위젯(tabTest 등)과 Python 코드로 삽입된 탭 위젯의 동작이 다름

### 수정 파일
- `scripts/main_window.py` (`_on_ai_ds435_frame`, `_on_ai_arducam_frame`)

---

## 2026-03-11 | AI Detection 탭 신규 생성

### 구현 내용
- 탭 위치: "Laser Scan" 탭 다음, "테스트" 탭 앞 (인덱스 9)
- 카메라 뷰: DS435 640×360 + ArduCam 640×360 (`setFixedSize` 고정, `#222` 배경)
- 그룹박스 2개: `"DS435 AI Detection"`, `"Arducam AI Detection"` (각 fixedHeight=100)
- `"Port hall detect"` 버튼: YOLOv8-seg 추론 토글 (checkable)
  - 모델: `config/AI_weights/ArduCam/best.onnx`
  - 추론: `scripts/AI/view_results.py`의 `overlay_masks()` 재사용
  - **백그라운드 `threading.Thread`로 추론 (CPU 1~5초/프레임 → UI 블로킹 방지)**
  - 추론 중에는 이전 결과 유지, 완료 시 세그멘테이션 오버레이 표시

### 이미지 크기 이슈 방지 설계
- `setFixedSize(640, 360)` 엄격 적용 (Expanding 정책 금지)
- `cv2.resize(frame, (640, 360))` — 표시 전 항상 명시적 리사이즈
- `currentWidget()` 기반 가시성 체크 (위 이슈 참조)

### 신규/수정 파일
- `scripts/tabs/tab_ai_detection.py` (신규)
- `scripts/tabs/__init__.py` (`TabAIDetection` import/export 추가)
- `scripts/main_window.py` (import, insertTab, 프레임 핸들러, 시그널 연결, 탭 활성화)
- 수동 고정 필요 시 `config/camera_config.json`에서 `"device_index": 0` 처럼 정수 지정

---

## 2026-03-21: 통합 정렬 코드 리팩토링

### 문제
- `_on_aruco_align_combined()`가 수평/수직 정렬 로직을 자체 구현 (Ry→BaseY→Rz 보정)
- 기존 `_on_aruco_align_y()`(수평), `_on_aruco_align_x()`(수직) 핸들러와 로직 중복
- 통합 정렬의 보정 항목(Ry+BaseY+Rz)이 개별 버튼과 불일치 (수직=Rz vs 수직=BaseZ)

### 수정
- `_on_aruco_align_combined()` 자체 보정 로직 삭제
- 기존 `_on_aruco_align_y()` → `_on_aruco_align_x()` 순차 호출로 변경
- 버튼 활성화/비활성화는 각 핸들러 내부에서 자체 관리

### 수정 파일
- `scripts/main_window.py` (`_on_aruco_align_combined` 간소화)

---

## 2026-03-21: ±180° Euler 정규화 잔존 이슈 일괄 수정

### 문제
- `normalize_angle()`이 `(-180, 180]` 반개구간 사용 → PRS while-loop `[-180, 180]` 폐구간과 불일치
  - `-180°` 입력 시 Python이 `+180°`으로 변환 → delta 계산에서 360° 오차 가능
- `set_rz.py` CLI의 target args 미정규화 → `--rz 185` 입력 시 범위 초과 전송
- 5개 테스트 스크립트에 인라인 `% 360` 복제 코드 산재
- `send_base_rotate`의 `before_pose` 미정규화 + threshold `> 180` (방향 모호 미처리)

### 수정
- `normalize_angle` → `math.remainder(angle, 360)` 전환 (IEEE 754 symmetric remainder, `[-180, 180]` 폐구간)
- `set_rz.py` target args + 현재 위치 모두 `math.remainder` 정규화
- `scripts/tests/angle_utils.py` 공유 모듈 생성, 5개 테스트 스크립트 인라인 코드 제거
- `send_base_rotate`: `before_pose` 정규화 + while-loop delta 래핑 + `>= 180` threshold

### 교훈
- Python `% 360`는 `(-180, 180]` (반개구간), `math.remainder`는 `[-180, 180]` (폐구간)
- PRS 컨트롤러와 범위 일치가 이중 보호 아키텍처의 핵심
- 정확히 ±180° delta는 방향 모호 → 증분 회전이 안전

### 수정 파일
- `scripts/Robot/communication/modbus_client.py` (`normalize_angle`, `send_base_rotate`)
- `scripts/tests/set_rz.py` (target args 정규화)
- `scripts/tests/angle_utils.py` (신규: 공유 정규화 유틸리티)
- `scripts/tests/` 내 5개 스크립트 (인라인 `% 360` → `angle_utils` 전환)
