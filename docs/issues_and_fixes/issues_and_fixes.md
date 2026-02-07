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
