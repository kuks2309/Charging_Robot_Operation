# Eye-in-Hand Data Capture Implementation Plan

## Context

### Original Request
Hand-in-Eye 캘리브레이션 탭(tab_eye_in_hand.py)에서 데이터 캡처 기능 구현

### Requirements
1. **로봇 베이스 좌표**: x,y,z,rx,ry,rz (기존 `read_current_pose()` 활용)
2. **TCP(TF4) 좌표**: TF4 좌표계로 변환된 x,y,z,rx,ry,rz
3. **카메라 이미지**: 기존 캡처 방식 활용
4. **저장 형식**: CSV 파일

### Research Findings

#### 현재 구현 분석
- `tab_eye_in_hand.py`: 이미 `on_capture_at_position()` 메서드가 존재하나 CSV 저장 없음
- `tab_calibration.py`: 자동 캡처 워크플로우 참고 가능 (`_on_run_auto_capture`)
- `modbus_client.py`: `read_current_pose()`는 베이스 좌표만 반환 (레지스터 158~169)
- 현재 툴프레임 읽기: `read_current_toolframe()` (레지스터 219)

#### TF4 좌표 읽기 방안: Modbus 레지스터 직접 읽기

**구현 방식**: 로봇 컨트롤러의 Modbus 레지스터에서 TF4 좌표를 직접 읽기

**필요한 레지스터 정보**:
- 베이스 좌표: 레지스터 158~169 (기존 `read_current_pose()`)
- TF4 좌표: 별도 레지스터 필요 (로봇 컨트롤러 확인 필요)
  - 예상 레지스터: 170~181 또는 유사 범위
  - 또는 TF4 설정 후 동일 레지스터(158~169)에서 TF4 기준 좌표 반환

**구현 순서**:
1. 로봇 컨트롤러(Main_task.prs)에서 TF4 좌표 출력 레지스터 확인
2. `read_tf4_pose()` 메서드 추가 - 해당 레지스터 직접 읽기
3. 레지스터가 없으면 `send_set_toolframe(4)` → `read_current_pose()` 방식 사용

---

## Work Objectives

### Core Objective
Eye-in-Hand 캘리브레이션 탭에서 로봇 포즈(베이스 + TCP) 및 카메라 이미지를 CSV 형식으로 저장하는 데이터 캡처 기능 구현

### Deliverables
1. CSV 저장 기능이 포함된 `on_capture_at_position()` 메서드
2. TF4 좌표 읽기 기능 (로봇 통신 확장)
3. CSV 파일 구조 및 헤더
4. UI 업데이트 (캡처 카운트 표시)

### Definition of Done
- [ ] 자동 캡처 시 각 위치에서 CSV 행 추가
- [ ] CSV에 타임스탬프, 베이스 좌표(6축), TF4 좌표(6축), 이미지 파일명 포함
- [ ] **이미지-CSV 동기화**: 각 CSV 행의 `image_filename`이 실제 저장된 이미지 파일명과 정확히 일치
- [ ] 이미지 파일과 CSV 파일이 같은 디렉토리에 저장
- [ ] 기존 자동 캡처 워크플로우와 통합
- [ ] 캡처 실패 시에도 CSV 행 기록 (chessboard_detected=False로 표시)

---

## Guardrails

### Must Have
- 기존 `on_capture_at_position()` 로직 유지 (체스보드 감지, solvePnP 등)
- CSV 헤더 포함
- 에러 시 로그 출력 및 graceful 실패

### Must NOT Have
- 기존 캘리브레이션 탭(tab_calibration.py) 수정 금지
- 로봇 통신 프로토콜 변경 최소화
- UI 레이아웃 대폭 변경 금지

---

## Task Flow

```
[Phase 1: TF4 좌표 읽기 기능]
    └── Task 1.1: ModbusClient에 TF4 좌표 읽기 메서드 추가
    └── Task 1.2: 로봇 컨트롤러 레지스터 확인/추가 (필요시)

[Phase 2: CSV 저장 구조]
    └── Task 2.1: CSV 파일 구조 정의
    └── Task 2.2: CSV 헬퍼 클래스/함수 작성

[Phase 3: 캡처 로직 통합]
    └── Task 3.1: on_capture_at_position() 메서드 수정
    └── Task 3.2: _on_run_auto_capture() 시작/종료 시 CSV 파일 관리
    └── Task 3.3: UI 업데이트 (캡처 상태 표시)

[Phase 4: 테스트]
    └── Task 4.1: 단위 테스트 (CSV 저장)
    └── Task 4.2: 통합 테스트 (자동 캡처 워크플로우)
```

---

## Flowchart: 자동 캡처 워크플로우

### 전체 흐름
```
┌─────────────────────────────────────────────────────────────────┐
│                    사용자: "Run Auto Capture" 클릭               │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│  _on_run_auto_capture() 시작                                    │
│  ├── 저장 디렉토리 생성: calibration/hand_eye_YYYYMMDD_HHMMSS/  │
│  ├── CSV 파일 초기화: hand_eye_data.csv                         │
│  └── 헤더 작성                                                   │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│  for each position in generated_positions:                      │
│  ├── 로봇 이동 (TCP Linear) ────────────────────────────────┐   │
│  └── on_capture_at_position() 호출 ◄────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ on_capture_at_position │
                    └────────────────────────┘
                                 │
        ┌────────────────────────┼────────────────────────┐
        ▼                        ▼                        ▼
┌──────────────┐        ┌──────────────┐        ┌──────────────┐
│ 1. 로봇 좌표 │        │ 2. TF4 좌표  │        │ 3. 카메라    │
│    읽기      │        │    읽기      │        │    이미지    │
└──────────────┘        └──────────────┘        └──────────────┘
        │                        │                        │
        ▼                        ▼                        ▼
┌──────────────┐        ┌──────────────┐        ┌──────────────┐
│read_current_ │        │read_tf4_pose │        │capture_frame │
│pose()        │        │() [NEW]      │        │()            │
│Reg 158~169   │        │Reg 170~181   │        │              │
└──────────────┘        └──────────────┘        └──────────────┘
        │                        │                        │
        └────────────────────────┼────────────────────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ 체스보드 감지 (기존)    │
                    │ cv2.solvePnP()         │
                    └────────────────────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ CSV 행 추가 [NEW]      │
                    │ _append_csv_row()      │
                    └────────────────────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ 이미지 파일 저장        │
                    │ {index}_pose.png       │
                    └────────────────────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ 다음 위치로 반복        │
                    └────────────────────────┘
```

### 데이터 흐름
```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Modbus    │     │   Camera    │     │   OpenCV    │
│  Registers  │     │   Frame     │     │  solvePnP   │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌──────────────────────────────────────────────────────┐
│                  on_capture_at_position()            │
│  ┌────────────┐  ┌────────────┐  ┌────────────────┐  │
│  │ base_pose  │  │ tf4_pose   │  │ camera_pose    │  │
│  │ (x,y,z,    │  │ (x,y,z,    │  │ (rvec, tvec)   │  │
│  │  rx,ry,rz) │  │  rx,ry,rz) │  │                │  │
│  └─────┬──────┘  └─────┬──────┘  └───────┬────────┘  │
│        └───────────────┼─────────────────┘           │
└────────────────────────┼─────────────────────────────┘
                         │
                         ▼
              ┌──────────────────────┐
              │    CSV 파일 저장     │
              │  hand_eye_data.csv   │
              └──────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│ timestamp | index | image | base_x...rz | tcp_x...rz | rvec  │
├──────────────────────────────────────────────────────────────┤
│ 2026-01-31T10:00:00 | 0 | 0000.png | 100,200,... | 50,100,...│
│ 2026-01-31T10:00:05 | 1 | 0001.png | 110,205,... | 55,105,...│
│ ...                                                          │
└──────────────────────────────────────────────────────────────┘
```

### Modbus 레지스터 맵
```
┌────────────────────────────────────────────────────────┐
│                    Modbus Registers                    │
├─────────────┬──────────────┬──────────────────────────┤
│  Register   │    Type      │       Description        │
├─────────────┼──────────────┼──────────────────────────┤
│  158~169    │  float32×6   │ 베이스 좌표 (x,y,z,rx,ry,rz) │
│  170~181    │  float32×6   │ TF4 좌표 [확인필요]       │
│  219        │  int16       │ 현재 툴프레임 번호        │
└─────────────┴──────────────┴──────────────────────────┘
```

---

## Detailed TODOs

### Phase 1: TF4 좌표 읽기 기능

#### Task 1.1: ModbusClient 확장 - Modbus 레지스터 직접 읽기
**파일**: `scripts/Robot/communication/modbus_client.py`

**작업 내용**:
```python
# TF4 좌표용 레지스터 상수 추가
REGISTER_TF4_POSE = 170  # 로봇 컨트롤러에서 확인 필요 (170~181: 12 registers, float32)

def read_tf4_pose(self) -> Optional[Tuple[float, ...]]:
    """
    TF4 좌표 직접 읽기 (Modbus 레지스터)

    Returns:
        (x, y, z, rx, ry, rz) in mm/deg or None if failed
    """
    result = self.read_registers(self.REGISTER_TF4_POSE, 12)
    if result is None:
        return None
    # float32 변환 (2 registers per float, 6 floats total)
    values = []
    for i in range(0, 12, 2):
        raw = (result[i] << 16) | result[i+1]
        values.append(struct.unpack('>f', struct.pack('>I', raw))[0])
    return tuple(values)
```

**수용 기준**:
- [ ] TF4 좌표 레지스터 주소 확인 (로봇 컨트롤러 Main_task.prs)
- [ ] `read_tf4_pose()` 메서드 구현
- [ ] 로봇 미연결 시 None 반환
- [ ] 기존 `read_current_pose()`와 동일한 형식 반환 (x,y,z,rx,ry,rz)

---

### Phase 2: CSV 저장 구조

#### Task 2.1: CSV 파일 구조 정의

**CSV 헤더**:
```
timestamp,index,image_filename,base_x,base_y,base_z,base_rx,base_ry,base_rz,tcp_x,tcp_y,tcp_z,tcp_rx,tcp_ry,tcp_rz,chessboard_detected,rvec_x,rvec_y,rvec_z,tvec_x,tvec_y,tvec_z
```

**필드 설명**:
| 필드 | 타입 | 설명 |
|------|------|------|
| timestamp | str | ISO 형식 타임스탬프 |
| index | int | 캡처 인덱스 (0부터) |
| image_filename | str | 저장된 이미지 파일명 |
| base_x~base_rz | float | 베이스 좌표계 (mm, deg) |
| tcp_x~tcp_rz | float | TF4 좌표계 (mm, deg) |
| chessboard_detected | bool | 체스보드 감지 여부 |
| rvec_x~rvec_z | float | 카메라-체스보드 회전 벡터 |
| tvec_x~tvec_z | float | 카메라-체스보드 변환 벡터 (mm) |

#### Task 2.2: CSV 헬퍼 구현
**파일**: `scripts/tabs/tab_eye_in_hand.py` (클래스 내 메서드)

```python
def _init_csv_file(self, save_dir: str) -> str:
    """CSV 파일 초기화 및 헤더 작성"""
    pass

def _append_csv_row(self, csv_path: str, data: dict):
    """CSV 파일에 행 추가"""
    pass
```

---

### Phase 3: 캡처 로직 통합

#### Task 3.1: on_capture_at_position() 수정
**파일**: `scripts/tabs/tab_eye_in_hand.py`

**현재 코드 (240~295행)** 수정:
```python
def on_capture_at_position(self, index: int, total: int, save_dir: str,
                            target_pose: tuple, csv_path: str = None) -> bool:
    """각 위치에서의 캡처 - 로봇 포즈 + 카메라 포즈 + CSV 저장"""

    # 1. 기존 로직 유지 (체스보드 감지, solvePnP)
    # 2. 베이스 좌표 읽기 (기존)
    # 3. TF4 좌표 읽기 (신규)
    # 4. 이미지 저장 (기존)
    # 5. CSV 행 추가 (신규)
```

**수용 기준**:
- [ ] 베이스 좌표 + TF4 좌표 모두 저장
- [ ] 체스보드 미감지 시에도 좌표 데이터는 저장
- [ ] solvePnP 결과 CSV에 포함

#### Task 3.2: _on_run_auto_capture() 수정
**파일**: `scripts/tabs/tab_eye_in_hand.py`

**CalibrationMixin 오버라이드** (297~309행 확장):
```python
def _on_run_auto_capture(self):
    """자동 캡처 실행 - CSV 파일 초기화 포함"""
    # 1. CSV 파일 경로 생성
    # 2. 헤더 작성
    # 3. 부모 클래스 메서드 호출 (super())
    # 4. 완료 시 CSV 경로 로그 출력
```

#### Task 3.3: UI 업데이트
**파일**: `ui/tab_eye_in_hand.ui` (필요시)

- 캡처된 데이터 수 표시 라벨 (기존 `labelPosePairCountValue` 활용 가능)
- CSV 파일 경로 표시 (선택적)

---

### Phase 4: 테스트

#### Task 4.1: 단위 테스트
- CSV 파일 생성 확인
- 헤더 형식 검증
- 행 추가 시 데이터 무결성

#### Task 4.2: 통합 테스트
- 자동 캡처 워크플로우 전체 실행
- 이미지 파일 + CSV 파일 매칭 확인
- 로봇 미연결 시 graceful 실패

---

## Commit Strategy

1. **Commit 1**: `[feat] Add TF4 pose reading to ModbusClient`
2. **Commit 2**: `[feat] Add CSV export to Eye-in-Hand data capture`
3. **Commit 3**: `[test] Add unit tests for Eye-in-Hand CSV export`

---

## Success Criteria

- [ ] 자동 캡처 완료 시 `hand_eye_YYYYMMDD_HHMMSS.csv` 파일 생성
- [ ] CSV 파일에 모든 캡처 포인트의 좌표 데이터 포함
- [ ] 이미지 파일명과 CSV 행이 1:1 매칭
- [ ] 기존 Hand-Eye 캘리브레이션 워크플로우 정상 동작

---

## Notes

### TF4 좌표 읽기 우선순위
1. **로봇 컨트롤러 레지스터 확인**: Main_task.prs에 TF4 좌표 출력 레지스터가 있는지 확인
2. **없으면 임시 해결책**: 베이스 좌표만 저장 후, 후처리로 TF4 변환 (Hand-Eye 행렬 사용)
3. **장기 해결책**: 로봇 컨트롤러에 TCP 좌표 출력 기능 추가

### 기존 코드 재사용
- `tab_calibration.py`의 `_on_run_auto_capture()` 패턴 참고
- `CalibrationMixin`의 자동 캡처 워크플로우 활용
- `modbus_client.py`의 레지스터 읽기 패턴 활용
