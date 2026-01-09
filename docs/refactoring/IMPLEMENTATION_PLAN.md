# Charging Robot Task Manager 리팩토링 실행 계획

## 작업 시작 전 확인

- [ ] 현재 아키텍처 분석 완료
- [ ] 백업 브랜치 생성 (`git checkout -b backup/before-refactoring`)
- [ ] 기존 기능 테스트 완료

---

## 현재 상태 분석

### MainWindow 현황
- **파일:** `scripts/main_window.py`
- **라인 수:** 1,914 lines
- **메서드 수:** 83개
- **주요 책임:** UI, 로봇 제어, 카메라, Vision, 정렬, 데이터 수집, 레시피 관리

### 현재 프로젝트 구조
```
scripts/
├── main.py                 # 진입점
├── main_window.py          # 메인 윈도우 (1,914 lines) ← 리팩토링 대상
├── Robot/
│   ├── communication/      # ModbusClient
│   ├── control/            # RobotController
│   └── pose/               # PoseManager, PoseType
├── Sensor/
│   └── aruco/              # ArucoCameraPoseEstimator
└── test_*.py               # 테스트 스크립트
```

### MainWindow 메서드 분류

| 카테고리 | 메서드 수 | 라인 수 (추정) | 분리 대상 |
|----------|----------|----------------|-----------|
| 초기화 | 8 | ~200 | - |
| Task 관리 | 15 | ~350 | TaskManager |
| 로봇 연결/상태 | 8 | ~150 | - |
| 카메라/Vision | 12 | ~400 | CameraManager |
| Aruco 감지 | 4 | ~150 | VisionManager |
| 정렬 (Alignment) | 5 | ~250 | AlignmentService |
| 데이터 수집 | 6 | ~200 | DataCollector |
| Pose 관리 | 5 | ~150 | PoseService |
| 레시피/파일 | 8 | ~100 | RecipeManager |
| 디버그/유틸 | 12 | ~150 | - |

---

## 단계별 실행 순서

### 🎯 1단계: CameraManager 분리

**목표:** 카메라 초기화, 시작/정지, 프레임 업데이트 로직 분리

#### 작업 순서
```bash
# 1. 브랜치 생성
git checkout -b feature/camera-manager

# 2. 서비스 디렉토리 생성
mkdir -p scripts/services
touch scripts/services/__init__.py

# 3. CameraManager 작성
# - 파일: scripts/services/camera_manager.py
```

#### 분리할 메서드
- `_init_camera()` → `CameraManager.__init__()`
- `_on_start_camera()` → `CameraManager.start()`
- `_on_stop_camera()` → `CameraManager.stop()`
- `_update_camera_frame()` → `CameraManager.get_frame()`
- `_create_intrinsics_object()` → `CameraManager` 내부
- `_on_gamma_changed()` → `CameraManager.set_gamma()`
- `_on_snapshot()` → `CameraManager.snapshot()`

#### 체크리스트
- [ ] `scripts/services/camera_manager.py` 작성
- [ ] Qt Signal로 프레임 업데이트 알림
- [ ] MainWindow에서 CameraManager 인스턴스 사용
- [ ] 카메라 시작/정지 테스트

**예상 코드 감소:** MainWindow ~250 lines

---

### 🎯 2단계: VisionManager 분리

**목표:** Aruco 태그 감지 로직 독립화

#### 분리할 메서드
- `_detect_aruco_markers()` → `VisionManager.detect_markers()`
- `detect_aruco_tag()` → `VisionManager.detect_tag()`
- `_calculate_average_marker()` → `VisionManager` 내부
- `_get_num_samples()` → `VisionManager.num_samples`

#### 체크리스트
- [ ] `scripts/services/vision_manager.py` 작성
- [ ] CameraManager와 연동
- [ ] 태그 감지 테스트

**예상 코드 감소:** MainWindow ~150 lines

---

### 🎯 3단계: AlignmentService 분리

**목표:** 정렬 로직 독립화

#### 분리할 메서드
- `_on_align_center()` → `AlignmentService.align_center()`
- `_on_align_pose()` → `AlignmentService.align_pose()`
- `_on_align_full()` → `AlignmentService.align_full()`
- `_update_align_status()` → Signal로 처리

#### 체크리스트
- [ ] `scripts/services/alignment_service.py` 작성
- [ ] VisionManager, RobotController 의존성 주입
- [ ] 정렬 테스트

**예상 코드 감소:** MainWindow ~250 lines

---

### 🎯 4단계: DataCollector 분리

**목표:** 데이터 수집 로직 독립화

#### 분리할 메서드
- `_on_start_collect()` → `DataCollector.start()`
- `_on_stop_collect()` → `DataCollector.stop()`
- `_collect_sample()` → `DataCollector.collect_sample()`
- `_print_collect_statistics()` → `DataCollector.get_statistics()`
- `_on_save_collect()` → `DataCollector.save()`

#### 체크리스트
- [ ] `scripts/services/data_collector.py` 작성
- [ ] 수집 시작/정지/저장 테스트

**예상 코드 감소:** MainWindow ~200 lines

---

### 🎯 5단계: TaskManager 분리

**목표:** Task 생성/수정/실행 로직 독립화

#### 분리할 메서드
- `_on_add_task()` → `TaskManager.add_task()`
- `_on_delete_task()` → `TaskManager.delete_task()`
- `_on_move_up()` → `TaskManager.move_up()`
- `_on_move_down()` → `TaskManager.move_down()`
- `_on_apply_params()` → `TaskManager.update_params()`
- `_update_task_ids()` → `TaskManager` 내부
- `_get_task_display_name()` → `TaskManager` 내부
- `_refresh_task_list()` → Signal로 처리

#### 체크리스트
- [ ] `scripts/services/task_manager.py` 작성
- [ ] JOB_TYPES 정의 이동
- [ ] Task 추가/삭제/이동 테스트

**예상 코드 감소:** MainWindow ~300 lines

---

### 🎯 6단계: PoseService 분리

**목표:** Pose 저장/로드/이동 로직 독립화

#### 분리할 메서드
- `_on_save_current_pose()` → `PoseService.save_pose()`
- `_on_delete_pose()` → `PoseService.delete_pose()`
- `_on_move_to_saved_pose()` → `PoseService.move_to_pose()`
- `_on_approach_pose()` → `PoseService.approach_pose()`
- `_refresh_saved_poses_list()` → Signal로 처리

#### 체크리스트
- [ ] `scripts/services/pose_service.py` 작성
- [ ] PoseManager와 연동
- [ ] Pose 저장/이동 테스트

**예상 코드 감소:** MainWindow ~200 lines

---

### 🎯 7단계: NetworkManager 분리 (선택적)

**목표:** 네트워크 유틸리티 독립화

#### 분리할 메서드
- `_get_network_interfaces()` → `NetworkManager.get_interfaces()`
- `_get_pc_ip_fallback()` → `NetworkManager.get_local_ip()`
- `_init_pc_ip_combo()` → MainWindow에서 NetworkManager 사용

**예상 코드 감소:** MainWindow ~80 lines

---

## 최종 목표

### 코드 감소 목표

| 단계 | 작업 | 코드 감소 | 누적 감소 | MainWindow 라인 수 |
|------|------|----------|-----------|-------------------|
| 현재 | - | 0 | 0 | 1,914 |
| 1단계 | CameraManager | 250 | 250 | 1,664 |
| 2단계 | VisionManager | 150 | 400 | 1,514 |
| 3단계 | AlignmentService | 250 | 650 | 1,264 |
| 4단계 | DataCollector | 200 | 850 | 1,064 |
| 5단계 | TaskManager | 300 | 1,150 | 764 |
| 6단계 | PoseService | 200 | 1,350 | 564 |
| 7단계 | NetworkManager (선택) | 80 | 1,430 | 484 |

**최종 목표:** MainWindow 800 lines 이하 (58% 감소)

---

## 제안 아키텍처

### 리팩토링 후 구조
```
scripts/
├── main.py
├── main_window.py              # UI 이벤트 핸들링만 (~800 lines)
├── Robot/                      # 기존 모듈 (변경 없음)
│   ├── communication/
│   │   └── modbus_client.py    # Modbus TCP 통신
│   ├── control/
│   │   └── robot_controller.py # 로봇 이동 명령
│   └── pose/
│       ├── pose_manager.py     # 포즈 저장/로드
│       └── constants.py        # 레지스터, 명령 상수
├── Sensor/
│   └── aruco/
└── services/                   # 새로 추가
    ├── __init__.py
    ├── camera_manager.py       # 카메라 관리
    ├── vision_manager.py       # Aruco 감지
    ├── alignment_service.py    # 정렬 로직
    ├── data_collector.py       # 데이터 수집
    ├── task_manager.py         # Task 관리
    ├── pose_service.py         # Pose 관리
    └── network_manager.py      # 네트워크 유틸 (선택)
```

### Robot 모듈 연동 관계

```
┌─────────────────────────────────────────────────────────────────┐
│                        MainWindow (UI)                          │
│  - UI 이벤트 핸들링                                              │
│  - Qt Signal/Slot 연결                                          │
└─────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│ CameraManager │    │  TaskManager  │    │  PoseService  │
│ - RealSense   │    │ - JOB_TYPES   │    │ - 포즈 저장   │
│ - 프레임 캡처 │    │ - Task 생성   │    │ - 포즈 이동   │
└───────────────┘    └───────────────┘    └───────────────┘
        │                     │                     │
        ▼                     │                     │
┌───────────────┐             │                     │
│ VisionManager │             │                     │
│ - Aruco 감지  │             │                     │
│ - 태그 추적   │             │                     │
└───────────────┘             │                     │
        │                     │                     │
        ▼                     ▼                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                    AlignmentService                             │
│  - align_center()  # VisionManager + RobotController            │
│  - align_pose()    # VisionManager + RobotController            │
│  - align_full()    # VisionManager + RobotController            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Robot 모듈 (기존)                            │
├─────────────────────────────────────────────────────────────────┤
│  RobotController                                                │
│  ├── move_to_pose(x, y, z, rx, ry, rz)                         │
│  ├── move_to_saved_pose(name)                                  │
│  ├── approach_pose(name, distance)                             │
│  └── get_current_camera_pose()                                 │
│                              │                                  │
│                              ▼                                  │
│  ModbusClient                                                   │
│  ├── connect() / disconnect()                                  │
│  ├── send_tcp_linear(axis, distance)    # command 10-13        │
│  ├── send_tcp_rotate(axis, angle)       # command 14-17        │
│  ├── send_move_to_pose(x,y,z,rx,ry,rz)  # command 20           │
│  ├── send_gripper(action)               # command 30-32        │
│  ├── send_go_home()                     # command 1            │
│  └── read_camera_pose()                 # register 158-169     │
│                              │                                  │
│                              ▼                                  │
│  PoseManager                                                    │
│  ├── save_pose(name, pose)                                     │
│  ├── get_pose(name)                                            │
│  └── delete_pose(name)                                         │
└─────────────────────────────────────────────────────────────────┘
```

### 서비스 ↔ Robot 모듈 연동 상세

| 서비스 | 사용하는 Robot 클래스 | 주요 메서드 |
|--------|----------------------|-------------|
| **TaskManager** | `ModbusClient` | `send_tcp_linear()`, `send_move_to_pose()`, `send_gripper()` |
| **AlignmentService** | `RobotController` | `move_to_pose()`, `get_current_camera_pose()` |
| **PoseService** | `RobotController`, `PoseManager` | `save_current_pose()`, `move_to_saved_pose()` |
| **DataCollector** | `RobotController` | `get_current_camera_pose()` |

### 의존성 주입 예시

```python
# main_window.py
class MainWindow(QMainWindow):
    def __init__(self):
        # Robot 모듈 초기화
        self.modbus = ModbusClient(ip="192.168.0.29", port=1502)
        self.pose_manager = PoseManager()
        self.robot = RobotController(self.modbus, self.pose_manager)

        # 서비스 초기화 (Robot 모듈 주입)
        self.camera_manager = CameraManager()
        self.vision_manager = VisionManager(self.camera_manager)
        self.alignment_service = AlignmentService(self.vision_manager, self.robot)
        self.task_manager = TaskManager(self.modbus)
        self.pose_service = PoseService(self.robot, self.pose_manager)
        self.data_collector = DataCollector(self.vision_manager, self.robot)
```

---

## 성공 기준

### 필수 조건
- [ ] 모든 기존 기능 정상 작동
- [ ] MainWindow 1,000 lines 이하
- [ ] 각 서비스 클래스 독립 테스트 가능

### 선택 조건
- [ ] 단위 테스트 추가
- [ ] 코드 문서화

---

## 긴급 롤백 절차

문제 발생 시:

```bash
# 1. 현재 작업 저장
git stash

# 2. 백업 브랜치로 복원
git checkout backup/before-refactoring

# 3. 문제 분석 후 재시도
```

---

## 다음 단계

1. **즉시 시작:** CameraManager 분리
2. **순차 진행:** 의존성 순서에 따라 VisionManager → AlignmentService → ...
3. **테스트:** 각 단계별 기능 테스트 필수

