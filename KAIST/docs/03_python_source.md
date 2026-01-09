# Python 소스 가이드

비전 시스템 및 Modbus 통신을 담당하는 Python 코드 설명입니다.

## 1. 파일 구조

```
src/
├── main_main_refactored.py     # 메인 제어 루프
├── modbus_robot_interface.py   # Modbus 통신 인터페이스
├── main_charuco_gun.py         # 충전건 비전 (독립 실행)
├── main_charuco_port.py        # 충전포트 비전 (독립 실행)
├── main_charuco.py             # ChArUco 보드 검출
├── main_main.py                # 이전 버전 메인
├── utils.py                    # 유틸리티 함수
├── kalman_filter/
│   └── kalman_filter.py        # 포즈 칼만 필터
├── keypoint_detector/
│   ├── aruco.py                # ArUco/ChArUco 검출
│   ├── pnp.py                  # PnP 포즈 추정
│   ├── config.py               # 설정
│   └── ...
└── vision_processor/
    ├── vision_processor_abstract.py
    ├── vision_processor_gun.py
    └── vision_processor_port.py
```

## 2. 주요 모듈

### 2.1 main_main_refactored.py

메인 제어 루프를 실행하는 엔트리 포인트입니다.

#### 클래스: RSFramesUtils
RealSense 카메라 프레임 캡처를 담당합니다.

```python
class RSFramesUtils:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 15)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

    def get_frames(self):
        """카메라 프레임 캡처 및 반환"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        return color_image, intrinsics
```

#### 클래스: CommandProcessor
명령 라우팅 및 비전 프로세서 관리를 담당합니다.

```python
class CommandProcessor:
    def __init__(self, modbus: ModbusRobotInterface):
        self.modbus = modbus
        self.processors: Dict[str, VisionProcessorAbstract] = {}
        self.command_map: Dict[int, dict] = {}

    def register_processor(self, name: str, processor: VisionProcessorAbstract):
        """비전 프로세서 등록"""
        self.processors[name] = processor

    def register_command(self, cmd: int, processor_name: str,
                        response_type: int, use_offset: bool = False):
        """명령-프로세서 매핑 등록"""
        self.command_map[cmd] = {
            'processor': processor_name,
            'response_type': response_type,
            'use_offset': use_offset
        }

    def process_command(self, cmd: int, frame_util: RSFramesUtils) -> bool:
        """명령 처리 실행"""
        config = self.command_map[cmd]
        processor = self.processors[config['processor']]

        # 비전 처리
        pose_result = self._run_vision(processor, frame_util)

        # 포즈 계산
        pose_main, pose_back = self._calculate_poses(pose_result, use_offset)

        # Modbus 전송
        if pose_result["is_converged"]:
            self.modbus.write_pose(pose_back, pose_back)
            self.modbus.write_response(config['response_type'])
```

#### 함수: setup_command_processor
명령 프로세서 초기화 및 설정입니다.

```python
def setup_command_processor(modbus, intrinsics) -> CommandProcessor:
    cp = CommandProcessor(modbus)

    # 비전 프로세서 초기화
    vp_gun = VisionProcessorGun(intrinsics)
    vp_port = VisionProcessorPort(intrinsics)

    # 프로세서 등록
    cp.register_processor('gun', vp_gun)
    cp.register_processor('port', vp_port)

    # 명령 매핑
    cp.register_command(1, 'gun', response_type=1, use_offset=True)   # Rilham get
    cp.register_command(2, 'gun', response_type=1, use_offset=True)   # Rilham put
    cp.register_command(3, 'gun', response_type=1, use_offset=True)   # Car get
    cp.register_command(4, 'gun', response_type=1, use_offset=True)   # Car put

    return cp
```

#### 함수: main_loop
메인 제어 루프입니다.

```python
def main_loop():
    with ModbusRobotInterface(ip="192.168.0.29", port=1502) as modbus:
        rs_utils = RSFramesUtils()
        command_processor = setup_command_processor(modbus, rs_utils.intrinsics)

        while True:
            # 로봇에서 명령 읽기
            cmd = modbus.read_command()

            if cmd is not None:
                # 명령 처리
                command_processor.process_command(cmd, rs_utils)
```

---

### 2.2 modbus_robot_interface.py

로봇과의 Modbus TCP 통신을 담당합니다.

#### 클래스: ModbusRobotInterface

```python
class ModbusRobotInterface:
    # 레지스터 주소
    REGISTER_CMD = 351          # 명령 레지스터
    REGISTER_RESP = 352         # 응답 레지스터
    REGISTER_POSE_MAIN = 301    # 메인 포즈 (301~306)
    REGISTER_POSE_BACK = 307    # 백오프 포즈 (307~312)
    REGISTER_BASE_CAM = 158     # 카메라 포즈 (158~169)

    def __init__(self, ip="192.168.0.29", port=1502, timeout=0.1):
        self.client = ModbusTcpClient(ip, port=port, timeout=timeout)

    def connect(self) -> bool:
        """Modbus 연결"""
        return self.client.connect()

    def read_command(self) -> Optional[int]:
        """명령 읽기 (레지스터 351)"""
        rr = self.client.read_holding_registers(address=351, count=1)
        return rr.registers[0]

    def write_response(self, value: int) -> bool:
        """응답 쓰기 (레지스터 352)"""
        self.client.write_registers(352, [value])

    def write_pose(self, pose_main: List[int], pose_back: List[int]) -> bool:
        """포즈 데이터 쓰기 (레지스터 301~312)"""
        self.client.write_registers(301, pose_main)
        self.client.write_registers(307, pose_back)

    def read_camera_pose(self) -> Tuple[float, ...]:
        """카메라 포즈 읽기 (레지스터 158~169)"""
        rr = self.client.read_holding_registers(address=158, count=12)
        return self._modbus_to_pose(rr)
```

#### 메서드: pose_to_modbus_data
포즈를 Modbus 레지스터 형식으로 변환합니다.

```python
@staticmethod
def pose_to_modbus_data(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg):
    """
    포즈 → Modbus 레지스터 변환
    - 위치: mm → int16 (×10000)
    - 회전: deg → int16 (×10)
    """
    def to_uint16(val):
        return int(val + 65536) if val < 0 else int(val)

    def wrap_deg(deg):
        return ((deg + 180) % 360) - 180

    x_val = int(round(x_mm * 10000))
    y_val = int(round(y_mm * 10000))
    z_val = int(round(z_mm * 10000))
    rx_val = int(round(wrap_deg(rx_deg) * 10))
    ry_val = int(round(wrap_deg(ry_deg) * 10))
    rz_val = int(round(wrap_deg(rz_deg) * 10))

    vals = [x_val, y_val, z_val, rx_val, ry_val, rz_val]
    return [to_uint16(v) for v in vals]
```

---

### 2.3 main_charuco_gun.py / main_charuco_port.py

독립 실행 가능한 비전 테스트 스크립트입니다.

#### 주요 구성요소

```python
# Kalman 필터 초기화
kalman_filter = StaticObjectPoseKalmanFilter(
    process_noise_std=1e-6,
    position_noise_std=0.02,    # 2cm 위치 불확실성
    rotation_noise_std=0.1      # 5.7도 회전 불확실성
)

# ChArUco 보드 설정
charuco_board_config = {
    'grid_size': (8, 6),
    'square_size': 0.029,       # 29mm
    'marker_size': 0.02,        # 20mm
}

# ArUco 검출기
aruco_estimator = ArucoCameraPoseEstimator()

# YOLO 모델
model = YOLO('best.pt')

# PnP 정렬기
pnp_aligner = PnPKeypointAligner(
    objp,
    camera_matrix,
    dist_coeffs,
    offset=np.array([0.03, -0.069, 0, 0, 0, 0])
)
```

#### 함수: get_port_normal_vector

```python
def get_port_normal_vector(pipeline, align, aruco_estimator, ...):
    # 1. 프레임 캡처
    frames = pipeline.wait_for_frames()
    color_frame = aligned_frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    # 2. YOLO 객체 검출
    results = model(color_image)

    # 3. ChArUco 마커 검출
    marker_poses = aruco_estimator.detect_and_estimate_charuco_pose(
        color_image, intrinsics, charuco_board_config
    )

    # 4. PnP 포즈 추정
    _, pose_info = pnp_aligner.align_keypoints_pnp_marker(
        color_image, marker_poses, marker_offset
    )

    # 5. Kalman 필터 업데이트
    kalman_filter.predict()
    kalman_filter.update(pose_info['world_pose'], ...)
    current_pose = kalman_filter.get_pose(marker_poses)

    return current_pose, color_image, intrinsics
```

---

### 2.4 kalman_filter.py

포즈 추정 안정화를 위한 칼만 필터입니다.

```python
class StaticObjectPoseKalmanFilter:
    def __init__(self, process_noise_std, position_noise_std, rotation_noise_std):
        # 상태: [x, y, z, qw, qx, qy, qz]
        self.state = np.zeros(7)
        self.covariance = np.eye(7)

    def predict(self):
        """예측 단계"""
        # 정적 객체이므로 상태 유지
        self.covariance += self.Q

    def update(self, measurement, measurement_quality):
        """업데이트 단계"""
        # 칼만 이득 계산
        K = self.covariance @ H.T @ inv(H @ self.covariance @ H.T + R)
        # 상태 업데이트
        self.state = self.state + K @ (measurement - H @ self.state)
        # 공분산 업데이트
        self.covariance = (I - K @ H) @ self.covariance

    def get_pose(self, marker_poses) -> dict:
        """현재 포즈 반환"""
        return {
            'position_object_to_world': self.state[:3],
            'rotation_object_to_world': quat_to_matrix(self.state[3:7]),
            'is_converged': self.check_convergence(),
            'convergence_score': self.get_convergence_score(),
            ...
        }
```

---

## 3. 데이터 흐름

```
┌──────────────┐
│  RealSense   │
│   Camera     │
└──────┬───────┘
       │ RGB 1920×1080
       ▼
┌──────────────┐
│    YOLO      │
│   Detection  │
└──────┬───────┘
       │ Bounding Box
       ▼
┌──────────────┐
│   ChArUco    │
│  Detection   │
└──────┬───────┘
       │ Marker Pose
       ▼
┌──────────────┐
│     PnP      │
│   Solver     │
└──────┬───────┘
       │ 6DoF Pose
       ▼
┌──────────────┐
│   Kalman     │
│   Filter     │
└──────┬───────┘
       │ Stabilized Pose
       ▼
┌──────────────┐
│   Modbus     │
│   Interface  │
└──────┬───────┘
       │ Register Write
       ▼
┌──────────────┐
│    Robot     │
│  Controller  │
└──────────────┘
```

## 4. 실행 방법

### 메인 시스템 실행
```bash
cd /home/amap/Project/Charging_Robot/KAIST/src
python main_main_refactored.py
```

### 비전 테스트 (충전건)
```bash
python main_charuco_gun.py
```

### 비전 테스트 (충전포트)
```bash
python main_charuco_port.py
```

## 5. 설정

### 카메라 설정
```python
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 15)
```

### Modbus 설정
```python
ModbusRobotInterface(ip="192.168.0.29", port=1502, timeout=0.1)
```

### ChArUco 보드 설정
```python
charuco_board_config = {
    'grid_size': (8, 6),
    'square_size': 0.029,   # 29mm
    'marker_size': 0.02,    # 20mm
}
```
