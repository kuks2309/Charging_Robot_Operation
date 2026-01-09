# 통신 프로토콜

Python 비전 시스템과 로봇 컨트롤러 간의 Modbus TCP 통신 프로토콜입니다.

## 1. 연결 정보

| 항목 | 값 |
|------|-----|
| 프로토콜 | Modbus TCP |
| IP 주소 | 192.168.0.29 |
| 포트 (기존) | 1502 |
| 포트 (Main_task) | 502 |
| Timeout | 0.1초 |

---

## 2. Main_task.prs 레지스터 맵 (신규)

PC에서 로봇을 직접 제어하는 통합 프로토콜입니다.

### 2.1 제어 레지스터

| 주소 | 이름 | 방향 | 타입 | 설명 |
|------|------|------|------|------|
| 301 | command | PC → Robot | uint16 | 명령 코드 |
| 302 | mode | PC → Robot | uint16 | 이동 모드 (0=상대, 1=절대) |
| 315 | status | Robot → PC | uint16 | 상태 코드 |

### 3.2 포즈 데이터 레지스터 (Float32)

| 주소 | 이름 | 방향 | 타입 | 설명 |
|------|------|------|------|------|
| 303-304 | X | PC → Robot | float32 | X 위치 (mm) |
| 305-306 | Y | PC → Robot | float32 | Y 위치 (mm) |
| 307-308 | Z | PC → Robot | float32 | Z 위치 (mm) |
| 309-310 | Rx | PC → Robot | float32 | X 회전 (deg) |
| 311-312 | Ry | PC → Robot | float32 | Y 회전 (deg) |
| 313-314 | Rz | PC → Robot | float32 | Z 회전 (deg) |

### 2.3 명령 코드 (레지스터 301)

| 코드 | 명령 | 설명 |
|------|------|------|
| 0 | IDLE | 대기 |
| 1 | GO_HOME | var.p(100)으로 이동 |
| 2 | SET_HOME | 현재 위치를 var.p(100)에 저장 |
| 10 | LINEAR_X | X축 직선 이동 |
| 11 | LINEAR_Y | Y축 직선 이동 |
| 12 | LINEAR_Z | Z축 직선 이동 |
| 13 | LINEAR_XYZ | XYZ 동시 직선 이동 |
| 14 | ROTATE_RX | Rx축 회전 |
| 15 | ROTATE_RY | Ry축 회전 |
| 16 | ROTATE_RZ | Rz축 회전 |
| 17 | ROTATE_RXRYRZ | RxRyRz 동시 회전 |
| 20 | MOVE_POSE | 6DOF 포즈 이동 |
| 30 | GRIPPER_OPEN | 그리퍼 열기 |
| 31 | GRIPPER_CLOSE | 그리퍼 닫기 |
| 32 | GRIPPER_HOME | 그리퍼 홈 |

### 2.4 상태 코드 (레지스터 315)

| 값 | 상수 | 의미 |
|----|------|------|
| 0 | STATUS_IDLE | 대기 중 |
| 1 | STATUS_RUNNING | 실행 중 |
| 2 | STATUS_DONE | 완료 |
| 3 | STATUS_ERROR | 오류 |

### 2.5 통신 시퀀스 (Main_task)

```
PC (Client)                        Robot (Server)
  │                                    │
  │  write(302, mode)                  │
  ├───────────────────────────────────►│
  │  write(303-304, X_float)           │
  ├───────────────────────────────────►│
  │  write(301, command)               │
  ├───────────────────────────────────►│
  │                                    │ status = RUNNING
  │                                    │ 로봇 이동
  │                                    │
  │         read(315) = RUNNING        │
  │◄───────────────────────────────────┤
  │                                    │
  │         (폴링 반복)                 │
  │                                    │
  │         read(315) = DONE           │
  │◄───────────────────────────────────┤ 이동 완료
  │                                    │ command = 0
```

### 2.6 Float32 변환 (Python)

```python
import struct

def float_to_registers(value):
    """Float32를 2개의 uint16 레지스터로 변환"""
    packed = struct.pack('>f', value)
    hi = int.from_bytes(packed[0:2], 'big')
    lo = int.from_bytes(packed[2:4], 'big')
    return [hi, lo]

def registers_to_float(regs):
    """2개의 uint16 레지스터를 Float32로 변환"""
    packed = regs[0].to_bytes(2, 'big') + regs[1].to_bytes(2, 'big')
    return struct.unpack('>f', packed)[0]

# 사용 예시
client.write_registers(303, float_to_registers(100.5))  # X = 100.5mm
```

---

## 3. Vision_task.prs 레지스터 맵 (기존)

### 3.1 명령/응답 레지스터

| 주소 | 이름 | 방향 | 설명 |
|------|------|------|------|
| 351 | task_number | Robot → Python | 작업 명령 번호 |
| 352 | task_done | Python → Robot | 완료 신호 |

### 3.2 포즈 데이터 레지스터

| 주소 | 이름 | 방향 | 설명 | 단위 |
|------|------|------|------|------|
| 301 | x | Python → Robot | X 위치 (Main) | mm × 10 |
| 302 | y | Python → Robot | Y 위치 (Main) | mm × 10 |
| 303 | z | Python → Robot | Z 위치 (Main) | mm × 10 |
| 304 | Rx | Python → Robot | X 회전 (Main) | deg × 10 |
| 305 | Ry | Python → Robot | Y 회전 (Main) | deg × 10 |
| 306 | Rz | Python → Robot | Z 회전 (Main) | deg × 10 |
| 307 | x2 | Python → Robot | X 위치 (Back) | mm × 10 |
| 308 | y2 | Python → Robot | Y 위치 (Back) | mm × 10 |
| 309 | z2 | Python → Robot | Z 위치 (Back) | mm × 10 |
| 310 | Rx2 | Python → Robot | X 회전 (Back) | deg × 10 |
| 311 | Ry2 | Python → Robot | Y 회전 (Back) | deg × 10 |
| 312 | Rz2 | Python → Robot | Z 회전 (Back) | deg × 10 |

### 3.3 카메라 포즈 레지스터

| 주소 | 이름 | 방향 | 설명 |
|------|------|------|------|
| 158-159 | x_cam | Robot → Python | 카메라 X 위치 (float32) |
| 160-161 | y_cam | Robot → Python | 카메라 Y 위치 (float32) |
| 162-163 | z_cam | Robot → Python | 카메라 Z 위치 (float32) |
| 164-165 | rx_cam | Robot → Python | 카메라 X 회전 (float32) |
| 166-167 | ry_cam | Robot → Python | 카메라 Y 회전 (float32) |
| 168-169 | rz_cam | Robot → Python | 카메라 Z 회전 (float32) |

## 4. 명령 코드 (Vision_task)

### 4.1 task_number (레지스터 351)

| 값 | 명령 | 설명 |
|-----|------|------|
| 0 | IDLE | 대기 상태 |
| 1 | RILHAM_GET | 충전 스테이션에서 충전건 픽업 |
| 2 | RILHAM_PUT | 충전 스테이션에 충전건 반납 |
| 3 | CAR_GET | 차량에서 충전건 분리 |
| 4 | CAR_PUT | 차량에 충전건 삽입 |

### 4.2 task_done (레지스터 352)

| 값 | 의미 | 설명 |
|-----|------|------|
| 0 | PROCESSING | 비전 처리 중 |
| 1 | POSE_BACK_READY | 백오프 포즈 준비됨 (301-306) |
| 2 | POSE_MAIN_READY | 메인 포즈 준비됨 (307-312) |

## 5. 데이터 변환 (Vision_task)

### 5.1 Python → Robot (쓰기)

#### 위치 데이터 (m → 레지스터)
```python
def position_to_register(value_m):
    value_mm = value_m * 1000       # m → mm
    value_scaled = value_mm * 10    # mm × 10
    value_int = int(round(value_scaled))

    # int16 → uint16 변환
    if value_int < 0:
        return value_int + 65536
    return value_int
```

#### 회전 데이터 (deg → 레지스터)
```python
def rotation_to_register(value_deg):
    # 각도 정규화 (-180 ~ 180)
    normalized = ((value_deg + 180) % 360) - 180

    value_scaled = normalized * 10  # deg × 10
    value_int = int(round(value_scaled))

    # int16 → uint16 변환
    if value_int < 0:
        return value_int + 65536
    return value_int
```

#### 예제
| 실제 값 | 계산 | 레지스터 값 |
|---------|------|-------------|
| x = 0.5m | 0.5 × 1000 × 10 = 5000 | 5000 |
| x = -0.3m | -0.3 × 1000 × 10 = -3000 → -3000 + 65536 | 62536 |
| rx = 45° | 45 × 10 = 450 | 450 |
| rx = -90° | -90 × 10 = -900 → -900 + 65536 | 64636 |

### 5.2 Robot → Python (읽기)

#### 레지스터 → 실제 값
```lua
-- Robot Script (Lua)
x = modserv.read_register('x')

-- uint16 → int16 변환
if x > 32767 then
    x = x - 65537
end

-- 스케일 복원
x_mm = x / 10
```

#### float32 읽기 (카메라 포즈)
```python
def read_float32(registers):
    # 2개의 uint16 → float32
    hi, lo = registers[0], registers[1]
    byte_data = lo.to_bytes(2, 'big') + hi.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]
```

## 6. 통신 시퀀스 (Vision_task)

### 6.1 정상 동작 시퀀스

```
Robot                              Python
  │                                  │
  │  task_number = 1 (351)           │
  ├─────────────────────────────────►│
  │                                  │ 비전 처리 시작
  │                                  │
  │      task_done = 0 (352)         │
  │◄─────────────────────────────────┤
  │                                  │
  │                                  │ 포즈 계산 중...
  │                                  │
  │   pose_main (301-306)            │
  │◄─────────────────────────────────┤
  │   pose_back (307-312)            │
  │◄─────────────────────────────────┤
  │                                  │
  │      task_done = 1 (352)         │
  │◄─────────────────────────────────┤
  │                                  │
  │ 로봇 이동                         │
  │                                  │
  │      task_done = 0 (352)         │
  │◄─────────────────────────────────┤
  │                                  │
  │   (반복)                          │
  │                                  │
  │      task_done = 2 (352)         │
  │◄─────────────────────────────────┤ 최종 위치
  │                                  │
  │ 그리퍼 동작                       │
  │                                  │
```

### 6.2 Robot Script 측 폴링 로직

```lua
while move_count < 10 do
    move_signal = modserv.read_register('task_done')

    if move_signal == 1 then
        -- 중간 포즈로 이동
        x = modserv.read_register('x')
        ...
        movel(pose(x/10, y/10, z/10, ...))
        modserv.write_register('task_done', 0)

    elseif move_signal == 2 then
        -- 최종 포즈로 이동
        x2 = modserv.read_register('x2')
        ...
        movel(pose(x2/10, y2/10, z2/10, ...))
        break
    end
end
```

### 6.3 Python 측 처리 로직

```python
def process_command(cmd):
    # 처리 시작 신호
    modbus.write_response(0)

    # 비전 처리
    pose_result = run_vision(processor)

    # 수렴 확인
    if pose_result["is_converged"]:
        # 포즈 전송
        modbus.write_pose(pose_main, pose_back)
        # 완료 신호
        modbus.write_response(1)  # 또는 2
```

## 7. 에러 처리

### 7.1 통신 에러

| 상황 | Python 동작 | Robot 동작 |
|------|-------------|------------|
| 연결 실패 | 재연결 시도 | 대기 |
| 읽기 실패 | None 반환 | 이전 값 유지 |
| 쓰기 실패 | 재시도 | 대기 |
| Timeout | 재연결 | 10회 이동 후 경고 |

### 7.2 비전 에러

| 상황 | Python 동작 |
|------|-------------|
| 마커 미검출 | task_done = 0 유지 |
| 포즈 미수렴 | task_done = 0 유지 |
| 예외 발생 | task_done = 0, 명령 리셋 |

```python
except Exception as e:
    modbus.write_response(0)
    modbus.reset_command()  # 351 = 0
```

## 8. 타이밍

| 항목 | 값 |
|------|-----|
| Modbus Timeout | 100ms |
| 카메라 프레임 레이트 | 15fps (66ms) |
| 칼만 필터 수렴 | ~25 프레임 (~1.7초) |
| 그리퍼 grip 동작 | ~4.8초 |
| 그리퍼 ungrip 동작 | ~2.5초 |
