# 로봇 스크립트 가이드

로봇 컨트롤러에서 실행되는 스크립트 파일들의 상세 설명입니다.

## 1. 파일 구조

| 확장자 | 설명 | 형식 |
|--------|------|------|
| `.prs` | 프로그램 스크립트 | Lua 기반 텍스트 |
| `.dat` | 위치 데이터 | 바이너리 |

## 2. 스크립트 디렉토리

| 디렉토리 | 용도 |
|----------|------|
| `KAIST/robot_scripts/` | 기존 비전 기반 작업 스크립트 |
| `Robot_scripts/robot_scripts/` | 신규 Modbus 통합 제어 스크립트 |

---

## 3. Main_task.prs (신규 통합 제어 스크립트)

**위치:** `Robot_scripts/robot_scripts/Main_task.prs`

PC에서 Modbus 명령을 통해 로봇을 원격 제어하는 통합 스크립트입니다. 기존 Vision_task.prs와 달리, 단일 스크립트에서 모든 동작을 처리합니다.

### 3.1 주요 특징

- Modbus Server 모드로 동작
- 명령 코드 기반 동작 선택
- 상대/절대 좌표 모드 지원
- Float32 데이터 타입 지원
- Tool Frame 3 사용

### 3.2 레지스터 맵

#### 위치 읽기 (Float32)
| 레지스터 | 용도 | 타입 |
|----------|------|------|
| 158-159 | 현재 X | float32 (2 regs) |
| 160-161 | 현재 Y | float32 (2 regs) |
| 162-163 | 현재 Z | float32 (2 regs) |
| 164-165 | 현재 Rx | float32 (2 regs) |
| 166-167 | 현재 Ry | float32 (2 regs) |
| 168-169 | 현재 Rz | float32 (2 regs) |

#### 명령 파라미터 (int16 × 10 스케일)
| 레지스터 | 이름 | 용도 |
|----------|------|------|
| 301 | x | X값 (실제값 × 10) |
| 302 | y | Y값 (실제값 × 10) |
| 303 | z | Z값 (실제값 × 10) |
| 304 | Rx | Rx값 (실제값 × 10) |
| 305 | Ry | Ry값 (실제값 × 10) |
| 306 | Rz | Rz값 (실제값 × 10) |
| 351 | task_number | 명령 코드 |
| 352 | task_done | 상태 코드 |

### 3.3 상태 코드 (레지스터 352)

| 값 | 의미 |
|----|------|
| 0 | Idle (대기) |
| 1 | Running (실행 중) |
| 2 | Done (완료) |
| 3 | Error (오류) |

### 3.4 명령 코드 (레지스터 351)

| 코드 | 명령 | 설명 | 좌표계 |
|------|------|------|--------|
| **1** | Go Home | var.p(100) 위치로 이동 | - |
| **2** | (Reserved) | - | - |
| **10** | tool.transx | Tool X축 상대 이동 | Tool |
| **11** | tool.transy | Tool Y축 상대 이동 | Tool |
| **12** | tool.transz | Tool Z축 상대 이동 | Tool |
| **13** | tool.trans | Tool XYZ 동시 상대 이동 | Tool |
| **20** | movel (절대) | 6DOF 절대 좌표 이동 | Base |
| **30** | Gripper Open | 그리퍼 열기 | - |
| **31** | Gripper Close | 그리퍼 닫기 | - |
| **32** | Gripper Home | 그리퍼 홈 + 열기 | - |
| **40** | toolframe(0) | 툴프레임 0 선택 | - |
| **41** | toolframe(1) | 툴프레임 1 선택 | - |
| **42** | toolframe(2) | 툴프레임 2 선택 | - |
| **43** | toolframe(3) | 툴프레임 3 선택 | - |

### 3.5 좌표계 설명

| 명령어 | 좌표계 | 설명 |
|--------|--------|------|
| `tool.transx/y/z(mm)` | Tool 좌표계 | 로봇 자세에 따라 축 방향 변함 |
| `trans(x, y, z)` | Base 좌표계 | 항상 고정된 방향 |

### 3.6 Python 테스트 스크립트

#### 절대 좌표 이동 (command 20)
```bash
python3 /home/amap/Project/KAIST/Charging_Robot/scripts/test_relative_move.py --dx 10 --dy 0 --dz 0
```

#### Tool Z축 상대 이동 (command 12)
```bash
python3 /home/amap/Project/KAIST/Charging_Robot/scripts/test_tool_trans.py --dz 100
```

#### 회전 설정
```bash
python3 /home/amap/Project/KAIST/Charging_Robot/scripts/set_rz.py --rx 45 --ry 0 --rz 90
```

#### 툴프레임 변경 테스트
```bash
python3 /home/amap/Project/KAIST/Charging_Robot/scripts/test_toolframe.py
```

#### 상태 모니터링
```bash
python3 /home/amap/Project/KAIST/Charging_Robot/scripts/debug_status.py
```

### 3.7 Python 클라이언트 예시

```python
from pymodbus.client import ModbusTcpClient
import struct
import time

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT)
client.connect()

# 현재 위치 읽기 (float32)
def read_float32(regs, idx):
    high, low = regs[idx+1], regs[idx]
    return struct.unpack('>f', high.to_bytes(2,'big') + low.to_bytes(2,'big'))[0]

rr = client.read_holding_registers(158, 12)
x = read_float32(rr.registers, 0)
y = read_float32(rr.registers, 2)
z = read_float32(rr.registers, 4)

# int16 변환 (음수 처리)
def to_int16(val):
    return int(val + 65536) if val < 0 else int(val)

# 절대 좌표 이동 (command 20)
regs = [
    to_int16(int(x * 10)),     # 301: X
    to_int16(int(y * 10)),     # 302: Y
    to_int16(int((z+10) * 10)), # 303: Z (+10mm)
    to_int16(int(45 * 10)),    # 304: Rx
    to_int16(int(0 * 10)),     # 305: Ry
    to_int16(int(90 * 10)),    # 306: Rz
]
client.write_registers(301, regs)
client.write_registers(351, [20])  # command 20

# 완료 대기 (Running → Done)
while True:
    time.sleep(0.1)
    rr = client.read_holding_registers(352, 1)
    if rr.registers[0] == 2:  # Done
        break
print("완료")
```

---

## 4. 기존 스크립트 목록

### 4.1 메인 작업 스크립트

#### Vision_task.prs
비전 시스템과 연동하여 로봇을 제어하는 핵심 스크립트입니다.

**주요 기능:**
- Modbus 통신으로 비전 시스템에서 좌표 수신
- 수신된 좌표로 로봇 이동
- 그리퍼 동작 제어

**코드 흐름:**
```lua
-- 1. Modbus 서버 초기화
modserv = modbus_server()
modserv.write_register('task_number', var.i(1))

-- 2. 비전 시스템 응답 대기
while move_count < 10 do
    move_signal = modserv.read_register('task_done')

    if move_signal == 1 then
        -- 좌표 읽기 및 이동
        x = modserv.read_register('x')
        y = modserv.read_register('y')
        z = modserv.read_register('z')
        Rx = modserv.read_register('Rx')
        Ry = modserv.read_register('Ry')
        Rz = modserv.read_register('Rz')

        -- 부호 변환 (uint16 → int16)
        if x > 32767 then x = x - 65537 end

        -- 이동 실행
        p1 = pose(x/10, y/10, z/10, Rx/10, Ry/10, Rz/10)
        movel(p1)

    elseif move_signal == 2 then
        -- 최종 위치 도달, 그리퍼 동작
        break
    end
end

-- 3. 그리퍼 제어
if var.i(3) == 1 then
    call('Gripper_grip')
elseif var.i(3) == 2 then
    call('Gripper_ungrip')
end
```

**변수 설명:**

| 변수 | 용도 | 값 |
|------|------|-----|
| `var.i(1)` | task_number | 1~4 |
| `var.i(2)` | toolframe 번호 | 2 또는 3 |
| `var.i(3)` | gripper_action | 1=grip, 2=ungrip |
| `sim_mode` | 시뮬레이션 모드 | 0=실제, 1=시뮬 |

---

### 4.2 충전기 Pick & Place 스크립트

#### Car_get.prs
차량에서 충전건을 분리하는 스크립트입니다.

```lua
toolframe(3)
movep(lp1)              -- 접근 위치
movep(lp7)              -- 대기 위치
var.i(1, 3)             -- task_number = 3
var.i(2, 3)             -- toolframe = 3
var.i(3, 1)             -- gripper = grip
call('Vision_task')     -- 비전 태스크 실행
toolframe(2)
movep(lp8)              -- 복귀 위치
```

| 항목 | 값 |
|------|-----|
| Command (351) | 3 |
| Toolframe | 3 |
| Gripper Action | grip |

---

#### Car_put.prs
차량 충전포트에 충전건을 삽입하는 스크립트입니다.

```lua
toolframe(2)
movep(lp1)
movep(lp2)
var.i(1, 4)             -- task_number = 4
var.i(2, 2)             -- toolframe = 2
var.i(3, 2)             -- gripper = ungrip
call('Vision_task')
toolframe(3)
movep(lp3)
```

| 항목 | 값 |
|------|-----|
| Command (351) | 4 |
| Toolframe | 2 |
| Gripper Action | ungrip |

---

#### Rilham_get.prs
충전 스테이션에서 충전건을 픽업하는 스크립트입니다.

```lua
toolframe(3)
movep(lp3)
movel(lp2)
var.i(1, 1)             -- task_number = 1
var.i(2, 3)             -- toolframe = 3
var.i(3, 1)             -- gripper = grip
call('Vision_task')
toolframe(2)
movel(lp5)
```

| 항목 | 값 |
|------|-----|
| Command (351) | 1 |
| Toolframe | 3 |
| Gripper Action | grip |

---

#### Rilham_put.prs
충전 스테이션에 충전건을 반납하는 스크립트입니다.

```lua
toolframe(2)
movep(lp1)
movel(lp4)
var.i(1, 2)             -- task_number = 2
var.i(2, 2)             -- toolframe = 2
var.i(3, 2)             -- gripper = ungrip
call('Vision_task')
toolframe(3)
movel(lp5)
```

| 항목 | 값 |
|------|-----|
| Command (351) | 2 |
| Toolframe | 2 |
| Gripper Action | ungrip |

---

### 4.3 그리퍼 제어 스크립트

#### Gripper_grip.prs
충전건을 잡는 동작입니다.

```lua
dio.set(1)
delay(20)
dio.set(4)
delay(20)
dio.reset(1)
dio.reset(4)
dio.in_wait(1,1)        -- DI 1 대기
delay(2300)
dio.set(9)
delay(2500)
dio.reset(9)
dio.set(7)
dio.reset(8)
```

**DIO 시퀀스:**
```
DO1 ON → DO4 ON → DO1,4 OFF → DI1 대기 → DO9 ON/OFF → DO7 ON, DO8 OFF
```

---

#### Gripper_ungrip.prs
충전건을 놓는 동작입니다.

```lua
dio.set(10)
delay(2500)
dio.reset(10)
dio.reset(7)
dio.set(8)
dio.set(2)
delay(20)
dio.set(4)
delay(20)
dio.reset(4)
dio.reset(2)
dio.in_wait(2, 1)       -- DI 2 대기
```

**DIO 시퀀스:**
```
DO10 ON/OFF → DO7 OFF, DO8 ON → DO2,4 ON/OFF → DI2 대기
```

---

#### Gripper_home.prs
그리퍼를 홈 위치로 복귀시킵니다.

```lua
dio.pulse(5, 1, 30)
dio.in_wait(8,1)
dio.set(6)
delay(20)
dio.pulse(3, 1, 30)
delay(500)
dio.in_wait(5,1)
call('Gripper_ungrip')
```

---

### 4.4 캘리브레이션/테스트 스크립트

#### S10_Cal.prs
로봇 캘리브레이션용 60개 포인트 순회 스크립트입니다.

```lua
motion.speed = 100
t = 11000               -- 11초 대기

movep(lp1)
delay(t)
movep(lp2)
delay(t)
...
movep(lp60)
delay(t)
```

---

#### S10_ISO_9283.prs
ISO 9283 정밀도 테스트 스크립트입니다.

```lua
motion.speed = 100
t1 = 11000

-- 초기 위치
p0 = joint(0, 0, 90, 90, 90, 0)
movep(p0)

-- 테스트 포인트 정의
p1 = pose(640, 160, 80, -90, 0, -90)
p2 = pose(640, -160, 80, -90, 0, -90)
p3 = pose(960, -160, 400, -90, 0, -90)
p4 = pose(960, 160, 400, -90, 0, -90)
p5 = pose(800, 0, 240, -90, 0, -90)

-- 30회 반복
for i = 1, 30 do
    movel(p1)
    delay(t1)
    movel(p2)
    delay(t1)
    movel(p3)
    delay(t1)
    movel(p4)
    delay(t1)
    movel(p5)
    delay(t1)
end
```

---

#### Set_home_position.prs
홈 위치 설정 스크립트입니다.

```lua
toolframe(3)
movel(lp2)
movel(lp3)
```

---

#### visiontest.prs
비전 좌표 수신 테스트용 스크립트입니다.

```lua
toolframe(1)
modserv = modbus_server()

-- 좌표 읽기
x = modserv.read_register('x')
y = modserv.read_register('y')
z = modserv.read_register('z')
Rx = modserv.read_register('Rx')
Ry = modserv.read_register('Ry')
Rz = modserv.read_register('Rz')

-- 부호 변환
if x > 32767 then x = x - 65537 end
...

-- 이동
toolframe(2)
motion.speed = 10
p1 = pose(x-150, y, z, Rx, Ry, Rz)
movel(p1)
```

---

## 5. 주요 함수 레퍼런스

| 함수 | 설명 | 예제 |
|------|------|------|
| `toolframe(n)` | 툴 좌표계 선택 | `toolframe(2)` |
| `movep(point)` | PTP 이동 | `movep(lp1)` |
| `movel(pose)` | 선형 이동 | `movel(p1)` |
| `pose(x,y,z,rx,ry,rz)` | 포즈 생성 | `pose(100,0,50,0,0,0)` |
| `joint(j1~j6)` | 조인트 각도 | `joint(0,0,90,90,90,0)` |
| `delay(ms)` | 대기 | `delay(1000)` |
| `call(script)` | 서브 스크립트 호출 | `call('Gripper_grip')` |
| `dio.set(n)` | DO 출력 ON | `dio.set(1)` |
| `dio.reset(n)` | DO 출력 OFF | `dio.reset(1)` |
| `dio.in_wait(n,v)` | DI 대기 | `dio.in_wait(1,1)` |
| `dio.pulse(n,v,ms)` | DO 펄스 출력 | `dio.pulse(5,1,30)` |
| `modbus_server()` | Modbus 서버 객체 | `modserv = modbus_server()` |
| `read_register(name)` | 레지스터 읽기 | `modserv.read_register('x')` |
| `write_register(name,v)` | 레지스터 쓰기 | `modserv.write_register('task_done',0)` |
| `tool.transz(mm)` | Z축 이동 | `tool.transz(30)` |
| `var.i(n)` | 정수 변수 읽기 | `var.i(1)` |
| `var.i(n, v)` | 정수 변수 쓰기 | `var.i(1, 3)` |
| `var.p(n)` | 포즈 변수 읽기 | `var.p(8)` |
| `var.p(n) = pose` | 포즈 변수 쓰기 | `var.p(100) = here()` |
| `here()` | 현재 TCP 포즈 반환 | `cur = here()` |
| `shift(pose, x,y,z,rx,ry,rz)` | 포즈 상대 이동 | `shift(here(), 10, 0, 0, 0, 0, 0)` |
| `read_float(addr)` | Float32 읽기 (2 regs) | `modserv.read_float(303)` |

---

## 6. 스크립트 비교: Vision_task vs Main_task

| 항목 | Vision_task.prs | Main_task.prs |
|------|-----------------|---------------|
| **위치** | KAIST/robot_scripts/ | Robot_scripts/robot_scripts/ |
| **통신 방식** | 비전 시스템이 좌표 전송 | PC가 명령 전송 |
| **데이터 타입** | int16 (×10 스케일) | float32 (직접 값) |
| **이동 모드** | 절대 좌표만 | 상대/절대 선택 |
| **명령 처리** | task_number 기반 | command 코드 기반 |
| **그리퍼 호출** | call('Gripper_xxx') | 내장 DIO 시퀀스 |
| **상태 피드백** | task_done (0,1,2) | status (0,1,2,3) |
| **폴링 주기** | 이벤트 기반 | 100ms 고정 |
