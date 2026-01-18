# Symphony Series 로봇 스크립터 제어 함수 정리

> 프로그램매뉴얼 V1.6 기준 정리

---

## 1. 동작 스크립트 (Motion)

### 1.1 속도/가속도 설정

| 함수 | 설명 |
|------|------|
| `motion.speed` | 이동 속도 설정 |
| `motion.accel` | 가속도 설정 |
| `motion.decel` | 감속도 설정 |

### 1.2 위치 변수 초기화

| 함수 | 설명 |
|------|------|
| `joint` | 관절형 위치 변수 초기화 |
| `pose` | 직교형 위치 변수 초기화 |
| `j2p` | 위치 값 변환 (관절 → 직교) |
| `inv` | 포즈 변수의 역행렬을 직교 좌표계로 변환 |

### 1.3 이동 명령 (Base 좌표계 기준)

| 함수 | 설명 |
|------|------|
| `movep` | 보간이동 (PTP) |
| `movel` | 선형이동 |
| `movec` | 원호이동 |
| `moves` | 스플라인 곡선 이동 |
| `rmovep` | 상대 보간이동 |
| `rmovel` | 상대 선형이동 |

### 1.4 병진/회전 이동 (Base 좌표계 기준)

| 함수 | 설명 |
|------|------|
| `trans` | 병진이동 (X, Y, Z) |
| `transx` | X축 기준 병진이동 |
| `transy` | Y축 기준 병진이동 |
| `transz` | Z축 기준 병진이동 |
| `rot` | 회전이동 (Rx, Ry, Rz) |
| `rotx` | X축 기준 회전이동 |
| `roty` | Y축 기준 회전이동 |
| `rotz` | Z축 기준 회전이동 |
| `rotj1` | 관절 1축 기준 회전이동 |
| `rotj2` | 관절 2축 기준 회전이동 |
| `rotj3` | 관절 3축 기준 회전이동 |
| `rotj4` | 관절 4축 기준 회전이동 |
| `rotj5` | 관절 5축 기준 회전이동 |
| `rotj6` | 관절 6축 기준 회전이동 |

### 1.5 Tool 좌표계 기준 이동 ⭐

| 함수 | 설명 |
|------|------|
| `tool.trans` | Tool 좌표계 기준 병진이동 (X, Y, Z) |
| `tool.transx` | Tool 좌표계 X축 기준 병진이동 |
| `tool.transy` | Tool 좌표계 Y축 기준 병진이동 |
| `tool.transz` | Tool 좌표계 Z축 기준 병진이동 |
| `tool.rot` | Tool 좌표계 기준 회전이동 |
| `tool.rotx` | Tool 좌표계 X축 기준 회전이동 |
| `tool.roty` | Tool 좌표계 Y축 기준 회전이동 |
| `tool.rotz` | Tool 좌표계 Z축 기준 회전이동 |

### 1.6 좌표계 설정

| 함수 | 설명 |
|------|------|
| `workframe` | 워크 좌표계 번호 설정 |
| `toolframe` | 툴 좌표계 번호 설정 |

### 1.7 기타 동작

| 함수 | 설명 |
|------|------|
| `delay` | 대기시간 |
| `home` | 홈 위치로 이동 |

---

## 2. 힘/순응 제어 스크립트 (Force)

| 함수 | 설명 |
|------|------|
| `force.enable` | 힘 제어 활성화 |
| `force.disable` | 힘 제어 비활성화 |
| `force.set` | 힘 크기 및 방향 설정 |
| `force.reset` | 힘 제어 설정 초기화 |
| `force.bias` | 외력 참조 모드 |
| `force.set_compliance_x` | X 방향 순응 제어 설정 |
| `force.set_compliance_y` | Y 방향 순응 제어 설정 |
| `force.set_compliance_z` | Z 방향 순응 제어 설정 |
| `force.set_compliance_rx` | Rx 방향 순응 제어 설정 |
| `force.set_compliance_ry` | Ry 방향 순응 제어 설정 |
| `force.set_compliance_rz` | Rz 방향 순응 제어 설정 |
| `force.collision` | 충돌 감지 설정 |
| `force.set_collision` | 충돌 감지 감도 설정 |
| `force.get_collision` | 현재 충돌 감지 감도 값 읽기 |

---

## 3. 입/출력 스크립트 (I/O)

### 3.1 디지털 I/O

| 함수 | 설명 |
|------|------|
| `dio.set` | 디지털 출력 신호 활성화 (ON) |
| `dio.reset` | 디지털 출력 신호 비활성화 (OFF) |
| `dio.set_bits` | 다중 디지털 출력 신호 활성화 |
| `dio.reset_bits` | 다중 디지털 출력 신호 비활성화 |
| `dio.get` | 디지털 입력 값 가져오기 |
| `dio.get_bits` | 모든 디지털 입력 값 읽기 |
| `dio.in_wait` | 디지털 입력 신호 대기 |
| `dio.out_wait` | 디지털 출력 신호 대기 |
| `dio.pulse` | 디지털 단일 펄스 출력 |

### 3.2 아날로그 I/O

| 함수 | 설명 |
|------|------|
| `aio.set` | 아날로그 출력 신호 설정 |
| `aio.get` | 아날로그 출력 신호 가져오기 |

### 3.3 Tool I/O

| 함수 | 설명 |
|------|------|
| `tool.set` | 툴 출력 신호 활성화 (ON) |
| `tool.reset` | 툴 출력 신호 비활성화 (OFF) |
| `tool.get` | 툴 입력 값 가져오기 |
| `tool.in_wait` | 툴 입력 신호 대기 |
| `tool.out_wait` | 툴 출력 신호 대기 |
| `tool.voltage` | 툴 출력 전압 값 설정 |
| `tool.modbus_write_register` | Holding Register에 값 쓰기 |
| `tool.modbus_read_register` | Holding Register에 값 읽기 |

---

## 4. 통신 스크립트

### 4.1 TCP/IP 서버

| 함수 | 설명 |
|------|------|
| `serv = server` | 서버 선언 |
| `serv.open` | 서버 열기 |
| `serv.close` | 서버 종료 |
| `serv.state` | 클라이언트 접속 상태 확인 |
| `serv.flush` | 송/수신 버퍼 초기화 |
| `serv.read` | 데이터 수신 |
| `serv.write` | 데이터 송신 |
| `serv.set_label` | 송신 데이터 접미사 설정 |

### 4.2 TCP/IP 클라이언트

| 함수 | 설명 |
|------|------|
| `cli = client` | 클라이언트 선언 |
| `cli.open` | 서버와 연결 |
| `cli.close` | 서버와 연결 해제 |
| `cli.read` | 데이터 수신 |
| `cli.write` | 데이터 송신 |
| `cli.get_float` | 32비트 실수형 데이터 읽기 |
| `cli.get_double` | 64비트 실수형 데이터 읽기 |

### 4.3 Modbus 서버

| 함수 | 설명 |
|------|------|
| `modserv = modbus_server` | Modbus 서버 선언 |
| `modserv.read_bit` | Coil 값 읽기 |
| `modserv.read_bits` | 여러 개의 Coil 값 읽기 |
| `modserv.read_bit_multi` | 해당 이름의 모든 Coil 값 읽기 |
| `modserv.read_register` | Holding Register 값 읽기 |
| `modserv.read_registers` | 여러 개의 Holding Register 값 읽기 |
| `modserv.read_register_multi` | 해당 이름의 모든 Holding Register 값 읽기 |
| `modserv.write_bit` | Coil 값 쓰기 |
| `modserv.write_bits` | 여러 개의 Coil 값 쓰기 |
| `modserv.write_bit_multi` | 해당 이름의 모든 Coil에 값 쓰기 |
| `modserv.write_register` | Holding Register 값 쓰기 |
| `modserv.write_registers` | 여러 개의 Holding Register 값 쓰기 |
| `modserv.write_register_multi` | 해당 이름의 모든 Holding Register에 값 쓰기 |

### 4.4 Modbus 클라이언트

| 함수 | 설명 |
|------|------|
| `modcli = modbus_client()` | Modbus 클라이언트 선언 |
| `modcli.open` | Modbus 서버와 연결 |
| `modcli.close` | Modbus 서버와 연결 해제 |
| `modcli.read_bit` | Coil 값 읽기 |
| `modcli.read_bits` | 여러 개의 Coil 값 읽기 |
| `modcli.read_register` | Holding Register 값 읽기 |
| `modcli.read_registers` | 여러 개의 Holding Register 값 읽기 |
| `modcli.write_bit` | Coil에 값 쓰기 |
| `modcli.write_bits` | 여러 개의 Coil에 값 쓰기 |
| `modcli.write_register` | Holding Register에 값 쓰기 |
| `modcli.write_registers` | 여러 개의 Holding Register에 값 쓰기 |

### 4.5 시리얼 통신

| 함수 | 설명 |
|------|------|
| `seri = serial` | 시리얼 포트 선언 |
| `seri.open` | 시리얼 통신 연결 |
| `seri.close` | 시리얼 통신 종료 |
| `seri.state` | 시리얼 통신 포트 상태 확인 |
| `seri.read` | 데이터 수신 |
| `seri.write` | 데이터 송신 |
| `seri.set_byte_timeout` | 수신 데이터 문자간 타임아웃 설정 |
| `seri.set_label` | 송신 데이터 접미사 설정 |
| `seri.count_rx` | 수신 버퍼 데이터 개수 반환 |
| `seri.count_tx` | 송신 버퍼 데이터 개수 반환 |
| `seri.flush_rx` | 수신 버퍼 초기화 |
| `seri.flush_tx` | 송신 버퍼 초기화 |
| `seri.flush` | 송/수신 버퍼 초기화 |

---

## 5. 다중처리 스크립트 (Thread)

| 함수 | 설명 |
|------|------|
| `thread_name = thread` | 스레드 선언 |
| `thread_name.start` | 스레드 시작 |
| `thread_name.stop` | 스레드 정지 |
| `thread_name.pause` | 스레드 일시 정지 |
| `thread_name.resume` | 스레드 재시작 |
| `thread_name.state` | 스레드 상태 확인 |
| `lock` | 스레드 잠금 |
| `unlock` | 스레드 잠금 해제 |

---

## 6. 수학 함수 스크립트 (Math)

| 함수 | 설명 |
|------|------|
| `math.pi` | 파이 값 |
| `math.deg` | Degree 각도 변환 |
| `math.rad` | Radian 각도 변환 |
| `math.sqrt` | 제곱근 |
| `math.pow` | 거듭제곱 |
| `math.abs` | 절대 값 |
| `math.ceil` | 올림 |
| `math.floor` | 내림 |
| `math.sin` | 사인함수 |
| `math.cos` | 코사인 함수 |
| `math.tan` | 탄젠트 함수 |
| `math.asin` | 아크사인 함수 |
| `math.acos` | 아크코사인 함수 |
| `math.atan` | 아크탄젠트 함수 |
| `math.atan2` | 제2아크탄젠트 함수 |

---

## 7. 메시지 출력 스크립트 (Message)

| 함수 | 설명 |
|------|------|
| `msg.error` | 에러 메시지 출력 |
| `msg.warn` | 경고 메시지 출력 |
| `msg.info` | 정보 메시지 출력 |

---

## 8. 타이머 스크립트 (Timer)

| 함수 | 설명 |
|------|------|
| `timer = time` | 타이머 생성 |
| `timer.start` | 타이머 시작 |
| `timer.stop` | 타이머 정지 |
| `timer.elapsed` | 소요시간 계산 |

---

## 9. 프로그램 제어 스크립트

| 함수 | 설명 |
|------|------|
| `robot.pause` | 프로그램 일시정지 |
| `robot.stop` | 프로그램 정지 |

---

## 10. 전역변수 스크립트 (Variable)

| 함수 | 설명 |
|------|------|
| `var.p` | 위치 값 읽기/쓰기 |
| `var.i` | 정수 값 읽기/쓰기 |
| `var.f` | 실수 값 읽기/쓰기 |
| `var.b` | 논리형 읽기/쓰기 |
| `var.s` | 문자열 읽기/쓰기 |

---

## 11. 서브루틴 스크립트

| 함수 | 설명 |
|------|------|
| `call` | 서브루틴 호출 |

---

## 참고: 좌표계 차이점

| 좌표계 | 함수 예시 | 설명 |
|--------|----------|------|
| **Base 좌표계** | `trans`, `transx`, `rot` | 로봇 베이스 기준 고정 좌표계 |
| **Tool 좌표계** | `tool.trans`, `tool.transx`, `tool.rot` | 현재 TCP 기준 동적 좌표계 (로봇 자세에 따라 변함) |

### 좌표계 사용 시 주의사항

1. **Base 좌표계**: 로봇 베이스를 기준으로 한 고정 좌표계로, 로봇의 자세와 관계없이 항상 동일한 방향을 유지합니다.

2. **Tool 좌표계**: 현재 TCP(Tool Center Point)를 기준으로 한 좌표계로, 로봇의 자세에 따라 축 방향이 변합니다. TCP 기준으로 상대적인 이동이 필요할 때 사용합니다.

---

## 트리뷰 프로그램 명령어 그룹

### 일반 (Statement)
- I/O 그룹: `aio.get`, `aio.set`, `dio.get`, `dio.set`, `dio.reset` 등
- Force 그룹: `enable`, `disable`, `set`, `reset`, `bias` 등
- Motion 그룹: `trans`, `transx`, `rot`, `rotx` 등 (Base 좌표계)
- Tool 그룹: `tool.trans`, `tool.transx`, `tool.rot` 등 (Tool 좌표계)
- Modbus Client 그룹: `read_bit`, `read_register`, `write_bit` 등
- Modbus Server 그룹: `read_bit`, `read_register`, `write_bit` 등

### 제어
- `For`: 반복문
- `While`: 조건 반복문
- `If/Else`: 조건문
- `Break`: 반복 탈출
- `Return`: 함수 반환

### 고급
- `Function`: 함수 생성
- `Call`: 다른 프로그램 호출

### 플러그인
- 설정 메뉴에서 생성한 플러그인 인스턴스들

---

> **출처**: Symphony Series 프로그램매뉴얼 V1.6
