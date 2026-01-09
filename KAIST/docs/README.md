# Charging Robot System Documentation

충전 로봇 시스템의 기술 문서입니다.

## 문서 목차

| 문서 | 설명 |
|------|------|
| [시스템 개요](01_system_overview.md) | 전체 시스템 아키텍처 및 구성요소 |
| [로봇 스크립트 가이드](02_robot_scripts.md) | 로봇 컨트롤러 스크립트 상세 설명 |
| [Python 소스 가이드](03_python_source.md) | 비전 시스템 및 Modbus 통신 코드 |
| [통신 프로토콜](04_communication_protocol.md) | Modbus 레지스터 매핑 및 데이터 포맷 |
| [작업 사이클](05_operation_cycle.md) | 충전 작업 흐름 및 시퀀스 |

## 빠른 시작

### 시스템 요구사항
- Python 3.8+
- Intel RealSense 카메라
- 로봇 컨트롤러 (Modbus TCP 지원)

### 실행 방법
```bash
cd /home/amap/Project/Charging_Robot/KAIST/src
python main_main_refactored.py
```

## 디렉토리 구조

```
Charging_Robot/
├── KAIST/
│   ├── src/                    # Python 소스 코드
│   │   ├── main_main_refactored.py   # 메인 제어 루프
│   │   ├── modbus_robot_interface.py # Modbus 통신
│   │   ├── main_charuco_gun.py       # 충전건 비전
│   │   ├── main_charuco_port.py      # 충전포트 비전
│   │   └── keypoint_detector/        # 키포인트 검출 모듈
│   ├── robot_scripts/          # 기존 비전 기반 스크립트
│   │   ├── Vision_task.prs           # 비전 연동 메인 태스크
│   │   ├── Car_get.prs               # 차량에서 충전건 분리
│   │   ├── Car_put.prs               # 차량에 충전건 삽입
│   │   ├── Rilham_get.prs            # 스테이션에서 충전건 픽업
│   │   ├── Rilham_put.prs            # 스테이션에 충전건 반납
│   │   └── Gripper_*.prs             # 그리퍼 제어
│   └── docs/                   # 문서
│
└── Robot_scripts/
    └── robot_scripts/          # 신규 통합 제어 스크립트
        └── Main_task.prs             # PC 원격 제어 통합 스크립트
```

## 스크립트 비교

| 항목 | Vision_task (기존) | Main_task (신규) |
|------|-------------------|------------------|
| 위치 | KAIST/robot_scripts/ | Robot_scripts/robot_scripts/ |
| 용도 | 비전 시스템 연동 | PC 직접 제어 |
| 데이터 타입 | int16 (×10 스케일) | float32 |
| 통신 포트 | 1502 | 502 |
