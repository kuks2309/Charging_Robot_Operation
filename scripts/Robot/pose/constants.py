#!/usr/bin/env python3
"""
Robot Constants - 레지스터 주소 및 상수 정의
"""

# Modbus 레지스터 주소
class Register:
    """Modbus 레지스터 주소"""
    POSE_MAIN = 301       # 301~306: pose_main (X,Y,Z,Rx,Ry,Rz)
    POSE_BACK = 307       # 307~312: pose_back (X,Y,Z,Rx,Ry,Rz)
    CMD = 351             # 커맨드 레지스터 (로봇 → PC)
    RESP = 352            # 응답 레지스터 (PC → 로봇)
    CAM_POSE = 158        # 158~169: 카메라 포즈 (12 registers, float32)


# 커맨드 값 정의
class Command:
    """로봇 커맨드 (레지스터 351)"""
    IDLE = 0
    GET_GUN_RILHAM = 1      # 릴함에서 충전건 가져오기
    PUT_GUN_RILHAM = 2      # 릴함에 충전건 넣기
    GET_GUN_CAR = 3         # 차량에서 충전건 가져오기
    PUT_PORT_CAR = 4        # 차량 충전포트에 삽입


# 응답 값 정의
class Response:
    """PC 응답 (레지스터 352)"""
    FAILED = 0              # 실패
    USE_POSE_BACK = 1       # pose_back (301~306) 위치로 이동
    USE_POSE_MAIN = 2       # pose_main (307~312) 위치로 이동


# 위치 타입
class PoseType:
    """저장할 위치 타입"""
    HOME = "home"
    AR_TAG = "ar_tag"
    CHARGING_GUN = "charging_gun"
    CHARGING_PORT = "charging_port"
    APPROACH = "approach"
    CUSTOM = "custom"
