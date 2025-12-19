#!/usr/bin/env python3
"""
Robot Module - Modbus 통신 및 로봇 제어

구조:
    robot/
    ├── communication/      # 통신 (Modbus TCP)
    │   └── modbus_client.py
    ├── control/            # 제어 (이동 명령)
    │   └── robot_controller.py
    └── pose/               # 포즈 데이터 관리
        ├── pose_manager.py
        └── constants.py
"""

# Communication
from .communication import ModbusClient

# Pose
from .pose import PoseManager, Pose, SavedPose, PoseType, Register, Command, Response

# Control
from .control import RobotController, MoveResult

__all__ = [
    # Communication
    'ModbusClient',
    # Pose
    'PoseManager', 'Pose', 'SavedPose', 'PoseType', 'Register', 'Command', 'Response',
    # Control
    'RobotController', 'MoveResult',
]
