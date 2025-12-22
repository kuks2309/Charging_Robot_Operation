#!/usr/bin/env python3
"""
Modbus TCP Client - 로봇 컨트롤러 통신
"""

import struct
from typing import Optional, Tuple, List
from pymodbus.client import ModbusTcpClient


class ModbusClient:
    """Modbus TCP 클라이언트"""

    # 레지스터 주소
    REGISTER_COMMAND = 301      # 명령 코드
    REGISTER_MODE = 302         # 모드 (0=상대, 1=절대)
    REGISTER_PARAM_X = 303      # X값 (float, 2 registers)
    REGISTER_PARAM_Y = 305      # Y값 (float, 2 registers)
    REGISTER_PARAM_Z = 307      # Z값 (float, 2 registers)
    REGISTER_PARAM_RX = 309     # Rx값 (float, 2 registers)
    REGISTER_PARAM_RY = 311     # Ry값 (float, 2 registers)
    REGISTER_PARAM_RZ = 313     # Rz값 (float, 2 registers)
    REGISTER_STATUS = 315       # 상태 (0=Idle, 1=Running, 2=Done, 3=Error)
    REGISTER_CAM_POSE = 158     # 158~169: 카메라 포즈 (12 registers, float32)

    # 상태 코드
    STATUS_IDLE = 0
    STATUS_RUNNING = 1
    STATUS_DONE = 2
    STATUS_ERROR = 3

    # 명령 코드
    CMD_GO_HOME = 1
    CMD_SET_HOME = 2
    CMD_TCP_LINEAR_X = 10
    CMD_TCP_LINEAR_Y = 11
    CMD_TCP_LINEAR_Z = 12
    CMD_TCP_LINEAR_XYZ = 13
    CMD_TCP_ROTATE_RX = 14
    CMD_TCP_ROTATE_RY = 15
    CMD_TCP_ROTATE_RZ = 16
    CMD_TCP_ROTATE_RXRYRZ = 17
    CMD_MOVE_TO_POSE = 20
    CMD_GRIPPER_OPEN = 30
    CMD_GRIPPER_CLOSE = 31
    CMD_GRIPPER_HOME = 32

    def __init__(self, ip: str = "192.168.0.29", port: int = 1502, timeout: float = 1.0):
        """
        Args:
            ip: 로봇 컨트롤러 IP
            port: Modbus TCP 포트
            timeout: 연결 타임아웃 (초)
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self._client: Optional[ModbusTcpClient] = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        """연결 상태"""
        return self._connected and self._client is not None

    def connect(self) -> Tuple[bool, str]:
        """
        로봇에 연결

        Returns:
            (성공 여부, 메시지)
        """
        try:
            self._client = ModbusTcpClient(
                host=self.ip,
                port=self.port,
                timeout=self.timeout
            )
            self._connected = self._client.connect()

            if self._connected:
                return True, f"연결 성공: {self.ip}:{self.port}"
            else:
                return False, f"연결 실패: {self.ip}:{self.port}"

        except Exception as e:
            self._connected = False
            return False, f"연결 오류: {e}"

    def disconnect(self) -> Tuple[bool, str]:
        """
        연결 해제

        Returns:
            (성공 여부, 메시지)
        """
        if self._client:
            self._client.close()
            self._client = None
        self._connected = False
        return True, "연결 해제됨"

    # ==================== 레지스터 읽기/쓰기 ====================

    def read_registers(self, address: int, count: int) -> Optional[List[int]]:
        """
        홀딩 레지스터 읽기

        Args:
            address: 시작 주소
            count: 읽을 레지스터 수

        Returns:
            레지스터 값 리스트 또는 None
        """
        if not self.is_connected:
            return None

        try:
            result = self._client.read_holding_registers(address=address, count=count)
            if result.isError():
                return None
            return list(result.registers)
        except Exception:
            return None

    def write_registers(self, address: int, values: List[int]) -> bool:
        """
        홀딩 레지스터 쓰기

        Args:
            address: 시작 주소
            values: 쓸 값 리스트

        Returns:
            성공 여부
        """
        if not self.is_connected:
            return False

        try:
            result = self._client.write_registers(address=address, values=values)
            return not result.isError()
        except Exception:
            return False

    def write_register(self, address: int, value: int) -> bool:
        """단일 레지스터 쓰기"""
        return self.write_registers(address, [value])

    # ==================== 상태/명령 ====================

    def read_status(self) -> Optional[int]:
        """상태 레지스터(315) 읽기"""
        result = self.read_registers(self.REGISTER_STATUS, 1)
        return result[0] if result else None

    def read_command(self) -> Optional[int]:
        """커맨드 레지스터(301) 읽기"""
        result = self.read_registers(self.REGISTER_COMMAND, 1)
        return result[0] if result else None

    def write_command(self, value: int) -> bool:
        """커맨드 레지스터(301) 쓰기"""
        return self.write_register(self.REGISTER_COMMAND, value)

    def write_float(self, address: int, value: float) -> bool:
        """float 값을 2개 레지스터에 쓰기 (big-endian)"""
        byte_data = struct.pack('>f', value)
        r2 = int.from_bytes(byte_data[0:2], 'big')
        r1 = int.from_bytes(byte_data[2:4], 'big')
        return self.write_registers(address, [r1, r2])

    def wait_for_done(self, timeout: float = 30.0) -> Tuple[bool, str]:
        """
        명령 완료 대기

        Args:
            timeout: 타임아웃 (초)

        Returns:
            (성공 여부, 메시지)
        """
        import time
        start = time.time()
        while time.time() - start < timeout:
            status = self.read_status()
            if status == self.STATUS_DONE:
                return True, "명령 완료"
            elif status == self.STATUS_ERROR:
                return False, "로봇 오류 발생"
            elif status == self.STATUS_IDLE:
                return True, "대기 상태"
            time.sleep(0.1)
        return False, "타임아웃"

    # ==================== 모션 명령 ====================

    def send_go_home(self, wait: bool = True) -> Tuple[bool, str]:
        """Go Home 명령"""
        self.write_command(self.CMD_GO_HOME)
        if wait:
            return self.wait_for_done()
        return True, "명령 전송됨"

    def send_set_home(self) -> Tuple[bool, str]:
        """Set Home 명령 (현재 위치 저장)"""
        self.write_command(self.CMD_SET_HOME)
        return self.wait_for_done()

    def send_tcp_linear(self, axis: str, distance: float, absolute: bool = False, wait: bool = True) -> Tuple[bool, str]:
        """
        TCP Linear 이동

        Args:
            axis: 'x', 'y', 'z', 'xyz'
            distance: 이동 거리 (mm) 또는 (x, y, z) 튜플
            absolute: True=절대, False=상대
            wait: 완료 대기 여부
        """
        mode = 1 if absolute else 0
        self.write_register(self.REGISTER_MODE, mode)

        if axis.lower() == 'x':
            self.write_float(self.REGISTER_PARAM_X, distance)
            self.write_command(self.CMD_TCP_LINEAR_X)
        elif axis.lower() == 'y':
            self.write_float(self.REGISTER_PARAM_X, distance)
            self.write_command(self.CMD_TCP_LINEAR_Y)
        elif axis.lower() == 'z':
            self.write_float(self.REGISTER_PARAM_X, distance)
            self.write_command(self.CMD_TCP_LINEAR_Z)
        elif axis.lower() == 'xyz' and isinstance(distance, (list, tuple)):
            self.write_float(self.REGISTER_PARAM_X, distance[0])
            self.write_float(self.REGISTER_PARAM_Y, distance[1])
            self.write_float(self.REGISTER_PARAM_Z, distance[2])
            self.write_command(self.CMD_TCP_LINEAR_XYZ)
        else:
            return False, "잘못된 축 지정"

        if wait:
            return self.wait_for_done()
        return True, "명령 전송됨"

    def send_tcp_rotate(self, axis: str, angle: float, absolute: bool = False, wait: bool = True) -> Tuple[bool, str]:
        """
        TCP Rotate 회전

        Args:
            axis: 'rx', 'ry', 'rz', 'rxryrz'
            angle: 회전 각도 (deg) 또는 (rx, ry, rz) 튜플
            absolute: True=절대, False=상대
            wait: 완료 대기 여부
        """
        mode = 1 if absolute else 0
        self.write_register(self.REGISTER_MODE, mode)

        if axis.lower() == 'rx':
            self.write_float(self.REGISTER_PARAM_X, angle)
            self.write_command(self.CMD_TCP_ROTATE_RX)
        elif axis.lower() == 'ry':
            self.write_float(self.REGISTER_PARAM_X, angle)
            self.write_command(self.CMD_TCP_ROTATE_RY)
        elif axis.lower() == 'rz':
            self.write_float(self.REGISTER_PARAM_X, angle)
            self.write_command(self.CMD_TCP_ROTATE_RZ)
        elif axis.lower() == 'rxryrz' and isinstance(angle, (list, tuple)):
            self.write_float(self.REGISTER_PARAM_X, angle[0])
            self.write_float(self.REGISTER_PARAM_Y, angle[1])
            self.write_float(self.REGISTER_PARAM_Z, angle[2])
            self.write_command(self.CMD_TCP_ROTATE_RXRYRZ)
        else:
            return False, "잘못된 축 지정"

        if wait:
            return self.wait_for_done()
        return True, "명령 전송됨"

    def send_move_to_pose(self, x: float, y: float, z: float,
                          rx: float, ry: float, rz: float, wait: bool = True) -> Tuple[bool, str]:
        """
        지정된 포즈로 이동

        Args:
            x, y, z: 위치 (mm)
            rx, ry, rz: 회전 (deg)
            wait: 완료 대기 여부
        """
        self.write_float(self.REGISTER_PARAM_X, x)
        self.write_float(self.REGISTER_PARAM_Y, y)
        self.write_float(self.REGISTER_PARAM_Z, z)
        self.write_float(self.REGISTER_PARAM_RX, rx)
        self.write_float(self.REGISTER_PARAM_RY, ry)
        self.write_float(self.REGISTER_PARAM_RZ, rz)
        self.write_command(self.CMD_MOVE_TO_POSE)

        if wait:
            return self.wait_for_done()
        return True, "명령 전송됨"

    def send_gripper(self, action: str, wait: bool = True) -> Tuple[bool, str]:
        """
        그리퍼 제어

        Args:
            action: 'open', 'close', 또는 'home'
            wait: 완료 대기 여부
        """
        if action.lower() == 'open':
            self.write_command(self.CMD_GRIPPER_OPEN)
        elif action.lower() == 'close':
            self.write_command(self.CMD_GRIPPER_CLOSE)
        elif action.lower() == 'home':
            self.write_command(self.CMD_GRIPPER_HOME)
        else:
            return False, "잘못된 action (open/close/home)"

        if wait:
            return self.wait_for_done()
        return True, "명령 전송됨"

    # ==================== 포즈 읽기/쓰기 ====================

    def read_pose_main(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        Pose Main (301~306) 읽기

        Returns:
            (X, Y, Z, Rx, Ry, Rz) 또는 None
        """
        result = self.read_registers(self.REGISTER_POSE_MAIN, 6)
        if not result:
            return None
        return self._registers_to_pose(result)

    def read_pose_back(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        Pose Back (307~312) 읽기

        Returns:
            (X, Y, Z, Rx, Ry, Rz) 또는 None
        """
        result = self.read_registers(self.REGISTER_POSE_BACK, 6)
        if not result:
            return None
        return self._registers_to_pose(result)

    def write_pose_main(self, x: float, y: float, z: float,
                        rx: float, ry: float, rz: float) -> bool:
        """Pose Main (301~306) 쓰기"""
        values = self._pose_to_registers(x, y, z, rx, ry, rz)
        return self.write_registers(self.REGISTER_POSE_MAIN, values)

    def write_pose_back(self, x: float, y: float, z: float,
                        rx: float, ry: float, rz: float) -> bool:
        """Pose Back (307~312) 쓰기"""
        values = self._pose_to_registers(x, y, z, rx, ry, rz)
        return self.write_registers(self.REGISTER_POSE_BACK, values)

    def write_pose_both(self,
                        main_pose: Tuple[float, float, float, float, float, float],
                        back_pose: Tuple[float, float, float, float, float, float]) -> bool:
        """
        Pose Main (301~306)과 Pose Back (307~312) 동시 쓰기

        KAIST 방식: back_pose(approach) → main_pose(target) 순서로 이동

        Args:
            main_pose: (x, y, z, rx, ry, rz) target 위치
            back_pose: (x, y, z, rx, ry, rz) approach 위치

        Returns:
            성공 여부
        """
        main_values = self._pose_to_registers(*main_pose)
        back_values = self._pose_to_registers(*back_pose)

        # pose_main (301~306) 쓰기
        if not self.write_registers(self.REGISTER_POSE_MAIN, main_values):
            return False

        # pose_back (307~312) 쓰기
        if not self.write_registers(self.REGISTER_POSE_BACK, back_values):
            return False

        return True

    def read_camera_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        카메라 포즈 (158~169) 읽기 - float32 형식

        Returns:
            (X, Y, Z, Rx, Ry, Rz) mm/deg 또는 None
        """
        result = self.read_registers(self.REGISTER_CAM_POSE, 12)
        if not result:
            return None
        return self._registers_to_float32_pose(result)

    # ==================== 데이터 변환 ====================

    @staticmethod
    def _pose_to_registers(x: float, y: float, z: float,
                           rx: float, ry: float, rz: float) -> List[int]:
        """
        포즈 → 레지스터 값 변환

        위치: mm * 10000 → int16 → uint16
        회전: deg * 10 → int16 → uint16 (-180~180 정규화)
        """
        def to_uint16(val: int) -> int:
            val = max(-32768, min(32767, val))
            return val + 65536 if val < 0 else val

        def normalize_angle(deg: float) -> float:
            return ((deg + 180) % 360) - 180

        x_val = int(round(x * 10000))
        y_val = int(round(y * 10000))
        z_val = int(round(z * 10000))
        rx_val = int(round(normalize_angle(rx) * 10))
        ry_val = int(round(normalize_angle(ry) * 10))
        rz_val = int(round(normalize_angle(rz) * 10))

        return [
            to_uint16(x_val), to_uint16(y_val), to_uint16(z_val),
            to_uint16(rx_val), to_uint16(ry_val), to_uint16(rz_val)
        ]

    @staticmethod
    def _registers_to_pose(regs: List[int]) -> Tuple[float, float, float, float, float, float]:
        """
        레지스터 값 → 포즈 변환

        uint16 → int16 → 위치: /10000 (mm), 회전: /10 (deg)
        """
        def to_int16(val: int) -> int:
            return val - 65536 if val > 32767 else val

        x = to_int16(regs[0]) / 10000.0
        y = to_int16(regs[1]) / 10000.0
        z = to_int16(regs[2]) / 10000.0
        rx = to_int16(regs[3]) / 10.0
        ry = to_int16(regs[4]) / 10.0
        rz = to_int16(regs[5]) / 10.0

        return (x, y, z, rx, ry, rz)

    @staticmethod
    def _registers_to_float32_pose(regs: List[int]) -> Tuple[float, float, float, float, float, float]:
        """
        12개 레지스터 → float32 포즈 변환 (카메라 포즈용)

        2개 레지스터 = 1개 float32 (big-endian)
        """
        def regs_to_float(r1: int, r2: int) -> float:
            byte_data = r2.to_bytes(2, 'big') + r1.to_bytes(2, 'big')
            return struct.unpack('>f', byte_data)[0]

        x = regs_to_float(regs[0], regs[1])
        y = regs_to_float(regs[2], regs[3])
        z = regs_to_float(regs[4], regs[5])
        rx = regs_to_float(regs[6], regs[7])
        ry = regs_to_float(regs[8], regs[9])
        rz = regs_to_float(regs[10], regs[11])

        return (x, y, z, rx, ry, rz)

    # ==================== 컨텍스트 매니저 ====================

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
