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
    REGISTER_POSE_MAIN = 301    # 301~306: pose_main (X,Y,Z,Rx,Ry,Rz)
    REGISTER_POSE_BACK = 307    # 307~312: pose_back (X,Y,Z,Rx,Ry,Rz)
    REGISTER_CMD = 351          # 커맨드 레지스터
    REGISTER_RESP = 352         # 응답 레지스터
    REGISTER_CAM_POSE = 158     # 158~169: 카메라 포즈 (12 registers, float32)

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

    # ==================== 커맨드/응답 ====================

    def read_command(self) -> Optional[int]:
        """커맨드 레지스터(351) 읽기"""
        result = self.read_registers(self.REGISTER_CMD, 1)
        return result[0] if result else None

    def write_command(self, value: int) -> bool:
        """커맨드 레지스터(351) 쓰기"""
        return self.write_register(self.REGISTER_CMD, value)

    def read_response(self) -> Optional[int]:
        """응답 레지스터(352) 읽기"""
        result = self.read_registers(self.REGISTER_RESP, 1)
        return result[0] if result else None

    def write_response(self, value: int) -> bool:
        """응답 레지스터(352) 쓰기"""
        return self.write_register(self.REGISTER_RESP, value)

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
