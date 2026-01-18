#!/usr/bin/env python3
"""
Modbus TCP Client - 로봇 컨트롤러 통신

Main_task.prs 레지스터 정의:
- 301~306: pose_main (x, y, z, Rx, Ry, Rz) - int16, mm×10/deg×10
- 307~312: pose_back (x2, y2, z2, Rx2, Ry2, Rz2) - int16, mm×10/deg×10
- 351: task_number (command)
- 352: task_done (status): 0=Idle, 1=Running, 2=Done, 3=Error
- 158~169: camera_pose (12 registers, float32)
"""

import struct
import time
from typing import Optional, Tuple, List
from pymodbus.client import ModbusTcpClient


class ModbusClient:
    """Modbus TCP 클라이언트"""

    # 레지스터 주소 (Main_task.prs와 일치)
    REGISTER_POSE_X = 301       # X (int16, mm×10)
    REGISTER_POSE_Y = 302       # Y (int16, mm×10)
    REGISTER_POSE_Z = 303       # Z (int16, mm×10)
    REGISTER_POSE_RX = 304      # Rx (int16, deg×10)
    REGISTER_POSE_RY = 305      # Ry (int16, deg×10)
    REGISTER_POSE_RZ = 306      # Rz (int16, deg×10)
    REGISTER_POSE_MAIN = 301    # 301~306: pose_main
    REGISTER_POSE_BACK = 307    # 307~312: pose_back
    REGISTER_COMMAND = 351      # 명령 코드 (task_number)
    REGISTER_STATUS = 352       # 상태 (task_done)
    REGISTER_CAM_POSE = 158     # 158~169: 카메라 포즈 (float32)
    REGISTER_TOOLFRAME = 219    # 현재 툴프레임 번호

    # 상태 코드
    STATUS_IDLE = 0
    STATUS_RUNNING = 1
    STATUS_DONE = 2
    STATUS_ERROR = 3

    # 명령 코드 (Main_task.prs와 일치)
    CMD_GO_HOME = 1             # movep(var.p(100))
    CMD_SET_HOME = 2            # (에러 반환)
    CMD_TCP_LINEAR_X = 10       # tool.transx(x) - 툴 좌표계 X 이동
    CMD_TCP_LINEAR_Y = 11       # tool.transy(y) - 툴 좌표계 Y 이동
    CMD_TCP_LINEAR_Z = 12       # tool.transz(z) - 툴 좌표계 Z 이동
    CMD_TCP_LINEAR_XYZ = 13     # tool.trans(x, y, z) - 툴 좌표계 XYZ 이동
    CMD_TCP_ROTATE_X = 14       # tool.rotx(rx) - 툴 좌표계 X축 회전
    CMD_TCP_ROTATE_Y = 15       # tool.roty(ry) - 툴 좌표계 Y축 회전
    CMD_TCP_ROTATE_Z = 16       # tool.rotz(rz) - 툴 좌표계 Z축 회전
    CMD_MOVE_TO_POSE = 20       # movel(pose) - 절대 좌표 이동
    CMD_GRIPPER_OPEN = 30       # 그리퍼 열기
    CMD_GRIPPER_CLOSE = 31      # 그리퍼 닫기
    CMD_GRIPPER_HOME = 32       # 그리퍼 홈
    CMD_TOOLFRAME_0 = 40        # toolframe(0)
    CMD_TOOLFRAME_1 = 41        # toolframe(1)
    CMD_TOOLFRAME_2 = 42        # toolframe(2)
    CMD_TOOLFRAME_3 = 43        # toolframe(3)
    CMD_WORKFRAME = 44          # workframe(0) - 워크프레임 설정
    CMD_BASEFRAME = 44          # alias (deprecated)
    CMD_BASE_LINEAR_X = 50      # transx(x) - 베이스 좌표계 X 이동
    CMD_BASE_LINEAR_Y = 51      # transy(y) - 베이스 좌표계 Y 이동
    CMD_BASE_LINEAR_Z = 52      # transz(z) - 베이스 좌표계 Z 이동
    CMD_BASE_LINEAR_XYZ = 53    # trans(x, y, z) - 베이스 좌표계 XYZ 이동
    CMD_BASE_ROTATE_X = 54      # rotx(rx) - 베이스 좌표계 X축 회전
    CMD_BASE_ROTATE_Y = 55      # roty(ry) - 베이스 좌표계 Y축 회전
    CMD_BASE_ROTATE_Z = 56      # rotz(rz) - 베이스 좌표계 Z축 회전

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
        """로봇에 연결"""
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
        """연결 해제"""
        if self._client:
            self._client.close()
            self._client = None
        self._connected = False
        return True, "연결 해제됨"

    # ==================== 레지스터 읽기/쓰기 ====================

    def read_registers(self, address: int, count: int) -> Optional[List[int]]:
        """홀딩 레지스터 읽기"""
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
        """홀딩 레지스터 쓰기"""
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
        """상태 레지스터(352) 읽기"""
        result = self.read_registers(self.REGISTER_STATUS, 1)
        return result[0] if result else None

    def read_command(self) -> Optional[int]:
        """커맨드 레지스터(351) 읽기"""
        result = self.read_registers(self.REGISTER_COMMAND, 1)
        return result[0] if result else None

    def write_command(self, value: int) -> bool:
        """커맨드 레지스터(351) 쓰기"""
        return self.write_register(self.REGISTER_COMMAND, value)

    def read_response(self) -> Optional[int]:
        """응답 레지스터(352) 읽기 - status와 동일"""
        return self.read_status()

    def read_current_toolframe(self) -> Optional[int]:
        """현재 툴프레임 번호(219) 읽기"""
        result = self.read_registers(self.REGISTER_TOOLFRAME, 1)
        return result[0] if result else None

    def write_response(self, response: int) -> bool:
        """응답 레지스터(352) 쓰기"""
        return self.write_register(self.REGISTER_STATUS, response)

    def wait_for_done(self, timeout: float = 30.0, process_events_callback=None) -> Tuple[bool, str]:
        """
        명령 완료 대기 (Running → Done/Idle 감지)

        Args:
            timeout: 타임아웃 (초)
            process_events_callback: UI 이벤트 처리 콜백 (예: QApplication.processEvents)
        """
        start = time.time()

        # 1단계: Running 상태 감지 대기 (최대 5초)
        # 빠른 명령(toolframe, workframe 등)은 즉시 완료되어 IDLE로 돌아갈 수 있음
        initial_status = self.read_status()
        while time.time() - start < 5.0:
            if process_events_callback:
                process_events_callback()

            status = self.read_status()
            if status == self.STATUS_RUNNING:
                break
            elif status == self.STATUS_DONE:
                return True, "명령 완료"
            elif status == self.STATUS_ERROR:
                return False, "로봇 오류 발생"
            elif status == self.STATUS_IDLE and time.time() - start > 0.3:
                # 빠른 명령이 이미 완료된 경우 (0.3초 후에도 IDLE이면 완료로 간주)
                return True, "명령 완료"

            time.sleep(0.1)
        else:
            return False, "Running 상태 감지 실패"

        # 2단계: 완료 대기 (Running → Done/Idle)
        while time.time() - start < timeout:
            if process_events_callback:
                process_events_callback()

            status = self.read_status()
            if status is None:
                time.sleep(0.02)
                continue

            if status == self.STATUS_DONE:
                return True, "명령 완료"
            elif status == self.STATUS_IDLE:
                return True, "명령 완료"
            elif status == self.STATUS_ERROR:
                return False, "로봇 오류 발생"

            time.sleep(0.1)

        return False, "타임아웃"

    # ==================== int16 변환 유틸리티 ====================

    @staticmethod
    def to_uint16(value: int) -> int:
        """int16 → uint16 변환 (음수 처리)"""
        if value < 0:
            return int(value + 65536)
        return int(value)

    @staticmethod
    def to_int16(value: int) -> int:
        """uint16 → int16 변환"""
        if value > 32767:
            return value - 65536
        return value

    # ==================== 모션 명령 ====================

    def send_go_home(self, wait: bool = True) -> Tuple[bool, str]:
        """Go Home 명령 (command 1)"""
        self.write_command(self.CMD_GO_HOME)
        if wait:
            return self.wait_for_done()
        return True, "명령 전송됨"

    def send_tcp_linear(self, axis: str, distance: float, wait: bool = True, process_events_callback=None) -> Tuple[bool, str]:
        """
        TCP 상대 이동 (툴 좌표계)

        Args:
            axis: 'x', 'y', 'z', 'xyz'
            distance: 이동 거리 (mm) 또는 (x, y, z) 튜플
            wait: 완료 대기 여부
            process_events_callback: UI 이벤트 처리 콜백

        Note:
            Main_task.prs의 tool.trans 명령은 mm 단위 그대로 사용 (×10 스케일링 없음)
        """
        if axis.lower() == 'x':
            # x 레지스터(301)에 값 쓰기 (mm 단위 정수)
            val = self.to_uint16(int(distance))
            self.write_register(self.REGISTER_POSE_X, val)
            self.write_command(self.CMD_TCP_LINEAR_X)
        elif axis.lower() == 'y':
            val = self.to_uint16(int(distance))
            self.write_register(self.REGISTER_POSE_Y, val)
            self.write_command(self.CMD_TCP_LINEAR_Y)
        elif axis.lower() == 'z':
            val = self.to_uint16(int(distance))
            self.write_register(self.REGISTER_POSE_Z, val)
            self.write_command(self.CMD_TCP_LINEAR_Z)
        elif axis.lower() == 'xyz' and isinstance(distance, (list, tuple)):
            x_val = self.to_uint16(int(distance[0]))
            y_val = self.to_uint16(int(distance[1]))
            z_val = self.to_uint16(int(distance[2]))
            self.write_registers(self.REGISTER_POSE_MAIN, [x_val, y_val, z_val])
            self.write_command(self.CMD_TCP_LINEAR_XYZ)
        else:
            return False, "잘못된 축 지정"

        if wait:
            return self.wait_for_done(process_events_callback=process_events_callback)
        return True, "명령 전송됨"

    def send_tcp_rotate(self, axis: str, angle: float, wait: bool = True, process_events_callback=None) -> Tuple[bool, str]:
        """
        TCP 상대 회전 (툴 좌표계)

        Args:
            axis: 'rx', 'ry', 'rz'
            angle: 회전 각도 (deg)
            wait: 완료 대기 여부
            process_events_callback: UI 이벤트 처리 콜백
        """
        # 회전 전 좌표 출력
        before_pose = self.read_current_pose()
        if before_pose:
            print(f"[TCP ROTATE] 회전 전: X={before_pose[0]:.2f}, Y={before_pose[1]:.2f}, Z={before_pose[2]:.2f}, "
                  f"Rx={before_pose[3]:.2f}, Ry={before_pose[4]:.2f}, Rz={before_pose[5]:.2f}")
        print(f"[TCP ROTATE] 명령: axis={axis}, angle={angle}deg")

        if axis.lower() == 'rx':
            val = self.to_uint16(int(angle))
            self.write_register(self.REGISTER_POSE_RX, val)
            print(f"[TCP ROTATE] 레지스터: Rx(304)={val}, CMD(351)={self.CMD_TCP_ROTATE_X}")
            self.write_command(self.CMD_TCP_ROTATE_X)
        elif axis.lower() == 'ry':
            val = self.to_uint16(int(angle))
            self.write_register(self.REGISTER_POSE_RY, val)
            print(f"[TCP ROTATE] 레지스터: Ry(305)={val}, CMD(351)={self.CMD_TCP_ROTATE_Y}")
            self.write_command(self.CMD_TCP_ROTATE_Y)
        elif axis.lower() == 'rz':
            val = self.to_uint16(int(angle))
            self.write_register(self.REGISTER_POSE_RZ, val)
            print(f"[TCP ROTATE] 레지스터: Rz(306)={val}, CMD(351)={self.CMD_TCP_ROTATE_Z}")
            self.write_command(self.CMD_TCP_ROTATE_Z)
        else:
            return False, "잘못된 축 지정 (rx/ry/rz)"

        if wait:
            result = self.wait_for_done(process_events_callback=process_events_callback)
            # 회전 후 좌표 출력
            after_pose = self.read_current_pose()
            if after_pose:
                print(f"[TCP ROTATE] 회전 후: X={after_pose[0]:.2f}, Y={after_pose[1]:.2f}, Z={after_pose[2]:.2f}, "
                      f"Rx={after_pose[3]:.2f}, Ry={after_pose[4]:.2f}, Rz={after_pose[5]:.2f}")
                if before_pose:
                    print(f"[TCP ROTATE] 변화량: dRx={after_pose[3]-before_pose[3]:.2f}, dRy={after_pose[4]-before_pose[4]:.2f}, "
                          f"dRz={after_pose[5]-before_pose[5]:.2f}")
            return result
        return True, "명령 전송됨"

    def send_move_to_pose(self, x: float, y: float, z: float,
                          rx: float, ry: float, rz: float, wait: bool = True,
                          process_events_callback=None) -> Tuple[bool, str]:
        """
        절대 좌표 이동 (command 20)

        Args:
            x, y, z: 위치 (mm)
            rx, ry, rz: 회전 (deg)
            wait: 완료 대기 여부
            process_events_callback: UI 이벤트 처리 콜백

        Note:
            Main_task.prs에서 movel(pose)로 이동 (베이스 좌표계 기준)
        """
        # mm×10, deg×10 스케일링 후 uint16 변환
        regs = [
            self.to_uint16(int(x * 10)),
            self.to_uint16(int(y * 10)),
            self.to_uint16(int(z * 10)),
            self.to_uint16(int(rx * 10)),
            self.to_uint16(int(ry * 10)),
            self.to_uint16(int(rz * 10)),
        ]

        # 레지스터 301~306에 쓰기
        self.write_registers(self.REGISTER_POSE_MAIN, regs)

        # command 20 전송
        self.write_command(self.CMD_MOVE_TO_POSE)

        if wait:
            return self.wait_for_done(process_events_callback=process_events_callback)
        return True, "명령 전송됨"

    def send_gripper(self, action: str, wait: bool = True) -> Tuple[bool, str]:
        """그리퍼 제어"""
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

    def send_set_toolframe(self, frame: int, wait: bool = True, process_events_callback=None) -> Tuple[bool, str]:
        """
        툴프레임 설정 (command 40-43)

        Args:
            frame: 툴프레임 번호 (0-3)
            wait: 완료 대기 여부
            process_events_callback: UI 이벤트 처리 콜백
        """
        if frame == 0:
            cmd = self.CMD_TOOLFRAME_0
        elif frame == 1:
            cmd = self.CMD_TOOLFRAME_1
        elif frame == 2:
            cmd = self.CMD_TOOLFRAME_2
        elif frame == 3:
            cmd = self.CMD_TOOLFRAME_3
        else:
            return False, f"잘못된 툴프레임 번호: {frame} (0-3만 가능)"

        self.write_command(cmd)
        if wait:
            return self.wait_for_done(process_events_callback=process_events_callback)
        return True, f"툴프레임 {frame} 설정"

    def send_set_workframe(self, wait: bool = True) -> Tuple[bool, str]:
        """
        워크프레임 0으로 설정 (command 44)
        workframe(0) 호출
        """
        self.write_command(self.CMD_BASEFRAME)
        if wait:
            return self.wait_for_done()
        return True, "워크프레임 설정"

    def send_base_linear(self, axis: str, distance: float, wait: bool = True,
                         absolute: bool = False, process_events_callback=None) -> Tuple[bool, str]:
        """
        베이스 좌표계 상대 이동 (transx, transy, transz, trans)

        Args:
            axis: 'x', 'y', 'z', 'xyz'
            distance: 이동 거리 (mm) 또는 (x, y, z) 튜플
            wait: 완료 대기 여부
            absolute: 미사용 (호환성 유지)
            process_events_callback: UI 이벤트 처리 콜백
        """
        # 이동 전 좌표 출력
        before_pose = self.read_current_pose()
        if before_pose:
            print(f"[BASE LINEAR] 이동 전: X={before_pose[0]:.2f}, Y={before_pose[1]:.2f}, Z={before_pose[2]:.2f}, "
                  f"Rx={before_pose[3]:.2f}, Ry={before_pose[4]:.2f}, Rz={before_pose[5]:.2f}")
        print(f"[BASE LINEAR] 명령: axis={axis}, distance={distance}mm")

        if axis.lower() == 'x':
            val = self.to_uint16(int(distance))
            self.write_register(self.REGISTER_POSE_X, val)
            print(f"[BASE LINEAR] 레지스터: X(301)={val}, CMD(351)={self.CMD_BASE_LINEAR_X}")
            self.write_command(self.CMD_BASE_LINEAR_X)
        elif axis.lower() == 'y':
            val = self.to_uint16(int(distance))
            self.write_register(self.REGISTER_POSE_Y, val)
            print(f"[BASE LINEAR] 레지스터: Y(302)={val}, CMD(351)={self.CMD_BASE_LINEAR_Y}")
            self.write_command(self.CMD_BASE_LINEAR_Y)
        elif axis.lower() == 'z':
            val = self.to_uint16(int(distance))
            self.write_register(self.REGISTER_POSE_Z, val)
            print(f"[BASE LINEAR] 레지스터: Z(303)={val}, CMD(351)={self.CMD_BASE_LINEAR_Z}")
            self.write_command(self.CMD_BASE_LINEAR_Z)
        elif axis.lower() == 'xyz' and isinstance(distance, (list, tuple)):
            x_val = self.to_uint16(int(distance[0]))
            y_val = self.to_uint16(int(distance[1]))
            z_val = self.to_uint16(int(distance[2]))
            self.write_registers(self.REGISTER_POSE_MAIN, [x_val, y_val, z_val])
            self.write_command(self.CMD_BASE_LINEAR_XYZ)
        else:
            return False, "잘못된 축 지정"

        if wait:
            result = self.wait_for_done(process_events_callback=process_events_callback)
            # 이동 후 좌표 출력
            after_pose = self.read_current_pose()
            if after_pose:
                print(f"[BASE LINEAR] 이동 후: X={after_pose[0]:.2f}, Y={after_pose[1]:.2f}, Z={after_pose[2]:.2f}, "
                      f"Rx={after_pose[3]:.2f}, Ry={after_pose[4]:.2f}, Rz={after_pose[5]:.2f}")
                if before_pose:
                    print(f"[BASE LINEAR] 변화량: dX={after_pose[0]-before_pose[0]:.2f}, dY={after_pose[1]-before_pose[1]:.2f}, "
                          f"dZ={after_pose[2]-before_pose[2]:.2f}")
            return result
        return True, "명령 전송됨"

    def send_base_rotate(self, axis: str, angle: float, wait: bool = True,
                         process_events_callback=None) -> Tuple[bool, str]:
        """
        베이스 좌표계 회전 (rotx, roty, rotz)

        Args:
            axis: 'rx', 'ry', 'rz'
            angle: 회전 각도 (deg)
            wait: 완료 대기 여부
            process_events_callback: UI 이벤트 처리 콜백
        """
        # 회전 전 좌표 출력
        before_pose = self.read_current_pose()
        if before_pose:
            print(f"[BASE ROTATE] 회전 전: X={before_pose[0]:.2f}, Y={before_pose[1]:.2f}, Z={before_pose[2]:.2f}, "
                  f"Rx={before_pose[3]:.2f}, Ry={before_pose[4]:.2f}, Rz={before_pose[5]:.2f}")
        print(f"[BASE ROTATE] 명령: axis={axis}, angle={angle}deg")

        if axis.lower() == 'rx':
            val = self.to_uint16(int(angle))
            self.write_register(self.REGISTER_POSE_RX, val)
            print(f"[BASE ROTATE] 레지스터: Rx(304)={val}, CMD(351)={self.CMD_BASE_ROTATE_X}")
            self.write_command(self.CMD_BASE_ROTATE_X)
        elif axis.lower() == 'ry':
            val = self.to_uint16(int(angle))
            self.write_register(self.REGISTER_POSE_RY, val)
            print(f"[BASE ROTATE] 레지스터: Ry(305)={val}, CMD(351)={self.CMD_BASE_ROTATE_Y}")
            self.write_command(self.CMD_BASE_ROTATE_Y)
        elif axis.lower() == 'rz':
            val = self.to_uint16(int(angle))
            self.write_register(self.REGISTER_POSE_RZ, val)
            print(f"[BASE ROTATE] 레지스터: Rz(306)={val}, CMD(351)={self.CMD_BASE_ROTATE_Z}")
            self.write_command(self.CMD_BASE_ROTATE_Z)
        else:
            return False, "잘못된 축 지정 (rx/ry/rz)"

        if wait:
            result = self.wait_for_done(process_events_callback=process_events_callback)
            # 회전 후 좌표 출력
            after_pose = self.read_current_pose()
            if after_pose:
                print(f"[BASE ROTATE] 회전 후: X={after_pose[0]:.2f}, Y={after_pose[1]:.2f}, Z={after_pose[2]:.2f}, "
                      f"Rx={after_pose[3]:.2f}, Ry={after_pose[4]:.2f}, Rz={after_pose[5]:.2f}")
                if before_pose:
                    print(f"[BASE ROTATE] 변화량: dRx={after_pose[3]-before_pose[3]:.2f}, dRy={after_pose[4]-before_pose[4]:.2f}, "
                          f"dRz={after_pose[5]-before_pose[5]:.2f}")
            return result
        return True, "명령 전송됨"

    # alias
    send_set_baseframe = send_set_workframe
    send_reset_base = send_set_workframe

    # ==================== 포즈 읽기/쓰기 ====================

    @staticmethod
    def _read_float32(registers, idx: int) -> float:
        """레지스터에서 float32 읽기 (Little Endian word order)"""
        high = registers[idx + 1]
        low = registers[idx]
        byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
        return struct.unpack('>f', byte_data)[0]

    def read_current_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        현재 로봇 위치 읽기 (레지스터 158~169, float32)

        Returns:
            (X, Y, Z, Rx, Ry, Rz) mm/deg 또는 None
        """
        result = self.read_registers(self.REGISTER_CAM_POSE, 12)
        if not result:
            return None

        x = self._read_float32(result, 0)
        y = self._read_float32(result, 2)
        z = self._read_float32(result, 4)
        rx = self._read_float32(result, 6)
        ry = self._read_float32(result, 8)
        rz = self._read_float32(result, 10)

        return (x, y, z, rx, ry, rz)

    def read_pose_main(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        Pose Main (301~306) 읽기 - 명령용 레지스터

        Returns:
            (X, Y, Z, Rx, Ry, Rz) mm/deg 또는 None
        """
        result = self.read_registers(self.REGISTER_POSE_MAIN, 6)
        if not result:
            return None

        # int16 → float 변환
        x = self.to_int16(result[0]) / 10.0
        y = self.to_int16(result[1]) / 10.0
        z = self.to_int16(result[2]) / 10.0
        rx = self.to_int16(result[3]) / 10.0
        ry = self.to_int16(result[4]) / 10.0
        rz = self.to_int16(result[5]) / 10.0

        return (x, y, z, rx, ry, rz)

    def read_pose_back(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """Pose Back (307~312) 읽기"""
        result = self.read_registers(self.REGISTER_POSE_BACK, 6)
        if not result:
            return None

        x = self.to_int16(result[0]) / 10.0
        y = self.to_int16(result[1]) / 10.0
        z = self.to_int16(result[2]) / 10.0
        rx = self.to_int16(result[3]) / 10.0
        ry = self.to_int16(result[4]) / 10.0
        rz = self.to_int16(result[5]) / 10.0

        return (x, y, z, rx, ry, rz)

    def write_pose_main(self, x: float, y: float, z: float,
                        rx: float, ry: float, rz: float) -> bool:
        """Pose Main (301~306) 쓰기"""
        regs = [
            self.to_uint16(int(x * 10)),
            self.to_uint16(int(y * 10)),
            self.to_uint16(int(z * 10)),
            self.to_uint16(int(rx * 10)),
            self.to_uint16(int(ry * 10)),
            self.to_uint16(int(rz * 10)),
        ]
        return self.write_registers(self.REGISTER_POSE_MAIN, regs)

    def write_pose_back(self, x: float, y: float, z: float,
                        rx: float, ry: float, rz: float) -> bool:
        """Pose Back (307~312) 쓰기"""
        regs = [
            self.to_uint16(int(x * 10)),
            self.to_uint16(int(y * 10)),
            self.to_uint16(int(z * 10)),
            self.to_uint16(int(rx * 10)),
            self.to_uint16(int(ry * 10)),
            self.to_uint16(int(rz * 10)),
        ]
        return self.write_registers(self.REGISTER_POSE_BACK, regs)

    def read_camera_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        카메라 포즈 (158~169) 읽기 - float32 형식

        Returns:
            (X, Y, Z, Rx, Ry, Rz) mm/deg 또는 None
        """
        result = self.read_registers(self.REGISTER_CAM_POSE, 12)
        if not result:
            return None

        def regs_to_float(r1: int, r2: int) -> float:
            byte_data = r2.to_bytes(2, 'big') + r1.to_bytes(2, 'big')
            return struct.unpack('>f', byte_data)[0]

        x = regs_to_float(result[0], result[1])
        y = regs_to_float(result[2], result[3])
        z = regs_to_float(result[4], result[5])
        rx = regs_to_float(result[6], result[7])
        ry = regs_to_float(result[8], result[9])
        rz = regs_to_float(result[10], result[11])

        return (x, y, z, rx, ry, rz)

    # ==================== 컨텍스트 매니저 ====================

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
