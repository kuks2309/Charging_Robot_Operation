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

    # 연결 검증 설정
    VERIFICATION_TIMEOUT = 2.0  # 검증 타임아웃 (초)
    VERIFICATION_RETRIES = 3    # 최대 재시도 횟수
    VERIFICATION_DELAY = 0.5    # 재시도 간 대기 시간 (초)

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
    CMD_TCP_ROTATE_RXRYRZ = 17  # tool.rotx/roty/rotz - 툴 좌표계 RxRyRz 회전
    CMD_MOVE_TO_POSE = 20       # movel(pose) - 절대 좌표 이동
    CMD_GRIPPER_OPEN = 30       # 그리퍼 열기
    CMD_GRIPPER_CLOSE = 31      # 그리퍼 닫기
    CMD_GRIPPER_HOME = 32       # 그리퍼 홈
    CMD_TOOLFRAME_0 = 40        # toolframe(0) - DEPRECATED, Eye-in-Hand 전용
    CMD_TOOLFRAME_1 = 41        # toolframe(1) - 비전/캘리브레이션
    CMD_TOOLFRAME_2 = 42        # toolframe(2) - 충전 작업
    CMD_TOOLFRAME_3 = 43        # toolframe(3) - 충전 작업
    CMD_WORKFRAME = 44          # workframe(0) - 워크프레임 설정
    CMD_TOOLFRAME_4 = 45        # toolframe(4) - 추가 툴프레임
    CMD_TOOLFRAME_5 = 46        # toolframe(5) - Hand-Eye Calibration
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
        """로봇에 연결 및 통신 검증"""
        try:
            # Step 1: Modbus TCP 클라이언트 생성
            self._client = ModbusTcpClient(
                host=self.ip,
                port=self.port,
                timeout=self.VERIFICATION_TIMEOUT  # 검증용 타임아웃 사용
            )

            # Step 2: TCP 연결
            print(f"[연결] TCP 연결 시도: {self.ip}:{self.port}")
            if not self._client.connect():
                self._connected = False
                self._client = None
                return False, f"TCP 연결 실패: {self.ip}:{self.port}"

            print(f"[연결] TCP 연결 성공")

            # Step 3: 통신 검증 (레지스터 읽기)
            verified, verify_msg = self._verify_communication()

            if verified:
                # 검증 성공 -> 연결 완료
                self._connected = True
                # 타임아웃을 원래 값으로 복원 (향후 operation용)
                self._client.timeout = self.timeout
                return True, f"연결 성공: {self.ip}:{self.port}"
            else:
                # 검증 실패 -> 연결 정리
                print(f"[연결] 통신 검증 실패, 연결 해제")
                self._client.close()
                self._client = None
                self._connected = False
                return False, verify_msg

        except Exception as e:
            # 예외 발생 시 정리
            self._connected = False
            if self._client:
                try:
                    self._client.close()
                except:
                    pass
                self._client = None
            return False, f"연결 오류: {e}"

    def _verify_communication(self) -> Tuple[bool, str]:
        """
        연결 후 통신 검증: Camera Pose 레지스터 읽기

        Returns:
            (success: bool, message: str)
        """
        import time

        print(f"[통신 검증] 시작 (최대 {self.VERIFICATION_RETRIES}회 시도)")

        for attempt in range(self.VERIFICATION_RETRIES):
            try:
                print(f"[통신 검증] 시도 {attempt + 1}/{self.VERIFICATION_RETRIES}")

                # Camera Pose 레지스터 읽기 (직접 호출, is_connected 체크 없음)
                result = self._client.read_holding_registers(
                    address=self.REGISTER_CAM_POSE,
                    count=12
                )

                # 성공 확인
                if result and not result.isError():
                    print(f"[통신 검증] 성공")
                    return True, "통신 검증 성공"
                else:
                    error_msg = str(result) if result else "응답 없음"
                    print(f"[통신 검증] 실패: {error_msg}")

            except Exception as e:
                print(f"[통신 검증] 예외: {e}")

            # 재시도 대기 (마지막 시도가 아닌 경우)
            if attempt < self.VERIFICATION_RETRIES - 1:
                print(f"[통신 검증] {self.VERIFICATION_DELAY}초 후 재시도...")
                time.sleep(self.VERIFICATION_DELAY)

                # UI 응답성 유지 (QApplication이 있는 경우)
                try:
                    from PyQt5.QtWidgets import QApplication
                    QApplication.processEvents()
                except:
                    pass

        # 모든 재시도 실패
        return False, f"통신 검증 실패: {self.VERIFICATION_RETRIES}회 시도 후 응답 없음"

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

    def wait_for_done(self, timeout: float = 30.0, process_events_callback=None, stop_flag_callback=None) -> Tuple[bool, str]:
        """
        명령 완료 대기 (Running → Done/Idle 감지)

        Args:
            timeout: 타임아웃 (초)
            process_events_callback: UI 이벤트 처리 콜백 (예: QApplication.processEvents)
            stop_flag_callback: 중지 확인 콜백 (True 반환 시 즉시 중지)
        """
        start = time.time()

        # 1단계: Running 상태 감지 대기 (최대 5초)
        # 빠른 명령(toolframe, workframe 등)은 즉시 완료되어 IDLE로 돌아갈 수 있음
        initial_status = self.read_status()
        while time.time() - start < 5.0:
            if process_events_callback:
                process_events_callback()

            # 중지 요청 확인
            if stop_flag_callback and stop_flag_callback():
                return False, "사용자 중지"

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

            # 중지 요청 확인
            if stop_flag_callback and stop_flag_callback():
                return False, "사용자 중지"

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

    def send_tcp_rotate(self, axis: str, angle, wait: bool = True, process_events_callback=None) -> Tuple[bool, str]:
        """
        TCP 상대 회전 (툴 좌표계)

        Args:
            axis: 'rx', 'ry', 'rz', 'rxryrz'
            angle: 회전 각도 (deg) 또는 (rx, ry, rz) 튜플
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
            val = self.to_uint16(int(round(angle * 10)))
            self.write_register(self.REGISTER_POSE_RX, val)
            print(f"[TCP ROTATE] Rx(304)={val} (×10→PRS÷10={angle:.1f}°), CMD={self.CMD_TCP_ROTATE_X}")
            self.write_command(self.CMD_TCP_ROTATE_X)
        elif axis.lower() == 'ry':
            val = self.to_uint16(int(round(angle * 10)))
            self.write_register(self.REGISTER_POSE_RY, val)
            print(f"[TCP ROTATE] Ry(305)={val} (×10→PRS÷10={angle:.1f}°), CMD={self.CMD_TCP_ROTATE_Y}")
            self.write_command(self.CMD_TCP_ROTATE_Y)
        elif axis.lower() == 'rz':
            val = self.to_uint16(int(round(angle * 10)))
            self.write_register(self.REGISTER_POSE_RZ, val)
            print(f"[TCP ROTATE] Rz(306)={val} (×10→PRS÷10={angle:.1f}°), CMD={self.CMD_TCP_ROTATE_Z}")
            self.write_command(self.CMD_TCP_ROTATE_Z)
        elif axis.lower() == 'rxryrz' and isinstance(angle, (list, tuple)):
            rx_val = self.to_uint16(int(round(angle[0] * 10)))
            ry_val = self.to_uint16(int(round(angle[1] * 10)))
            rz_val = self.to_uint16(int(round(angle[2] * 10)))
            self.write_registers(self.REGISTER_POSE_RX, [rx_val, ry_val, rz_val])
            print(f"[TCP ROTATE] Rx={rx_val},Ry={ry_val},Rz={rz_val} (×10→PRS÷10), CMD={self.CMD_TCP_ROTATE_RXRYRZ}")
            self.write_command(self.CMD_TCP_ROTATE_RXRYRZ)
        else:
            return False, "잘못된 축 지정 (rx/ry/rz/rxryrz) 또는 각도 형식"

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
                          process_events_callback=None, stop_flag_callback=None) -> Tuple[bool, str]:
        """
        절대 좌표 이동 (command 20)

        Args:
            x, y, z: 위치 (mm)
            rx, ry, rz: 회전 (deg)
            wait: 완료 대기 여부
            process_events_callback: UI 이벤트 처리 콜백
            stop_flag_callback: 중지 확인 콜백 (True 반환 시 즉시 중지)

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
            return self.wait_for_done(
                process_events_callback=process_events_callback,
                stop_flag_callback=stop_flag_callback
            )
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
        툴프레임 설정 (command 40-43, 45-46)

        Args:
            frame: 툴프레임 번호 (0-5, 0은 Eye-in-Hand 전용, 5는 Hand-Eye Calibration)
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
        elif frame == 4:
            cmd = self.CMD_TOOLFRAME_4
        elif frame == 5:
            cmd = self.CMD_TOOLFRAME_5
        else:
            return False, f"잘못된 툴프레임 번호: {frame} (0-5만 가능)"

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
        베이스 좌표계 Euler 각도 직접 변경 (movel 방식)

        현재 TCP pose를 읽고, 해당 Euler 축 값에 angle을 더한 뒤
        movel(CMD 20)로 절대 이동. Euler Rx/Ry/Rz 값이 직접 변경됨.

        Args:
            axis: 'rx', 'ry', 'rz'
            angle: 회전 각도 (deg)
            wait: 완료 대기 여부
            process_events_callback: UI 이벤트 처리 콜백
        """
        axis_index = {'rx': 3, 'ry': 4, 'rz': 5}
        if axis.lower() not in axis_index:
            return False, "잘못된 축 지정 (rx/ry/rz)"

        # 1. 현재 pose 읽기
        before_pose = self.read_current_pose()
        if not before_pose:
            return False, "현재 자세 읽기 실패"

        print(f"[BASE ROTATE] 회전 전: X={before_pose[0]:.2f}, Y={before_pose[1]:.2f}, Z={before_pose[2]:.2f}, "
              f"Rx={before_pose[3]:.2f}, Ry={before_pose[4]:.2f}, Rz={before_pose[5]:.2f}")

        # 2. 목표 pose 계산 (Euler 값 직접 수정)
        target = list(before_pose)
        idx = axis_index[axis.lower()]
        target[idx] += angle

        print(f"[BASE ROTATE] 명령: {axis.upper()} {'+' if angle > 0 else ''}{angle}° (movel 방식)")
        print(f"[BASE ROTATE] 목표: X={target[0]:.2f}, Y={target[1]:.2f}, Z={target[2]:.2f}, "
              f"Rx={target[3]:.2f}, Ry={target[4]:.2f}, Rz={target[5]:.2f}")

        # 3. movel 전송 (×10 스케일, CMD 20)
        regs = [
            self.to_uint16(int(round(target[0] * 10))),
            self.to_uint16(int(round(target[1] * 10))),
            self.to_uint16(int(round(target[2] * 10))),
            self.to_uint16(int(round(target[3] * 10))),
            self.to_uint16(int(round(target[4] * 10))),
            self.to_uint16(int(round(target[5] * 10))),
        ]
        self.write_registers(self.REGISTER_POSE_MAIN, regs)
        self.write_command(self.CMD_MOVE_TO_POSE)

        if wait:
            result = self.wait_for_done(process_events_callback=process_events_callback)
            after_pose = self.read_current_pose()
            if after_pose:
                print(f"[BASE ROTATE] 회전 후: X={after_pose[0]:.2f}, Y={after_pose[1]:.2f}, Z={after_pose[2]:.2f}, "
                      f"Rx={after_pose[3]:.2f}, Ry={after_pose[4]:.2f}, Rz={after_pose[5]:.2f}")
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
