#!/usr/bin/env python3
"""
Robot TCP Controller Library
Control robot TCP position and orientation via Modbus communication
"""

from pymodbus.client import ModbusTcpClient
from typing import Optional, Tuple, List, Dict
import struct
import numpy as np
from scipy.spatial.transform import Rotation as R
import time


class RobotTCPController:
    """
    Robot TCP (Tool Center Point) Controller

    Controls robot movement and position via Modbus TCP protocol.
    Supports reading current TCP position and moving to target positions.
    """

    # Modbus Register Addresses (Based on Modbus Server Manual V1.01)
    # Robot State & Control (0-282)
    REGISTER_ROBOT_STATE = 0        # Robot state
    REGISTER_ROBOT_ERROR = 1        # Robot error code
    REGISTER_OPERATION_MODE = 75    # Operation mode
    REGISTER_PROGRAM_CONTROL = 206  # Program control
    REGISTER_SERVO_ON = 211         # Servo on/off

    # TCP Position (158-169) - IEEE 754 32-bit float, 2 registers per value
    REGISTER_TCP_X = 158            # TCP X position (mm)
    REGISTER_TCP_Y = 160            # TCP Y position (mm)
    REGISTER_TCP_Z = 162            # TCP Z position (mm)
    REGISTER_TCP_RX = 164           # TCP Rx rotation (deg)
    REGISTER_TCP_RY = 166           # TCP Ry rotation (deg)
    REGISTER_TCP_RZ = 168           # TCP Rz rotation (deg)

    # User Define Area (301-556)
    REGISTER_POSE_MAIN = 301        # Main pose registers (301-306)
    REGISTER_POSE_BACK = 307        # Back pose registers (307-312)
    REGISTER_CMD = 351              # Command register
    REGISTER_RESP = 352             # Response register

    # Command codes (Application-specific)
    CMD_GRAB_GUN_EMPTY = 1          # Grab gun from empty port
    CMD_EMPTY_PORT = 2              # Move to empty port
    CMD_GRAB_GUN_CAR = 3            # Grab gun from car
    CMD_CAR_CHARGING_PORT = 4       # Move to car charging port

    # Response codes (Application-specific)
    RESP_POSE_BACK = 1
    RESP_POSE_MAIN = 2

    def __init__(self, ip: str = "192.168.0.29", port: int = 1502, timeout: float = 0.1):
        """
        Initialize Robot TCP Controller

        Args:
            ip: Robot controller IP address
            port: Modbus TCP port
            timeout: Connection timeout in seconds
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.client: Optional[ModbusTcpClient] = None
        self._connected = False
        self._current_tcp = None  # Cache current TCP position

    # ==================== Connection Management ====================

    def connect(self) -> bool:
        """
        Establish connection to robot Modbus server

        Returns:
            bool: True if connected successfully, False otherwise
        """
        try:
            self.client = ModbusTcpClient(self.ip, port=self.port, timeout=self.timeout)
            self._connected = self.client.connect()

            if self._connected:
                print(f"✅ Robot connected at {self.ip}:{self.port}")
                # Read initial TCP position
                self.read_tcp_position()
            else:
                print(f"❌ Failed to connect to robot at {self.ip}:{self.port}")

            return self._connected

        except Exception as e:
            print(f"❌ Connection error: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """Close Modbus connection"""
        if self.client:
            self.client.close()
            self._connected = False
            print("🔌 Robot disconnected")

    def is_connected(self) -> bool:
        """Check if connection is active"""
        return self._connected and self.client is not None

    # ==================== TCP Position Operations ====================

    def read_tcp_position(self) -> Optional[Dict[str, float]]:
        """
        Read current TCP position from robot

        Returns:
            Dictionary with TCP position:
            {
                'x': x_mm, 'y': y_mm, 'z': z_mm,
                'rx': rx_deg, 'ry': ry_deg, 'rz': rz_deg
            }
            or None if failed
        """
        if not self.is_connected():
            print("⚠️  Not connected to robot")
            return None

        try:
            # Read TCP position from registers 158-169 (12 registers = 6 float32 values)
            # Based on Modbus Server Manual: TCP Position (X, Y, Z, Rx, Ry, Rz)
            rr = self.client.read_holding_registers(address=self.REGISTER_TCP_X, count=12)

            if rr.isError():
                print("⚠️  Failed to read TCP position")
                return None

            # Parse pose from Modbus registers (IEEE 754 32-bit float format)
            x, y, z, rx, ry, rz = self._modbus_to_pose(rr)

            self._current_tcp = {
                'x': x,
                'y': y,
                'z': z,
                'rx': rx,
                'ry': ry,
                'rz': rz
            }

            print(f"📍 TCP Position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, "
                  f"Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")

            return self._current_tcp

        except Exception as e:
            print(f"❌ Error reading TCP position: {e}")
            return None

    def get_current_tcp(self) -> Optional[Dict[str, float]]:
        """
        Get cached current TCP position (without reading from robot)

        Returns:
            Dictionary with last known TCP position or None
        """
        return self._current_tcp

    # ==================== TCP Movement Operations ====================

    def move_tcp_to(self, x: float, y: float, z: float,
                    rx: float = None, ry: float = None, rz: float = None,
                    use_main: bool = True) -> bool:
        """
        Move robot TCP to target position

        Args:
            x: Target X position in mm
            y: Target Y position in mm
            z: Target Z position in mm
            rx: Target Rx rotation in degrees (optional, uses current if None)
            ry: Target Ry rotation in degrees (optional, uses current if None)
            rz: Target Rz rotation in degrees (optional, uses current if None)
            use_main: If True, use pose_main (301-306), else pose_back (307-312)

        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            print("⚠️  Not connected to robot")
            return False

        # Use current rotation if not specified
        current = self.get_current_tcp()
        if current is None:
            current = self.read_tcp_position()
            if current is None:
                print("⚠️  Cannot read current TCP position")
                return False

        if rx is None:
            rx = current['rx']
        if ry is None:
            ry = current['ry']
        if rz is None:
            rz = current['rz']

        try:
            # Convert pose to Modbus format
            pose_data = self._pose_to_modbus_data(x, y, z, rx, ry, rz)

            # Write to appropriate register
            register = self.REGISTER_POSE_MAIN if use_main else self.REGISTER_POSE_BACK

            success = self.client.write_registers(register, pose_data)

            if success.isError():
                print(f"❌ Failed to write pose to register {register}")
                return False

            print(f"✅ TCP move command sent: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, "
                  f"Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")

            # Send response command
            resp_code = self.RESP_POSE_MAIN if use_main else self.RESP_POSE_BACK
            self.write_response(resp_code)

            return True

        except Exception as e:
            print(f"❌ Error moving TCP: {e}")
            return False

    def move_tcp_relative(self, dx: float = 0, dy: float = 0, dz: float = 0,
                         drx: float = 0, dry: float = 0, drz: float = 0,
                         use_main: bool = True) -> bool:
        """
        Move robot TCP relative to current position

        Args:
            dx, dy, dz: Relative position change in mm
            drx, dry, drz: Relative rotation change in degrees
            use_main: If True, use pose_main, else pose_back

        Returns:
            bool: True if command sent successfully, False otherwise
        """
        current = self.get_current_tcp()
        if current is None:
            current = self.read_tcp_position()
            if current is None:
                print("⚠️  Cannot read current TCP position")
                return False

        # Calculate target position
        target_x = current['x'] + dx
        target_y = current['y'] + dy
        target_z = current['z'] + dz
        target_rx = current['rx'] + drx
        target_ry = current['ry'] + dry
        target_rz = current['rz'] + drz

        return self.move_tcp_to(target_x, target_y, target_z,
                               target_rx, target_ry, target_rz, use_main)

    def write_pose_both(self, pose_main: List[float], pose_back: List[float]) -> bool:
        """
        Write both main and back poses simultaneously

        Args:
            pose_main: [x, y, z, rx, ry, rz] for main pose
            pose_back: [x, y, z, rx, ry, rz] for back pose

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected():
            return False

        try:
            # Convert poses to Modbus format
            main_data = self._pose_to_modbus_data(*pose_main)
            back_data = self._pose_to_modbus_data(*pose_back)

            # Write both poses
            self.client.write_registers(self.REGISTER_POSE_MAIN, main_data)
            self.client.write_registers(self.REGISTER_POSE_BACK, back_data)

            print(f"✅ Both poses written: Main={pose_main}, Back={pose_back}")
            return True

        except Exception as e:
            print(f"❌ Error writing poses: {e}")
            return False

    # ==================== Robot State Operations ====================

    def read_robot_state(self) -> Optional[int]:
        """
        Read robot state from register 0

        Returns:
            Robot state code or None if failed
            State codes (from manual):
            - 1: Emergency stop
            - 2: Standby
            - 3: Ready
            - 4: Running
            - 5: Paused
            - 6: Error
        """
        if not self.is_connected():
            return None

        try:
            rr = self.client.read_holding_registers(address=self.REGISTER_ROBOT_STATE, count=1)
            if rr.isError():
                print("⚠️  Failed to read robot state")
                return None

            state = rr.registers[0]
            state_names = {1: "Emergency Stop", 2: "Standby", 3: "Ready",
                          4: "Running", 5: "Paused", 6: "Error"}
            print(f"🤖 Robot State: {state} ({state_names.get(state, 'Unknown')})")
            return state

        except Exception as e:
            print(f"❌ Error reading robot state: {e}")
            return None

    def read_error_code(self) -> Optional[int]:
        """
        Read robot error code from register 1

        Returns:
            Error code or None if failed
        """
        if not self.is_connected():
            return None

        try:
            rr = self.client.read_holding_registers(address=self.REGISTER_ROBOT_ERROR, count=1)
            if rr.isError():
                print("⚠️  Failed to read error code")
                return None

            error = rr.registers[0]
            if error != 0:
                print(f"⚠️  Robot Error Code: {error}")
            return error

        except Exception as e:
            print(f"❌ Error reading error code: {e}")
            return None

    def set_servo_on(self, enable: bool) -> bool:
        """
        Enable/disable servo motors

        Args:
            enable: True to enable servo, False to disable

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected():
            return False

        try:
            value = 1 if enable else 0
            self.client.write_registers(self.REGISTER_SERVO_ON, [value])
            print(f"🔧 Servo: {'ON' if enable else 'OFF'}")
            return True
        except Exception as e:
            print(f"❌ Error setting servo: {e}")
            return False

    # ==================== Command Operations ====================

    def send_command(self, command: int) -> bool:
        """
        Send command to robot

        Args:
            command: Command code (1-4)

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected():
            return False

        try:
            self.client.write_registers(self.REGISTER_CMD, [command])
            print(f"📤 Command sent: {command}")
            return True
        except Exception as e:
            print(f"❌ Error sending command: {e}")
            return False

    def read_command(self) -> Optional[int]:
        """Read command from register 351"""
        if not self.is_connected():
            return None

        try:
            rr = self.client.read_holding_registers(address=self.REGISTER_CMD, count=1)
            if rr.isError():
                print("⚠️  Failed to read command")
                return None

            val = rr.registers[0]
            print(f"📖 Read command: {val}")
            return val

        except Exception as e:
            print(f"❌ Error reading command: {e}")
            return None

    def write_response(self, value: int) -> bool:
        """Write response to register 352"""
        if not self.is_connected():
            return False

        try:
            self.client.write_registers(self.REGISTER_RESP, [value])
            print(f"✍️  Response written: {value}")
            return True
        except Exception as e:
            print(f"❌ Error writing response: {e}")
            return False

    # ==================== Transformation Utilities ====================

    def get_transform_matrices(self) -> Optional[Dict]:
        """
        Get transformation matrices for vision processing

        Returns:
            Dictionary with transformation matrices or None
        """
        tcp = self.read_tcp_position()
        if tcp is None:
            return None

        x, y, z = tcp['x'], tcp['y'], tcp['z']
        rx, ry, rz = tcp['rx'], tcp['ry'], tcp['rz']

        # Translation vector (convert mm to meters)
        tvec = np.array([[x/1000.0, y/1000.0, z/1000.0]], dtype=np.float64)

        # Rotation matrix from Euler angles
        rotation_matrix = R.from_euler("xyz", [rx, ry, rz], degrees=True).as_matrix()

        # Camera to end-effector transform
        cam_to_ee = np.array([[-1, 0, 0],
                              [0, -1, 0],
                              [0, 0, 1]])

        rotation_matrix = rotation_matrix @ cam_to_ee.T
        robot_rotation_in_camera = rotation_matrix.T
        robot_position_in_camera = -robot_rotation_in_camera @ tvec.T

        return {
            'camera_position': tvec,
            'camera_rotation': rotation_matrix,
            'robot_rotation_in_camera': robot_rotation_in_camera,
            'robot_position_in_camera': robot_position_in_camera,
        }

    # ==================== Internal Helper Methods ====================

    @staticmethod
    def _pose_to_modbus_data(x_mm: float, y_mm: float, z_mm: float,
                             rx_deg: float, ry_deg: float, rz_deg: float) -> List[int]:
        """
        Convert pose to Modbus register format for user define area (301-312)

        Based on existing implementation in modbus_robot_interface.py:
        - Position: millimeters * 1000 (micrometers) as int16
        - Rotation: degrees as int16, normalized to (-180, 180]
        - Each value stored as int16, then mapped to uint16 register

        Args:
            x_mm, y_mm, z_mm: Position in millimeters
            rx_deg, ry_deg, rz_deg: Rotation in degrees

        Returns:
            List of 6 uint16 register values for pose_main or pose_back
        """
        def to_uint16(val: int):
            """Convert signed int16 to unsigned uint16"""
            return int(val + 65536) if val < 0 else int(val)

        def wrap_deg(deg: float):
            """Normalize angle to (-180, 180]"""
            return ((deg + 180) % 360) - 180

        # Position values (convert mm to micrometers)
        x_val = int(round(x_mm * 1000))  # mm to micrometers
        y_val = int(round(y_mm * 1000))
        z_val = int(round(z_mm * 1000))

        # Rotation values (normalize to -180 to 180)
        rx_val = int(round(wrap_deg(rx_deg)))
        ry_val = int(round(wrap_deg(ry_deg)))
        rz_val = int(round(wrap_deg(rz_deg)))

        # Clamp to int16 range and convert to uint16
        vals = [x_val, y_val, z_val, rx_val, ry_val, rz_val]
        regs = [
            to_uint16(max(min(v, 32767), -32768))
            for v in vals
        ]

        return regs

    @staticmethod
    def _modbus_to_pose(modbus_response) -> Tuple[float, float, float, float, float, float]:
        """
        Parse pose from Modbus registers (IEEE 754 32-bit float format)

        Based on Modbus Server Manual V1.01:
        - Data format: IEEE 754 32-bit float
        - Endian: Big Endian within 16-bit word, Little Endian for word arrays
        - Each float32 value uses 2 consecutive registers

        Args:
            modbus_response: Modbus response with 12 registers (6 float32 values)

        Returns:
            Tuple of (x, y, z, rx, ry, rz)
            - x, y, z: Position in mm
            - rx, ry, rz: Rotation in degrees
        """
        registers = modbus_response.registers

        # Parse float32 values from register pairs
        # Each value spans 2 registers: [reg_low, reg_high]
        # Endian: Big Endian within word, Little Endian for array
        def parse_float32(reg_low: int, reg_high: int) -> float:
            byte_data = reg_high.to_bytes(2, 'big') + reg_low.to_bytes(2, 'big')
            return struct.unpack('>f', byte_data)[0]

        x = parse_float32(registers[0], registers[1])
        y = parse_float32(registers[2], registers[3])
        z = parse_float32(registers[4], registers[5])
        rx = parse_float32(registers[6], registers[7])
        ry = parse_float32(registers[8], registers[9])
        rz = parse_float32(registers[10], registers[11])

        return (x, y, z, rx, ry, rz)

    # ==================== Context Manager Support ====================

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()

    # ==================== String Representation ====================

    def __str__(self):
        status = "Connected" if self.is_connected() else "Disconnected"
        tcp_info = ""
        if self._current_tcp:
            tcp = self._current_tcp
            tcp_info = f"\n  TCP: X={tcp['x']:.2f}, Y={tcp['y']:.2f}, Z={tcp['z']:.2f}"

        return (f"RobotTCPController({self.ip}:{self.port}) - {status}{tcp_info}")


# ==================== Example Usage ====================

def example_usage():
    """Example of how to use RobotTCPController"""

    # Create controller instance
    robot = RobotTCPController(ip="192.168.0.29", port=1502)

    # Connect to robot
    if not robot.connect():
        print("Failed to connect to robot")
        return

    try:
        # Read current TCP position
        current_tcp = robot.read_tcp_position()
        print(f"Current TCP: {current_tcp}")

        # Move to absolute position
        robot.move_tcp_to(x=100.0, y=200.0, z=300.0, rx=90.0, ry=0.0, rz=180.0)

        # Wait for movement to complete
        time.sleep(2)

        # Move relative to current position
        robot.move_tcp_relative(dx=10.0, dy=5.0, dz=0.0)  # Move 10mm in X, 5mm in Y

    finally:
        # Disconnect
        robot.disconnect()


if __name__ == "__main__":
    example_usage()
