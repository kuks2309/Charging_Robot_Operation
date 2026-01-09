#!/usr/bin/env python3
"""
로봇 현재 위치 읽기 테스트 스크립트
- 카메라 TCP 위치 (레지스터 158~169)를 읽어서 표시
"""

from pymodbus.client import ModbusTcpClient
import struct

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

# 카메라 TCP 위치 레지스터 (float32, 12개 레지스터 = 6개 값)
REGISTER_BASE_CAM = 158


def read_float32_from_registers(registers, start_idx):
    """2개의 16비트 레지스터에서 float32 값 추출"""
    high = registers[start_idx + 1]
    low = registers[start_idx]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def main():
    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!\n")

    try:
        # 카메라 TCP 위치 읽기 (12개 레지스터 = 6개 float32 값)
        rr = client.read_holding_registers(address=REGISTER_BASE_CAM, count=12)

        if rr.isError():
            print(f"레지스터 읽기 실패: {rr}")
            return

        # float32 값으로 변환
        x = read_float32_from_registers(rr.registers, 0)
        y = read_float32_from_registers(rr.registers, 2)
        z = read_float32_from_registers(rr.registers, 4)
        rx = read_float32_from_registers(rr.registers, 6)
        ry = read_float32_from_registers(rr.registers, 8)
        rz = read_float32_from_registers(rr.registers, 10)

        print("=" * 50)
        print("현재 로봇 카메라 TCP 위치 (레지스터 158~169)")
        print("=" * 50)
        print(f"  X  = {x:10.3f} mm")
        print(f"  Y  = {y:10.3f} mm")
        print(f"  Z  = {z:10.3f} mm")
        print(f"  Rx = {rx:10.3f} deg")
        print(f"  Ry = {ry:10.3f} deg")
        print(f"  Rz = {rz:10.3f} deg")
        print("=" * 50)

        # 원본 레지스터 값도 표시
        print("\n원본 레지스터 값:")
        for i, val in enumerate(rr.registers):
            print(f"  레지스터 {REGISTER_BASE_CAM + i}: {val} (0x{val:04X})")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
