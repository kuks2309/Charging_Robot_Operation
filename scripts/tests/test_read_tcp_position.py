#!/usr/bin/env python3
"""
TCP Position 레지스터(158~169) 읽기 테스트
로봇이 자동으로 업데이트하는지 확인
"""

import time
import struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

REG_TCP_POSE = 158  # 158~169 (12 registers)


def read_float32(registers, offset):
    """2개 레지스터를 IEEE 754 float32로 변환 (Little Endian Word, Big Endian Byte)"""
    low = registers[offset]
    high = registers[offset + 1]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def main():
    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("❌ 연결 실패!")
        return

    print("✓ 연결 성공!\n")

    try:
        # TCP Position 레지스터 읽기 (158~169)
        result = client.read_holding_registers(address=REG_TCP_POSE, count=12)

        if result.isError():
            print("❌ 레지스터 읽기 실패!")
            return

        print("=== 레지스터 Raw 값 ===")
        print(f"Registers 158~169: {result.registers}")
        print()

        # Float32로 변환
        x = read_float32(result.registers, 0)
        y = read_float32(result.registers, 2)
        z = read_float32(result.registers, 4)
        rx = read_float32(result.registers, 6)
        ry = read_float32(result.registers, 8)
        rz = read_float32(result.registers, 10)

        print("=== 현재 TCP Position (Base 좌표계) ===")
        print(f"X:  {x:8.2f} mm")
        print(f"Y:  {y:8.2f} mm")
        print(f"Z:  {z:8.2f} mm")
        print(f"Rx: {rx:8.2f} deg")
        print(f"Ry: {ry:8.2f} deg")
        print(f"Rz: {rz:8.2f} deg")
        print()

        # 모든 값이 0인지 확인
        all_zero = all(v == 0.0 for v in [x, y, z, rx, ry, rz])

        if all_zero:
            print("⚠️  경고: 모든 값이 0입니다!")
            print("   → 로봇이 레지스터를 업데이트하지 않고 있을 수 있습니다.")
            print("   → 로봇 모드를 확인하세요 (자동/수동/외부).")
        else:
            print("✓ 정상: TCP 위치가 읽혔습니다!")

        # 연속 읽기 테스트 (5회)
        print("\n=== 연속 읽기 테스트 (5회, 1초 간격) ===")
        for i in range(5):
            time.sleep(1)
            result = client.read_holding_registers(address=REG_TCP_POSE, count=12)
            if not result.isError():
                x = read_float32(result.registers, 0)
                y = read_float32(result.registers, 2)
                z = read_float32(result.registers, 4)
                print(f"{i+1}. X={x:7.2f}, Y={y:7.2f}, Z={z:7.2f}")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
