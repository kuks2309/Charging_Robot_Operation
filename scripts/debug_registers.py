#!/usr/bin/env python3
"""
레지스터 값 디버깅 스크립트
- 301-306, 351-352 레지스터 값을 읽어서 표시
"""

import time
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

def from_uint16(value):
    """uint16 → int16 변환"""
    if value > 32767:
        return value - 65536
    return value


def main():
    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!\n")

    try:
        # 위치 레지스터 읽기 (301-306)
        rr = client.read_holding_registers(address=301, count=6)
        if not rr.isError():
            print("위치 레지스터 (301-306):")
            labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
            for i, val in enumerate(rr.registers):
                signed = from_uint16(val)
                print(f"  {301+i} ({labels[i]}): {val} (signed: {signed}, /10: {signed/10})")

        # 명령/상태 레지스터 읽기 (351-352)
        rr = client.read_holding_registers(address=351, count=2)
        if not rr.isError():
            print(f"\n명령/상태 레지스터:")
            print(f"  351 (command): {rr.registers[0]}")
            print(f"  352 (status):  {rr.registers[1]}")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
