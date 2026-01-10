#!/usr/bin/env python3
"""로봇 상태 및 레지스터 확인"""

import struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502


def read_float32(registers, idx):
    high = registers[idx + 1]
    low = registers[idx]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def main():
    print(f"로봇 연결: {ROBOT_IP}:{ROBOT_PORT}")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!\n")

    try:
        # 주요 레지스터 읽기
        print("="*60)
        print("주요 레지스터 상태")
        print("="*60)

        # 레지스터 351 (task_number / command)
        rr = client.read_holding_registers(address=351, count=2)
        if not rr.isError():
            print(f"351 (task_number/command): {rr.registers[0]}")
            print(f"352 (task_done/status):    {rr.registers[1]}")

        # 레지스터 301-306 (포즈 데이터)
        rr = client.read_holding_registers(address=301, count=12)
        if not rr.isError():
            print(f"\n301-306 (pose_main):")
            print(f"  301 (X):  {rr.registers[0]}")
            print(f"  302 (Y):  {rr.registers[1]}")
            print(f"  303 (Z):  {rr.registers[2]}")
            print(f"  304 (Rx): {rr.registers[3]}")
            print(f"  305 (Ry): {rr.registers[4]}")
            print(f"  306 (Rz): {rr.registers[5]}")
            print(f"\n307-312 (pose_back):")
            print(f"  307 (X2):  {rr.registers[6]}")
            print(f"  308 (Y2):  {rr.registers[7]}")
            print(f"  309 (Z2):  {rr.registers[8]}")
            print(f"  310 (Rx2): {rr.registers[9]}")
            print(f"  311 (Ry2): {rr.registers[10]}")
            print(f"  312 (Rz2): {rr.registers[11]}")

        # 카메라 포즈 (158-169)
        rr = client.read_holding_registers(address=158, count=12)
        if not rr.isError():
            print(f"\n158-169 (camera_pose):")
            x = read_float32(rr.registers, 0)
            y = read_float32(rr.registers, 2)
            z = read_float32(rr.registers, 4)
            rx = read_float32(rr.registers, 6)
            ry = read_float32(rr.registers, 8)
            rz = read_float32(rr.registers, 10)
            print(f"  X:  {x:.3f} mm")
            print(f"  Y:  {y:.3f} mm")
            print(f"  Z:  {z:.3f} mm")
            print(f"  Rx: {rx:.3f} deg")
            print(f"  Ry: {ry:.3f} deg")
            print(f"  Rz: {rz:.3f} deg")

        # 레지스터 315 (status - Main_task용)
        rr = client.read_holding_registers(address=315, count=1)
        if not rr.isError():
            print(f"\n315 (status - Main_task): {rr.registers[0]}")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
