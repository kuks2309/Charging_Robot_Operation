#!/usr/bin/env python3
"""
툴프레임별 위치 읽기 테스트
- command 40-43으로 툴프레임 변경 후 위치 읽기
"""

import argparse
import time
import struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

REG_POS_BASE = 158
REG_COMMAND = 351
REG_STATUS = 352


def read_float32(registers, idx):
    """2개의 16비트 레지스터에서 float32 값 추출"""
    high = registers[idx + 1]
    low = registers[idx]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def read_position(client):
    """현재 위치 읽기"""
    rr = client.read_holding_registers(address=REG_POS_BASE, count=12)
    if rr.isError():
        return None
    return {
        'x': read_float32(rr.registers, 0),
        'y': read_float32(rr.registers, 2),
        'z': read_float32(rr.registers, 4),
        'rx': read_float32(rr.registers, 6),
        'ry': read_float32(rr.registers, 8),
        'rz': read_float32(rr.registers, 10),
    }


def send_command(client, cmd):
    """명령 전송 및 완료 대기"""
    client.write_registers(REG_COMMAND, [cmd])

    for _ in range(50):
        time.sleep(0.1)
        rr = client.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            return False
        status = rr.registers[0]
        if status == 2:
            return True
        elif status == 3:
            return False
    return False


def main():
    parser = argparse.ArgumentParser(description='툴프레임별 위치 테스트')
    parser.add_argument('--frame', type=int, choices=[0, 1, 2, 3, 4, 5], help='특정 프레임만 테스트')
    args = parser.parse_args()

    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!\n")

    try:
        # 현재 위치 (변경 전)
        print("=" * 60)
        print("현재 위치 (툴프레임 변경 전):")
        pos = read_position(client)
        if pos:
            print(f"  X={pos['x']:.2f}, Y={pos['y']:.2f}, Z={pos['z']:.2f}")
            print(f"  Rx={pos['rx']:.2f}, Ry={pos['ry']:.2f}, Rz={pos['rz']:.2f}")
        print("=" * 60)

        frames = [args.frame] if args.frame is not None else [0, 1, 2, 3]

        for frame in frames:
            cmd = 40 + frame
            print(f"\n툴프레임 {frame} 설정 중 (command={cmd})...")

            if send_command(client, cmd):
                print(f"툴프레임 {frame} 설정 완료")
                time.sleep(0.3)

                pos = read_position(client)
                if pos:
                    print(f"  X  = {pos['x']:10.2f} mm")
                    print(f"  Y  = {pos['y']:10.2f} mm")
                    print(f"  Z  = {pos['z']:10.2f} mm")
                    print(f"  Rx = {pos['rx']:10.2f} deg")
                    print(f"  Ry = {pos['ry']:10.2f} deg")
                    print(f"  Rz = {pos['rz']:10.2f} deg")
            else:
                print(f"툴프레임 {frame} 설정 실패")

        # 원래 프레임(3)으로 복귀
        if args.frame is None:
            print("\n툴프레임 3으로 복귀...")
            send_command(client, 43)

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
