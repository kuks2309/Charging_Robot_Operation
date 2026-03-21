#!/usr/bin/env python3
"""
회전 절대값 설정 (현재 XYZ 위치 유지, 회전만 변경)

사용법:
  python3 /home/amap/Project/KAIST/Charging_Robot/scripts/set_rz.py --rx 45 --ry 0 --rz 90
"""

import argparse
import math
import time
import struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

REG_POS_BASE = 158
REG_X = 301
REG_COMMAND = 351
REG_STATUS = 352


def read_float32(registers, idx):
    high = registers[idx + 1]
    low = registers[idx]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def to_int16(value):
    if value < 0:
        return int(value + 65536)
    return int(value)


def read_current_position(client):
    rr = client.read_holding_registers(address=REG_POS_BASE, count=12)
    if rr.isError():
        raise Exception(f"위치 읽기 실패: {rr}")
    return {
        'x': read_float32(rr.registers, 0),
        'y': read_float32(rr.registers, 2),
        'z': read_float32(rr.registers, 4),
        'rx': read_float32(rr.registers, 6),
        'ry': read_float32(rr.registers, 8),
        'rz': read_float32(rr.registers, 10),
    }


def main():
    parser = argparse.ArgumentParser(description='회전 절대값 설정')
    parser.add_argument('--rx', type=float, default=45, help='Rx (deg)')
    parser.add_argument('--ry', type=float, default=0, help='Ry (deg)')
    parser.add_argument('--rz', type=float, default=90, help='Rz (deg)')
    args = parser.parse_args()

    # target 각도 정규화 [-180, 180]
    args.rx = math.remainder(args.rx, 360)
    args.ry = math.remainder(args.ry, 360)
    args.rz = math.remainder(args.rz, 360)

    target_rx = args.rx
    target_ry = args.ry
    target_rz = args.rz

    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!")

    try:
        pos = read_current_position(client)
        print(f"\n현재 위치:")
        print(f"  X={pos['x']:.2f}, Y={pos['y']:.2f}, Z={pos['z']:.2f}")
        print(f"  Rx={pos['rx']:.2f}, Ry={pos['ry']:.2f}, Rz={pos['rz']:.2f}")

        print(f"\n목표 회전: Rx={target_rx}, Ry={target_ry}, Rz={target_rz} deg")

        input("\nEnter를 누르면 이동을 시작합니다...")

        # 현재 위치 각도 정규화 [-180, 180] (math.remainder: PRS 범위 일치)
        for key in ['rx', 'ry', 'rz']:
            pos[key] = math.remainder(pos[key], 360)

        # 레지스터 전송 (x10 스케일)
        regs = [
            to_int16(int(round(pos['x'] * 10))),
            to_int16(int(round(pos['y'] * 10))),
            to_int16(int(round(pos['z'] * 10))),
            to_int16(int(round(target_rx * 10))),
            to_int16(int(round(target_ry * 10))),
            to_int16(int(round(target_rz * 10))),
        ]
        client.write_registers(REG_X, regs)
        client.write_registers(REG_COMMAND, [20])
        print("command=20 전송")

        # 완료 대기
        print("로봇 동작 대기 중", end="", flush=True)
        for _ in range(50):
            time.sleep(0.1)
            rr = client.read_holding_registers(address=REG_STATUS, count=1)
            if not rr.isError() and rr.registers[0] == 1:
                break

        for _ in range(300):
            time.sleep(0.1)
            rr = client.read_holding_registers(address=REG_STATUS, count=1)
            if rr.isError():
                continue
            status = rr.registers[0]
            if status == 2:
                print("\n완료!")
                break
            elif status == 3:
                print("\n오류!")
                break
            elif status == 1:
                print(".", end="", flush=True)

        time.sleep(0.5)
        new_pos = read_current_position(client)
        print(f"\n이동 후 위치:")
        print(f"  X={new_pos['x']:.2f}, Y={new_pos['y']:.2f}, Z={new_pos['z']:.2f}")
        print(f"  Rx={new_pos['rx']:.2f}, Ry={new_pos['ry']:.2f}, Rz={new_pos['rz']:.2f}")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
