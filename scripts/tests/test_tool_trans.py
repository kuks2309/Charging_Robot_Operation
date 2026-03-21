#!/usr/bin/env python3
"""
상대 이동 테스트 (tool.trans 명령)
- command 10: X축 상대 이동
- command 11: Y축 상대 이동
- command 12: Z축 상대 이동
- command 13: XYZ 동시 상대 이동

사용법:
  python3 /home/amap/Project/KAIST/Charging_Robot/scripts/test_tool_trans.py --dx 50
  python3 /home/amap/Project/KAIST/Charging_Robot/scripts/test_tool_trans.py --dz -30
  python3 /home/amap/Project/KAIST/Charging_Robot/scripts/test_tool_trans.py --dx 10 --dy 20 --dz 30
"""

import argparse
import time
import struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

REG_POS_BASE = 158
REG_X = 301
REG_Y = 302
REG_Z = 303
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


def send_tool_trans(client, cmd, dx=0, dy=0, dz=0):
    """상대 이동 명령 전송"""
    # 레지스터에 이동량 쓰기 (mm 단위, 정수)
    client.write_registers(REG_X, [to_int16(int(dx))])
    client.write_registers(REG_Y, [to_int16(int(dy))])
    client.write_registers(REG_Z, [to_int16(int(dz))])

    print(f"\n레지스터 전송값:")
    print(f"  301 (X): {to_int16(int(dx))} ({dx} mm)")
    print(f"  302 (Y): {to_int16(int(dy))} ({dy} mm)")
    print(f"  303 (Z): {to_int16(int(dz))} ({dz} mm)")

    # command 전송
    client.write_registers(REG_COMMAND, [cmd])
    print(f"\ncommand={cmd} 전송")

    # 완료 대기
    print("로봇 동작 대기 중", end="", flush=True)

    # Running 상태 대기
    for _ in range(50):
        time.sleep(0.1)
        rr = client.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            print("\n상태 읽기 실패")
            return None
        status = rr.registers[0]
        if status == 1:
            print(".", end="", flush=True)
            break
        elif status == 3:
            print("\n오류 발생!")
            return None
    else:
        print("\nRunning 상태 감지 실패")
        return None

    # Done 상태 대기
    for _ in range(300):
        time.sleep(0.1)
        rr = client.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            print("\n상태 읽기 실패")
            return None
        status = rr.registers[0]
        if status == 2:
            print("\n완료!")
            return True
        elif status == 3:
            print("\n오류 발생!")
            return None
        elif status == 1:
            print(".", end="", flush=True)

    print("\n타임아웃")
    return None


def main():
    parser = argparse.ArgumentParser(description='상대 이동 테스트 (tool.trans)')
    parser.add_argument('--dx', type=float, default=0, help='X축 이동량 (mm)')
    parser.add_argument('--dy', type=float, default=0, help='Y축 이동량 (mm)')
    parser.add_argument('--dz', type=float, default=0, help='Z축 이동량 (mm)')
    args = parser.parse_args()

    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!")

    try:
        # 현재 위치 읽기
        pos = read_current_position(client)
        print("\n현재 위치:")
        print(f"  X  = {pos['x']:10.3f} mm")
        print(f"  Y  = {pos['y']:10.3f} mm")
        print(f"  Z  = {pos['z']:10.3f} mm")

        # 명령 결정
        if args.dx != 0 and args.dy == 0 and args.dz == 0:
            cmd = 10  # X축만
            print(f"\nX축 상대 이동: {args.dx} mm (command 10)")
        elif args.dy != 0 and args.dx == 0 and args.dz == 0:
            cmd = 11  # Y축만
            print(f"\nY축 상대 이동: {args.dy} mm (command 11)")
        elif args.dz != 0 and args.dx == 0 and args.dy == 0:
            cmd = 12  # Z축만
            print(f"\nZ축 상대 이동: {args.dz} mm (command 12)")
        else:
            cmd = 13  # XYZ 동시
            print(f"\nXYZ 동시 상대 이동: dx={args.dx}, dy={args.dy}, dz={args.dz} mm (command 13)")

        # Tool 좌표계 이동은 Base 좌표계로 단순 변환 불가
        print(f"\n(Tool 좌표계 기준 이동 - Base 좌표계 예상값 계산 생략)")

        # 확인
        input("\nEnter를 누르면 이동을 시작합니다...")

        # 명령 전송
        send_tool_trans(client, cmd, args.dx, args.dy, args.dz)

        # 1초 대기 후 위치 확인
        time.sleep(1.0)
        new_pos = read_current_position(client)
        print("\n이동 후 위치 (Base 좌표계):")
        print(f"  X  = {new_pos['x']:10.3f} mm")
        print(f"  Y  = {new_pos['y']:10.3f} mm")
        print(f"  Z  = {new_pos['z']:10.3f} mm")

        # 실제 이동량 (Base 좌표계 기준)
        delta_x = new_pos['x'] - pos['x']
        delta_y = new_pos['y'] - pos['y']
        delta_z = new_pos['z'] - pos['z']
        total_dist = (delta_x**2 + delta_y**2 + delta_z**2)**0.5

        print("\nBase 좌표계 기준 이동량:")
        print(f"  dX = {delta_x:+.3f} mm")
        print(f"  dY = {delta_y:+.3f} mm")
        print(f"  dZ = {delta_z:+.3f} mm")
        print(f"  총 이동거리 = {total_dist:.3f} mm")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
