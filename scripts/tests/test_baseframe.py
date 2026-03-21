#!/usr/bin/env python3
"""
베이스프레임 전환 및 절대 이동 테스트

1. 현재 위치 확인
2. 베이스프레임 설정 (command 44)
3. 절대 좌표 이동 테스트
"""

import sys
import os
import time
import struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

# 레지스터 주소
REG_POS_BASE = 158
REG_X = 301
REG_COMMAND = 351
REG_STATUS = 352


def read_float32(registers, idx):
    """2개의 16비트 레지스터에서 float32 값 추출"""
    high = registers[idx + 1]
    low = registers[idx]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def to_int16(value):
    """int16 → uint16 변환"""
    if value < 0:
        return int(value + 65536)
    return int(value)


def read_current_position(client):
    """현재 로봇 위치 읽기"""
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


def send_command_and_wait(client, command, timeout=10.0):
    """명령 전송 및 완료 대기"""
    client.write_registers(REG_COMMAND, [command])
    print(f"  command={command} 전송됨")

    # Running 상태 대기
    print("  대기 중", end="", flush=True)
    for _ in range(int(timeout * 10)):
        time.sleep(0.1)
        rr = client.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            print("\n  상태 읽기 실패")
            return False
        status = rr.registers[0]
        if status == 1:  # Running
            print(".", end="", flush=True)
            break
        elif status == 3:  # Error
            print("\n  오류 발생!")
            return False
    else:
        print("\n  Running 감지 실패")
        return False

    # Done 상태 대기
    for _ in range(int(timeout * 10)):
        time.sleep(0.1)
        rr = client.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            return False
        status = rr.registers[0]
        if status == 2:  # Done
            print("\n  완료!")
            return True
        elif status == 3:
            print("\n  오류!")
            return False
        elif status == 1:
            print(".", end="", flush=True)

    print("\n  타임아웃")
    return False


def main():
    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!\n")

    try:
        # 현재 위치 확인
        pos = read_current_position(client)
        print("현재 위치:")
        print(f"  X  = {pos['x']:10.3f} mm")
        print(f"  Y  = {pos['y']:10.3f} mm")
        print(f"  Z  = {pos['z']:10.3f} mm")
        print(f"  Rx = {pos['rx']:10.3f} deg")
        print(f"  Ry = {pos['ry']:10.3f} deg")
        print(f"  Rz = {pos['rz']:10.3f} deg")

        # 베이스프레임 설정
        print("\n" + "="*50)
        print("[1단계] 베이스프레임 설정 (command 44)")
        print("="*50)
        input("Enter를 누르면 base(0)을 호출합니다...")

        if send_command_and_wait(client, 44):
            print("베이스프레임 설정 완료")
        else:
            print("베이스프레임 설정 실패")
            return

        # 위치 변화 확인
        time.sleep(0.5)
        new_pos = read_current_position(client)
        print("\n베이스프레임 설정 후 위치:")
        print(f"  X  = {new_pos['x']:10.3f} mm (변화: {new_pos['x']-pos['x']:+.3f})")
        print(f"  Y  = {new_pos['y']:10.3f} mm (변화: {new_pos['y']-pos['y']:+.3f})")
        print(f"  Z  = {new_pos['z']:10.3f} mm (변화: {new_pos['z']-pos['z']:+.3f})")
        print(f"  Rx = {new_pos['rx']:10.3f} deg (변화: {new_pos['rx']-pos['rx']:+.3f})")
        print(f"  Ry = {new_pos['ry']:10.3f} deg (변화: {new_pos['ry']-pos['ry']:+.3f})")
        print(f"  Rz = {new_pos['rz']:10.3f} deg (변화: {new_pos['rz']-pos['rz']:+.3f})")

        # 절대 좌표 이동 테스트
        print("\n" + "="*50)
        print("[2단계] 절대 좌표 이동 테스트 (command 20)")
        print("="*50)

        # 현재 위치에서 X +50mm
        target_x = new_pos['x'] + 50
        print(f"\n목표: X = {target_x:.3f} mm (현재에서 +50mm)")
        input("Enter를 누르면 이동합니다...")

        # ±180° 정규화 (Rx, Ry, Rz)
        def normalize_angle(a):
            a = a % 360
            if a > 180:
                a -= 360
            return a

        rx_val = normalize_angle(new_pos['rx'])
        ry_val = normalize_angle(new_pos['ry'])
        rz_val = normalize_angle(new_pos['rz'])

        # 레지스터 쓰기
        regs = [
            to_int16(int(round(target_x * 10))),
            to_int16(int(round(new_pos['y'] * 10))),
            to_int16(int(round(new_pos['z'] * 10))),
            to_int16(int(round(rx_val * 10))),
            to_int16(int(round(ry_val * 10))),
            to_int16(int(round(rz_val * 10))),
        ]
        client.write_registers(REG_X, regs)

        if send_command_and_wait(client, 20, timeout=30.0):
            time.sleep(0.5)
            final_pos = read_current_position(client)
            print("\n이동 후 위치:")
            print(f"  X  = {final_pos['x']:10.3f} mm (목표 대비 오차: {final_pos['x']-target_x:+.3f})")
            print(f"  Y  = {final_pos['y']:10.3f} mm")
            print(f"  Z  = {final_pos['z']:10.3f} mm")

            if abs(final_pos['x'] - target_x) < 1.0:
                print("\n이동 성공!")
            else:
                print("\n이동 오차 발생!")
        else:
            print("이동 실패")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
