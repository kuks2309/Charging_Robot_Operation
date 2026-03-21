#!/usr/bin/env python3
"""
절대 좌표 이동 테스트 (Base 좌표계)

사용법:
  python test_absolute_move.py --x 650 --y -150 --z 740  # 절대 좌표로 이동
  python test_absolute_move.py  # 현재 위치 유지 (테스트용)
"""

import argparse
import time
import struct
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from angle_utils import normalize_angle
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

# 현재 위치 읽기용 레지스터 (float32)
REG_POS_BASE = 158

# 명령 전송용 레지스터 (int16, mm×10, deg×10)
REG_X = 301
REG_Y = 302
REG_Z = 303
REG_RX = 304
REG_RY = 305
REG_RZ = 306
REG_COMMAND = 351
REG_STATUS = 352


def read_float32(registers, idx):
    """2개의 16비트 레지스터에서 float32 값 추출"""
    high = registers[idx + 1]
    low = registers[idx]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def to_int16(value):
    """int16 → uint16 변환 (음수 처리)"""
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


def send_absolute_move(client, x, y, z, rx, ry, rz):
    """절대좌표 이동 명령 전송 (command 20)"""
    # ±180° 정규화
    rx = normalize_angle(rx)
    ry = normalize_angle(ry)
    rz = normalize_angle(rz)

    # mm×10, deg×10 스케일링
    regs = [
        to_int16(int(round(x * 10))),      # x (mm×10)
        to_int16(int(round(y * 10))),      # y (mm×10)
        to_int16(int(round(z * 10))),      # z (mm×10)
        to_int16(int(round(rx * 10))),     # Rx (deg×10)
        to_int16(int(round(ry * 10))),     # Ry (deg×10)
        to_int16(int(round(rz * 10))),     # Rz (deg×10)
    ]

    print(f"\n레지스터 전송값:")
    print(f"  301 (X):  {regs[0]} ({x:.1f} mm)")
    print(f"  302 (Y):  {regs[1]} ({y:.1f} mm)")
    print(f"  303 (Z):  {regs[2]} ({z:.1f} mm)")
    print(f"  304 (Rx): {regs[3]} ({rx:.1f} deg)")
    print(f"  305 (Ry): {regs[4]} ({ry:.1f} deg)")
    print(f"  306 (Rz): {regs[5]} ({rz:.1f} deg)")

    # 위치 레지스터 쓰기
    client.write_registers(REG_X, regs)

    # command 20 전송 (절대좌표 이동)
    client.write_registers(REG_COMMAND, [20])
    print("\ncommand=20 (절대좌표 이동) 전송")

    # 완료 대기
    print("로봇 동작 대기 중", end="", flush=True)

    # Running(1) 상태 감지 후 완료(Done/Idle) 대기
    saw_running = False
    for _ in range(1500):  # 30초 타임아웃 (20ms × 1500 = 30초)
        time.sleep(0.02)
        rr = client.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            print("\n상태 읽기 실패")
            return None
        status = rr.registers[0]
        if status == 1:  # Running
            saw_running = True
            print(".", end="", flush=True)
        elif status == 2:  # Done
            print("\n완료!")
            return True
        elif status == 0 and saw_running:  # Idle (Running 후 완료)
            print("\n완료!")
            return True
        elif status == 3:  # Error
            print("\n오류 발생!")
            return None

    print("\n타임아웃")
    return None


def main():
    parser = argparse.ArgumentParser(description='절대 좌표 이동 테스트')
    parser.add_argument('--x', type=float, default=None, help='목표 X 좌표 (mm)')
    parser.add_argument('--y', type=float, default=None, help='목표 Y 좌표 (mm)')
    parser.add_argument('--z', type=float, default=None, help='목표 Z 좌표 (mm)')
    parser.add_argument('--rx', type=float, default=None, help='목표 Rx 각도 (deg)')
    parser.add_argument('--ry', type=float, default=None, help='목표 Ry 각도 (deg)')
    parser.add_argument('--rz', type=float, default=None, help='목표 Rz 각도 (deg)')
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
        print("\n현재 위치 (Base 좌표계):")
        print(f"  X  = {pos['x']:10.3f} mm")
        print(f"  Y  = {pos['y']:10.3f} mm")
        print(f"  Z  = {pos['z']:10.3f} mm")
        print(f"  Rx = {pos['rx']:10.3f} deg")
        print(f"  Ry = {pos['ry']:10.3f} deg")
        print(f"  Rz = {pos['rz']:10.3f} deg")

        # 목표 위치 설정 (인자가 없으면 현재 위치 유지)
        target_x = args.x if args.x is not None else pos['x']
        target_y = args.y if args.y is not None else pos['y']
        target_z = args.z if args.z is not None else pos['z']
        target_rx = args.rx if args.rx is not None else pos['rx']
        target_ry = args.ry if args.ry is not None else pos['ry']
        target_rz = args.rz if args.rz is not None else pos['rz']

        print(f"\n목표 위치 (절대 좌표):")
        print(f"  X  = {target_x:10.3f} mm")
        print(f"  Y  = {target_y:10.3f} mm")
        print(f"  Z  = {target_z:10.3f} mm")
        print(f"  Rx = {target_rx:10.3f} deg")
        print(f"  Ry = {target_ry:10.3f} deg")
        print(f"  Rz = {target_rz:10.3f} deg")

        # 변화량 계산
        print(f"\n변화량:")
        print(f"  dX  = {target_x - pos['x']:+.3f} mm")
        print(f"  dY  = {target_y - pos['y']:+.3f} mm")
        print(f"  dZ  = {target_z - pos['z']:+.3f} mm")
        print(f"  dRx = {target_rx - pos['rx']:+.3f} deg")
        print(f"  dRy = {target_ry - pos['ry']:+.3f} deg")
        print(f"  dRz = {target_rz - pos['rz']:+.3f} deg")

        # 확인
        input("\nEnter를 누르면 이동을 시작합니다...")

        # 절대좌표 이동 명령 전송
        result = send_absolute_move(client, target_x, target_y, target_z,
                                     target_rx, target_ry, target_rz)

        # 1초 대기 후 현재 위치 확인
        time.sleep(1.0)
        new_pos = read_current_position(client)
        print("\n이동 후 위치:")
        print(f"  X  = {new_pos['x']:10.3f} mm")
        print(f"  Y  = {new_pos['y']:10.3f} mm")
        print(f"  Z  = {new_pos['z']:10.3f} mm")
        print(f"  Rx = {new_pos['rx']:10.3f} deg")
        print(f"  Ry = {new_pos['ry']:10.3f} deg")
        print(f"  Rz = {new_pos['rz']:10.3f} deg")

        # 목표와 실제 위치 비교
        err_x = new_pos['x'] - target_x
        err_y = new_pos['y'] - target_y
        err_z = new_pos['z'] - target_z
        err_rx = new_pos['rx'] - target_rx
        err_ry = new_pos['ry'] - target_ry
        err_rz = new_pos['rz'] - target_rz

        print("\n목표 대비 오차:")
        print(f"  dX  = {err_x:+.3f} mm")
        print(f"  dY  = {err_y:+.3f} mm")
        print(f"  dZ  = {err_z:+.3f} mm")
        print(f"  dRx = {err_rx:+.3f} deg")
        print(f"  dRy = {err_ry:+.3f} deg")
        print(f"  dRz = {err_rz:+.3f} deg")

        # 위치 정확도 판정 (허용 오차: 1mm, 0.5deg)
        pos_ok = abs(err_x) < 1.0 and abs(err_y) < 1.0 and abs(err_z) < 1.0
        rot_ok = abs(err_rx) < 0.5 and abs(err_ry) < 0.5 and abs(err_rz) < 0.5

        if pos_ok and rot_ok:
            print("\n이동 성공!")
        else:
            print("\n이동 오차 발생!")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
