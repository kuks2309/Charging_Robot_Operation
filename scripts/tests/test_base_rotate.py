#!/usr/bin/env python3
"""
베이스 좌표계 회전 테스트
- 현재 TCP 자세 읽기 → 베이스 Rx/Ry/Rz에 각도 더하기 → movel(CMD 20)로 절대 이동
- CMD 54-56(rotx/roty/rotz)이 툴 기준으로 동작하므로 movel 방식 사용
"""

import time
import sys
import struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

REG_X = 301
REG_Y = 302
REG_Z = 303
REG_RX = 304
REG_RY = 305
REG_RZ = 306
REG_COMMAND = 351
REG_STATUS = 352

CMD_MOVEL = 20  # movel(pose) - 절대 좌표 이동 (×10 스케일, PRS ÷10)


def to_uint16(value):
    """int16 → uint16 변환"""
    if value < 0:
        return int(value + 65536)
    return int(value)


def wait_done(client, timeout=10.0):
    """명령 완료 대기"""
    start = time.time()
    while time.time() - start < timeout:
        rr = client.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            time.sleep(0.1)
            continue
        status = rr.registers[0]
        if status == 2:  # Done
            return True, "완료"
        elif status == 3:  # Error
            return False, "로봇 오류"
        elif status == 0 and time.time() - start > 0.3:
            return True, "완료 (IDLE)"
        time.sleep(0.1)
    return False, "타임아웃"


def read_pose(client):
    """현재 TCP 자세 읽기 (레지스터 158~169, float32 Little Endian word order)"""
    rr = client.read_holding_registers(address=158, count=12)
    if rr.isError():
        return None
    vals = []
    for i in range(6):
        high = rr.registers[i * 2 + 1]
        low = rr.registers[i * 2]
        byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
        vals.append(struct.unpack('>f', byte_data)[0])
    return vals


def main():
    # 파라미터: 축(rx/ry/rz), 각도(deg)
    axis = sys.argv[1] if len(sys.argv) > 1 else 'rx'
    angle_deg = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    axis_index = {'rx': 3, 'ry': 4, 'rz': 5}
    if axis not in axis_index:
        print(f"잘못된 축: {axis} (rx/ry/rz)")
        return

    print(f"=== 베이스 좌표계 회전 테스트 (movel 방식) ===")
    print(f"축: {axis}, 각도: {angle_deg}°")
    print(f"방법: 현재 자세 읽기 → 베이스 {axis.upper()}에 {angle_deg}° 더하기 → movel")
    print(f"연결: {ROBOT_IP}:{ROBOT_PORT}")
    print()

    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)
    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!")

    try:
        # 1. 현재 자세 읽기
        before = read_pose(client)
        if not before:
            print("자세 읽기 실패!")
            return

        print(f"\n[1] 현재 자세:")
        print(f"    X={before[0]:.2f}, Y={before[1]:.2f}, Z={before[2]:.2f}")
        print(f"    Rx={before[3]:.2f}, Ry={before[4]:.2f}, Rz={before[5]:.2f}")

        # 2. 목표 자세 계산 (베이스 Rx/Ry/Rz에 각도 더하기, 위치는 유지)
        target = list(before)
        idx = axis_index[axis]
        target[idx] += angle_deg

        # ±180° 정규화
        for i in range(3, 6):
            target[i] = target[i] % 360
            if target[i] > 180:
                target[i] -= 360

        print(f"\n[2] 목표 자세 ({axis.upper()} += {angle_deg}°):")
        print(f"    X={target[0]:.2f}, Y={target[1]:.2f}, Z={target[2]:.2f}")
        print(f"    Rx={target[3]:.2f}, Ry={target[4]:.2f}, Rz={target[5]:.2f}")

        # 3. movel 명령 전송 (CMD 20, ×10 스케일)
        x_val = to_uint16(int(round(target[0] * 10)))
        y_val = to_uint16(int(round(target[1] * 10)))
        z_val = to_uint16(int(round(target[2] * 10)))
        rx_val = to_uint16(int(round(target[3] * 10)))
        ry_val = to_uint16(int(round(target[4] * 10)))
        rz_val = to_uint16(int(round(target[5] * 10)))

        print(f"\n[3] movel 전송 (CMD={CMD_MOVEL})...")
        print(f"    레지스터: X={x_val}, Y={y_val}, Z={z_val}, Rx={rx_val}, Ry={ry_val}, Rz={rz_val}")

        # 레지스터 쓰기 (301~306)
        client.write_registers(REG_X, [x_val, y_val, z_val, rx_val, ry_val, rz_val])
        # 명령 전송
        client.write_registers(REG_COMMAND, [CMD_MOVEL])

        ok, msg = wait_done(client)
        print(f"    결과: {msg}")

        # 4. 회전 후 자세
        after = read_pose(client)
        if after:
            print(f"\n[4] 이동 후 자세:")
            print(f"    X={after[0]:.2f}, Y={after[1]:.2f}, Z={after[2]:.2f}")
            print(f"    Rx={after[3]:.2f}, Ry={after[4]:.2f}, Rz={after[5]:.2f}")
            print(f"\n[5] 변화량:")
            print(f"    dX={after[0]-before[0]:.2f}, dY={after[1]-before[1]:.2f}, dZ={after[2]-before[2]:.2f}")
            print(f"    dRx={after[3]-before[3]:.2f}, dRy={after[4]-before[4]:.2f}, dRz={after[5]-before[5]:.2f}")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    print("사용법: python test_base_rotate.py [축] [각도]")
    print("예시:")
    print("  python test_base_rotate.py rx 5.0    # 베이스 Rx +5°")
    print("  python test_base_rotate.py ry -2.0   # 베이스 Ry -2°")
    print("  python test_base_rotate.py rz 1.0    # 베이스 Rz +1°")
    print()
    main()
