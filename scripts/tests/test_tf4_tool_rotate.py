#!/usr/bin/env python3
"""
TF4 tool.rotx/roty/rotz 개별 테스트
- TF4 (command=45) 설정 후 각 축 회전 테스트
- 기존 test_tool1_rz_rotate.py 참조: deg×10 스케일링 사용
"""

import time
import sys
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

REG_RX = 304
REG_RY = 305
REG_RZ = 306
REG_COMMAND = 351
REG_STATUS = 352

# 툴프레임 명령
CMD_TOOLFRAME_4 = 45
CMD_TOOLFRAME_5 = 46

# 회전 명령
CMD_ROTX = 14
CMD_ROTY = 15
CMD_ROTZ = 16


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
    import struct
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
    # 파라미터: 축(rx/ry/rz), 각도(deg), 스케일(10 or 1), 툴프레임(4 or 5)
    axis = sys.argv[1] if len(sys.argv) > 1 else 'rz'
    angle_deg = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    scale = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    tf = int(sys.argv[4]) if len(sys.argv) > 4 else 4

    cmd_map = {'rx': (CMD_ROTX, REG_RX), 'ry': (CMD_ROTY, REG_RY), 'rz': (CMD_ROTZ, REG_RZ)}
    tf_cmd = CMD_TOOLFRAME_4 if tf == 4 else CMD_TOOLFRAME_5

    if axis not in cmd_map:
        print(f"잘못된 축: {axis} (rx/ry/rz)")
        return

    rot_cmd, rot_reg = cmd_map[axis]
    scaled_value = int(angle_deg * scale)
    reg_value = to_uint16(scaled_value)

    print(f"=== TF{tf} tool.rot{axis[-1]}() 테스트 ===")
    print(f"축: {axis}, 각도: {angle_deg}°, 스케일: ×{scale}, 레지스터값: {reg_value}")
    print(f"연결: {ROBOT_IP}:{ROBOT_PORT}")
    print()

    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)
    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!")

    try:
        # 1. 툴프레임 설정
        print(f"\n[1] TF{tf} 설정 (command={tf_cmd})...")
        client.write_registers(REG_COMMAND, [tf_cmd])
        ok, msg = wait_done(client)
        print(f"    결과: {msg}")
        if not ok:
            return
        time.sleep(0.3)  # 클린업 대기

        # 2. 회전 전 자세
        before = read_pose(client)
        if before:
            print(f"\n[2] 회전 전 자세:")
            print(f"    X={before[0]:.2f}, Y={before[1]:.2f}, Z={before[2]:.2f}")
            print(f"    Rx={before[3]:.2f}, Ry={before[4]:.2f}, Rz={before[5]:.2f}")

        # 3. 회전 명령 전송
        print(f"\n[3] tool.rot{axis[-1]}({scaled_value}) 전송...")
        print(f"    레지스터 {rot_reg} = {reg_value}")
        client.write_registers(rot_reg, [reg_value])
        print(f"    command = {rot_cmd}")
        client.write_registers(REG_COMMAND, [rot_cmd])

        ok, msg = wait_done(client)
        print(f"    결과: {msg}")

        # 4. 회전 후 자세
        after = read_pose(client)
        if after:
            print(f"\n[4] 회전 후 자세:")
            print(f"    X={after[0]:.2f}, Y={after[1]:.2f}, Z={after[2]:.2f}")
            print(f"    Rx={after[3]:.2f}, Ry={after[4]:.2f}, Rz={after[5]:.2f}")
            if before:
                print(f"\n[5] 변화량:")
                print(f"    dX={after[0]-before[0]:.2f}, dY={after[1]-before[1]:.2f}, dZ={after[2]-before[2]:.2f}")
                print(f"    dRx={after[3]-before[3]:.2f}, dRy={after[4]-before[4]:.2f}, dRz={after[5]-before[5]:.2f}")

        # 최종 레지스터 상태
        rr = client.read_holding_registers(address=REG_COMMAND, count=2)
        if not rr.isError():
            print(f"\n최종: command={rr.registers[0]}, status={rr.registers[1]}")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    print("사용법: python test_tf4_tool_rotate.py [축] [각도] [스케일] [TF]")
    print("예시:")
    print("  python test_tf4_tool_rotate.py rx 3.0 10 4   # TF4, Rx 3°, ×10")
    print("  python test_tf4_tool_rotate.py rx 3.0 1 4    # TF4, Rx 3°, ×1")
    print("  python test_tf4_tool_rotate.py rz 1.0 10 5   # TF5, Rz 1°, ×10")
    print()
    main()
