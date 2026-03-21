#!/usr/bin/env python3
"""레이저 캘리브레이션 자동 스캔 재현: Rz=180°에서 Z 2mm 간격 70mm 이동."""
import sys, os, time, struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = '192.168.0.39'
ROBOT_PORT = 1502

REG_POSE = 158
REG_CMD = 351
REG_STATUS = 352
CMD_BASE_TRANS_Z = 52
CMD_BASE_ROT_Z = 56

Z_STEP_MM = -2.0   # 2mm 하강
Z_TOTAL_MM = 70.0   # 총 70mm
Z_STEPS = int(Z_TOTAL_MM / abs(Z_STEP_MM))  # 35 스텝


def read_pose(c):
    rr = c.read_holding_registers(address=REG_POSE, count=12)
    if rr.isError():
        return None
    pose = []
    for i in range(6):
        low, high = rr.registers[i*2], rr.registers[i*2+1]
        val = struct.unpack('<f', struct.pack('<HH', low, high))[0]
        pose.append(val)
    return pose


def wait_done(c, timeout=15):
    start = time.time()
    saw_running = False
    while time.time() - start < timeout:
        rr = c.read_holding_registers(address=REG_STATUS, count=1)
        if rr.isError():
            time.sleep(0.1)
            continue
        s = rr.registers[0]
        if s == 1:
            saw_running = True
        elif s == 2:
            return True
        elif s == 3:
            return False
        elif s == 0 and saw_running:
            return True
        time.sleep(0.1)
    return False


def move_z(c, delta_mm):
    val = int(round(delta_mm * 10))
    if val < 0:
        val = val + 65536
    c.write_register(303, val)
    c.write_register(REG_CMD, CMD_BASE_TRANS_Z)
    return wait_done(c)


def move_rz(c, delta_deg):
    val = int(round(delta_deg * 10))
    if val < 0:
        val = val + 65536
    c.write_register(306, val)
    c.write_register(REG_CMD, CMD_BASE_ROT_Z)
    return wait_done(c)


def main():
    c = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=3)
    if not c.connect():
        print("연결 실패")
        return

    pose = read_pose(c)
    if not pose:
        print("포즈 읽기 실패")
        c.close()
        return

    print(f"현재: X={pose[0]:.1f} Y={pose[1]:.1f} Z={pose[2]:.1f} Rx={pose[3]:.1f} Ry={pose[4]:.1f} Rz={pose[5]:.1f}")

    # Rz를 180°로 맞추기 (현재값과의 차이만큼 회전)
    current_rz = pose[5]
    target_rz = 180.0
    rz_delta = target_rz - current_rz
    while rz_delta > 180: rz_delta -= 360
    while rz_delta < -180: rz_delta += 360

    if abs(rz_delta) > 0.5:
        print(f"\nRz 보정: {current_rz:.1f}° → 180.0° (delta={rz_delta:+.1f}°)")
        ok = move_rz(c, rz_delta)
        if not ok:
            print("Rz 보정 실패!")
            c.close()
            return
        pose = read_pose(c)
        print(f"보정 후: Rz={pose[5]:.1f}°")
    else:
        print(f"Rz={current_rz:.1f}° — 이미 180° 근처")

    # Z 2mm 간격 70mm 하강
    print(f"\n=== Z 스캔 시작: {Z_STEP_MM}mm × {Z_STEPS}스텝 = {Z_TOTAL_MM}mm ===")
    print("=" * 70)

    total = 0
    for i in range(Z_STEPS):
        total += Z_STEP_MM
        print(f"스텝 {i+1:>2}/{Z_STEPS}: Z {Z_STEP_MM:+.1f}mm (누적 {total:+.1f}mm)...", end=" ", flush=True)

        ok = move_z(c, Z_STEP_MM)
        pose = read_pose(c)

        if ok and pose:
            print(f"Z={pose[2]:.1f} Rz={pose[5]:.1f}° — OK")
        else:
            print(f"*** 보호정지! ***")
            if pose:
                print(f"  정지: X={pose[0]:.1f} Y={pose[1]:.1f} Z={pose[2]:.1f} Rz={pose[5]:.1f}°")
            print(f"  누적: {total:+.1f}mm (스텝 {i+1})")
            break
        time.sleep(0.2)
    else:
        print(f"\n{Z_STEPS}스텝 완료 — 보호정지 없음!")

    c.close()


if __name__ == '__main__':
    main()
