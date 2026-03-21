#!/usr/bin/env python3
"""단축 이동(transx/y/z) 정밀 검증 — 다른 축 변동 여부 확인.

로그에서 transx 명령인데 Y/Z도 크게 변하는 이상 현상 분석용.
각 축을 개별 이동 후 모든 축의 변화량을 정밀 측정.
"""
import sys, os, time, struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = '192.168.0.39'
ROBOT_PORT = 1502

REG_POSE = 158
REG_CMD = 351
REG_STATUS = 352


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


def read_status(c):
    """task_number, task_done 읽기"""
    rr = c.read_holding_registers(address=REG_CMD, count=2)
    if rr.isError():
        return None, None
    return rr.registers[0], rr.registers[1]


def wait_done(c, timeout=20):
    start = time.time()
    saw_running = False
    while time.time() - start < timeout:
        # status + task_number 동시 읽기
        rr = c.read_holding_registers(address=REG_CMD, count=2)
        if rr.isError():
            time.sleep(0.05)
            continue
        cmd, status = rr.registers[0], rr.registers[1]
        if status == 1:
            saw_running = True
        elif status == 2:
            return True, "완료"
        elif status == 3:
            return False, "로봇 오류"
        elif status == 0:
            if saw_running:
                return True, "완료(IDLE)"
            if cmd == 0 and time.time() - start > 0.3:
                return True, "완료(고속)"  # TF 전환 등 빠른 명령
        time.sleep(0.05)
    return False, "타임아웃"


def send_single_axis(c, axis, delta_mm):
    """단축 이동 — 레지스터 직접 기록."""
    reg_map = {'x': (301, 50), 'y': (302, 51), 'z': (303, 52)}
    reg, cmd = reg_map[axis]

    # 이동 전 상태 확인
    cmd_num, cmd_done = read_status(c)
    if cmd_num != 0:
        print(f"  ⚠ 이동 전 task_number={cmd_num} (dirty!) — 리셋")
        c.write_register(REG_CMD, 0)
        time.sleep(0.2)  # PRS 1사이클(50ms) 대기

    # 이동 전 포즈
    before = read_pose(c)

    # 레지스터 기록 + 명령 전송
    val = int(round(delta_mm * 10))
    if val < 0:
        val += 65536
    c.write_register(reg, val)
    time.sleep(0.05)  # 레지스터 안정화
    c.write_register(REG_CMD, cmd)

    # 완료 대기
    ok, msg = wait_done(c)

    # 이동 후 포즈
    time.sleep(0.1)
    after = read_pose(c)

    return before, after, ok, msg


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

    # TF4 설정 (CMD 45 = toolframe(4))
    rr = c.read_holding_registers(address=219, count=1)
    tf = rr.registers[0] if not rr.isError() else '?'
    print(f"현재 TF: {tf}")
    if tf != 4:
        print(f"TF4로 전환 중...")
        c.write_register(REG_CMD, 45)  # CMD 45 = toolframe(4)
        ok, msg = wait_done(c)
        if not ok:
            print(f"TF4 전환 실패: {msg}")
            c.close()
            return
        time.sleep(0.3)
        rr = c.read_holding_registers(address=219, count=1)
        tf = rr.registers[0] if not rr.isError() else '?'
    print(f"TF: {tf}")
    print(f"현재: X={pose[0]:.2f} Y={pose[1]:.2f} Z={pose[2]:.2f} Rx={pose[3]:.2f} Ry={pose[4]:.2f} Rz={pose[5]:.2f}")
    print()

    tests = [
        ('z', +10.0, "Z +10mm"),
        ('z', -10.0, "Z -10mm (복귀)"),
        ('y', +10.0, "Y +10mm"),
        ('y', -10.0, "Y -10mm (복귀)"),
        ('x', +10.0, "X +10mm"),
        ('x', -10.0, "X -10mm (복귀)"),
    ]

    print(f"{'테스트':<20} {'결과':>4}  {'dX':>8} {'dY':>8} {'dZ':>8} {'dRx':>8} {'dRy':>8} {'dRz':>8}  이상")
    print("=" * 100)

    for axis, delta, desc in tests:
        before, after, ok, msg = send_single_axis(c, axis, delta)
        if not ok or not before or not after:
            print(f"{desc:<20} FAIL  {msg}")
            print("  *** 보호정지 — 테스트 중단 ***")
            break

        dx = after[0] - before[0]
        dy = after[1] - before[1]
        dz = after[2] - before[2]
        drx = after[3] - before[3]
        dry = after[4] - before[4]
        drz = after[5] - before[5]

        # 이상 판정: 명령한 축 외에 0.5mm/° 이상 변동
        anomalies = []
        if axis != 'x' and abs(dx) > 0.5: anomalies.append(f"X={dx:+.1f}")
        if axis != 'y' and abs(dy) > 0.5: anomalies.append(f"Y={dy:+.1f}")
        if axis != 'z' and abs(dz) > 0.5: anomalies.append(f"Z={dz:+.1f}")
        if abs(drx) > 0.5: anomalies.append(f"Rx={drx:+.1f}")
        if abs(dry) > 0.5: anomalies.append(f"Ry={dry:+.1f}")
        if abs(drz) > 0.5: anomalies.append(f"Rz={drz:+.1f}")

        anomaly_str = ", ".join(anomalies) if anomalies else "정상"
        print(f"{desc:<20} {'OK':>4}  {dx:>+8.2f} {dy:>+8.2f} {dz:>+8.2f} {drx:>+8.2f} {dry:>+8.2f} {drz:>+8.2f}  {anomaly_str}")

        time.sleep(0.3)

    print()
    pose = read_pose(c)
    if pose:
        print(f"최종: X={pose[0]:.2f} Y={pose[1]:.2f} Z={pose[2]:.2f} Rx={pose[3]:.2f} Ry={pose[4]:.2f} Rz={pose[5]:.2f}")

    c.close()


if __name__ == '__main__':
    main()
