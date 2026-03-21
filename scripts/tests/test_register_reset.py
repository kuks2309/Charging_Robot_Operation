#!/usr/bin/env python3
"""
PRS stuck 상태 수동 복구 + TF 명령 재시도

진단 결과: Status=RUNNING(1), CMD=52(transz) stuck
→ 레지스터 클리어(CMD=0, Status=0) 후 TF 명령 재시도
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from Robot.communication.modbus_client import ModbusClient

IP = "192.168.0.39"
PORT = 1502


def show_state(robot, label=""):
    status = robot.read_status()
    cmd = robot.read_command()
    tf = robot.read_current_toolframe()
    sname = {0: "IDLE", 1: "RUNNING", 2: "DONE", 3: "ERROR"}.get(status, f"?({status})")
    print(f"  {label:20s} Status={status}({sname})  CMD={cmd}  TF={tf}")
    return status, cmd, tf


def main():
    robot = ModbusClient(ip=IP, port=PORT, timeout=1.0)
    success, msg = robot.connect()
    if not success:
        print(f"연결 실패: {msg}")
        return
    print("연결 성공\n")

    # 현재 상태
    print("[1] 현재 상태:")
    show_state(robot, "before")

    # 레지스터 강제 클리어
    print("\n[2] 레지스터 강제 클리어:")
    print("  CMD(351) ← 0")
    robot.write_register(robot.REGISTER_COMMAND, 0)
    time.sleep(0.05)
    print("  Status(352) ← 0 (IDLE)")
    robot.write_register(robot.REGISTER_STATUS, 0)
    time.sleep(0.2)

    show_state(robot, "after clear")

    # PRS가 다시 덮어쓰는지 1초 관찰
    print("\n[3] 1초간 안정성 확인 (PRS 덮어쓰기 감시):")
    t0 = time.time()
    stable = True
    while time.time() - t0 < 1.0:
        status, cmd, tf = show_state(robot, f"{time.time()-t0:.2f}s")
        if status != 0 or cmd != 0:
            print(f"  ⚠ PRS가 다시 덮어씀!")
            stable = False
            break
        time.sleep(0.1)

    if not stable:
        print("\n  PRS 스크립트가 계속 CMD/Status를 덮어쓰고 있음")
        print("  → PRS 스크립트 재시작 필요 (로봇 컨트롤러에서)")
        robot.disconnect()
        return

    # TF4 명령 재시도 (send_set_toolframe 사용)
    print(f"\n[4] TF4 설정 재시도 (send_set_toolframe):")
    t0 = time.time()
    success, msg = robot.send_set_toolframe(4, wait=True)
    elapsed = time.time() - t0
    print(f"  결과: success={success}, msg='{msg}' ({elapsed:.2f}s)")
    show_state(robot, "after TF4")

    if success:
        # TF3도 테스트
        print(f"\n[5] TF3 설정 테스트:")
        time.sleep(0.3)
        t0 = time.time()
        success2, msg2 = robot.send_set_toolframe(3, wait=True)
        elapsed2 = time.time() - t0
        print(f"  결과: success={success2}, msg='{msg2}' ({elapsed2:.2f}s)")
        show_state(robot, "after TF3")

    robot.disconnect()
    print("\n완료")


def quick_check():
    """PRS 재시작 후 간단 확인용"""
    robot = ModbusClient(ip=IP, port=PORT, timeout=1.0)
    success, msg = robot.connect()
    if not success:
        print(f"연결 실패: {msg}")
        return
    print("연결 성공\n")

    show_state(robot, "현재 상태")

    # TF4 → TF3 왕복 테스트
    for tf in [4, 3]:
        time.sleep(0.3)
        t0 = time.time()
        ok, msg = robot.send_set_toolframe(tf, wait=True)
        elapsed = time.time() - t0
        status, cmd, cur_tf = show_state(robot, f"TF{tf} 설정")
        result = "✓" if ok else "✗"
        print(f"  {result} TF{tf}: {msg} ({elapsed:.2f}s)\n")

    robot.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true", help="PRS 재시작 후 빠른 확인")
    args = parser.parse_args()

    if args.quick:
        quick_check()
    else:
        main()
