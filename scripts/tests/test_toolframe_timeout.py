#!/usr/bin/env python3
"""
Tool Frame 설정 타임아웃 진단 테스트

증상: 연결 후 Tool Frame 설정 시 "타임아웃" (wait_for_done phase 2)
목적: 레지스터 상태를 실시간 추적하여 PRS 응답 여부 확인
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from Robot.communication.modbus_client import ModbusClient

IP = "192.168.0.39"
PORT = 1502


def read_all_status(robot: ModbusClient):
    """현재 레지스터 상태 스냅샷"""
    status = robot.read_status()
    cmd = robot.read_command()
    tf = robot.read_current_toolframe()
    status_name = {0: "IDLE", 1: "RUNNING", 2: "DONE", 3: "ERROR"}.get(status, f"UNKNOWN({status})")
    return status, cmd, tf, status_name


def test_register_state(robot: ModbusClient):
    """1단계: 연결 직후 레지스터 상태 확인"""
    print("=" * 60)
    print("[1] 현재 레지스터 상태 확인")
    print("=" * 60)
    status, cmd, tf, status_name = read_all_status(robot)
    print(f"  Status(352) = {status} ({status_name})")
    print(f"  Command(351) = {cmd}")
    print(f"  ToolFrame(219) = {tf}")
    print()

    if status != 0:
        print(f"  ⚠ Status가 IDLE(0)이 아님! PRS가 이전 명령을 처리 중이거나 정리 안 됨")
        print(f"  → PRS 스크립트가 실행 중인지 확인 필요")
    if cmd != 0:
        print(f"  ⚠ Command가 0이 아님! 이전 명령({cmd})이 남아있음")
    return status, cmd, tf


def test_write_command_polling(robot: ModbusClient, target_tf: int = 4):
    """2단계: TF 명령 직접 쓰고 레지스터 변화 관찰"""
    tf_cmd_map = {0: 40, 1: 41, 2: 42, 3: 43, 4: 45, 5: 46}
    cmd_value = tf_cmd_map[target_tf]

    print("=" * 60)
    print(f"[2] TF{target_tf} 명령(CMD={cmd_value}) 쓰기 + 레지스터 폴링")
    print("=" * 60)

    # write_command의 pre-flight 체크를 우회하지 않고 그대로 사용
    print(f"  write_command({cmd_value}) 호출...")
    t0 = time.time()
    ok = robot.write_command(cmd_value)
    t_write = time.time() - t0
    print(f"  write_command 결과: {ok} ({t_write*1000:.0f}ms)")

    if not ok:
        print("  ✗ 명령 쓰기 실패 — Modbus 통신 문제")
        return False

    # 10초간 20ms 간격 폴링
    print(f"\n  10초간 폴링 시작 (20ms 간격):")
    print(f"  {'시간':>8s}  {'Status':>10s}  {'Command':>10s}  {'TF':>5s}")
    print(f"  {'-'*8}  {'-'*10}  {'-'*10}  {'-'*5}")

    saw_running = False
    saw_done = False
    t_start = time.time()

    while time.time() - t_start < 10.0:
        elapsed = time.time() - t_start
        status, cmd, tf, status_name = read_all_status(robot)
        print(f"  {elapsed:7.3f}s  {status_name:>10s}  {cmd:>10d}  TF{tf}")

        if status == 1:  # RUNNING
            saw_running = True
        if status == 2:  # DONE
            saw_done = True
        if status == 0 and (saw_running or saw_done):
            print(f"\n  ✓ 완료! IDLE 복귀 ({elapsed:.3f}s)")
            break
        if status == 2:
            print(f"\n  ✓ DONE 감지 ({elapsed:.3f}s)")
            # PRS cleanup 대기 (DONE→IDLE)
            time.sleep(0.5)
            status2, cmd2, tf2, sname2 = read_all_status(robot)
            print(f"  cleanup 후: {sname2}, CMD={cmd2}, TF={tf2}")
            break
        if status == 3:
            print(f"\n  ✗ ERROR 발생!")
            break

        time.sleep(0.02)
    else:
        print(f"\n  ✗ 10초 타임아웃 — PRS가 명령을 처리하지 않음")
        print(f"    saw_running={saw_running}, saw_done={saw_done}")

    # 최종 상태
    print(f"\n  최종 TF: {robot.read_current_toolframe()}")
    return saw_running or saw_done


def test_manual_status_clear(robot: ModbusClient):
    """3단계: Status 레지스터 수동 클리어 후 재시도"""
    status = robot.read_status()
    if status != 0:
        print("=" * 60)
        print(f"[3] Status={status} (비정상) → 수동 클리어 시도")
        print("=" * 60)
        robot.write_response(0)  # Status=0 (IDLE) 강제 쓰기
        time.sleep(0.1)
        new_status = robot.read_status()
        print(f"  클리어 후 Status: {new_status}")
        if new_status == 0:
            print("  → Status IDLE 복구 완료, TF 명령 재시도 가능")
        return new_status == 0
    return True


def main():
    print(f"로봇 연결: {IP}:{PORT}")
    robot = ModbusClient(ip=IP, port=PORT, timeout=1.0)
    success, msg = robot.connect()
    if not success:
        print(f"연결 실패: {msg}")
        return

    print(f"연결 성공\n")

    # 1단계: 현재 상태 확인
    status, cmd, tf = test_register_state(robot)

    # 2단계: TF 명령 쓰기 + 폴링
    result = test_write_command_polling(robot, target_tf=4)

    if not result:
        # 3단계: Status 클리어 후 재시도
        if test_manual_status_clear(robot):
            print("\n재시도:")
            test_write_command_polling(robot, target_tf=4)

    # 연결 해제
    robot.disconnect()
    print("\n테스트 완료")


if __name__ == "__main__":
    main()
