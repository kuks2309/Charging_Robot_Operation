#!/usr/bin/env python3
"""
Base 좌표계 절대 이동 테스트

Main_task 포트(502)를 사용하여 절대 좌표로 이동합니다.
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from Robot import ModbusClient


def read_current_pose_main_task(robot):
    """Main_task에서 현재 위치 읽기 (303-314 레지스터)"""
    import struct

    # X, Y, Z, Rx, Ry, Rz (각 2레지스터, float32)
    regs = robot.read_registers(303, 12)
    if not regs:
        return None

    def regs_to_float(r1, r2):
        byte_data = r1.to_bytes(2, 'big') + r2.to_bytes(2, 'big')
        return struct.unpack('>f', byte_data)[0]

    x = regs_to_float(regs[0], regs[1])
    y = regs_to_float(regs[2], regs[3])
    z = regs_to_float(regs[4], regs[5])
    rx = regs_to_float(regs[6], regs[7])
    ry = regs_to_float(regs[8], regs[9])
    rz = regs_to_float(regs[10], regs[11])

    return (x, y, z, rx, ry, rz)


def main():
    # 로봇 연결 (Main_task 포트 사용)
    ip = "192.168.0.29"
    port = 502  # Main_task.prs 포트 (직접 제어용)

    print(f"로봇 연결 중: {ip}:{port}")
    robot = ModbusClient(ip=ip, port=port, timeout=1.0)
    success, msg = robot.connect()
    print(f"  {msg}")

    if not success:
        print("로봇 연결 실패")
        return

    try:
        # 현재 상태 확인
        status = robot.read_status()
        print(f"\n현재 상태: {status} (0=Idle, 1=Running, 2=Done, 3=Error)")

        # Vision_task 포트로 현재 위치 읽기 (카메라 포즈)
        robot_vision = ModbusClient(ip=ip, port=1502, timeout=1.0)
        success_v, _ = robot_vision.connect()
        if success_v:
            pose = robot_vision.read_camera_pose()
            if pose:
                print(f"\n현재 카메라 포즈 (Vision_task에서 읽음):")
                print(f"  X: {pose[0]:.3f} mm")
                print(f"  Y: {pose[1]:.3f} mm")
                print(f"  Z: {pose[2]:.3f} mm")
                print(f"  Rx: {pose[3]:.2f} deg")
                print(f"  Ry: {pose[4]:.2f} deg")
                print(f"  Rz: {pose[5]:.2f} deg")
            robot_vision.disconnect()

        # 테스트: 절대 좌표계로 X축 이동
        print("\n" + "="*50)
        print("[테스트] 절대 좌표 모드로 X축 이동")
        print("="*50)

        if pose:
            # 절대 모드로 현재 X 위치로 이동 (사실상 제자리)
            target_x = pose[0]
            print(f"\n현재 X 위치로 절대 이동 테스트: {target_x:.3f} mm")

            # 모드 설정: 1 = 절대
            robot.write_register(robot.REGISTER_MODE, 1)
            print("  모드: 절대(1)")

            # X 값 쓰기 (float32)
            robot.write_float(robot.REGISTER_PARAM_X, target_x)
            print(f"  X 파라미터: {target_x:.3f} mm")

            # 명령 전송: LINEAR_X (10)
            robot.write_command(robot.CMD_TCP_LINEAR_X)
            print("  명령: LINEAR_X (10) 전송됨")

            # 완료 대기
            print("\n  완료 대기 중...")
            success, msg = robot.wait_for_done(timeout=10.0)
            print(f"  결과: {msg}")

            # 이동 후 위치 확인
            time.sleep(0.5)
            if success_v:
                robot_vision.connect()
                new_pose = robot_vision.read_camera_pose()
                if new_pose:
                    print(f"\n이동 후 카메라 포즈:")
                    print(f"  X: {new_pose[0]:.3f} mm (변화: {new_pose[0]-pose[0]:+.3f})")
                    print(f"  Y: {new_pose[1]:.3f} mm (변화: {new_pose[1]-pose[1]:+.3f})")
                    print(f"  Z: {new_pose[2]:.3f} mm (변화: {new_pose[2]-pose[2]:+.3f})")
                robot_vision.disconnect()

    finally:
        # 연결 해제
        robot.disconnect()
        print("\n로봇 연결 해제됨")


if __name__ == "__main__":
    main()
