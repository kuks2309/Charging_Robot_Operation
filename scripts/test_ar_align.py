#!/usr/bin/env python3
"""
AR Tag 법선 정렬 테스트 스크립트
- 로봇만 이동
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from Robot.communication.modbus_client import ModbusClient


def main():
    print("=" * 60)
    print("AR Tag 법선 정렬 테스트 (로봇 이동)")
    print("=" * 60)

    # 로봇 연결
    robot = ModbusClient(ip="192.168.0.200", port=1502)
    success, msg = robot.connect()
    if not success:
        print(f"로봇 연결 실패: {msg}")
        return

    print(f"로봇 연결 성공")

    # Tool Frame 1 설정
    print("Tool Frame 1 설정 중...")
    success, msg = robot.send_set_toolframe(1, wait=False)
    time.sleep(0.5)
    print("Tool Frame 1 설정 완료")

    # 현재 로봇 포즈 읽기
    robot_pose = robot.read_current_pose()
    if robot_pose:
        print(f"\n현재 로봇 포즈 (Tool Frame 1):")
        print(f"  X={robot_pose[0]:.2f}, Y={robot_pose[1]:.2f}, Z={robot_pose[2]:.2f}")
        print(f"  Rx={robot_pose[3]:.2f}, Ry={robot_pose[4]:.2f}, Rz={robot_pose[5]:.2f}")

    # 이동할 거리 입력
    print("\n" + "=" * 60)
    print("Tool Frame 1 기준으로 이동합니다.")
    print("=" * 60)

    while True:
        print("\n명령어:")
        print("  x [mm]  - X축 이동 (예: x 10)")
        print("  y [mm]  - Y축 이동 (예: y -5)")
        print("  z [mm]  - Z축 이동 (예: z 20)")
        print("  xyz [x] [y] [z] - XYZ 동시 이동 (예: xyz 10 -5 0)")
        print("  p       - 현재 포즈 출력")
        print("  q       - 종료")

        user_input = input("\n입력: ").strip().lower()

        if user_input == 'q':
            break
        elif user_input == 'p':
            robot_pose = robot.read_current_pose()
            if robot_pose:
                print(f"현재 포즈: X={robot_pose[0]:.2f}, Y={robot_pose[1]:.2f}, Z={robot_pose[2]:.2f}")
                print(f"         Rx={robot_pose[3]:.2f}, Ry={robot_pose[4]:.2f}, Rz={robot_pose[5]:.2f}")
        elif user_input.startswith('x '):
            try:
                dist = float(user_input.split()[1])
                print(f"X축 {dist}mm 이동 중...")
                success, msg = robot.send_tcp_linear('x', dist)
                print(f"결과: {msg}")
            except:
                print("잘못된 입력")
        elif user_input.startswith('y '):
            try:
                dist = float(user_input.split()[1])
                print(f"Y축 {dist}mm 이동 중...")
                success, msg = robot.send_tcp_linear('y', dist)
                print(f"결과: {msg}")
            except:
                print("잘못된 입력")
        elif user_input.startswith('z '):
            try:
                dist = float(user_input.split()[1])
                print(f"Z축 {dist}mm 이동 중...")
                success, msg = robot.send_tcp_linear('z', dist)
                print(f"결과: {msg}")
            except:
                print("잘못된 입력")
        elif user_input.startswith('xyz '):
            try:
                parts = user_input.split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                print(f"XYZ ({x}, {y}, {z})mm 이동 중...")
                success, msg = robot.send_tcp_linear('xyz', (x, y, z))
                print(f"결과: {msg}")
            except:
                print("잘못된 입력 (예: xyz 10 -5 0)")
        else:
            print("알 수 없는 명령")

    # 정리
    robot.disconnect()
    print("\n테스트 종료")


if __name__ == "__main__":
    main()
