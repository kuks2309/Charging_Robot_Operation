#!/usr/bin/env python3
"""TF4 TCP 좌표 읽기 테스트"""

import time
import sys
sys.path.insert(0, '/home/argoon/Project/Charging_Robot_Operation/scripts')

from Robot.communication.modbus_client import ModbusClient

def main():
    print("로봇 연결 중...")
    robot = ModbusClient()
    success, msg = robot.connect()

    if not success:
        print(f"연결 실패: {msg}")
        return

    print(f"연결 성공: {msg}\n")

    try:
        # TF4 설정
        print("TF4 설정...")
        robot.send_set_toolframe(4, wait=True)
        time.sleep(0.3)

        # TCP 좌표 읽기
        tcp_pose = robot.read_current_pose()

        print("=" * 50)
        print("TF4 TCP 좌표")
        print("=" * 50)
        print(f"  X  = {tcp_pose[0]:.3f} mm")
        print(f"  Y  = {tcp_pose[1]:.3f} mm")
        print(f"  Z  = {tcp_pose[2]:.3f} mm")
        print(f"  Rx = {tcp_pose[3]:.3f} deg")
        print(f"  Ry = {tcp_pose[4]:.3f} deg")
        print(f"  Rz = {tcp_pose[5]:.3f} deg")
        print("=" * 50)

    finally:
        robot.disconnect()
        print("\n연결 해제됨")

if __name__ == "__main__":
    main()
