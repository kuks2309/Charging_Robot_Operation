#!/usr/bin/env python3
"""Detection Pose 이동 테스트 — 분할 이동 + Rz=-179° 검증.

사용법:
    charging_robot/bin/python scripts/tests/test_rz_minus179.py
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from Robot.communication.modbus_client import ModbusClient

# Detection Pose (Rz=-179°)
TARGET_X = 459.67
TARGET_Y = 608.60
TARGET_Z = 612.79
TARGET_RX = 90.0
TARGET_RY = 0.0
TARGET_RZ = -179.0

ROBOT_IP = '192.168.0.39'
ROBOT_PORT = 1502


def main():
    print("=" * 60)
    print(f"Detection Pose 이동 테스트 (Rz={TARGET_RZ}°)")
    print(f"분할 이동 활성: MAX_STEP={ModbusClient.MOVEL_MAX_STEP_MM}mm")
    print("=" * 60)

    client = ModbusClient(ROBOT_IP, ROBOT_PORT)
    if not client.connect():
        print("로봇 연결 실패!")
        return

    try:
        # 1. TF5 설정
        print(f"\n[1] TF5 설정...")
        ok, msg = client.send_set_toolframe(5, wait=True)
        if not ok:
            print(f"    실패: {msg}")
            return
        print(f"    완료")
        time.sleep(0.3)

        # 2. 현재 포즈 읽기 (원점)
        origin = client.read_current_pose()
        if not origin:
            print("현재 포즈 읽기 실패!")
            return
        print(f"\n[2] 원점 (TF5):")
        print(f"    X={origin[0]:.2f} Y={origin[1]:.2f} Z={origin[2]:.2f}")
        print(f"    Rx={origin[3]:.2f} Ry={origin[4]:.2f} Rz={origin[5]:.2f}")

        import math
        dx = TARGET_X - origin[0]
        dy = TARGET_Y - origin[1]
        dz = TARGET_Z - origin[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        print(f"    목표까지 거리: {dist:.1f}mm")

        # 3. Detection Pose 이동
        print(f"\n[3] Detection Pose 이동 (Rz={TARGET_RZ}°)...")
        ok, msg = client.send_move_to_pose(
            TARGET_X, TARGET_Y, TARGET_Z,
            TARGET_RX, TARGET_RY, TARGET_RZ,
            wait=True
        )
        if ok:
            pose = client.read_current_pose()
            if pose:
                print(f"    성공! 도착: X={pose[0]:.2f} Y={pose[1]:.2f} Z={pose[2]:.2f}")
                print(f"              Rx={pose[3]:.2f} Ry={pose[4]:.2f} Rz={pose[5]:.2f}")
            else:
                print(f"    성공!")
        else:
            print(f"    *** 실패: {msg} ***")
            return

        # 4. 원점 복귀
        print(f"\n[4] 원점 복귀...")
        time.sleep(0.5)
        ok, msg = client.send_move_to_pose(
            origin[0], origin[1], origin[2],
            origin[3], origin[4], origin[5],
            wait=True
        )
        if ok:
            print(f"    복귀 완료!")
        else:
            print(f"    복귀 실패: {msg}")

    except KeyboardInterrupt:
        print("\n\n사용자 중단")
    except Exception as e:
        print(f"\n오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        client.disconnect()
        print("\n연결 해제")


if __name__ == '__main__':
    main()
