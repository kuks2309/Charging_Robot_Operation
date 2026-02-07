#!/usr/bin/env python3
"""CSV 저장 테스트 - TF4 좌표만 저장"""

import os
import csv
import time
import sys
sys.path.insert(0, '/home/argoon/Project/Charging_Robot_Operation/scripts')

from Robot.communication.modbus_client import ModbusClient
from datetime import datetime

def test_csv_save():
    print("로봇 연결 중...")
    robot = ModbusClient()
    success, msg = robot.connect()

    if not success:
        print(f"연결 실패: {msg}")
        return

    print(f"연결 성공: {msg}\n")

    try:
        # TF4 설정 및 좌표 읽기
        print("TF4 설정...")
        robot.send_set_toolframe(4, wait=True)
        time.sleep(0.3)

        tcp_pose = robot.read_current_pose()
        print(f"TCP 좌표: X={tcp_pose[0]:.2f}, Y={tcp_pose[1]:.2f}, Z={tcp_pose[2]:.2f}")
        print(f"          Rx={tcp_pose[3]:.2f}, Ry={tcp_pose[4]:.2f}, Rz={tcp_pose[5]:.2f}")

        # 테스트 CSV 저장
        test_dir = "/home/argoon/Project/Charging_Robot_Operation/calibration/test_csv"
        os.makedirs(test_dir, exist_ok=True)
        csv_path = os.path.join(test_dir, "test_hand_eye.csv")

        headers = [
            "timestamp", "index", "image_filename",
            "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
            "chessboard_detected",
            "rvec_x", "rvec_y", "rvec_z",
            "tvec_x", "tvec_y", "tvec_z"
        ]

        # 헤더 쓰기
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(headers)

        # 데이터 행 쓰기
        row = [
            datetime.now().isoformat(),
            0,
            "test_0000.png",
            tcp_pose[0], tcp_pose[1], tcp_pose[2],
            tcp_pose[3], tcp_pose[4], tcp_pose[5],
            True,
            0.1, 0.2, 0.3,
            100.0, 50.0, 500.0
        ]

        with open(csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(row)

        print(f"\n저장 완료: {csv_path}")

        # 저장된 내용 확인
        print("\n" + "=" * 60)
        print("저장된 CSV 내용:")
        print("=" * 60)
        with open(csv_path, 'r') as f:
            print(f.read())

    finally:
        robot.disconnect()
        print("\n연결 해제됨")


if __name__ == "__main__":
    test_csv_save()
