# -*- coding: utf-8 -*-
"""
Robot Tool Frame Investigation 모듈

목적: 로봇의 여러 Tool Frame (TF0, TF1, ...) 정보를 읽고 관계 분석
용도: 좌표계 변환을 위한 기초 자료

저장 위치: config/tf_config.json (프로젝트 루트 기준)
"""

import sys
import os
import json
from datetime import datetime

# 프로젝트 경로 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Robot.communication.modbus_client import ModbusClient


# =============================================================================
# 함수: read_pose_with_tf(robot, tf_num)
# =============================================================================
# 목적: 특정 Tool Frame으로 전환 후 TCP 읽기
# 입력: robot (ModbusClient), tf_num (int)
# 출력: (X, Y, Z, Rx, Ry, Rz) 또는 None
# =============================================================================
def read_pose_with_tf(robot, tf_num):
    """
    특정 Tool Frame으로 전환 후 TCP 읽기

    Args:
        robot: ModbusClient 인스턴스
        tf_num: Tool Frame 번호 (0, 1, 2, ...)

    Returns:
        (X, Y, Z, Rx, Ry, Rz) 또는 None
    """
    import time

    # Tool Frame 전환
    success, msg = robot.send_set_toolframe(tf_num, wait=False)
    if not success:
        print(f"  TF{tf_num} 전환 실패: {msg}")
        return None

    time.sleep(0.3)  # 전환 대기

    # TCP 읽기
    pose = robot.read_current_pose()
    return pose


# =============================================================================
# 함수: read_all_tool_frames(robot, tf_list)
# =============================================================================
# 목적: 모든 Tool Frame의 TCP 조회
# 입력: robot (ModbusClient), tf_list ([0, 1, 2, ...])
# 출력: dict {tf_num: pose}
# =============================================================================
def read_all_tool_frames(robot, tf_list):
    """
    모든 Tool Frame의 TCP 조회

    Args:
        robot: ModbusClient 인스턴스
        tf_list: Tool Frame 번호 리스트 [0, 1, 2, ...]

    Returns:
        dict: {tf_num: (X, Y, Z, Rx, Ry, Rz)}
    """
    tf_data = {}

    for tf_num in tf_list:
        print(f"  TF{tf_num} 조회 중...")
        pose = read_pose_with_tf(robot, tf_num)
        if pose:
            tf_data[tf_num] = pose
            print(f"    X={pose[0]:.2f}, Y={pose[1]:.2f}, Z={pose[2]:.2f}")
            print(f"    Rx={pose[3]:.2f}, Ry={pose[4]:.2f}, Rz={pose[5]:.2f}")
        else:
            print(f"    읽기 실패")

    return tf_data


# =============================================================================
# 함수: calculate_tf_offset(pose1, pose2)
# =============================================================================
# 목적: 두 Tool Frame 간 오프셋 계산
# 입력: pose1, pose2 (각각 6-tuple)
# 출력: (dX, dY, dZ, dRx, dRy, dRz)
# =============================================================================
def calculate_tf_offset(pose1, pose2):
    """
    두 Tool Frame 간 오프셋 계산

    Args:
        pose1: 기준 pose (X, Y, Z, Rx, Ry, Rz)
        pose2: 대상 pose (X, Y, Z, Rx, Ry, Rz)

    Returns:
        (dX, dY, dZ, dRx, dRy, dRz)
    """
    dx = pose2[0] - pose1[0]
    dy = pose2[1] - pose1[1]
    dz = pose2[2] - pose1[2]
    drx = pose2[3] - pose1[3]
    dry = pose2[4] - pose1[4]
    drz = pose2[5] - pose1[5]

    return (dx, dy, dz, drx, dry, drz)


# =============================================================================
# 함수: print_tf_report(tf_data)
# =============================================================================
# 목적: 조사 결과 콘솔 출력
# 입력: tf_data (dict)
# 출력: None
# =============================================================================
def print_tf_report(tf_data):
    """
    조사 결과 콘솔 출력

    Args:
        tf_data: {tf_num: (X, Y, Z, Rx, Ry, Rz)}
    """
    print("\n" + "=" * 60)
    print("Robot Tool Frame Investigation Report")
    print("=" * 60)

    # 각 TF 정보 출력
    for tf_num, pose in sorted(tf_data.items()):
        print(f"\n[TF{tf_num}]")
        print(f"  Position: X={pose[0]:.2f}, Y={pose[1]:.2f}, Z={pose[2]:.2f} mm")
        print(f"  Rotation: Rx={pose[3]:.2f}, Ry={pose[4]:.2f}, Rz={pose[5]:.2f} deg")

    # TF 간 오프셋 출력
    tf_nums = sorted(tf_data.keys())
    if len(tf_nums) >= 2:
        print("\n" + "-" * 60)
        print("Tool Frame Offsets")
        print("-" * 60)

        base_tf = tf_nums[0]
        for tf_num in tf_nums[1:]:
            offset = calculate_tf_offset(tf_data[base_tf], tf_data[tf_num])
            print(f"\n[TF{base_tf} → TF{tf_num}]")
            print(f"  dX={offset[0]:+.2f}, dY={offset[1]:+.2f}, dZ={offset[2]:+.2f} mm")
            print(f"  dRx={offset[3]:+.2f}, dRy={offset[4]:+.2f}, dRz={offset[5]:+.2f} deg")

    print("\n" + "=" * 60)


# =============================================================================
# 함수: save_tf_report(tf_data, filepath)
# =============================================================================
# 목적: 조사 결과 JSON 저장
# 입력: tf_data (dict), filepath (str)
# 출력: None
# =============================================================================
def save_tf_report(tf_data, filepath):
    """
    조사 결과 JSON 저장

    Args:
        tf_data: {tf_num: (X, Y, Z, Rx, Ry, Rz)}
        filepath: 저장 경로
    """
    # 저장용 데이터 구조 생성
    save_data = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "tool_frames": {},
        "offsets": {}
    }

    # Tool Frame 정보
    for tf_num, pose in tf_data.items():
        save_data["tool_frames"][f"TF{tf_num}"] = {
            "X": pose[0],
            "Y": pose[1],
            "Z": pose[2],
            "Rx": pose[3],
            "Ry": pose[4],
            "Rz": pose[5]
        }

    # 오프셋 계산
    tf_nums = sorted(tf_data.keys())
    if len(tf_nums) >= 2:
        base_tf = tf_nums[0]
        for tf_num in tf_nums[1:]:
            offset = calculate_tf_offset(tf_data[base_tf], tf_data[tf_num])
            save_data["offsets"][f"TF{base_tf}_to_TF{tf_num}"] = {
                "dX": offset[0],
                "dY": offset[1],
                "dZ": offset[2],
                "dRx": offset[3],
                "dRy": offset[4],
                "dRz": offset[5]
            }

    # 디렉토리 생성
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    # JSON 저장
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(save_data, f, indent=2, ensure_ascii=False)

    print(f"\n저장 완료: {filepath}")


# =============================================================================
# 함수: load_tf_config(filepath)
# =============================================================================
# 목적: 기존 JSON 파일에서 TF 설정 읽기
# 입력: filepath (str)
# 출력: dict 또는 None
# =============================================================================
def load_tf_config(filepath):
    """
    기존 JSON 파일에서 TF 설정 읽기

    Args:
        filepath: JSON 파일 경로

    Returns:
        dict: 저장된 데이터 또는 None
    """
    if not os.path.exists(filepath):
        return None

    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        print(f"파일 읽기 오류: {e}")
        return None


# =============================================================================
# 함수: compare_tf_data(current_data, saved_data)
# =============================================================================
# 목적: 현재 조회 결과와 기존 저장 데이터 비교
# 입력: current_data (dict), saved_data (dict)
# 출력: None (콘솔 출력)
# =============================================================================
def compare_tf_data(current_data, saved_data):
    """
    현재 조회 결과와 기존 저장 데이터 비교

    Args:
        current_data: 현재 조회한 {tf_num: (X, Y, Z, Rx, Ry, Rz)}
        saved_data: 기존 저장된 JSON 데이터
    """
    print("\n" + "=" * 60)
    print("Comparison: Current vs Saved")
    print("=" * 60)
    print(f"저장 시간: {saved_data.get('timestamp', 'N/A')}")

    saved_tfs = saved_data.get('tool_frames', {})

    for tf_num, current_pose in sorted(current_data.items()):
        tf_key = f"TF{tf_num}"
        saved_tf = saved_tfs.get(tf_key)

        print(f"\n[{tf_key}]")

        if saved_tf:
            saved_pose = (
                saved_tf['X'], saved_tf['Y'], saved_tf['Z'],
                saved_tf['Rx'], saved_tf['Ry'], saved_tf['Rz']
            )

            # 차이 계산
            diff = calculate_tf_offset(saved_pose, current_pose)

            print(f"  Current:  X={current_pose[0]:.2f}, Y={current_pose[1]:.2f}, Z={current_pose[2]:.2f} mm")
            print(f"            Rx={current_pose[3]:.2f}, Ry={current_pose[4]:.2f}, Rz={current_pose[5]:.2f} deg")
            print(f"  Saved:    X={saved_pose[0]:.2f}, Y={saved_pose[1]:.2f}, Z={saved_pose[2]:.2f} mm")
            print(f"            Rx={saved_pose[3]:.2f}, Ry={saved_pose[4]:.2f}, Rz={saved_pose[5]:.2f} deg")
            print(f"  Diff:     dX={diff[0]:+.2f}, dY={diff[1]:+.2f}, dZ={diff[2]:+.2f} mm")
            print(f"            dRx={diff[3]:+.2f}, dRy={diff[4]:+.2f}, dRz={diff[5]:+.2f} deg")
        else:
            print(f"  Current:  X={current_pose[0]:.2f}, Y={current_pose[1]:.2f}, Z={current_pose[2]:.2f} mm")
            print(f"            Rx={current_pose[3]:.2f}, Ry={current_pose[4]:.2f}, Rz={current_pose[5]:.2f} deg")
            print(f"  Saved:    (없음)")

    print("\n" + "=" * 60)


# =============================================================================
# 함수: main(compare, save)
# =============================================================================
# 목적: 1회 실행 (연결 → 조사 → 출력 → [비교] → [저장] → 종료)
# 입력: compare (bool) - 기존 파일과 비교 여부
#       save (bool) - 결과 저장 여부
# =============================================================================
def main(compare=False, save=False):
    """
    메인 실행 함수

    Args:
        compare: 기존 파일과 비교 여부 (기본값: False)
        save: 결과 저장 여부 (기본값: False)
    """
    print("=" * 60)
    print("Robot Tool Frame Investigation")
    print("=" * 60)

    # 설정
    ROBOT_IP = "192.168.0.29"
    ROBOT_PORT = 1502
    TF_LIST = [0, 1]  # 조사할 Tool Frame 목록
    SAVE_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'tf_config.json')

    # 로봇 연결
    print(f"\n로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    robot = ModbusClient(ip=ROBOT_IP, port=ROBOT_PORT)
    success, msg = robot.connect()

    if not success:
        print(f"연결 실패: {msg}")
        return

    print("연결 성공")

    try:
        # 모든 TF 조회
        print("\nTool Frame 조회 중...")
        tf_data = read_all_tool_frames(robot, TF_LIST)

        if tf_data:
            # 결과 출력
            print_tf_report(tf_data)

            # 기존 파일과 비교 (요청 시에만)
            if compare:
                saved_data = load_tf_config(SAVE_PATH)
                if saved_data:
                    compare_tf_data(tf_data, saved_data)
                else:
                    print("\n기존 저장 파일 없음 - 비교 생략")

            # 결과 저장 (요청 시에만)
            if save:
                save_tf_report(tf_data, SAVE_PATH)
        else:
            print("조회된 Tool Frame이 없습니다.")

    finally:
        # 연결 해제
        robot.disconnect()
        print("\n로봇 연결 해제")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Robot Tool Frame Investigation')
    parser.add_argument('--compare', '-c', action='store_true',
                        help='기존 저장 파일과 비교')
    parser.add_argument('--save', '-s', action='store_true',
                        help='결과를 JSON 파일로 저장')
    args = parser.parse_args()

    main(compare=args.compare, save=args.save)
