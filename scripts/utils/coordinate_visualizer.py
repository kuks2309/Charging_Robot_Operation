#!/usr/bin/env python3
"""
3D 좌표계 시각화 도구

ArUco 마커(카메라), Vision TCP, 로봇 베이스 좌표계를
3D 그래프에 동시 표시하여 좌표 변환을 시각적으로 이해.

사용법:
  실시간: python scripts/utils/coordinate_visualizer.py
  CSV:    python scripts/utils/coordinate_visualizer.py --csv data/dual_aruco_xxx.csv
"""

import os
import sys
import csv
import argparse
import numpy as np
import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 프로젝트 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(SCRIPT_DIR, '..')
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPTS_DIR, '..'))
sys.path.insert(0, SCRIPTS_DIR)

from utils.ar_to_base_tf import (
    euler_to_rotation_matrix, camera_to_vision, vision_to_base, ar_to_base
)


def draw_frame(ax, origin, R, label, scale=50.0, linewidth=2):
    """
    3D 좌표 프레임 그리기 (X:빨강, Y:초록, Z:파랑)

    Args:
        ax: matplotlib 3D axes
        origin: (3,) 원점 위치 (mm)
        R: (3,3) 회전 행렬
        label: 라벨 텍스트
        scale: 축 길이 (mm)
    """
    colors = ['r', 'g', 'b']
    axis_labels = ['X', 'Y', 'Z']

    for i in range(3):
        direction = R[:, i] * scale
        ax.quiver(
            origin[0], origin[1], origin[2],
            direction[0], direction[1], direction[2],
            color=colors[i], linewidth=linewidth, arrow_length_ratio=0.15
        )

    ax.text(origin[0], origin[1], origin[2] + scale * 0.3, label,
            fontsize=18, fontweight='bold', ha='center')


def draw_connection(ax, p1, p2, color='gray', linestyle='--', linewidth=1):
    """두 점 사이 점선 연결"""
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
            color=color, linestyle=linestyle, linewidth=linewidth, alpha=0.5)


def visualize_frames(robot_pose, markers_cam, title="Coordinate Frames"):
    """
    좌표계 시각화 (정적)

    Args:
        robot_pose: (X, Y, Z, Rx, Ry, Rz) mm/deg - 로봇 TCP
        markers_cam: list of dict {'tag_id', 'tvec'(m), 'rvec'(rad)} - 카메라 좌표계
        title: 그래프 제목
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 1. Robot Base Frame (원점)
    R_base = np.eye(3)
    origin_base = np.array([0, 0, 0])
    draw_frame(ax, origin_base, R_base, "Base", scale=80, linewidth=3)

    # 2. TCP Frame (로봇 자세)
    tcp_pos = np.array([robot_pose[0], robot_pose[1], robot_pose[2]])  # mm
    R_tcp = euler_to_rotation_matrix(robot_pose[3], robot_pose[4], robot_pose[5])
    draw_frame(ax, tcp_pos, R_tcp, "TCP (TF5)", scale=50, linewidth=2)
    draw_connection(ax, origin_base, tcp_pos, color='orange', linestyle='-', linewidth=1.5)

    # 3. ArUco Markers - 3가지 좌표계로 표시
    marker_colors_cam = ['cyan', 'magenta']
    marker_colors_base = ['darkblue', 'darkred']

    for idx, m in enumerate(markers_cam):
        tvec = np.array(m['tvec'])  # meters
        rvec = np.array(m['rvec'])  # Rodrigues
        tag_id = m.get('tag_id', idx)
        color_idx = idx % 2

        # (a) Camera 좌표계에서의 마커 (TCP 위치 기준으로 표시)
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        cam_pos_mm = tvec * 1000.0  # m -> mm
        # Camera frame에서의 위치를 TCP 기준으로 변환하여 표시
        marker_in_tcp = tcp_pos + R_tcp @ np.array([1, -1, -1]) * cam_pos_mm
        # 참고용 점으로 표시
        ax.scatter(*marker_in_tcp, color=marker_colors_cam[color_idx], s=30, alpha=0.5)

        # (b) Camera → Vision 변환
        vision_tvec, vision_rvec = camera_to_vision(tvec, rvec)

        # (c) Vision → Base 변환
        base_tvec, base_rvec = vision_to_base(vision_tvec, vision_rvec, robot_pose)
        R_base_marker, _ = cv2.Rodrigues(base_rvec)
        base_pos_mm = base_tvec * 1000.0

        draw_frame(ax, base_pos_mm, R_base_marker,
                   f"Marker {tag_id}\n(Base)", scale=30, linewidth=2)

        # TCP → Marker 연결선
        draw_connection(ax, tcp_pos, base_pos_mm,
                        color=marker_colors_base[color_idx], linestyle='--')

        # 정보 텍스트
        info = (f"ID{tag_id} Base: "
                f"[{base_pos_mm[0]:.1f}, {base_pos_mm[1]:.1f}, {base_pos_mm[2]:.1f}]mm")
        ax.text(base_pos_mm[0], base_pos_mm[1], base_pos_mm[2] - 20,
                info, fontsize=14, color=marker_colors_base[color_idx])

    # 축 설정
    ax.set_xlabel('X (mm)', fontsize=22)
    ax.set_ylabel('Y (mm)', fontsize=22)
    ax.set_zlabel('Z (mm)', fontsize=22)
    ax.set_title(title, fontsize=26)

    # 범위 자동 조정
    all_pts = [origin_base, tcp_pos]
    for m in markers_cam:
        base_result = ar_to_base(m['tvec'], m['rvec'], robot_pose)
        all_pts.append(np.array([base_result[0], base_result[1], base_result[2]]))

    all_pts = np.array(all_pts)
    center = all_pts.mean(axis=0)
    max_range = max(np.ptp(all_pts, axis=0)) * 0.7
    if max_range < 100:
        max_range = 200

    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)

    # 범례
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='r', linewidth=2, label='X axis'),
        Line2D([0], [0], color='g', linewidth=2, label='Y axis'),
        Line2D([0], [0], color='b', linewidth=2, label='Z axis'),
        Line2D([0], [0], color='orange', linewidth=1.5, label='Base → TCP'),
        Line2D([0], [0], color='gray', linestyle='--', label='TCP → Marker'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=16)

    # 줌 컨트롤 (+/- 키)
    def on_key(event):
        if event.key in ['+', '=']:
            ax.set_xlim(ax.get_xlim()[0] * 0.8, ax.get_xlim()[1] * 0.8)
            ax.set_ylim(ax.get_ylim()[0] * 0.8, ax.get_ylim()[1] * 0.8)
            ax.set_zlim(ax.get_zlim()[0] * 0.8, ax.get_zlim()[1] * 0.8)
            fig.canvas.draw_idle()
        elif event.key == '-':
            ax.set_xlim(ax.get_xlim()[0] * 1.25, ax.get_xlim()[1] * 1.25)
            ax.set_ylim(ax.get_ylim()[0] * 1.25, ax.get_ylim()[1] * 1.25)
            ax.set_zlim(ax.get_zlim()[0] * 1.25, ax.get_zlim()[1] * 1.25)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', on_key)

    fig.text(0.5, 0.01, 'Drag: rotate | Scroll: zoom | +/-: zoom in/out',
             ha='center', fontsize=18, color='gray')

    plt.tight_layout()
    return fig, ax


def load_csv_sample(csv_path, sample_idx=0, tvec_scale=1.0):
    """
    CSV 파일에서 샘플 로드

    Args:
        csv_path: CSV 파일 경로
        sample_idx: 샘플 인덱스
        tvec_scale: tvec 보정 비율 (예: 0.015/0.04 = 0.375)

    Returns:
        robot_pose: (X, Y, Z, Rx, Ry, Rz)
        markers_cam: list of marker dicts
    """
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("CSV 파일이 비어있습니다.")
        return None, None

    if sample_idx >= len(rows):
        sample_idx = len(rows) - 1

    row = rows[sample_idx]

    # 로봇 자세
    robot_pose = (
        float(row.get('robot_x', 0) or 0),
        float(row.get('robot_y', 0) or 0),
        float(row.get('robot_z', 0) or 0),
        float(row.get('robot_rx', 0) or 0),
        float(row.get('robot_ry', 0) or 0),
        float(row.get('robot_rz', 0) or 0),
    )

    markers = []
    for prefix in ['m1', 'm2']:
        tag_id_key = f'{prefix}_tag_id'
        if tag_id_key in row and row[tag_id_key]:
            tvec = np.array([
                float(row[f'{prefix}_tvec_x']),
                float(row[f'{prefix}_tvec_y']),
                float(row[f'{prefix}_tvec_z']),
            ]) * tvec_scale
            markers.append({
                'tag_id': int(float(row[tag_id_key])),
                'tvec': tvec,
                'rvec': np.array([
                    float(row[f'{prefix}_rvec_x']),
                    float(row[f'{prefix}_rvec_y']),
                    float(row[f'{prefix}_rvec_z']),
                ]),
            })

    return robot_pose, markers


def run_csv_mode(csv_path, sample_idx=0, tvec_scale=1.0):
    """CSV 모드: 저장된 데이터로 정적 시각화"""
    print(f"CSV 로드: {csv_path}")
    if tvec_scale != 1.0:
        print(f"tvec 보정 비율: {tvec_scale:.4f}")

    robot_pose, markers = load_csv_sample(csv_path, sample_idx, tvec_scale)
    if robot_pose is None:
        return

    print(f"Robot TCP: X={robot_pose[0]:.1f}, Y={robot_pose[1]:.1f}, Z={robot_pose[2]:.1f}, "
          f"Rx={robot_pose[3]:.1f}, Ry={robot_pose[4]:.1f}, Rz={robot_pose[5]:.1f}")

    for m in markers:
        base = ar_to_base(m['tvec'], m['rvec'], robot_pose)
        print(f"Marker {m['tag_id']}: "
              f"Camera tvec={m['tvec']*1000} mm | "
              f"Base=[{base[0]:.1f}, {base[1]:.1f}, {base[2]:.1f}]mm "
              f"[{base[3]:.1f}, {base[4]:.1f}, {base[5]:.1f}]deg")

    fig, ax = visualize_frames(
        robot_pose, markers,
        title=f"Coordinate Frames (CSV sample #{sample_idx})"
    )
    plt.show()


def run_realtime_mode():
    """실시간 모드: ArduCam + Modbus로 실시간 업데이트"""
    from Robot.communication.modbus_client import ModbusClient

    CALIB_FILE = os.path.join(PROJECT_ROOT, 'config', 'arducam_calibration.yaml')
    from utils.camera_utils import detect_arducam_index
    _detected = detect_arducam_index()
    DEVICE_INDEX = _detected if _detected is not None else 6  # fallback: 6 (기존 동작 보존)
    MARKER_SIZE = 0.015  # m (15mm)

    import yaml
    with open(CALIB_FILE, 'r') as f:
        data = yaml.safe_load(f)
    cam_data = data['camera_matrix']['data']
    camera_matrix = np.array(cam_data, dtype=np.float64).reshape(3, 3)
    dist_data = data['distortion_coefficients']['data']
    dist_coeffs = np.array(dist_data, dtype=np.float64)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    obj_points = np.array([
        [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    ], dtype=np.float32)

    # Modbus
    robot = ModbusClient()
    success, msg = robot.connect()
    print(f"Robot: {msg}")

    # ArduCam
    cap = cv2.VideoCapture(DEVICE_INDEX)
    if not cap.isOpened():
        print(f"ArduCam 열기 실패 (device={DEVICE_INDEX})")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print("\n실시간 3D 시각화 시작 (창 닫기로 종료)")

    plt.ion()
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    fig.canvas.manager.set_window_title('3D Coordinate Frames (Realtime)')
    plt.show(block=False)

    try:
        while plt.fignum_exists(fig.number):
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            markers = []
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    success_pnp, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i].reshape(-1, 2),
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if not success_pnp:
                        continue
                    rvec, tvec = cv2.solvePnPRefineLM(
                        obj_points, corners[i].reshape(-1, 2),
                        camera_matrix, dist_coeffs, rvec, tvec
                    )
                    markers.append({
                        'tag_id': int(marker_id),
                        'tvec': tvec.flatten(),
                        'rvec': rvec.flatten(),
                    })

            robot_pose = (0, 0, 0, 0, 0, 0)
            if robot.is_connected:
                pose = robot.read_current_pose()
                if pose:
                    robot_pose = pose

            if markers:
                ax.clear()
                visualize_on_ax(ax, robot_pose, markers)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

    except KeyboardInterrupt:
        pass

    cap.release()
    if robot.is_connected:
        robot.disconnect()
    plt.close('all')
    print("종료")


def visualize_on_ax(ax, robot_pose, markers_cam):
    """기존 ax에 좌표계 그리기 (실시간용)"""

    # Base
    draw_frame(ax, np.array([0, 0, 0]), np.eye(3), "Base", scale=80, linewidth=3)

    # TCP
    tcp_pos = np.array([robot_pose[0], robot_pose[1], robot_pose[2]])
    R_tcp = euler_to_rotation_matrix(robot_pose[3], robot_pose[4], robot_pose[5])
    draw_frame(ax, tcp_pos, R_tcp, "TCP", scale=50, linewidth=2)
    draw_connection(ax, np.array([0, 0, 0]), tcp_pos, color='orange', linestyle='-')

    # Markers
    all_pts = [np.array([0, 0, 0]), tcp_pos]
    for idx, m in enumerate(markers_cam):
        base_result = ar_to_base(m['tvec'], m['rvec'], robot_pose)
        base_pos = np.array([base_result[0], base_result[1], base_result[2]])

        base_rvec_full = vision_to_base(
            *camera_to_vision(m['tvec'], m['rvec']), robot_pose
        )[1]
        R_marker, _ = cv2.Rodrigues(base_rvec_full)

        draw_frame(ax, base_pos, R_marker,
                   f"M{m['tag_id']}", scale=30, linewidth=2)
        draw_connection(ax, tcp_pos, base_pos, color='gray', linestyle='--')
        all_pts.append(base_pos)

    # 축 설정
    all_pts = np.array(all_pts)
    center = all_pts.mean(axis=0)
    max_range = max(np.ptp(all_pts, axis=0)) * 0.7
    if max_range < 100:
        max_range = 200

    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('3D Coordinate Frames (Realtime)')


def main():
    parser = argparse.ArgumentParser(description='3D 좌표계 시각화')
    parser.add_argument('--csv', type=str, help='CSV 파일 경로')
    parser.add_argument('--sample', type=int, default=0, help='CSV 샘플 인덱스 (기본: 0)')
    parser.add_argument('--scale', type=float, default=1.0,
                        help='tvec 보정 비율 (예: 기존 0.04m로 수집 → 0.015m 보정: --scale 0.375)')
    args = parser.parse_args()

    if args.csv:
        csv_path = args.csv
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(PROJECT_ROOT, csv_path)
        run_csv_mode(csv_path, args.sample, args.scale)
    else:
        run_realtime_mode()


if __name__ == "__main__":
    main()
