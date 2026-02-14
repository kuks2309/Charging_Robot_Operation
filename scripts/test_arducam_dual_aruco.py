#!/usr/bin/env python3
"""
ArduCam 2개 ArUco 마커 인식 + 로봇 TCP 자세 동시 저장
- ArduCam USB 카메라 (device_index=6)
- 캘리브레이션: config/arducam_calibration.yaml
- Modbus TCP로 로봇 TCP pose (TF5) 읽기
- matplotlib 실시간 시각화
- 's' 키: 마커 데이터 + 로봇 자세 CSV 저장
"""

import os
import sys
import csv
import time
from datetime import datetime
import cv2
import yaml
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# 프로젝트 루트 & sys.path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)
CALIB_FILE = os.path.join(PROJECT_ROOT, '..', 'config', 'arducam_calibration.yaml')

from Robot.communication.modbus_client import ModbusClient
from utils.ar_to_base_tf import euler_to_rotation_matrix, rotation_matrix_to_euler


def marker_to_base(tvec, rvec, robot_pose):
    """마커의 카메라 좌표계 pose → 베이스 좌표계 변환.

    ar_to_base_tf.camera_to_vision의 diag(1,-1,1) (det=-1 반사행렬)이
    Rodrigues 왕복 시 회전행렬을 깨뜨리는 버그를 우회하기 위해
    회전행렬을 직접 합성한다.

    회전: R_base = R_tcp @ R_camera  (marker→base)
           euler = decompose(R_base.T)  (base→marker, TCP와 동일 관점)
    위치: P_base = R_tcp @ tvec + T_tcp  (카메라 = 툴 프레임 가정)

    Note: R_base.T를 분해하는 이유:
      - TCP euler = base→tool 회전 (Doosan 관례)
      - R_base.T euler = base→marker 회전 (동일 관례)
      → 마커 orientation을 TCP Rx/Ry/Rz와 직접 비교 가능
    """
    R_camera, _ = cv2.Rodrigues(np.array(rvec).flatten())
    R_tcp = euler_to_rotation_matrix(robot_pose[3], robot_pose[4], robot_pose[5])
    T_tcp = np.array(robot_pose[:3]) / 1000.0

    tvec = np.array(tvec).flatten()

    # 회전: R_base.T 분해 (base→marker = TCP와 동일 관점)
    R_base = R_tcp @ R_camera
    rx, ry, rz = rotation_matrix_to_euler(R_base.T)

    # 위치: 카메라 → 베이스 직접 변환
    base_pos = R_tcp @ tvec + T_tcp

    return (base_pos[0]*1000, base_pos[1]*1000, base_pos[2]*1000, rx, ry, rz)

# ArUco 설정
ARUCO_DICT_TYPE = cv2.aruco.DICT_5X5_50
MARKER_SIZE = 0.015  # m (15mm)
DEVICE_INDEX = 6
KNOWN_MARKER_DISTANCE = 0.059  # m (59mm) - 두 마커 중심 간 실제 거리 (실측)


def load_calibration(filepath):
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    cam_data = data['camera_matrix']['data']
    camera_matrix = np.array(cam_data, dtype=np.float64).reshape(3, 3)
    dist_data = data['distortion_coefficients']['data']
    dist_coeffs = np.array(dist_data, dtype=np.float64)
    return camera_matrix, dist_coeffs


def rotation_matrix_to_euler(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])


# 이전 프레임 기준 각도 연속성 보정 (temporal unwrapping)
_prev_euler = {}  # marker_id -> [rx, ry, rz]

def unwrap_euler(marker_id, euler):
    """이전 프레임 대비 ±180° 점프 방지"""
    euler = np.array(euler, dtype=float)
    if marker_id in _prev_euler:
        for i in range(3):
            while euler[i] - _prev_euler[marker_id][i] > 180:
                euler[i] -= 360
            while euler[i] - _prev_euler[marker_id][i] < -180:
                euler[i] += 360
    _prev_euler[marker_id] = euler.copy()
    return euler


def compute_plane_pose(m1, m2):
    """두 마커로 정의되는 평면의 위치(중점)와 자세(회전행렬) 계산.

    좌표계 정의:
      - Origin: 두 마커 중점
      - X축: m1 → m2 방향
      - Z축: 두 마커 법선 평균 (카메라 방향)
      - Y축: Z × X (오른손 법칙)

    Returns: (midpoint_tvec, R_plane, euler_deg)
    """
    mid = (m1['tvec'] + m2['tvec']) / 2.0

    # X축: m1 → m2
    x_axis = m2['tvec'] - m1['tvec']
    x_axis = x_axis / np.linalg.norm(x_axis)

    # 각 마커의 Z축 법선 (카메라 좌표계)
    R1, _ = cv2.Rodrigues(m1['rvec'])
    R2, _ = cv2.Rodrigues(m2['rvec'])
    z1 = R1[:, 2]
    z2 = R2[:, 2]
    z_avg = (z1 + z2) / 2.0
    z_avg = z_avg / np.linalg.norm(z_avg)

    # Y축 = Z × X, 재정규화
    y_axis = np.cross(z_avg, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)

    # Z축 재계산 (직교성 보장)
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)

    R_plane = np.column_stack([x_axis, y_axis, z_axis])
    euler = rotation_matrix_to_euler(R_plane)

    return mid, R_plane, euler


def draw_axes_on_frame(frame, camera_matrix, dist_coeffs, rvec, tvec, length):
    axis_points = np.float32([
        [length, 0, 0], [0, length, 0], [0, 0, length], [0, 0, 0]
    ]).reshape(-1, 3)
    img_pts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    img_pts = img_pts.astype(int).reshape(-1, 2)
    origin = tuple(img_pts[3])
    cv2.line(frame, origin, tuple(img_pts[0]), (0, 0, 255), 2)
    cv2.line(frame, origin, tuple(img_pts[1]), (0, 255, 0), 2)
    cv2.line(frame, origin, tuple(img_pts[2]), (255, 0, 0), 2)


def main():
    # 캘리브레이션 로드
    calib_path = os.path.normpath(CALIB_FILE)
    if not os.path.exists(calib_path):
        print(f"캘리브레이션 파일 없음: {calib_path}")
        sys.exit(1)

    camera_matrix, dist_coeffs = load_calibration(calib_path)
    print(f"캘리브레이션 로드: {calib_path}")

    # Modbus 연결 (로봇 TCP 자세 읽기용)
    robot = ModbusClient()
    success, msg = robot.connect()
    if success:
        print(f"로봇 연결 성공: {msg}")
    else:
        print(f"로봇 연결 실패: {msg} (로봇 자세 없이 진행)")

    # ArUco 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    obj_points = np.array([
        [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    ], dtype=np.float32)

    # ArduCam 열기
    cap = cv2.VideoCapture(DEVICE_INDEX)
    if not cap.isOpened():
        print(f"ArduCam 열기 실패 (device_index={DEVICE_INDEX})")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # CSV 저장 설정 (프로젝트루트/data/)
    project_root = os.path.normpath(os.path.join(PROJECT_ROOT, '..'))
    save_dir = os.path.join(project_root, 'data', 'aruco')
    os.makedirs(save_dir, exist_ok=True)
    csv_path = os.path.join(save_dir, f"dual_aruco_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    saved_samples = []
    save_requested = [False]

    CSV_FIELDS = [
        'timestamp',
        # 로봇 TCP (TF5)
        'robot_x', 'robot_y', 'robot_z', 'robot_rx', 'robot_ry', 'robot_rz',
        # 마커 1
        'm1_tag_id', 'm1_tvec_x', 'm1_tvec_y', 'm1_tvec_z',
        'm1_rvec_x', 'm1_rvec_y', 'm1_rvec_z',
        'm1_euler_rx', 'm1_euler_ry', 'm1_euler_rz',
        # 마커 2
        'm2_tag_id', 'm2_tvec_x', 'm2_tvec_y', 'm2_tvec_z',
        'm2_rvec_x', 'm2_rvec_y', 'm2_rvec_z',
        'm2_euler_rx', 'm2_euler_ry', 'm2_euler_rz',
        # 거리
        'distance_mm',
    ]

    def on_key_press(event):
        if event.key == 's':
            save_requested[0] = True

    print("\n" + "=" * 60)
    print("ArduCam 2개 ArUco + 로봇 TCP 자세 저장")
    print("'s' 키: 현재 데이터 저장 | 창 닫기: 종료")
    print(f"CSV: {os.path.normpath(csv_path)}")
    print("=" * 60 + "\n")

    # matplotlib 설정
    plt.ion()
    fig, ax = plt.subplots(1, 1, figsize=(12, 7))
    fig.canvas.manager.set_window_title('ArduCam Dual ArUco + Robot TCP')
    fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)
    fig.canvas.mpl_connect('key_press_event', on_key_press)

    ret, frame = cap.read()
    if not ret:
        print("프레임 읽기 실패")
        sys.exit(1)

    img_display = ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    ax.axis('off')

    info_text = ax.text(0.5, 0.02, '', fontsize=10, color='lime',
                        fontfamily='monospace', transform=ax.transAxes,
                        ha='center', va='bottom',
                        bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))

    save_text = ax.text(0.99, 0.01, '', fontsize=12, color='yellow',
                        fontfamily='monospace', transform=ax.transAxes,
                        ha='right', va='bottom')

    plt.tight_layout()
    plt.show(block=False)

    console_start_time = time.time()
    console_count = 0
    console_max = 50

    # Temporal consistency: 이전 프레임의 평면 rx 저장
    _prev_plane_rx = [None]  # mutable container for closure

    try:
        while plt.fignum_exists(fig.number):
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            markers = {}

            # 로봇 TCP 자세 읽기
            robot_pose = None
            if robot.is_connected:
                robot_pose = robot.read_current_pose()

            if ids is not None:
                # Phase 1: 각 마커별 IPPE 후보 수집 (z22 < 0 필터)
                marker_candidates = {}  # marker_id -> [{'rv', 'tv', 'rx', 'corners'}, ...]
                for i, marker_id in enumerate(ids.flatten()):
                    mid = int(marker_id)
                    n_solutions, rvecs, tvecs, reproj_errors = cv2.solvePnPGeneric(
                        obj_points, corners[i].reshape(-1, 2),
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if n_solutions == 0:
                        continue

                    candidates = []
                    for s in range(n_solutions):
                        rv = rvecs[s].flatten()
                        tv = tvecs[s].flatten()
                        R, _ = cv2.Rodrigues(rv)
                        if R[2, 2] < 0:  # Z축이 카메라 방향
                            rx = np.degrees(np.arctan2(R[2, 1], R[2, 2])) % 360
                            candidates.append({'rv': rv, 'tv': tv, 'rx': rx})

                    if not candidates:
                        # z22 < 0 만족하는 해가 없으면 sol[0] 폴백
                        rv = rvecs[0].flatten()
                        tv = tvecs[0].flatten()
                        R, _ = cv2.Rodrigues(rv)
                        rx = np.degrees(np.arctan2(R[2, 1], R[2, 2])) % 360
                        candidates.append({'rv': rv, 'tv': tv, 'rx': rx})

                    marker_candidates[mid] = (candidates, corners[i])

                # Phase 2: Coplanarity + Temporal disambiguation (2개 마커)
                mids_all = sorted(marker_candidates.keys())
                if len(mids_all) >= 2:
                    mid_a, mid_b = mids_all[0], mids_all[1]
                    sols_a, corners_a = marker_candidates[mid_a]
                    sols_b, corners_b = marker_candidates[mid_b]

                    # 모든 조합 평가
                    best_combo = (0, 0)
                    best_score = 999.0
                    for ia in range(len(sols_a)):
                        for ib in range(len(sols_b)):
                            # Coplanarity: rx 차이 최소
                            rx_diff = abs(sols_a[ia]['rx'] - sols_b[ib]['rx'])
                            if rx_diff > 180:
                                rx_diff = 360 - rx_diff
                            score = rx_diff

                            # Temporal: 이전 프레임 plane rx와의 차이 가산
                            if _prev_plane_rx[0] is not None:
                                avg_rx = (sols_a[ia]['rx'] + sols_b[ib]['rx']) / 2.0
                                temporal_diff = abs(avg_rx - _prev_plane_rx[0])
                                if temporal_diff > 180:
                                    temporal_diff = 360 - temporal_diff
                                # temporal weight: coplanarity가 비슷할 때 temporal로 결정
                                score += temporal_diff * 0.5

                            if score < best_score:
                                best_score = score
                                best_combo = (ia, ib)

                    ia_best, ib_best = best_combo
                    selected = {
                        mid_a: (sols_a[ia_best], corners_a),
                        mid_b: (sols_b[ib_best], corners_b),
                    }

                    # 평면 rx 업데이트 (temporal용)
                    _prev_plane_rx[0] = (sols_a[ia_best]['rx'] + sols_b[ib_best]['rx']) / 2.0

                else:
                    # 1개 마커: temporal consistency로 선택
                    selected = {}
                    for mid in mids_all:
                        cands, crn = marker_candidates[mid]
                        if len(cands) == 1 or _prev_plane_rx[0] is None:
                            selected[mid] = (cands[0], crn)
                        else:
                            # 이전 plane rx에 가장 가까운 후보
                            best_c = min(cands, key=lambda c: min(
                                abs(c['rx'] - _prev_plane_rx[0]),
                                360 - abs(c['rx'] - _prev_plane_rx[0])))
                            selected[mid] = (best_c, crn)

                # Phase 3: LM refinement + euler 계산
                for mid, (sol, crn) in selected.items():
                    corner_2d = crn.reshape(-1, 2)
                    best_rv, best_tv = cv2.solvePnPRefineLM(
                        obj_points, corner_2d,
                        camera_matrix, dist_coeffs,
                        sol['rv'].reshape(3, 1), sol['tv'].reshape(3, 1)
                    )
                    best_rv = best_rv.flatten()
                    best_tv = best_tv.flatten()

                    R, _ = cv2.Rodrigues(best_rv)
                    euler = rotation_matrix_to_euler(R)
                    euler = unwrap_euler(mid, euler)
                    euler[0] = euler[0] % 360
                    markers[mid] = {
                        'tvec': best_tv, 'rvec': best_rv,
                        'euler': euler, 'corners': crn,
                    }

                # 시각화
                for marker_id, m in markers.items():
                    rvec = m['rvec'].reshape(3, 1)
                    tvec = m['tvec'].reshape(3, 1)
                    cv2.aruco.drawDetectedMarkers(frame, [m['corners']], np.array([[marker_id]]))
                    draw_axes_on_frame(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE * 0.5)

                    c = np.mean(m['corners'][0], axis=0).astype(int)
                    # 마커 중심 이미지 좌표 (픽셀) 표시
                    cv2.circle(frame, (c[0], c[1]), 4, (0, 255, 255), -1)
                    cv2.putText(frame, f"ID:{marker_id}", (c[0]-20, c[1]-40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame,
                                f"px:[{c[0]}, {c[1]}]",
                                (c[0]-60, c[1]-55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
                    cv2.putText(frame,
                                f"t:[{m['tvec'][0]*1000:.1f}, {m['tvec'][1]*1000:.1f}, {m['tvec'][2]*1000:.1f}]mm",
                                (c[0]-60, c[1]-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                    cv2.putText(frame,
                                f"r:[{m['euler'][0]:.1f}, {m['euler'][1]:.1f}, {m['euler'][2]:.1f}]deg",
                                (c[0]-60, c[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                    # 베이스 기준 자세 표시
                    if robot_pose:
                        bx, by, bz, brx, bry, brz = marker_to_base(m['tvec'], m['rvec'], robot_pose)
                        cv2.putText(frame,
                                    f"base r:[{brx:.1f}, {bry:.1f}, {brz:.1f}]deg",
                                    (c[0]-60, c[1]+15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)

                # 평면 자세 계산 및 시각화 (2개 이상 마커 검출 시)
                if len(markers) >= 2:
                    mids_sorted = sorted(markers.keys())
                    plane_mid, R_plane, plane_euler = compute_plane_pose(
                        markers[mids_sorted[0]], markers[mids_sorted[1]])

                    # 평면 중점에 좌표축 그리기 (마젠타 색상으로 구분)
                    plane_rvec, _ = cv2.Rodrigues(R_plane)
                    plane_tvec = plane_mid.reshape(3, 1)
                    draw_axes_on_frame(frame, camera_matrix, dist_coeffs,
                                       plane_rvec, plane_tvec, MARKER_SIZE * 0.8)

                    # 중점 위치에 "PLANE" 라벨 + 자세 표시
                    mid_2d, _ = cv2.projectPoints(
                        np.zeros((1, 3), dtype=np.float32),
                        plane_rvec, plane_tvec, camera_matrix, dist_coeffs)
                    mx, my = int(mid_2d[0][0][0]), int(mid_2d[0][0][1])
                    cv2.putText(frame, "PLANE", (mx - 30, my + 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    cv2.putText(frame,
                                f"t:[{plane_mid[0]*1000:.1f}, {plane_mid[1]*1000:.1f}, {plane_mid[2]*1000:.1f}]mm",
                                (mx - 80, my + 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
                    cv2.putText(frame,
                                f"r:[{plane_euler[0]:.1f}, {plane_euler[1]:.1f}, {plane_euler[2]:.1f}]deg",
                                (mx - 80, my + 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
                    # 평면 베이스 기준 자세 표시
                    if robot_pose:
                        pb_x, pb_y, pb_z, pb_rx, pb_ry, pb_rz = marker_to_base(
                            plane_mid, plane_rvec.flatten(), robot_pose)
                        cv2.putText(frame,
                                    f"base r:[{pb_rx:.1f}, {pb_ry:.1f}, {pb_rz:.1f}]deg",
                                    (mx - 80, my + 85),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)

            # 's' 키 저장 처리
            if save_requested[0] and len(markers) >= 2:
                mids = sorted(markers.keys())
                m1, m2 = markers[mids[0]], markers[mids[1]]
                dist_mm = np.linalg.norm(m1['tvec'] - m2['tvec']) * 1000

                sample = {'timestamp': time.time()}

                # 로봇 TCP
                if robot_pose:
                    sample['robot_x'] = robot_pose[0]
                    sample['robot_y'] = robot_pose[1]
                    sample['robot_z'] = robot_pose[2]
                    sample['robot_rx'] = robot_pose[3]
                    sample['robot_ry'] = robot_pose[4]
                    sample['robot_rz'] = robot_pose[5]
                else:
                    for k in ['robot_x', 'robot_y', 'robot_z', 'robot_rx', 'robot_ry', 'robot_rz']:
                        sample[k] = ''

                # 마커 1
                sample['m1_tag_id'] = mids[0]
                sample['m1_tvec_x'] = m1['tvec'][0]
                sample['m1_tvec_y'] = m1['tvec'][1]
                sample['m1_tvec_z'] = m1['tvec'][2]
                sample['m1_rvec_x'] = m1['rvec'][0]
                sample['m1_rvec_y'] = m1['rvec'][1]
                sample['m1_rvec_z'] = m1['rvec'][2]
                sample['m1_euler_rx'] = m1['euler'][0]
                sample['m1_euler_ry'] = m1['euler'][1]
                sample['m1_euler_rz'] = m1['euler'][2]

                # 마커 2
                sample['m2_tag_id'] = mids[1]
                sample['m2_tvec_x'] = m2['tvec'][0]
                sample['m2_tvec_y'] = m2['tvec'][1]
                sample['m2_tvec_z'] = m2['tvec'][2]
                sample['m2_rvec_x'] = m2['rvec'][0]
                sample['m2_rvec_y'] = m2['rvec'][1]
                sample['m2_rvec_z'] = m2['rvec'][2]
                sample['m2_euler_rx'] = m2['euler'][0]
                sample['m2_euler_ry'] = m2['euler'][1]
                sample['m2_euler_rz'] = m2['euler'][2]

                sample['distance_mm'] = dist_mm
                saved_samples.append(sample)

                # CSV 즉시 기록
                write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
                with open(csv_path, 'a', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
                    if write_header:
                        writer.writeheader()
                    writer.writerow(sample)

                rp = f"Robot=[{robot_pose[0]:.1f},{robot_pose[1]:.1f},{robot_pose[2]:.1f},{robot_pose[3]:.1f},{robot_pose[4]:.1f},{robot_pose[5]:.1f}]" if robot_pose else "Robot=N/A"
                print(f"\n  >> SAVED #{len(saved_samples)} dist={dist_mm:.1f}mm {rp}")
                save_text.set_text(f"SAVED #{len(saved_samples)}")
                save_requested[0] = False

            elif save_requested[0]:
                print("\n  >> 2개 마커가 검출되어야 저장 가능")
                save_requested[0] = False

            # 콘솔 출력: 3개 알고리즘 + 듀얼마커 거리제약 비교 (1초 후 50개)
            elapsed = time.time() - console_start_time
            if elapsed >= 1.0 and console_count < console_max and ids is not None and len(ids) >= 2:
                console_count += 1
                algo_flags = [
                    ("IPPE", cv2.SOLVEPNP_IPPE_SQUARE),
                    ("SQPNP", cv2.SOLVEPNP_SQPNP),
                    ("ITER", cv2.SOLVEPNP_ITERATIVE),
                ]
                print(f"\n[{console_count:3d}/{console_max}] === 알고리즘 비교 ===")

                # 각 마커별 알고리즘 비교
                for i, marker_id in enumerate(ids.flatten()):
                    mid = int(marker_id)
                    corner_2d = corners[i].reshape(-1, 2)
                    parts = []
                    for algo_name, algo_flag in algo_flags:
                        try:
                            n_sol, rvs, tvs, errs = cv2.solvePnPGeneric(
                                obj_points, corner_2d,
                                camera_matrix, dist_coeffs,
                                flags=algo_flag
                            )
                            sol_strs = []
                            for s in range(n_sol):
                                R_s, _ = cv2.Rodrigues(rvs[s].flatten())
                                rx_s = np.degrees(np.arctan2(R_s[2, 1], R_s[2, 2]))
                                rx_s = rx_s % 360
                                zz = R_s[2, 2]
                                err_s = errs[s][0] if errs is not None else -1
                                sol_strs.append(f"rx={rx_s:6.1f} z22={zz:+.3f} err={err_s:.4f}")
                            parts.append(f"{algo_name}({n_sol}sol): " + " | ".join(sol_strs))
                        except Exception as e:
                            parts.append(f"{algo_name}: FAIL({e})")
                    print(f"  ID{mid}: " + "  //  ".join(parts))

                # 듀얼 마커 거리 제약 disambiguation (IPPE 2해 × 2마커 = 4조합)
                if len(ids) >= 2:
                    id_list = ids.flatten().tolist()
                    # 각 마커의 IPPE 2해 수집
                    marker_solutions = {}
                    for i, mid in enumerate(id_list):
                        corner_2d = corners[i].reshape(-1, 2)
                        n_sol, rvs, tvs, errs = cv2.solvePnPGeneric(
                            obj_points, corner_2d,
                            camera_matrix, dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )
                        sols = []
                        for s in range(n_sol):
                            R_s, _ = cv2.Rodrigues(rvs[s].flatten())
                            rx_s = np.degrees(np.arctan2(R_s[2, 1], R_s[2, 2])) % 360
                            sols.append({'tv': tvs[s].flatten(), 'rx': rx_s, 'err': errs[s][0]})
                        marker_solutions[int(mid)] = sols

                    mids = sorted(marker_solutions.keys())
                    if len(mids) >= 2 and len(marker_solutions[mids[0]]) >= 2 and len(marker_solutions[mids[1]]) >= 2:
                        sols_a = marker_solutions[mids[0]]
                        sols_b = marker_solutions[mids[1]]
                        # 4가지 조합 + coplanarity disambiguation
                        print(f"  DUAL 4조합 + Coplanarity:")
                        best_combo = None
                        best_rx_diff = 999.0
                        for ia in range(len(sols_a)):
                            for ib in range(len(sols_b)):
                                dist = np.linalg.norm(sols_a[ia]['tv'] - sols_b[ib]['tv']) * 1000
                                rx_diff = abs(sols_a[ia]['rx'] - sols_b[ib]['rx'])
                                if rx_diff > 180:
                                    rx_diff = 360 - rx_diff
                                tag = ""
                                if rx_diff < best_rx_diff:
                                    best_rx_diff = rx_diff
                                    best_combo = (ia, ib)
                                print(f"    [{ia},{ib}] ID{mids[0]}=rx{sols_a[ia]['rx']:6.1f} + "
                                      f"ID{mids[1]}=rx{sols_b[ib]['rx']:6.1f} → dist={dist:.1f}mm  Δrx={rx_diff:.1f}°")
                        if best_combo is not None:
                            ia, ib = best_combo
                            print(f"    ★ Coplanarity best: [{ia},{ib}] "
                                  f"ID{mids[0]}=rx{sols_a[ia]['rx']:6.1f} + "
                                  f"ID{mids[1]}=rx{sols_b[ib]['rx']:6.1f}  Δrx={best_rx_diff:.1f}°")

            # 정보 텍스트
            robot_info = ""
            if robot_pose:
                robot_info = (f"Robot TCP: [{robot_pose[0]:.1f}, {robot_pose[1]:.1f}, {robot_pose[2]:.1f}]mm "
                              f"[{robot_pose[3]:.1f}, {robot_pose[4]:.1f}, {robot_pose[5]:.1f}]deg\n")

            if len(markers) >= 2:
                mids = sorted(markers.keys())
                m1, m2 = markers[mids[0]], markers[mids[1]]
                dist = np.linalg.norm(m1['tvec'] - m2['tvec']) * 1000
                p_mid, R_p, p_euler = compute_plane_pose(m1, m2)
                # 베이스 기준 변환
                base_info = ""
                if robot_pose:
                    b1 = marker_to_base(m1['tvec'], m1['rvec'], robot_pose)
                    b2 = marker_to_base(m2['tvec'], m2['rvec'], robot_pose)
                    p_rvec_flat, _ = cv2.Rodrigues(R_p)
                    bp = marker_to_base(p_mid, p_rvec_flat.flatten(), robot_pose)
                    base_info = (f"[Base] ID{mids[0]}: pos=[{b1[0]:.1f}, {b1[1]:.1f}, {b1[2]:.1f}]mm "
                                 f"r=[{b1[3]:.1f}, {b1[4]:.1f}, {b1[5]:.1f}]deg\n"
                                 f"[Base] ID{mids[1]}: pos=[{b2[0]:.1f}, {b2[1]:.1f}, {b2[2]:.1f}]mm "
                                 f"r=[{b2[3]:.1f}, {b2[4]:.1f}, {b2[5]:.1f}]deg\n"
                                 f"[Base] PLANE: pos=[{bp[0]:.1f}, {bp[1]:.1f}, {bp[2]:.1f}]mm "
                                 f"r=[{bp[3]:.1f}, {bp[4]:.1f}, {bp[5]:.1f}]deg\n")
                info = (f"{robot_info}"
                        f"ID{mids[0]}: t=[{m1['tvec'][0]*1000:.1f}, {m1['tvec'][1]*1000:.1f}, {m1['tvec'][2]*1000:.1f}]mm "
                        f"r=[{m1['euler'][0]:.1f}, {m1['euler'][1]:.1f}, {m1['euler'][2]:.1f}]deg\n"
                        f"ID{mids[1]}: t=[{m2['tvec'][0]*1000:.1f}, {m2['tvec'][1]*1000:.1f}, {m2['tvec'][2]*1000:.1f}]mm "
                        f"r=[{m2['euler'][0]:.1f}, {m2['euler'][1]:.1f}, {m2['euler'][2]:.1f}]deg\n"
                        f"PLANE: t=[{p_mid[0]*1000:.1f}, {p_mid[1]*1000:.1f}, {p_mid[2]*1000:.1f}]mm "
                        f"r=[{p_euler[0]:.1f}, {p_euler[1]:.1f}, {p_euler[2]:.1f}]deg\n"
                        f"{base_info}"
                        f"Distance: {dist:.1f}mm (ref:{KNOWN_MARKER_DISTANCE*1000:.0f}mm, err:{abs(dist-KNOWN_MARKER_DISTANCE*1000):.1f}mm) | Saved: {len(saved_samples)}")
            elif len(markers) == 1:
                mid = list(markers.keys())[0]
                m = markers[mid]
                base_info_1 = ""
                if robot_pose:
                    b = marker_to_base(m['tvec'], m['rvec'], robot_pose)
                    base_info_1 = (f"[Base] ID{mid}: pos=[{b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f}]mm "
                                   f"r=[{b[3]:.1f}, {b[4]:.1f}, {b[5]:.1f}]deg\n")
                info = (f"{robot_info}"
                        f"ID{mid}: t=[{m['tvec'][0]*1000:.1f}, {m['tvec'][1]*1000:.1f}, {m['tvec'][2]*1000:.1f}]mm "
                        f"r=[{m['euler'][0]:.1f}, {m['euler'][1]:.1f}, {m['euler'][2]:.1f}]deg\n"
                        f"{base_info_1}"
                        f"(1/2 markers) | Saved: {len(saved_samples)}")
            else:
                info = f"{robot_info}No markers detected | Saved: {len(saved_samples)}"

            info_text.set_text(info)
            img_display.set_data(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

    except KeyboardInterrupt:
        pass

    cap.release()
    if robot.is_connected:
        robot.disconnect()
    plt.close('all')
    print(f"\n종료. 총 {len(saved_samples)}개 저장됨: {os.path.normpath(csv_path)}")


if __name__ == "__main__":
    main()
