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

    try:
        while plt.fignum_exists(fig.number):
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            markers = {}

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    n_solutions, rvecs, tvecs, reproj_errors = cv2.solvePnPGeneric(
                        obj_points, corners[i].reshape(-1, 2),
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if n_solutions == 0:
                        continue

                    # Z축 법선 기준 disambiguation: 마커 Z축이 카메라를 향하는 해 선택
                    # 카메라 좌표계에서 z_axis.z < 0 → 마커가 카메라를 바라봄
                    best_rv, best_tv = rvecs[0].flatten(), tvecs[0].flatten()
                    for s in range(n_solutions):
                        rv = rvecs[s].flatten()
                        R, _ = cv2.Rodrigues(rv)
                        z_axis = R[:, 2]
                        if z_axis[2] < 0:  # Z축이 카메라 방향
                            best_rv = rv
                            best_tv = tvecs[s].flatten()
                            break

                    # LM refinement
                    best_rv, best_tv = cv2.solvePnPRefineLM(
                        obj_points, corners[i].reshape(-1, 2),
                        camera_matrix, dist_coeffs,
                        best_rv.reshape(3, 1), best_tv.reshape(3, 1)
                    )
                    best_rv = best_rv.flatten()
                    best_tv = best_tv.flatten()

                    R, _ = cv2.Rodrigues(best_rv)
                    euler = rotation_matrix_to_euler(R)
                    marker_id = int(marker_id)
                    euler = unwrap_euler(marker_id, euler)
                    # rx를 [0, 360) 범위로 정규화 (±180° 경계 일관성)
                    euler[0] = euler[0] % 360
                    markers[marker_id] = {
                        'tvec': best_tv, 'rvec': best_rv,
                        'euler': euler, 'corners': corners[i],
                    }

                # 시각화
                for marker_id, m in markers.items():
                    rvec = m['rvec'].reshape(3, 1)
                    tvec = m['tvec'].reshape(3, 1)
                    cv2.aruco.drawDetectedMarkers(frame, [m['corners']], np.array([[marker_id]]))
                    draw_axes_on_frame(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE * 0.5)

                    c = np.mean(m['corners'][0], axis=0).astype(int)
                    cv2.putText(frame, f"ID:{marker_id}", (c[0]-20, c[1]-40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame,
                                f"t:[{m['tvec'][0]*1000:.1f}, {m['tvec'][1]*1000:.1f}, {m['tvec'][2]*1000:.1f}]mm",
                                (c[0]-60, c[1]-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                    cv2.putText(frame,
                                f"r:[{m['euler'][0]:.1f}, {m['euler'][1]:.1f}, {m['euler'][2]:.1f}]deg",
                                (c[0]-60, c[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            # 로봇 TCP 자세 읽기
            robot_pose = None
            if robot.is_connected:
                robot_pose = robot.read_current_pose()

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
                info = (f"{robot_info}"
                        f"ID{mids[0]}: t=[{m1['tvec'][0]*1000:.1f}, {m1['tvec'][1]*1000:.1f}, {m1['tvec'][2]*1000:.1f}]mm "
                        f"r=[{m1['euler'][0]:.1f}, {m1['euler'][1]:.1f}, {m1['euler'][2]:.1f}]deg\n"
                        f"ID{mids[1]}: t=[{m2['tvec'][0]*1000:.1f}, {m2['tvec'][1]*1000:.1f}, {m2['tvec'][2]*1000:.1f}]mm "
                        f"r=[{m2['euler'][0]:.1f}, {m2['euler'][1]:.1f}, {m2['euler'][2]:.1f}]deg\n"
                        f"Distance: {dist:.1f}mm (ref:{KNOWN_MARKER_DISTANCE*1000:.0f}mm, err:{abs(dist-KNOWN_MARKER_DISTANCE*1000):.1f}mm) | Saved: {len(saved_samples)}")
            elif len(markers) == 1:
                mid = list(markers.keys())[0]
                m = markers[mid]
                info = (f"{robot_info}"
                        f"ID{mid}: t=[{m['tvec'][0]*1000:.1f}, {m['tvec'][1]*1000:.1f}, {m['tvec'][2]*1000:.1f}]mm "
                        f"r=[{m['euler'][0]:.1f}, {m['euler'][1]:.1f}, {m['euler'][2]:.1f}]deg\n"
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
