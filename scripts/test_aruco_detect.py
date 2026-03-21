#!/usr/bin/env python3
"""ArUco 검출 단위 테스트 - ArduCam 마커 인식 확인"""
import cv2
import numpy as np
import yaml
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.camera_utils import detect_arducam_index
_detected = detect_arducam_index()
DEVICE_INDEX = _detected if _detected is not None else 6  # fallback: 6 (기존 동작 보존)
CALIB_FILE = os.path.join(os.path.dirname(__file__), '..', 'config', 'calibration', 'arducam', 'arducam_calibration.yaml')

def main():
    print(f"OpenCV: {cv2.__version__}")
    ver = tuple(map(int, cv2.__version__.split('.')[:2]))

    # 카메라
    cap = cv2.VideoCapture(DEVICE_INDEX)
    if not cap.isOpened():
        print(f"[FAIL] 카메라 {DEVICE_INDEX} 열기 실패")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    print(f"[OK] 카메라: {int(cap.get(3))}x{int(cap.get(4))}")

    for _ in range(10):
        cap.read()

    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("[FAIL] 프레임 캡처 실패")
        return
    print(f"[OK] 프레임: {frame.shape}")

    # 캘리브레이션
    camera_matrix, dist_coeffs = None, None
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE) as f:
            data = yaml.safe_load(f)
        cm = data['camera_matrix']
        camera_matrix = np.array(cm['data'], dtype=np.float64).reshape(cm['rows'], cm['cols'])
        dc = data['distortion_coefficients']
        dist_coeffs = np.array(dc['data'], dtype=np.float64).reshape(dc['rows'], dc['cols'])
        print(f"[OK] 캘리브레이션 로드")
    else:
        print(f"[WARN] 캘리브레이션 없음")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 원본 프레임 검출
    print(f"\n--- 원본 프레임 검출 ---")
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    if ver >= (4, 7):
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
    print(f"  rejected={len(rejected) if rejected else 0}")

    if ids is not None:
        print(f"  [OK] 검출: {ids.flatten().tolist()}")
        for i, mid in enumerate(ids.flatten()):
            c = corners[i][0] if len(corners[i].shape) == 3 else corners[i]
            center = c.mean(axis=0)
            print(f"    ID {mid}: center=({center[0]:.1f}, {center[1]:.1f})")
    else:
        print(f"  [FAIL] 검출 실패")

    # undistort 프레임 검출
    if camera_matrix is not None:
        print(f"\n--- undistort 프레임 검출 ---")
        undist = cv2.undistort(frame, camera_matrix, dist_coeffs)
        gray_u = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        if ver >= (4, 7):
            corners2, ids2, rejected2 = detector.detectMarkers(gray_u)
        else:
            corners2, ids2, rejected2 = cv2.aruco.detectMarkers(gray_u, aruco_dict)
        print(f"  rejected={len(rejected2) if rejected2 else 0}")

        if ids2 is not None:
            print(f"  [OK] 검출: {ids2.flatten().tolist()}")
            for i, mid in enumerate(ids2.flatten()):
                c = corners2[i][0] if len(corners2[i].shape) == 3 else corners2[i]
                center = c.mean(axis=0)
                print(f"    ID {mid}: center=({center[0]:.1f}, {center[1]:.1f})")
        else:
            print(f"  [FAIL] 검출 실패")

    # 이미지 저장
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imwrite('/tmp/aruco_test.png', frame)
    print(f"\n결과: /tmp/aruco_test.png")

if __name__ == '__main__':
    main()
