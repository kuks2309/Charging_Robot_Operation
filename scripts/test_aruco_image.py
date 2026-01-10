#!/usr/bin/env python3
"""
이미지에서 AR Tag 감지 테스트
"""

import cv2
import numpy as np

# 테스트할 사전 타입들
DICT_TYPES = [
    ("DICT_4X4_50", cv2.aruco.DICT_4X4_50),
    ("DICT_4X4_100", cv2.aruco.DICT_4X4_100),
    ("DICT_4X4_250", cv2.aruco.DICT_4X4_250),
    ("DICT_5X5_50", cv2.aruco.DICT_5X5_50),
    ("DICT_6X6_250", cv2.aruco.DICT_6X6_250),
]

def test_detection(gray, color_image, label):
    print(f"\n{'='*60}")
    print(f"테스트: {label}")
    print(f"{'='*60}")

    detected_any = False
    for dict_name, dict_type in DICT_TYPES:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)

        # 검출 파라미터 조정
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = 0.01
        params.maxMarkerPerimeterRate = 4.0

        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            detected_any = True
            print(f"\n[{dict_name}]")
            print(f"  감지된 마커 수: {len(ids)}")
            print(f"  IDs: {ids.flatten().tolist()}")

            # 결과 이미지 저장
            result_image = color_image.copy()
            cv2.aruco.drawDetectedMarkers(result_image, corners, ids)
            cv2.imwrite(f"/tmp/aruco_{label}_{dict_name}.jpg", result_image)

    if not detected_any:
        print("  감지된 마커 없음")

    return detected_any

def main():
    # 이미지 로드
    image_path = "/home/amap/Project/KAIST/Charging_Robot/images/20260110_120527.jpg"
    color_image = cv2.imread(image_path)

    if color_image is None:
        print(f"이미지를 로드할 수 없습니다: {image_path}")
        return

    print(f"원본 이미지 크기: {color_image.shape}")

    # 이미지 리사이즈
    max_width = 1280
    if color_image.shape[1] > max_width:
        scale = max_width / color_image.shape[1]
        color_image = cv2.resize(color_image, None, fx=scale, fy=scale)
        print(f"리사이즈 후 크기: {color_image.shape}")

    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # 1. 원본 테스트
    test_detection(gray, color_image, "original")

    # 2. 반전 테스트 (흑백 반전)
    gray_inverted = cv2.bitwise_not(gray)
    color_inverted = cv2.bitwise_not(color_image)
    test_detection(gray_inverted, color_inverted, "inverted")

    # 3. 히스토그램 평활화 테스트
    gray_eq = cv2.equalizeHist(gray)
    test_detection(gray_eq, color_image, "equalized")

    # 4. CLAHE 테스트
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_clahe = clahe.apply(gray)
    test_detection(gray_clahe, color_image, "clahe")

    # 5. 이진화 테스트
    _, gray_thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    test_detection(gray_thresh, color_image, "threshold")

    print("\n" + "=" * 60)
    print("테스트 완료")
    print("=" * 60)

if __name__ == "__main__":
    main()
