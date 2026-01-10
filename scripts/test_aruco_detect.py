#!/usr/bin/env python3
"""
AR Tag 감지 테스트 스크립트
- 모든 ArUco 사전 타입 테스트
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import time

# 테스트할 사전 타입들
DICT_TYPES = [
    ("DICT_4X4_50", cv2.aruco.DICT_4X4_50),
    ("DICT_4X4_100", cv2.aruco.DICT_4X4_100),
    ("DICT_4X4_250", cv2.aruco.DICT_4X4_250),
    ("DICT_5X5_50", cv2.aruco.DICT_5X5_50),
    ("DICT_5X5_100", cv2.aruco.DICT_5X5_100),
    ("DICT_5X5_250", cv2.aruco.DICT_5X5_250),
    ("DICT_6X6_50", cv2.aruco.DICT_6X6_50),
    ("DICT_6X6_100", cv2.aruco.DICT_6X6_100),
    ("DICT_6X6_250", cv2.aruco.DICT_6X6_250),
    ("DICT_7X7_50", cv2.aruco.DICT_7X7_50),
    ("DICT_7X7_100", cv2.aruco.DICT_7X7_100),
    ("DICT_7X7_250", cv2.aruco.DICT_7X7_250),
    ("DICT_ARUCO_ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
]

def main():
    # RealSense 카메라 초기화
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
        print("=" * 60)
        print("AR Tag 감지 테스트 (모든 사전 타입)")
        print("카메라 시작됨. 마커를 카메라에 비춰주세요.")
        print("10초 동안 테스트합니다...")
        print("=" * 60)

        start_time = time.time()
        detected_results = {}  # {dict_name: set of ids}
        frame_count = 0

        while time.time() - start_time < 10.0:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame_count += 1
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # 모든 사전 타입으로 감지 시도
            for dict_name, dict_type in DICT_TYPES:
                aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
                params = cv2.aruco.DetectorParameters()
                params.minMarkerPerimeterRate = 0.005
                params.maxMarkerPerimeterRate = 4.0
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                corners, ids, rejected = detector.detectMarkers(gray)

                if ids is not None and len(ids) > 0:
                    if dict_name not in detected_results:
                        detected_results[dict_name] = set()
                    detected_results[dict_name].update(ids.flatten().tolist())

            # 실시간 출력
            if detected_results:
                current = ", ".join([f"{k}:{sorted(v)}" for k, v in detected_results.items()])
                print(f"\r프레임 {frame_count}: {current[:70]}...", end="")
            else:
                print(f"\r프레임 {frame_count}: 감지된 마커 없음", end="")

            time.sleep(0.05)

        # 최종 결과 출력
        print("\n\n" + "=" * 60)
        print("최종 결과:")
        print("=" * 60)
        print(f"총 프레임 수: {frame_count}")

        if detected_results:
            print("\n감지된 마커:")
            for dict_name, ids in sorted(detected_results.items()):
                print(f"  [{dict_name}] IDs: {sorted(ids)}")
        else:
            print("감지된 마커 없음")

        # 마지막 프레임 저장
        cv2.imwrite("/tmp/aruco_final_frame.jpg", color_image)
        print(f"\n마지막 프레임 저장: /tmp/aruco_final_frame.jpg")
        print("=" * 60)

    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()
