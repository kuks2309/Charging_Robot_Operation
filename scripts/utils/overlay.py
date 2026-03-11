# -*- coding: utf-8 -*-
"""
공통 OpenCV overlay 드로잉 함수

반복되는 시각화 패턴(십자선, 중심 마커, 반투명 텍스트 등)을
함수화하여 재사용 가능하게 제공한다.
"""

import cv2
import numpy as np


class OverlayColors:
    """BGR 색상 상수"""
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    CYAN = (0, 255, 255)
    ORANGE = (255, 200, 0)
    PINK = (180, 105, 255)
    GRAY = (128, 128, 128)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)


def draw_crosshair(frame, cx, cy, color, thickness=1, full_frame=True, arm_length=20):
    """십자선을 그린다.

    full_frame=True: 프레임 전체 가로/세로선 (cx, cy를 지나는)
    full_frame=False: cx±arm_length, cy±arm_length 범위의 짧은 십자선
    """
    h, w = frame.shape[:2]
    if full_frame:
        cv2.line(frame, (0, cy), (w, cy), color, thickness)
        cv2.line(frame, (cx, 0), (cx, h), color, thickness)
    else:
        cv2.line(frame, (cx - arm_length, cy), (cx + arm_length, cy), color, thickness)
        cv2.line(frame, (cx, cy - arm_length), (cx, cy + arm_length), color, thickness)


def draw_center_marker(frame, cx, cy, color, radius=8, thickness=-1,
                       draw_crosshair_flag=True, crosshair_thickness=1):
    """십자선 + circle 조합으로 중심점을 표시한다.

    체스보드 중심, 이미지 중심 등에 사용.
    draw_crosshair_flag: True면 full-frame 십자선도 함께 그린다.
    """
    if draw_crosshair_flag:
        draw_crosshair(frame, cx, cy, color, thickness=crosshair_thickness, full_frame=True)
    cv2.circle(frame, (cx, cy), radius, color, thickness)


def draw_image_center_crosshair(frame, color=(128, 128, 128), thickness=1):
    """이미지 중심(w//2, h//2)에 full-frame 십자선을 그린다."""
    h, w = frame.shape[:2]
    img_cx, img_cy = w // 2, h // 2
    cv2.line(frame, (img_cx, 0), (img_cx, h), color, thickness)
    cv2.line(frame, (0, img_cy), (w, img_cy), color, thickness)


def draw_text_with_background(image, text_lines, pos, font_scale=0.4, alpha=0.5,
                              text_color=(255, 255, 255), bg_color=(0, 0, 0)):
    """반투명 배경 위에 텍스트를 그린다.

    text_lines: 표시할 문자열 리스트
    pos: (x, y) 첫 번째 줄 위치
    alpha: 배경 불투명도 (0=투명, 1=불투명)
    """
    if not text_lines:
        return
    text_height = 20
    max_text_width = max(cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)[0][0]
                         for line in text_lines)

    bg_top_left = (pos[0] - 5, pos[1] - 15)
    bg_bottom_right = (pos[0] + max_text_width + 10,
                       pos[1] + len(text_lines) * text_height + 5)

    overlay = image.copy()
    cv2.rectangle(overlay, bg_top_left, bg_bottom_right, bg_color, -1)
    cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)

    cv2.rectangle(image, bg_top_left, bg_bottom_right, (255, 255, 255), 1)

    for j, line in enumerate(text_lines):
        line_pos = (pos[0], pos[1] + j * text_height)
        cv2.putText(image, line, line_pos, cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, bg_color, 2)
        cv2.putText(image, line, line_pos, cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, text_color, 1)
