#!/usr/bin/env python3
"""
view_results.py - YOLOv8-seg 추론 결과 시각화 (파일 / ArduCam 카메라 모드)

Usage:
    # 파일 모드 (이미지 디렉토리 네비게이션)
    python view_results.py
    python view_results.py --source ../../data/AI/ArduCam/images/val
    python view_results.py --model ../../config/AI_weights/ArduCam/best.onnx

    # 카메라 모드 (ArduCam 실시간)
    python view_results.py --source camera       # 기본 인덱스 0
    python view_results.py --source 0            # 카메라 인덱스 직접 지정
    python view_results.py --source 2 --conf 0.3

조작 (파일 모드):
    아무 키       → 다음 이미지
    a / ←         → 이전 이미지
    q / ESC       → 종료
    s             → 현재 화면 저장 (comparison_results/)

조작 (카메라 모드):
    q / ESC       → 종료
    s             → 현재 화면 저장
    Space         → 일시정지 / 재개
"""

import argparse
import glob
import math
import os
from dataclasses import dataclass
from itertools import combinations
import cv2
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO  # type: ignore

# GPU 환경이라도 CPU 강제 사용
os.environ['CUDA_VISIBLE_DEVICES'] = ''

PROJECT    = Path(__file__).parent.resolve()
CLASSES    = ['Circle_A', 'Port_T']
COLORS_BGR = [(80, 220, 0), (255, 100, 0)]
COLORS_RGB = [(0, 220, 80), (0, 100, 255)]
MASK_ALPHA = 0.45
FONT_PATH  = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


@dataclass
class Det:
    cls: str    # 'Circle_A' or 'Port_T'
    cx: float
    cy: float
    conf: float


def get_font(size=18):
    try:
        return ImageFont.truetype(FONT_PATH, size)
    except Exception:
        return ImageFont.load_default()


def put_text_ko(img_bgr, text, pos, font_size=18, color_rgb=(240, 240, 240), bg_color=None):
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)
    draw    = ImageDraw.Draw(pil_img)
    font    = get_font(font_size)
    x, y   = pos
    if bg_color:
        bbox = draw.textbbox((x, y), text, font=font)
        pad  = 3
        draw.rectangle([bbox[0]-pad, bbox[1]-pad, bbox[2]+pad, bbox[3]+pad], fill=bg_color)
    draw.text((x, y), text, font=font, fill=color_rgb)
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


def parse_detections(result) -> list:
    """YOLO result → Det 리스트"""
    dets = []
    if result.boxes is None:
        return dets
    clss  = result.boxes.cls.cpu().numpy().astype(int)
    confs = result.boxes.conf.cpu().numpy()
    xyxy  = result.boxes.xyxy.cpu().numpy()
    for cls_idx, conf, box in zip(clss, confs, xyxy):
        cx = (box[0] + box[2]) / 2
        cy = (box[1] + box[3]) / 2
        dets.append(Det(cls=CLASSES[cls_idx % len(CLASSES)],
                        cx=cx, cy=cy, conf=conf))
    return dets


def draw_center_crosshair(vis):
    """이미지 중심 십자선 표시"""
    h, w = vis.shape[:2]
    cx, cy = w // 2, h // 2
    color = (200, 200, 200)  # 밝은 회색
    # 수평선 (중앙 30% 구간 빈 공간)
    gap = 40
    cv2.line(vis, (0, cy), (cx - gap, cy), color, 1, cv2.LINE_AA)
    cv2.line(vis, (cx + gap, cy), (w, cy), color, 1, cv2.LINE_AA)
    # 수직선 (중앙 30% 구간 빈 공간)
    cv2.line(vis, (cx, 0), (cx, cy - gap), color, 1, cv2.LINE_AA)
    cv2.line(vis, (cx, cy + gap), (cx, h), color, 1, cv2.LINE_AA)
    # 중심 원
    cv2.circle(vis, (cx, cy), 8, color, 1, cv2.LINE_AA)
    cv2.circle(vis, (cx, cy), 2, color, -1)
    return vis


def draw_align_overlay(vis, port_cy, img_cy, dy_px):
    """Port Center Align 결과 오버레이: port_cy 선 + dy 화살표"""
    h, w = vis.shape[:2]
    port_y = int(port_cy)
    img_y  = int(img_cy)

    # Port 중심선 (주황색 실선)
    cv2.line(vis, (0, port_y), (w, port_y), (0, 140, 255), 2, cv2.LINE_AA)

    # dy 화살표 (이미지 우측)
    arrow_x = w - 60
    cv2.arrowedLine(vis,
                    (arrow_x, img_y),
                    (arrow_x, port_y),
                    (0, 220, 255), 2, cv2.LINE_AA, tipLength=0.15)

    # dy 수치 텍스트 (우측 패널)
    label = f"dy {dy_px:+.1f}px"
    tx = w - 150
    ty = (img_y + port_y) // 2 - 10
    vis = put_text_ko(vis, label, (tx, ty),
                      font_size=16, color_rgb=(0, 220, 255),
                      bg_color=(20, 20, 20))
    return vis


def draw_horizontal_align_overlay(vis, port_cx, img_cx, dx_px):
    """수평 정렬 오버레이: 시안색 수직선 + 하단 화살표 + dx 텍스트.

    draw_align_overlay(수직, 오렌지, 우측 엣지)와 시각적으로 구분:
      - 색상: 시안 (255, 200, 0) BGR
      - 기준선: 수직선 (full height) at ref_cx
      - 화살표: 이미지 하단 수평 화살표 (ref_cx → img_cx)
      - 텍스트: dx 값 (하단 좌측)

    Args:
        vis: BGR 이미지 (draw_align_overlay 이후 전달 권장)
        port_cx: 커플러 수평 기준점 X (px)
        img_cx:  이미지 수평 중심 X (px)
        dx_px:   수평 오프셋 (port_cx - img_cx, px)

    Returns:
        vis (수정된 이미지)
    """
    h, w = vis.shape[:2]
    ref_cx_int = max(0, min(w - 1, int(port_cx)))
    img_cx_int = int(img_cx)

    COLOR = (255, 200, 0)  # 시안 BGR

    # 수직 기준선 (full height)
    cv2.line(vis, (ref_cx_int, 0), (ref_cx_int, h), COLOR, 2, cv2.LINE_AA)

    # 하단 수평 화살표 (ref_cx → img_cx)
    arrow_y = h - 40
    cv2.arrowedLine(vis,
                    (ref_cx_int, arrow_y),
                    (img_cx_int, arrow_y),
                    COLOR, 2, cv2.LINE_AA, tipLength=0.15)

    # dx 수치 텍스트 (하단 좌측 — dy 텍스트 우측과 겹치지 않도록)
    label = f"dx={dx_px:+.1f}px"
    tx = 10
    ty = h - 70
    vis = put_text_ko(vis, label, (tx, ty),
                      font_size=16, color_rgb=(0, 200, 255),
                      bg_color=(20, 20, 20))
    return vis


def overlay_ry_angles(vis, dets: list):
    h, w = vis.shape[:2]

    circles = [d for d in dets if d.cls == 'Circle_A']
    ports   = [d for d in dets if d.cls == 'Port_T']

    angles = {}  # source_key -> angle_deg

    # --- 소스 1: Circle_A 수평 쌍 (가장 수평 거리가 먼 2개) ---
    color_h = (0, 220, 0)
    if len(circles) >= 2:
        best_pair = max(combinations(circles, 2),
                        key=lambda p: abs(p[0].cx - p[1].cx))
        ca, cb = best_pair
        dx = cb.cx - ca.cx
        dy = cb.cy - ca.cy
        angle_h = -math.degrees(math.atan2(dy, dx))
        angles['H'] = angle_h

        # 선 그리기
        pt1 = (int(ca.cx), int(ca.cy))
        pt2 = (int(cb.cx), int(cb.cy))
        cv2.line(vis, pt1, pt2, color_h, 2)
        cv2.circle(vis, pt1, 6, color_h, -1)
        cv2.circle(vis, pt2, 6, color_h, -1)
        mid_x = int((ca.cx + cb.cx) / 2)
        mid_y = int((ca.cy + cb.cy) / 2)
        label_h = f"H {angle_h:+.1f}"
        vis = put_text_ko(vis, label_h, (mid_x + 8, mid_y - 12),
                          font_size=16, color_rgb=tuple(reversed(color_h)),
                          bg_color=(0, 30, 0))

    # --- 소스 2: Circle_A 삼각형 대칭축 ---
    color_s = (0, 200, 255)
    if len(circles) == 3:
        sorted_c = sorted(circles, key=lambda d: d.cy)
        top1, top2, bottom = sorted_c[0], sorted_c[1], sorted_c[2]
        mid_cx = (top1.cx + top2.cx) / 2
        mid_cy = (top1.cy + top2.cy) / 2
        dx = bottom.cx - mid_cx
        dy = bottom.cy - mid_cy
        angle_s = -math.degrees(math.atan2(dx, dy))
        angles['S'] = angle_s

        pt_top = (int(mid_cx), int(mid_cy))
        pt_bot = (int(bottom.cx), int(bottom.cy))
        cv2.line(vis, pt_top, pt_bot, color_s, 2)
        cv2.circle(vis, pt_top, 6, color_s, -1)
        cv2.circle(vis, pt_bot, 6, color_s, -1)
        sym_mid_x = int((mid_cx + bottom.cx) / 2)
        sym_mid_y = int((mid_cy + bottom.cy) / 2)
        label_s = f"S {angle_s:+.1f}"
        vis = put_text_ko(vis, label_s, (sym_mid_x + 8, sym_mid_y - 12),
                          font_size=16, color_rgb=tuple(reversed(color_s)),
                          bg_color=(0, 20, 30))

    # --- 소스 3: Port_T 수직 쌍 ---
    color_v = (255, 100, 0)
    if len(ports) >= 2:
        pt_top    = min(ports, key=lambda d: d.cy)
        pt_bottom = max(ports, key=lambda d: d.cy)
        dx = pt_bottom.cx - pt_top.cx
        dy = pt_bottom.cy - pt_top.cy
        angle_v = -math.degrees(math.atan2(dx, dy))
        angles['V'] = angle_v

        pp1 = (int(pt_top.cx),    int(pt_top.cy))
        pp2 = (int(pt_bottom.cx), int(pt_bottom.cy))
        cv2.line(vis, pp1, pp2, color_v, 2)
        cv2.circle(vis, pp1, 6, color_v, -1)
        cv2.circle(vis, pp2, 6, color_v, -1)
        v_mid_x = int((pt_top.cx + pt_bottom.cx) / 2)
        v_mid_y = int((pt_top.cy + pt_bottom.cy) / 2)
        label_v = f"V {angle_v:+.1f}"
        vis = put_text_ko(vis, label_v, (v_mid_x + 8, v_mid_y - 12),
                          font_size=16, color_rgb=tuple(reversed(color_v)),
                          bg_color=(30, 10, 0))

    if not angles:
        return vis

    # --- 아웃라이어 필터: 2소스 이상 있을 때 median ±5° 초과 제거 ---
    if len(angles) >= 2:
        vals = list(angles.values())
        median_val = sorted(vals)[len(vals) // 2]
        filtered = {k: v for k, v in angles.items() if abs(v - median_val) <= 5.0}
        if filtered:
            angles = filtered

    # --- 우상단 요약 패널 ---
    panel_x = w - 160
    panel_y = 40
    line_h  = 24

    source_colors = {'H': (0, 220, 0), 'S': (0, 200, 255), 'V': (255, 100, 0)}
    valid_keys = [k for k in ('H', 'S', 'V') if k in angles]

    panel_lines = len(valid_keys) + (1 if valid_keys else 0)
    panel_h = panel_lines * line_h + 8
    overlay_bg = vis.copy()
    cv2.rectangle(overlay_bg, (panel_x - 4, panel_y - 4),
                  (w - 4, panel_y + panel_h), (30, 30, 30), -1)
    vis = cv2.addWeighted(vis, 0.4, overlay_bg, 0.6, 0)

    row = 0
    for key in valid_keys:
        ang = angles[key]
        color_bgr = source_colors[key]
        text = f"{key} {ang:+.1f}"
        vis = put_text_ko(vis, text,
                          (panel_x, panel_y + row * line_h),
                          font_size=16,
                          color_rgb=tuple(reversed(color_bgr)),
                          bg_color=None)
        row += 1

    if valid_keys:
        ry_mean = sum(angles[k] for k in valid_keys) / len(valid_keys)
        ry_text = f"Ry {ry_mean:+.1f}"
        vis = put_text_ko(vis, ry_text,
                          (panel_x, panel_y + row * line_h),
                          font_size=16,
                          color_rgb=(0, 255, 255),
                          bg_color=None)

        # --- 수직 기준 회전각도 지시계 ---
        # 커플러 중심: 전체 Det 평균
        all_cx = [d.cx for d in dets]
        all_cy = [d.cy for d in dets]
        cx0 = int(sum(all_cx) / len(all_cx))
        cy0 = int(sum(all_cy) / len(all_cy))

        ARM = 80  # 지시선 길이 (px)
        θ   = math.radians(ry_mean)

        # 수직 기준선 (위쪽, 흰색 점선)
        v_end = (cx0, cy0 - ARM)
        for i in range(0, ARM, 10):
            p1 = (cx0, cy0 - i)
            p2 = (cx0, max(cy0 - i - 6, cy0 - ARM))
            cv2.line(vis, p1, p2, (200, 200, 200), 1, cv2.LINE_AA)

        # 측정 방향선 (노란색)
        r_end = (int(cx0 + ARM * math.sin(θ)),
                 int(cy0 - ARM * math.cos(θ)))
        cv2.line(vis, (cx0, cy0), r_end, (0, 220, 220), 2, cv2.LINE_AA)
        cv2.circle(vis, (cx0, cy0), 4, (0, 220, 220), -1)

        # 호(arc): 수직(-90°)에서 ry_mean만큼 회전
        arc_r = 40
        start_ang = -90.0
        end_ang   = -90.0 + ry_mean
        if abs(ry_mean) > 0.5:
            cv2.ellipse(vis, (cx0, cy0), (arc_r, arc_r), 0,
                        min(start_ang, end_ang),
                        max(start_ang, end_ang),
                        (0, 220, 220), 1, cv2.LINE_AA)

        # 각도 수치 (호 바깥쪽)
        label_ang = f"{ry_mean:+.1f}"
        lx = cx0 + int((arc_r + 6) * math.sin(θ / 2))
        ly = cy0 - int((arc_r + 6) * math.cos(θ / 2))
        vis = put_text_ko(vis, label_ang, (lx + 4, ly - 10),
                          font_size=15, color_rgb=(0, 220, 220),
                          bg_color=(20, 20, 20))

    return vis


def overlay_masks(img_bgr, result):
    vis = img_bgr.copy()

    if result.boxes is None:
        return vis

    boxes = result.boxes
    clss  = boxes.cls.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()
    xyxy  = boxes.xyxy.cpu().numpy().astype(int)

    # 마스크: masks.xy는 원본 이미지 좌표계의 polygon 점 목록
    # masks.data (letterbox 640x640 포함)를 직접 resize하면 위치 틀어짐 → xy 사용
    seg_polygons = result.masks.xy if result.masks is not None else [None] * len(clss)

    # 1단계: 마스크 오버레이 (bbox/레이블보다 먼저)
    for i, (seg_pts, cls) in enumerate(zip(seg_polygons, clss)):
        if seg_pts is not None and len(seg_pts) >= 3:
            color_bgr = COLORS_BGR[cls % len(COLORS_BGR)]
            pts = seg_pts.astype(np.int32).reshape(-1, 1, 2)
            overlay = vis.copy()
            cv2.fillPoly(overlay, [pts], color_bgr)
            vis = cv2.addWeighted(vis, 1 - MASK_ALPHA, overlay, MASK_ALPHA, 0)
            cv2.polylines(vis, [pts], True, color_bgr, 2)

    # bbox + 레이블 (나중에, 마스크 위에)
    dets = []
    used_label_y: list = []   # 사용된 y 위치 추적 (겹침 방지)
    for seg_pts, cls, conf, box in zip(seg_polygons, clss, confs, xyxy):
        color_bgr = COLORS_BGR[cls % len(COLORS_BGR)]
        color_rgb = COLORS_RGB[cls % len(COLORS_RGB)]

        x1, y1, x2, y2 = box
        cv2.rectangle(vis, (x1, y1), (x2, y2), color_bgr, 2)

        # 레이블 y 위치 — bbox 위에, 화면 상단 벗어나면 bbox 안으로
        # 이미 사용된 y와 겹치면 26px씩 아래로 이동
        label_y = y1 - 26 if y1 - 26 >= 4 else y1 + 4
        while any(abs(label_y - uy) < 22 for uy in used_label_y):
            label_y += 26
        used_label_y.append(label_y)

        label = f"{CLASSES[cls]} {conf:.2f}"
        vis = put_text_ko(vis, label, (x1 + 2, label_y),
                          font_size=18, color_rgb=(255, 255, 255),
                          bg_color=color_rgb)

        dets.append(Det(cls=CLASSES[cls % len(CLASSES)],
                        cx=(x1 + x2) / 2.0,
                        cy=(y1 + y2) / 2.0,
                        conf=conf))

    vis = overlay_ry_angles(vis, dets)
    vis = draw_center_crosshair(vis)
    return vis


def draw_hud(vis, info_text, guide_text):
    h, w = vis.shape[:2]

    bar = vis.copy()
    cv2.rectangle(bar, (0, 0), (w, 32), (30, 30, 30), -1)
    vis = cv2.addWeighted(vis, 0.5, bar, 0.5, 0)
    vis = put_text_ko(vis, info_text, (8, 6), font_size=17, color_rgb=(240, 240, 240))

    legend_h  = len(CLASSES) * 26 + 8
    legend_bg = vis.copy()
    cv2.rectangle(legend_bg, (0, h - legend_h), (200, h), (30, 30, 30), -1)
    vis = cv2.addWeighted(vis, 0.5, legend_bg, 0.5, 0)

    for ci, (cls_name, color_bgr) in enumerate(zip(CLASSES, COLORS_BGR)):
        ly = h - legend_h + 8 + ci * 26
        cv2.rectangle(vis, (8, ly + 2), (24, ly + 18), color_bgr, -1)
        vis = put_text_ko(vis, cls_name, (30, ly), font_size=16, color_rgb=(220, 220, 220))

    vis = put_text_ko(vis, guide_text, (w - 340, h - 26), font_size=15, color_rgb=(180, 180, 180))
    return vis


# ──────────────────────────────────────────────────────
# 파일 모드
# ──────────────────────────────────────────────────────

def run_file_mode(img_files, model, conf, iou, out_dir):
    total = len(img_files)
    print(f"이미지 {total}장 로드 | any:next  a:prev  s:save  q:quit")

    state = {'idx': 0, 'vis_bgr': None, 'imgf': None}
    fig, ax = plt.subplots(figsize=(6.5, 4))
    fig.patch.set_facecolor('black')
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax.axis('off')
    im_handle = [None]

    def update_display():
        idx     = state['idx']
        imgf    = img_files[idx]
        img_bgr = cv2.imread(imgf)
        if img_bgr is None:
            return

        results = model(img_bgr, conf=conf, iou=iou, verbose=False, device='cpu')
        result  = results[0]
        vis     = overlay_masks(img_bgr, result)
        n_det   = len(result.boxes) if result.boxes is not None else 0

        info  = f"[{idx+1}/{total}]  {Path(imgf).name}  |  검출: {n_det}개"
        guide = "any:next  a:prev  s:save  q:quit"
        vis   = draw_hud(vis, info, guide)

        state['vis_bgr'] = vis
        state['imgf']    = imgf

        vis_rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        if im_handle[0] is None:
            im_handle[0] = ax.imshow(vis_rgb)
        else:
            im_handle[0].set_data(vis_rgb)
            im_handle[0].set_extent([0, vis_rgb.shape[1], vis_rgb.shape[0], 0])
            ax.set_xlim(0, vis_rgb.shape[1])
            ax.set_ylim(vis_rgb.shape[0], 0)

        ax.set_title(f"[{idx+1}/{total}] {Path(imgf).name}  검출:{n_det}개",
                     color='white', fontsize=11, pad=4)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        print(f"  [{idx+1}/{total}] {Path(imgf).name}  검출:{n_det}개")

    def on_key(event):
        key = event.key
        if key in ('q', 'escape'):
            plt.close(fig)
        elif key == 's':
            vis_bgr = state.get('vis_bgr')
            imgf    = state.get('imgf')
            if vis_bgr is not None and imgf is not None:
                save_path = out_dir / f"result_{Path(imgf).stem}.png"
                cv2.imwrite(str(save_path), vis_bgr)
                print(f"  저장: {save_path}")
            state['idx'] = min(state['idx'] + 1, total - 1)
            update_display()
        elif key in ('a', 'left'):
            state['idx'] = max(0, state['idx'] - 1)
            update_display()
        else:
            if state['idx'] < total - 1:
                state['idx'] += 1
                update_display()
            else:
                print("마지막 이미지입니다. q로 종료하세요.")

    fig.canvas.mpl_connect('key_press_event', on_key)
    update_display()
    plt.show()


# ──────────────────────────────────────────────────────
# 카메라 모드 (ArduCam)
# ──────────────────────────────────────────────────────

def run_camera_mode(cam_index, model, conf, iou, out_dir):
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"ERROR: 카메라 {cam_index} 열기 실패")
        return

    # ArduCam 해상도 요청 (1920x1080)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"카메라 {cam_index} 시작 ({w}x{h}) | s:save  Space:일시정지  q:quit")

    state = {'paused': False, 'frame_cnt': 0, 'last_vis': None, 'running': True}

    fig, ax = plt.subplots(figsize=(6.5, 4))
    fig.patch.set_facecolor('black')
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax.axis('off')
    im_handle = [None]

    def on_key(event):
        key = event.key
        if key in ('q', 'escape'):
            state['running'] = False
            plt.close(fig)
        elif key == 's':
            vis = state['last_vis']
            if vis is not None:
                save_path = out_dir / f"result_cam{cam_index}_{state['frame_cnt']:06d}.png"
                cv2.imwrite(str(save_path), vis)
                print(f"  저장: {save_path}")
        elif key == ' ':
            state['paused'] = not state['paused']
            print("일시정지" if state['paused'] else "재개")

    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_):
        if not state['running']:
            return
        if state['paused']:
            return
        ret, frame = cap.read()
        if not ret:
            print("프레임 읽기 실패")
            state['running'] = False
            plt.close(fig)
            return

        state['frame_cnt'] += 1

        if state['frame_cnt'] % 3 == 0:
            results = model(frame, conf=conf, iou=iou, verbose=False, device='cpu')
            result  = results[0]
            vis     = overlay_masks(frame, result)
            n_det   = len(result.boxes) if result.boxes is not None else 0

            info  = f"카메라 {cam_index}  {w}x{h}  |  검출: {n_det}개  [#{state['frame_cnt']}]"
            guide = "s:save  Space:일시정지  q:quit"
            vis_hud = draw_hud(vis, info, guide)
            state['last_vis'] = vis_hud
        elif state['last_vis'] is None:
            return

        vis_hud = state['last_vis']
        vis_rgb = cv2.cvtColor(vis_hud, cv2.COLOR_BGR2RGB)
        if im_handle[0] is None:
            im_handle[0] = ax.imshow(vis_rgb)
            ax.set_xlim(0, vis_rgb.shape[1])
            ax.set_ylim(vis_rgb.shape[0], 0)
        else:
            im_handle[0].set_data(vis_rgb)
        fig.canvas.draw_idle()

    ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()
    cap.release()


# ──────────────────────────────────────────────────────
# 진입점
# ──────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="YOLOv8-seg 추론 결과 시각화 (파일 / ArduCam 카메라)"
    )
    parser.add_argument(
        "--model",
        default="../../config/AI_weights/ArduCam/best.onnx",
        help="모델 경로 (.pt 또는 .onnx)"
    )
    parser.add_argument(
        "--source",
        default="../../data/AI/ArduCam/images/val",
        help="이미지 디렉토리 경로, 'camera', 또는 카메라 인덱스 (0, 1, ...)"
    )
    parser.add_argument("--conf", type=float, default=0.50, help="신뢰도 임계값")
    parser.add_argument("--iou",  type=float, default=0.65, help="NMS IOU 임계값")
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = PROJECT / "comparison_results"
    out_dir.mkdir(exist_ok=True)

    model_path = PROJECT / args.model
    if not model_path.exists():
        print(f"ERROR: 모델 파일 없음 → {model_path}")
        return

    print(f"모델   : {model_path}")
    print(f"소스   : {args.source}")
    print(f"Conf   : {args.conf}  IOU : {args.iou}")
    print(f"디바이스: CPU (강제)")

    model = YOLO(str(model_path), task='segment')

    src = args.source.strip()
    if src == 'camera' or src.isdigit():
        cam_index = 0 if src == 'camera' else int(src)
        run_camera_mode(cam_index, model, args.conf, args.iou, out_dir)
    else:
        source_dir = PROJECT / src
        img_files  = sorted(
            glob.glob(str(source_dir / "*.jpg")) +
            glob.glob(str(source_dir / "*.JPG")) +
            glob.glob(str(source_dir / "*.png")) +
            glob.glob(str(source_dir / "*.PNG"))
        )
        if not img_files:
            print(f"이미지 없음: {source_dir}")
            return
        run_file_mode(img_files, model, args.conf, args.iou, out_dir)


if __name__ == "__main__":
    main()
