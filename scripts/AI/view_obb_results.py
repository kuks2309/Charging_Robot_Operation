#!/home/amap/YOLO_Projects/v8/yolov8_custom/yolov8_env_custom/bin/python
"""
view_results.py - YOLOv8-OBB 추론 결과 시각화 (키 입력으로 이미지 전환)

Usage:
    python view_results.py
    python view_results.py --model runs/obb/yolov8s_obb_robot_sq_mosaic-on_202603116/weights/best.pt
    python view_results.py --source images/val
    python view_results.py --conf 0.3

조작:
    아무 키       → 다음 이미지
    a / ←         → 이전 이미지
    q / ESC       → 종료
    s             → 현재 화면 저장 (comparison_results/)
"""

import argparse
import glob
import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO  # type: ignore

PROJECT    = Path(__file__).parent.resolve()
CLASSES    = ['charging-port']
COLORS_BGR = [(0, 200, 255)]
COLORS_RGB = [(255, 200, 0)]
FONT_PATH  = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


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
    x, y = pos
    if bg_color:
        bbox = draw.textbbox((x, y), text, font=font)
        pad = 3
        draw.rectangle([bbox[0]-pad, bbox[1]-pad, bbox[2]+pad, bbox[3]+pad], fill=bg_color)
    draw.text((x, y), text, font=font, fill=color_rgb)
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


def overlay_obb(img_bgr, result):
    vis = img_bgr.copy()

    if result.obb is None or len(result.obb) == 0:
        return vis

    # xyxyxyxy: [N, 4, 2] 형태의 4 꼭짓점 좌표
    corners = result.obb.xyxyxyxy.cpu().numpy().astype(int)  # [N, 4, 2]
    clss    = result.obb.cls.cpu().numpy().astype(int)
    confs   = result.obb.conf.cpu().numpy()

    for pts, cls, conf in zip(corners, clss, confs):
        color_bgr = COLORS_BGR[cls % len(COLORS_BGR)]
        color_rgb = COLORS_RGB[cls % len(COLORS_RGB)]

        # OBB 박스 그리기 (4 꼭짓점 연결)
        poly = pts.reshape((-1, 1, 2))
        cv2.polylines(vis, [poly], isClosed=True, color=color_bgr, thickness=2)

        # 라벨 표시 (첫 번째 꼭짓점 기준)
        x_min = pts[:, 0].min()
        y_min = pts[:, 1].min()
        label = f"{CLASSES[cls]} {conf:.2f}"
        vis = put_text_ko(vis, label, (x_min + 2, max(y_min - 24, 2)),
                          font_size=18, color_rgb=(255, 255, 255),
                          bg_color=color_rgb)

    return vis


def draw_hud(vis, idx, total, filename, n_det):
    h, w = vis.shape[:2]

    bar = vis.copy()
    cv2.rectangle(bar, (0, 0), (w, 32), (30, 30, 30), -1)
    vis = cv2.addWeighted(vis, 0.5, bar, 0.5, 0)

    info = f"[{idx+1}/{total}]  {filename}  |  검출: {n_det}개"
    vis = put_text_ko(vis, info, (8, 6), font_size=17, color_rgb=(240, 240, 240))

    legend_bg = vis.copy()
    legend_h = len(CLASSES) * 26 + 8
    cv2.rectangle(legend_bg, (0, h - legend_h), (200, h), (30, 30, 30), -1)
    vis = cv2.addWeighted(vis, 0.5, legend_bg, 0.5, 0)

    for ci, (cls_name, color_bgr) in enumerate(zip(CLASSES, COLORS_BGR)):
        ly = h - legend_h + 8 + ci * 26
        cv2.rectangle(vis, (8, ly + 2), (24, ly + 18), color_bgr, -1)
        vis = put_text_ko(vis, cls_name, (30, ly), font_size=16, color_rgb=(220, 220, 220))

    guide = "any:next  a:prev  s:save  q:quit"
    vis = put_text_ko(vis, guide, (w - 280, h - 26), font_size=15, color_rgb=(180, 180, 180))
    return vis


def render_frame(img_files, idx, total, model, conf, iou):
    imgf    = img_files[idx]
    img_bgr = cv2.imread(imgf)
    if img_bgr is None:
        return None, 0, imgf, None

    results = model(img_bgr, conf=conf, iou=iou, verbose=False)
    result  = results[0]

    vis   = overlay_obb(img_bgr, result)
    n_det = len(result.obb) if result.obb is not None else 0
    vis   = draw_hud(vis, idx, total, Path(imgf).name, n_det)

    vis_rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
    return vis_rgb, n_det, imgf, vis


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model",  default="runs/obb/yolov8s_obb_robot_sq_mosaic-on_202603116/weights/best.pt")
    parser.add_argument("--source", default="images/val")
    parser.add_argument("--conf",   type=float, default=0.25)
    parser.add_argument("--iou",    type=float, default=0.65)
    return parser.parse_args()


def main():
    args = parse_args()
    model_path = PROJECT / args.model
    source_dir = PROJECT / args.source
    out_dir    = PROJECT / "comparison_results"
    out_dir.mkdir(exist_ok=True)

    print(f"모델 : {model_path}")
    print(f"소스 : {source_dir}")
    print(f"Conf : {args.conf}")

    if not model_path.exists():
        print(f"ERROR: 모델 파일 없음 → {model_path}")
        return

    model = YOLO(str(model_path))

    img_files = sorted(
        glob.glob(str(source_dir / "*.jpg")) +
        glob.glob(str(source_dir / "*.JPG")) +
        glob.glob(str(source_dir / "*.png")) +
        glob.glob(str(source_dir / "*.PNG"))
    )
    if not img_files:
        print(f"이미지 없음: {source_dir}")
        return

    total = len(img_files)
    print(f"이미지 {total}장 | any:next  a:prev  s:save  q:quit")

    state = {'idx': 0, 'vis_bgr': None, 'imgf': None}

    fig, ax = plt.subplots(figsize=(13, 8))
    fig.patch.set_facecolor('black')
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax.axis('off')

    im_handle: list = [None]

    def update_display():
        idx = state['idx']
        result = render_frame(img_files, idx, total, model, args.conf, args.iou)
        if result[0] is None:
            return
        vis_rgb, n_det, imgf, vis_bgr = result
        state['vis_bgr'] = vis_bgr
        state['imgf']    = imgf

        if im_handle[0] is None:
            im_handle[0] = ax.imshow(vis_rgb)
        else:
            im_handle[0].set_data(vis_rgb)
            im_handle[0].set_extent([0, vis_rgb.shape[1], vis_rgb.shape[0], 0])
            ax.set_xlim(0, vis_rgb.shape[1])
            ax.set_ylim(vis_rgb.shape[0], 0)

        ax.set_title(
            f"[{idx+1}/{total}] {Path(imgf).name}  검출:{n_det}개",
            color='white', fontsize=11, pad=4
        )
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        print(f"  [{idx+1}/{total}] {Path(imgf).name}  검출:{n_det}개")

    def on_key(event):
        key = event.key
        if key in ('q', 'escape'):
            print("종료")
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


if __name__ == "__main__":
    main()
