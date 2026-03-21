"""DS435 YOLOv8-OBB 오버레이 유틸리티 (matplotlib 의존성 없음, 앱 import 가능)"""
import sys
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# scripts/ 디렉토리를 경로에 추가 (utils.overlay import용)
_SCRIPTS_DIR = str(Path(__file__).parent.parent)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from utils.overlay import draw_image_center_crosshair

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
    x, y   = pos
    if bg_color:
        bbox = draw.textbbox((x, y), text, font=font)
        pad  = 3
        draw.rectangle([bbox[0]-pad, bbox[1]-pad, bbox[2]+pad, bbox[3]+pad], fill=bg_color)
    draw.text((x, y), text, font=font, fill=color_rgb)
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


def overlay_obb(img_bgr, result):
    """OBB 추론 결과를 이미지에 오버레이."""
    vis = img_bgr.copy()
    if result.obb is None or len(result.obb) == 0:
        draw_image_center_crosshair(vis)
        return vis

    corners = result.obb.xyxyxyxy.cpu().numpy().astype(int)  # [N, 4, 2]
    clss    = result.obb.cls.cpu().numpy().astype(int)
    confs   = result.obb.conf.cpu().numpy()

    for pts, cls, conf in zip(corners, clss, confs):
        color_bgr = COLORS_BGR[cls % len(COLORS_BGR)]
        color_rgb = COLORS_RGB[cls % len(COLORS_RGB)]
        cv2.polylines(vis, [pts.reshape(-1, 1, 2)], isClosed=True, color=color_bgr, thickness=2)
        x_min = int(pts[:, 0].min())
        y_min = int(pts[:, 1].min())
        label = f"{CLASSES[cls]} {conf:.2f}"
        vis = put_text_ko(vis, label, (x_min + 2, max(y_min - 24, 2)),
                          font_size=18, color_rgb=(255, 255, 255), bg_color=color_rgb)
    draw_image_center_crosshair(vis)
    return vis
