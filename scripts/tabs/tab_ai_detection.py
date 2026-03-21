"""AI Detection 탭 — 카메라 피드 표시 + ArduCam YOLOv8-seg 검출"""
import enum
import time
import threading
from pathlib import Path
from typing import Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox, QPushButton
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
import json
import numpy as np
import cv2

_AI_DIR = Path(__file__).parent.parent / "AI"
_MODEL_PATH = Path(__file__).parent.parent.parent / "config/AI_weights/ArduCam/best.onnx"
_DS435_MODEL_PATH = Path(__file__).parent.parent.parent / "config/AI_weights/DS435/best.pt"


from services.port_alignment_service import (
    compute_vertical_alignment, compute_horizontal_alignment,
    compute_ry_angle, PortAlignmentService,
)
from services.laser_detection_service import LaserDetectionService
from utils.image_processing import (
    draw_laser_calib_roi as _draw_laser_calib_roi,
    draw_laser_fit_line,
    compute_roi_rects,
    draw_laser_reference_line,
)


class LaserState(enum.IntEnum):
    OFF         = 0   # 레이저 기능 비활성
    ROI_VISIBLE = 1   # ROI 박스 표시 중 (실시간 검출 진행)
    ALIGNING    = 2   # 로봇 정렬 스레드 실행 중


class TabAIDetection(QWidget):
    """AI Detection 탭: DS435 + ArduCam 듀얼 카메라 뷰, ArduCam YOLOv8-seg 검출"""

    log_message = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.robot = None
        self._align_svc = PortAlignmentService(log_fn=self._emit_log)
        # 검출 상태
        self._detect_active = False
        self._yolo_model = None
        self._overlay_fn = None
        self._parse_fn = None
        self._align_overlay_fn = None
        self._horiz_align_overlay_fn = None
        self._bbox_center_fn = None
        # 백그라운드 추론 상태
        self._detect_running = False
        self._laser_state = LaserState.OFF        # 레이저 모드 상태 머신
        self._laser_calib_roi = None              # laser_vertical_calib_roi.json roi 섹션 캐시
        self._laser_ref_cfg = None                # laser_vertical_calib_roi.json reference_line 섹션 캐시
        self._last_laser_results: dict = {'left': None, 'right': None}
        self._last_overlay = None   # 마지막 추론 결과 프레임
        self._last_dets = []        # 마지막 Det 리스트
        self._last_frame_shape = (1080, 1920)  # (h, w) 기본값
        self._detect_lock = threading.Lock()
        self._align_result = None   # (port_cy, img_cy, dy_px) or None
        self._last_port_t_ymax = None  # 최하단 Port_T bbox bottom y (px)
        self._last_port_t_xmin = None  # Port_T 전체 좌측 경계 x (px)
        self._last_port_t_xmax = None  # Port_T 전체 우측 경계 x (px)
        self._last_raw_frame = None    # 레이저 검출용 원본 프레임
        # DS435 OBB 검출 상태
        self._ds435_detect_active = False
        self._ds435_model = None
        self._ds435_overlay_fn = None
        self._ds435_detect_running = False
        self._ds435_last_overlay = None
        self._ds435_last_center = None   # OBB 중심 (cx, cy) px
        self._ds435_frame_shape = (480, 640)  # DS435 기본 해상도 (h, w)
        self._ds435_detect_lock = threading.Lock()
        self._build_layout()

    def _emit_log(self, msg: str):
        """서비스 레이어 로그 → Qt signal 브릿지"""
        self.log_message.emit(msg)

    def _build_layout(self):
        outer = QVBoxLayout(self)

        camera_row = QHBoxLayout()

        # DS435 뷰
        ds435_col = QVBoxLayout()
        ds435_title = QLabel("DS435")
        ds435_title.setAlignment(Qt.AlignCenter)
        ds435_col.addWidget(ds435_title)
        self.labelDS435View = QLabel()
        self.labelDS435View.setFixedSize(640, 360)
        self.labelDS435View.setAlignment(Qt.AlignCenter)
        self.labelDS435View.setStyleSheet("background-color: #222;")
        ds435_col.addWidget(self.labelDS435View)
        camera_row.addLayout(ds435_col)

        # ArduCam 뷰
        arducam_col = QVBoxLayout()
        arducam_title = QLabel("ArduCam")
        arducam_title.setAlignment(Qt.AlignCenter)
        arducam_col.addWidget(arducam_title)
        self.labelArduCamView = QLabel()
        self.labelArduCamView.setFixedSize(640, 360)
        self.labelArduCamView.setAlignment(Qt.AlignCenter)
        self.labelArduCamView.setStyleSheet("background-color: #222;")
        arducam_col.addWidget(self.labelArduCamView)
        camera_row.addLayout(arducam_col)

        camera_row.addStretch()
        outer.addLayout(camera_row)

        # DS435 AI Detection 그룹박스
        self.groupDS435AI = QGroupBox("DS435 AI Detection")
        self.groupDS435AI.setFixedHeight(100)
        ds435_ai_layout = QHBoxLayout(self.groupDS435AI)

        self.btnPortDetect = QPushButton("Port detect")
        self.btnPortDetect.setCheckable(True)
        self.btnPortDetect.clicked.connect(self._on_port_detect_toggled)
        ds435_ai_layout.addWidget(self.btnPortDetect)

        self.btnDS435VertAlign = QPushButton("수직 정렬")
        self.btnDS435VertAlign.clicked.connect(self._on_ds435_vert_align_clicked)
        ds435_ai_layout.addWidget(self.btnDS435VertAlign)

        self.btnDS435HorizAlign = QPushButton("수평 정렬")
        self.btnDS435HorizAlign.clicked.connect(self._on_ds435_horiz_align_clicked)
        ds435_ai_layout.addWidget(self.btnDS435HorizAlign)

        ds435_ai_layout.addStretch()
        outer.addWidget(self.groupDS435AI)

        # Arducam AI Detection 그룹박스
        self.groupArducamAI = QGroupBox("Arducam AI Detection")
        self.groupArducamAI.setFixedHeight(100)
        arducam_ai_layout = QHBoxLayout(self.groupArducamAI)

        self.btnPortHallDetect = QPushButton("Port hall detect")
        self.btnPortHallDetect.setCheckable(True)
        self.btnPortHallDetect.clicked.connect(self._on_port_hall_detect_toggled)
        arducam_ai_layout.addWidget(self.btnPortHallDetect)

        self.btnPortCenterAlign = QPushButton("수직 정렬")
        self.btnPortCenterAlign.clicked.connect(self._on_port_center_align_clicked)
        arducam_ai_layout.addWidget(self.btnPortCenterAlign)

        self.btnRyCorrect = QPushButton("Ry 보정")
        self.btnRyCorrect.clicked.connect(self._on_ry_correct_clicked)
        arducam_ai_layout.addWidget(self.btnRyCorrect)

        self.btnHorizontalAlign = QPushButton("수평 정렬")
        self.btnHorizontalAlign.clicked.connect(self._on_horizontal_align_clicked)
        arducam_ai_layout.addWidget(self.btnHorizontalAlign)

        arducam_ai_layout.addStretch()

        outer.addWidget(self.groupArducamAI)

        # 자동차 충전건 결합 그룹박스
        self.groupCoupling = QGroupBox("자동차 충전건 결합")
        self.groupCoupling.setFixedHeight(100)
        coupling_layout = QHBoxLayout(self.groupCoupling)

        self.btnMoveToChargingPos = QPushButton("차량 충전 위치 이동")
        coupling_layout.addWidget(self.btnMoveToChargingPos)

        self.btnCouplingPortDetect = QPushButton("DS435 포트 탐색")
        coupling_layout.addWidget(self.btnCouplingPortDetect)

        self.btnDS435ChargingAlign = QPushButton("DS435 충전 정렬")
        self.btnDS435ChargingAlign.clicked.connect(self._on_ds435_charging_align_clicked)
        coupling_layout.addWidget(self.btnDS435ChargingAlign)

        self.btnHandoff = QPushButton("ArduCam 충전위치 변환")
        self.btnHandoff.clicked.connect(self._on_handoff_clicked)
        coupling_layout.addWidget(self.btnHandoff)

        self.btnArducamChargingAlign = QPushButton("ArduCam 충전 정렬")
        self.btnArducamChargingAlign.clicked.connect(self._on_arducam_charging_align_clicked)
        coupling_layout.addWidget(self.btnArducamChargingAlign)

        self.btnLaserAlign = QPushButton("Laser 정렬")
        self.btnLaserAlign.clicked.connect(self._on_laser_align_clicked)
        coupling_layout.addWidget(self.btnLaserAlign)

        self.btnLaserHorizontalScan = QPushButton("Laser horizontal scan")
        self.btnLaserHorizontalScan.clicked.connect(self._on_laser_horizontal_scan_clicked)
        coupling_layout.addWidget(self.btnLaserHorizontalScan)

        self.btnLaserVerticalScan = QPushButton("Laser vertical scan")
        self.btnLaserVerticalScan.clicked.connect(self._on_laser_vertical_scan_clicked)
        coupling_layout.addWidget(self.btnLaserVerticalScan)

        coupling_layout.addStretch()
        outer.addWidget(self.groupCoupling)
        outer.addStretch()

    # ──────────────────────────────────────────────────────
    # 로봇 설정 (main_window 연결/해제 시 호출)
    # ──────────────────────────────────────────────────────

    def set_robot(self, robot):
        """로봇 클라이언트 설정."""
        self.robot = robot
        self._align_svc.set_robot(robot)

    # ──────────────────────────────────────────────────────
    # 버튼 핸들러
    # ──────────────────────────────────────────────────────

    def _on_port_center_align_clicked(self):
        if not self._detect_active:
            self.log_message.emit("[Port Center Align] Port hall detect가 켜져 있지 않습니다.")
            return

        # 오버레이용 현재 결과 즉시 저장
        with self._detect_lock:
            dets = list(self._last_dets)
            img_h, _ = self._last_frame_shape

        result = compute_vertical_alignment(dets, img_h)
        if result is None:
            self.log_message.emit("[Port Center Align] 검출된 객체 없음 — 정렬 불가")
            return

        with self._detect_lock:
            self._align_result = result

        _, _, dy_px = result
        n_circles = sum(1 for d in dets if d.cls == 'Circle_A')
        n_ports   = sum(1 for d in dets if d.cls == 'Port_T')
        self.log_message.emit(
            f"[Port Center Align] Circle_A={n_circles} Port_T={n_ports} | dy={dy_px:+.1f}px"
        )

        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[Port Center Align] 로봇 미연결 — 오버레이만 표시")
            return

        # 로봇 정렬은 백그라운드 스레드에서 실행 (send_tcp_linear wait=True 블로킹)
        t = threading.Thread(
            target=self._run_align,
            daemon=True
        )
        t.start()

    def _run_align(self):
        """백그라운드 스레드: PortAlignmentService를 통한 수직 정렬 (coarse + fine)."""
        success, msg = self._align_svc.align_vertical(self._measure_dy)
        self.log_message.emit(f"[Port Center Align] {'완료' if success else '실패'}: {msg}")

    def _run_ry_correct(self):
        """백그라운드 스레드: PortAlignmentService를 통한 Ry 각도 보정."""
        success, msg = self._align_svc.align_ry(self._measure_ry)
        self.log_message.emit(f"[Ry 보정] {'완료' if success else '실패'}: {msg}")

    def _run_horizontal_align(self):
        """백그라운드 스레드: PortAlignmentService를 통한 수평 정렬 (coarse + fine)."""
        success, msg = self._align_svc.align_horizontal(self._measure_dx)
        self.log_message.emit(f"[수평 정렬] {'완료' if success else '실패'}: {msg}")

    # ──────────────────────────────────────────────────────
    # 정렬 측정 콜백 (PortAlignmentService.measure_fn)
    # ──────────────────────────────────────────────────────

    def _measure_dy(self) -> Optional[float]:
        """3-phase wait 후 현재 dy_px 반환.

        Phase 1: 현재 진행 중인 YOLO 추론 완료 대기
        Phase 2: 새 추론 시작 대기 (로봇 이동 후 새 프레임 도착)
        Phase 3: 새 추론 완료 대기
        → 보장: 반환값은 로봇 이동 후 새 프레임의 검출 결과
        """
        deadline = time.time() + 5.0

        # Phase 1: 현재 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 2: 새 추론 시작 대기 (0.05s 간격으로 폴링)
        time.sleep(0.05)
        while time.time() < deadline:
            with self._detect_lock:
                if self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 3: 새 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        with self._detect_lock:
            dets = list(self._last_dets)
            img_h, _ = self._last_frame_shape

        result = compute_vertical_alignment(dets, img_h)
        return result[2] if result is not None else None

    def _measure_ry(self) -> 'Optional[float]':
        """3-phase wait 후 현재 Ry 기울기 각도 반환.

        Phase 1: 현재 진행 중인 YOLO 추론 완료 대기
        Phase 2: 새 추론 시작 대기 (로봇 이동 후 새 프레임 도착)
        Phase 3: 새 추론 완료 대기
        → 보장: 반환값은 로봇 이동 후 새 프레임의 검출 결과
        """
        deadline = time.time() + 5.0

        # Phase 1: 현재 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 2: 새 추론 시작 대기
        time.sleep(0.05)
        while time.time() < deadline:
            with self._detect_lock:
                if self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 3: 새 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        with self._detect_lock:
            dets = list(self._last_dets)

        return compute_ry_angle(dets)

    def _measure_dx(self) -> 'Optional[float]':
        """3-phase wait 후 현재 dx_px 반환.

        Phase 1: 현재 진행 중인 YOLO 추론 완료 대기
        Phase 2: 새 추론 시작 대기 (로봇 이동 후 새 프레임 도착)
        Phase 3: 새 추론 완료 대기
        → 보장: 반환값은 로봇 이동 후 새 프레임의 검출 결과
        """
        deadline = time.time() + 5.0

        # Phase 1: 현재 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 2: 새 추론 시작 대기
        time.sleep(0.05)
        while time.time() < deadline:
            with self._detect_lock:
                if self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 3: 새 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        with self._detect_lock:
            dets = list(self._last_dets)
            _, img_w = self._last_frame_shape

        result = compute_horizontal_alignment(dets, img_w)
        return result[2] if result is not None else None

    def _on_ry_correct_clicked(self):
        if not self._detect_active:
            self.log_message.emit("[Ry 보정] Port hall detect가 켜져 있지 않습니다.")
            return
        if self._yolo_model is None:
            self.log_message.emit("[Ry 보정] 모델 미로드.")
            return
        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[Ry 보정] 로봇 미연결.")
            return
        t = threading.Thread(target=self._run_ry_correct, daemon=True)
        t.start()

    def _on_horizontal_align_clicked(self):
        if not self._detect_active:
            self.log_message.emit("[수평 정렬] Port hall detect가 켜져 있지 않습니다.")
            return
        if self._yolo_model is None:
            self.log_message.emit("[수평 정렬] 모델 미로드.")
            return
        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[수평 정렬] 로봇 미연결.")
            return
        t = threading.Thread(target=self._run_horizontal_align, daemon=True)
        t.start()

    def _on_port_hall_detect_toggled(self, checked):
        if checked:
            if self._yolo_model is None:
                self._load_yolo_model()
            self._detect_active = (self._yolo_model is not None)
            if not self._detect_active:
                self.btnPortHallDetect.setChecked(False)
        else:
            self._detect_active = False
            with self._detect_lock:
                self._last_overlay = None

    def _on_port_detect_toggled(self, checked):
        if checked:
            if self._ds435_model is None:
                self._load_ds435_model()
            self._ds435_detect_active = (self._ds435_model is not None)
            if not self._ds435_detect_active:
                self.btnPortDetect.setChecked(False)
        else:
            self._ds435_detect_active = False
            with self._ds435_detect_lock:
                self._ds435_last_overlay = None

    def _load_ds435_model(self):
        """DS435 YOLOv8-OBB 모델 로드 (최초 1회)"""
        try:
            import sys
            ai_dir = str(_AI_DIR)
            if ai_dir not in sys.path:
                sys.path.insert(0, ai_dir)
            from obb_overlay import overlay_obb  # noqa: F401
            self._ds435_overlay_fn = overlay_obb

            from ultralytics import YOLO  # type: ignore
            if not _DS435_MODEL_PATH.exists():
                self.log_message.emit(f"[DS435 Detect] 모델 파일 없음: {_DS435_MODEL_PATH}")
                return
            self._ds435_model = YOLO(str(_DS435_MODEL_PATH), task='obb')
            self.log_message.emit("[DS435 Detect] YOLOv8-OBB 모델 로드 완료")
        except Exception as e:
            self.log_message.emit(f"[DS435 Detect] 모델 로드 실패: {e}")

    def _run_ds435_detection(self, frame):
        """백그라운드 스레드: DS435 OBB 추론 + 오버레이"""
        try:
            results = self._ds435_model(frame, conf=0.25, iou=0.65, verbose=False)
            result = results[0]
            overlay = self._ds435_overlay_fn(frame, result)
            # OBB 중심 계산 (신뢰도 최고 박스 꼭짓점 평균)
            center = None
            if result.obb is not None and len(result.obb) > 0:
                corners = result.obb.xyxyxyxy.cpu().numpy()  # [N, 4, 2]
                best_idx = int(result.obb.conf.cpu().numpy().argmax())
                best_corners = corners[best_idx]  # [4, 2]
                center = (float(best_corners[:, 0].mean()),
                          float(best_corners[:, 1].mean()))
            with self._ds435_detect_lock:
                self._ds435_last_overlay = overlay
                self._ds435_last_center = center
                self._ds435_frame_shape = frame.shape[:2]
        except Exception as e:
            self.log_message.emit(f"[DS435 Detect] 추론 실패: {e}")
            with self._ds435_detect_lock:
                self._ds435_last_overlay = frame
        finally:
            with self._ds435_detect_lock:
                self._ds435_detect_running = False

    def _on_ds435_vert_align_clicked(self):
        if not self._ds435_detect_active:
            self.log_message.emit("[DS435 수직 정렬] Port detect가 켜져 있지 않습니다.")
            return
        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[DS435 수직 정렬] 로봇 미연결.")
            return
        threading.Thread(target=self._run_ds435_vert_align, daemon=True).start()

    def _on_ds435_horiz_align_clicked(self):
        if not self._ds435_detect_active:
            self.log_message.emit("[DS435 수평 정렬] Port detect가 켜져 있지 않습니다.")
            return
        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[DS435 수평 정렬] 로봇 미연결.")
            return
        threading.Thread(target=self._run_ds435_horiz_align, daemon=True).start()

    def _run_ds435_vert_align(self):
        success, msg = self._align_svc.align_vertical(self._measure_ds435_dy)
        self.log_message.emit(f"[DS435 수직 정렬] {'완료' if success else '실패'}: {msg}")

    def _run_ds435_horiz_align(self):
        success, msg = self._align_svc.align_horizontal(self._measure_ds435_dx)
        self.log_message.emit(f"[DS435 수평 정렬] {'완료' if success else '실패'}: {msg}")

    def _measure_ds435_dy(self) -> 'Optional[float]':
        """3-phase wait 후 DS435 OBB 중심 dy_px 반환.

        Phase 1: 현재 진행 중인 OBB 추론 완료 대기
        Phase 2: 새 추론 시작 대기 (로봇 이동 후 새 프레임 도착)
        Phase 3: 새 추론 완료 대기
        → 보장: 반환값은 로봇 이동 후 새 프레임의 검출 결과
        """
        deadline = time.time() + 5.0

        # Phase 1: 현재 추론 완료 대기
        while time.time() < deadline:
            with self._ds435_detect_lock:
                if not self._ds435_detect_running:
                    break
            time.sleep(0.05)

        # Phase 2: 새 추론 시작 대기 (0.05s 간격으로 폴링)
        time.sleep(0.05)
        while time.time() < deadline:
            with self._ds435_detect_lock:
                if self._ds435_detect_running:
                    break
            time.sleep(0.05)

        # Phase 3: 새 추론 완료 대기
        while time.time() < deadline:
            with self._ds435_detect_lock:
                if not self._ds435_detect_running:
                    break
            time.sleep(0.05)

        with self._ds435_detect_lock:
            center = self._ds435_last_center
            h, _ = self._ds435_frame_shape
        return (center[1] - h / 2.0) if center is not None else None

    def _measure_ds435_dx(self) -> 'Optional[float]':
        """3-phase wait 후 DS435 OBB 중심 dx_px 반환.

        Phase 1: 현재 진행 중인 OBB 추론 완료 대기
        Phase 2: 새 추론 시작 대기 (로봇 이동 후 새 프레임 도착)
        Phase 3: 새 추론 완료 대기
        → 보장: 반환값은 로봇 이동 후 새 프레임의 검출 결과
        """
        deadline = time.time() + 5.0

        # Phase 1: 현재 추론 완료 대기
        while time.time() < deadline:
            with self._ds435_detect_lock:
                if not self._ds435_detect_running:
                    break
            time.sleep(0.05)

        # Phase 2: 새 추론 시작 대기 (0.05s 간격으로 폴링)
        time.sleep(0.05)
        while time.time() < deadline:
            with self._ds435_detect_lock:
                if self._ds435_detect_running:
                    break
            time.sleep(0.05)

        # Phase 3: 새 추론 완료 대기
        while time.time() < deadline:
            with self._ds435_detect_lock:
                if not self._ds435_detect_running:
                    break
            time.sleep(0.05)

        with self._ds435_detect_lock:
            center = self._ds435_last_center
            _, w = self._ds435_frame_shape
        return (center[0] - w / 2.0) if center is not None else None

    def _load_yolo_model(self):
        """YOLOv8 모델 및 overlay_masks 함수 로드 (최초 1회)"""
        try:
            import sys
            ai_dir = str(_AI_DIR)
            if ai_dir not in sys.path:
                sys.path.insert(0, ai_dir)

            from view_results import overlay_masks, parse_detections, draw_align_overlay, draw_horizontal_align_overlay, draw_bbox_centers  # noqa: F401
            self._overlay_fn = overlay_masks
            self._parse_fn = parse_detections
            self._align_overlay_fn = draw_align_overlay
            self._horiz_align_overlay_fn = draw_horizontal_align_overlay
            self._bbox_center_fn = draw_bbox_centers

            from ultralytics import YOLO  # type: ignore
            if not _MODEL_PATH.exists():
                self.log_message.emit(f"[AI Detection] 모델 파일 없음: {_MODEL_PATH}")
                return
            self._yolo_model = YOLO(str(_MODEL_PATH), task='segment')
            self.log_message.emit("[AI Detection] YOLOv8-seg 모델 로드 완료")
        except Exception as e:
            self.log_message.emit(f"[AI Detection] 모델 로드 실패: {e}")

    # ──────────────────────────────────────────────────────
    # 프레임 표시
    # ──────────────────────────────────────────────────────

    def _display_fixed(self, frame, label):
        """640x360 고정 크기로 라벨에 표시"""
        if frame is None:
            return
        resized = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        q_image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        label.setPixmap(QPixmap.fromImage(q_image))

    def update_ds435_frame(self, frame):
        if self._ds435_detect_active and self._ds435_model is not None:
            with self._ds435_detect_lock:
                display = self._ds435_last_overlay if self._ds435_last_overlay is not None else frame
                should_launch = not self._ds435_detect_running
                if should_launch:
                    self._ds435_detect_running = True
            if should_launch:
                t = threading.Thread(
                    target=self._run_ds435_detection,
                    args=(frame.copy(),),
                    daemon=True
                )
                t.start()
        else:
            display = frame
        self._display_fixed(display, self.labelDS435View)

    def update_arducam_frame(self, frame):
        if self._detect_active and self._yolo_model is not None:
            # 마지막 추론 결과 표시 (백그라운드 추론 중에도 화면 유지)
            with self._detect_lock:
                display = self._last_overlay if self._last_overlay is not None else frame

            # 아직 추론 중이지 않으면 새 추론 시작
            with self._detect_lock:
                should_launch = not self._detect_running
                if should_launch:
                    self._detect_running = True
            if should_launch:
                t = threading.Thread(
                    target=self._run_detection,
                    args=(frame.copy(),),
                    daemon=True
                )
                t.start()
        else:
            display = frame

        self._display_fixed(display, self.labelArduCamView)

    def _run_detection(self, frame):
        """백그라운드 스레드: YOLOv8 추론 + 오버레이"""
        try:
            results = self._yolo_model(frame, conf=0.5, iou=0.65, verbose=False, device='cpu')
            result = results[0]
            overlay = self._overlay_fn(frame, result)
            dets = self._parse_fn(result) if self._parse_fn else []
            # 최하단 Port_T bbox 하단 y + x 범위 (레이저 정렬용)
            port_t_ymax = None
            port_t_xmin = None
            port_t_xmax = None
            if result.boxes is not None and len(result.boxes):
                from AI.view_results import CLASSES as _CLS
                clss = result.boxes.cls.cpu().numpy().astype(int)
                xyxy = result.boxes.xyxy.cpu().numpy()
                port_t_boxes = [box for ci, box in zip(clss, xyxy)
                                if _CLS[ci % len(_CLS)] == 'Port_T']
                if port_t_boxes:
                    port_t_ymax = max(box[3] for box in port_t_boxes)
                    port_t_xmin = min(box[0] for box in port_t_boxes)
                    port_t_xmax = max(box[2] for box in port_t_boxes)
            # laser_vertical_calib_roi.json 고정 ROI (레이저 정렬 모드)
            laser_results = {'left': None, 'right': None}
            if self._laser_state != LaserState.OFF and self._laser_calib_roi:
                h, w = frame.shape[:2]
                _draw_laser_calib_roi(overlay, self._laser_calib_roi)
                _ref_y = self._laser_calib_roi.get("center_y_px", h // 2)
                draw_laser_reference_line(overlay, _ref_y, self._laser_ref_cfg)
                rects = compute_roi_rects(w, h, self._laser_calib_roi)
                keys = ['left', 'right'] if len(rects) == 2 else ['left']
                for key, rect in zip(keys, rects):
                    res = LaserDetectionService.detect_in_roi(frame, rect)
                    laser_results[key] = res
                    if res is not None:
                        for cx, cy in zip(res['inlier_cols'].astype(int),
                                          res['inlier_y'].astype(int)):
                            cv2.circle(overlay, (cx, cy), 2, (0, 255, 0), -1)
                        draw_laser_fit_line(overlay, res['coeffs'],
                                            res['inlier_cols'], (0, 255, 0))
            with self._detect_lock:
                self._last_overlay = overlay
                self._last_dets = dets
                self._last_frame_shape = frame.shape[:2]
                self._last_port_t_ymax = port_t_ymax
                self._last_port_t_xmin = port_t_xmin
                self._last_port_t_xmax = port_t_xmax
                self._last_raw_frame = frame
                self._last_laser_results = laser_results
        except Exception as e:
            self.log_message.emit(f"[AI Detection] 추론 실패: {e}")
            with self._detect_lock:
                self._last_overlay = frame
        finally:
            with self._detect_lock:
                self._detect_running = False

    # ──────────────────────────────────────────────────────
    # 자동차 충전건 결합
    # ──────────────────────────────────────────────────────

    def _on_ds435_charging_align_clicked(self):
        if not self._ds435_detect_active:
            self._on_port_detect_toggled(True)
            self.btnPortDetect.setChecked(self._ds435_detect_active)
        if not self._ds435_detect_active:
            self.log_message.emit("[DS435 충전 정렬] Port detect 활성화 실패.")
            return
        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[DS435 충전 정렬] 로봇 미연결.")
            return
        threading.Thread(target=self._run_ds435_charging_align, daemon=True).start()

    def _run_ds435_charging_align(self):
        ok, msg = self._align_svc.align_vertical(self._measure_ds435_dy)
        self.log_message.emit(f"[DS435 충전 정렬] 수직: {'완료' if ok else '실패'}: {msg}")
        if not ok:
            return
        ok, msg = self._align_svc.align_horizontal(self._measure_ds435_dx)
        self.log_message.emit(f"[DS435 충전 정렬] 수평: {'완료' if ok else '실패'}: {msg}")

    def _on_handoff_clicked(self):
        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[ArduCam 충전위치 변환] 로봇 미연결.")
            return
        threading.Thread(target=self._run_handoff, daemon=True).start()

    def _run_handoff(self):
        cfg_path = Path(__file__).parent.parent.parent / "config/charging/charging_gun_coupling.json"
        try:
            with open(cfg_path) as f:
                cfg = json.load(f)
        except Exception as e:
            self.log_message.emit(f"[ArduCam 충전위치 변환] config 로드 실패: {e}")
            return
        off = cfg["offsets"]["ds435_to_arducam"]
        for axis, val in [('x', off['x']), ('y', off['y']), ('z', off['z'])]:
            if abs(val) < 0.01:
                continue
            ok, msg = self.robot.send_base_linear(axis, val)
            if not ok:
                self.log_message.emit(f"[ArduCam 충전위치 변환] 실패 ({axis}): {msg}")
                return
        self.log_message.emit("[ArduCam 충전위치 변환] 완료")

    def _on_arducam_charging_align_clicked(self):
        if not self._detect_active:
            self._on_port_hall_detect_toggled(True)
            self.btnPortHallDetect.setChecked(self._detect_active)
        if not self._detect_active:
            self.log_message.emit("[ArduCam 충전 정렬] Port hall detect 활성화 실패.")
            return
        if self.robot is None or not self.robot.is_connected:
            self.log_message.emit("[ArduCam 충전 정렬] 로봇 미연결.")
            return
        threading.Thread(target=self._run_arducam_charging_align, daemon=True).start()

    def _on_laser_align_clicked(self):
        if self._laser_state == LaserState.OFF:
            # OFF -> ROI_VISIBLE: detect 활성화 + JSON ROI 로드
            if not self._detect_active:
                self._on_port_hall_detect_toggled(True)
                self.btnPortHallDetect.setChecked(self._detect_active)
            if not self._detect_active:
                self.log_message.emit("[Laser 정렬] Port hall detect 활성화 실패.")
                return
            try:
                _p = Path(__file__).parent.parent.parent / "config/laser/align/laser_vertical_calib_roi.json"
                with open(_p) as _f:
                    _data = json.load(_f)
                self._laser_calib_roi = _data["roi"]
                self._laser_ref_cfg = _data.get("reference_line")
                self._laser_state = LaserState.ROI_VISIBLE
                self.log_message.emit("[Laser 정렬] ROI 표시 ON — 다시 클릭하면 정렬 시작")
            except Exception as e:
                self._laser_calib_roi = None
                self._laser_ref_cfg = None
                self.log_message.emit(f"[Laser 정렬] ROI 로드 실패: {e}")

        elif self._laser_state == LaserState.ROI_VISIBLE:
            # ROI_VISIBLE -> ALIGNING (로봇 연결 시) 또는 OFF (미연결)
            if self.robot is None or not self.robot.is_connected:
                self._laser_state = LaserState.OFF
                self._laser_calib_roi = None
                self.log_message.emit("[Laser 정렬] 로봇 미연결 — ROI 표시 OFF")
                return
            self._laser_state = LaserState.ALIGNING
            self.log_message.emit("[Laser 정렬] 자동 정렬 시작...")
            threading.Thread(target=self._run_laser_align, daemon=True).start()

        else:  # ALIGNING
            self.log_message.emit("[Laser 정렬] 정렬 진행 중 — 완료를 기다려 주세요")

    def _run_laser_align(self):
        # 진입 시 이미 ALIGNING 상태
        try:
            ok, msg = self._align_svc.align_vertical(self._measure_laser_dy, tcp_axis='x', frame='base')
            self.log_message.emit(f"[Laser 정렬] {'완료' if ok else '실패'}: {msg}")
        finally:
            self._laser_state = LaserState.OFF
            self._laser_calib_roi = None
            self._laser_ref_cfg = None

    def _on_laser_horizontal_scan_clicked(self):
        if not self._detect_active:
            self._on_port_hall_detect_toggled(True)
            self.btnPortHallDetect.setChecked(self._detect_active)
        if not self._detect_active:
            self.log_message.emit("[Laser 수평 스캔] Port hall detect 활성화 실패.")
            return
        self.log_message.emit("[Laser 수평 스캔] YOLO 검출 활성화 — 레이저 ROI 표시 중")

    def _on_laser_vertical_scan_clicked(self):
        if not self._detect_active:
            self._on_port_hall_detect_toggled(True)
            self.btnPortHallDetect.setChecked(self._detect_active)
        if not self._detect_active:
            self.log_message.emit("[Laser 수직 스캔] Port hall detect 활성화 실패.")
            return
        self.log_message.emit("[Laser 수직 스캔] YOLO 검출 활성화 — 레이저 ROI 표시 중")

    def _measure_laser_dy(self) -> Optional[float]:
        """3-phase wait 후 JSON ROI 기반 레이저 Y 오프셋 반환.

        center_y_px 기준으로 레이저 중심 Y의 오프셋(px)을 반환.
        align_vertical은 이 값이 0에 수렴하도록 로봇을 이동시킨다.
        """
        if self._laser_calib_roi is None:
            self._emit_log("[Laser 측정] _laser_calib_roi=None — ROI 미설정")
            return None

        deadline = time.time() + 5.0

        # Phase 1: 현재 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 2: 새 추론 시작 대기 (로봇 이동 후 새 프레임 도착)
        time.sleep(0.05)
        while time.time() < deadline:
            with self._detect_lock:
                if self._detect_running:
                    break
            time.sleep(0.05)

        # Phase 3: 새 추론 완료 대기
        while time.time() < deadline:
            with self._detect_lock:
                if not self._detect_running:
                    break
            time.sleep(0.05)

        # _last_laser_results 안전 복사
        with self._detect_lock:
            results = dict(self._last_laser_results)
            _, frame_w = self._last_frame_shape

        center_x = frame_w // 2
        laser_ys = []
        for key in ('left', 'right'):
            res = results.get(key)
            if res is not None:
                laser_ys.append(float(np.polyval(res['coeffs'], center_x)))

        if not laser_ys:
            self._emit_log("[Laser 측정] 좌/우 ROI 모두 레이저 미검출")
            return None

        laser_y_mean = float(np.mean(laser_ys))
        center_y_px  = self._laser_calib_roi.get("center_y_px",
                                                  self._last_frame_shape[0] // 2)
        dy = laser_y_mean - center_y_px
        self._emit_log(
            f"[Laser 측정] laser_y={laser_y_mean:.1f}, center_y={center_y_px}, "
            f"dy={dy:+.1f}px (검출: {len(laser_ys)}개 ROI)"
        )
        return float(dy)

    def _run_arducam_charging_align(self):
        ok, msg = self._align_svc.align_vertical(self._measure_dy)
        self.log_message.emit(f"[ArduCam 충전 정렬] 수직: {'완료' if ok else '실패'}: {msg}")
        if not ok:
            return
        ok, msg = self._align_svc.align_ry(self._measure_ry)
        self.log_message.emit(f"[ArduCam 충전 정렬] Ry: {'완료' if ok else '실패'}: {msg}")
        if not ok:
            return
        ok, msg = self._align_svc.align_horizontal(self._measure_dx)
        self.log_message.emit(f"[ArduCam 충전 정렬] 수평: {'완료' if ok else '실패'}: {msg}")

