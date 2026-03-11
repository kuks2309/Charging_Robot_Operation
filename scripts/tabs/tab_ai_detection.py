"""AI Detection 탭 — 카메라 피드 표시 + ArduCam YOLOv8-seg 검출"""
import time
import threading
from pathlib import Path
from typing import Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox, QPushButton
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
import cv2

_AI_DIR = Path(__file__).parent.parent / "AI"
_MODEL_PATH = Path(__file__).parent.parent.parent / "config/AI_weights/ArduCam/best.onnx"

from services.port_alignment_service import (
    compute_vertical_alignment, compute_horizontal_alignment,
    compute_ry_angle, PortAlignmentService,
)


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
        # 백그라운드 추론 상태
        self._detect_running = False
        self._last_overlay = None   # 마지막 추론 결과 프레임
        self._last_dets = []        # 마지막 Det 리스트
        self._last_frame_shape = (1080, 1920)  # (h, w) 기본값
        self._detect_lock = threading.Lock()
        self._align_result = None   # (port_cy, img_cy, dy_px) or None
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
        QHBoxLayout(self.groupDS435AI)
        outer.addWidget(self.groupDS435AI)

        # Arducam AI Detection 그룹박스
        self.groupArducamAI = QGroupBox("Arducam AI Detection")
        self.groupArducamAI.setFixedHeight(100)
        arducam_ai_layout = QHBoxLayout(self.groupArducamAI)

        self.btnPortHallDetect = QPushButton("Port hall detect")
        self.btnPortHallDetect.setCheckable(True)
        self.btnPortHallDetect.clicked.connect(self._on_port_hall_detect_toggled)
        arducam_ai_layout.addWidget(self.btnPortHallDetect)

        self.btnPortCenterAlign = QPushButton("Port center align")
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

    def _load_yolo_model(self):
        """YOLOv8 모델 및 overlay_masks 함수 로드 (최초 1회)"""
        try:
            import sys
            ai_dir = str(_AI_DIR)
            if ai_dir not in sys.path:
                sys.path.insert(0, ai_dir)

            from view_results import overlay_masks, parse_detections, draw_align_overlay, draw_horizontal_align_overlay  # noqa: F401
            self._overlay_fn = overlay_masks
            self._parse_fn = parse_detections
            self._align_overlay_fn = draw_align_overlay
            self._horiz_align_overlay_fn = draw_horizontal_align_overlay

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
        self._display_fixed(frame, self.labelDS435View)

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
            # 항상 현재 검출 기반 정렬 오버레이 표시 (dy offset 실시간)
            current_align = compute_vertical_alignment(dets, frame.shape[0])
            if current_align is not None and self._align_overlay_fn is not None:
                overlay = self._align_overlay_fn(overlay, *current_align)
            current_horiz = compute_horizontal_alignment(dets, frame.shape[1])
            if current_horiz is not None and self._horiz_align_overlay_fn is not None:
                overlay = self._horiz_align_overlay_fn(overlay, *current_horiz)
            with self._detect_lock:
                self._last_overlay = overlay
                self._last_dets = dets
                self._last_frame_shape = frame.shape[:2]
        except Exception as e:
            self.log_message.emit(f"[AI Detection] 추론 실패: {e}")
            with self._detect_lock:
                self._last_overlay = frame
        finally:
            with self._detect_lock:
                self._detect_running = False
