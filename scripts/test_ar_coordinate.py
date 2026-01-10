#!/usr/bin/env python3
"""
AR Tag 좌표 테스트
1/Q: Rx +/-1도, 2/W: Ry +/-1도, 3/E: Rz +/-1도
R: 홈 등록, H: 홈 이동
"""

import sys
import os
import time

from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from Robot.communication.modbus_client import ModbusClient
from Sensor.d435.d435_controller import D435Controller
from Sensor.aruco.aruco_detector import ArucoCameraPoseEstimator


class Viewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AR Test")
        self.setGeometry(100, 100, 640, 480)

        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 640, 480)

        # 로봇
        self.robot = ModbusClient(ip="192.168.0.29", port=1502)
        self.robot.connect()
        self.robot.send_set_toolframe(1, wait=False)

        # 카메라
        self.camera = D435Controller()
        self.camera.start()
        self.intrinsics = self.camera.get_intrinsics()

        # ArUco
        self.aruco = ArucoCameraPoseEstimator(marker_size_meters=0.03, dictionary_type=cv2.aruco.DICT_5X5_50)

        # 상태
        self.pose = None
        self.ar_rot = None
        self.home = None
        self.moving = False

        # 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

        print("1/Q:Rx, 2/W:Ry, 3/E:Rz, R:홈등록, H:홈이동")

    def update(self):
        if self.moving:
            return

        frame = self.camera.get_frame()
        if frame is None:
            return

        # 로봇 포즈
        self.pose = self.robot.read_current_pose()

        # AR 마커
        markers = self.aruco.detect_and_estimate_pose(frame, self.intrinsics)
        if markers:
            rot_mat = markers[0]['rotation_matrix']
            self.ar_rot = R.from_matrix(rot_mat).as_euler('xyz', degrees=True)

        # 화면
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, c = rgb.shape
        img = QImage(rgb.data, w, h, c * w, QImage.Format_RGB888)
        self.label.setPixmap(QPixmap.fromImage(img))

    def keyPressEvent(self, e):
        if self.moving or not self.pose:
            return

        x, y, z, rx, ry, rz = self.pose
        k = e.key()

        if k == Qt.Key_1:
            self.move("1", "Rx", +1, x, y, z, rx+1, ry, rz)
        elif k == Qt.Key_Q:
            self.move("Q", "Rx", -1, x, y, z, rx-1, ry, rz)
        elif k == Qt.Key_2:
            self.move("2", "Ry", +1, x, y, z, rx, ry+1, rz)
        elif k == Qt.Key_W:
            self.move("W", "Ry", -1, x, y, z, rx, ry-1, rz)
        elif k == Qt.Key_3:
            self.move("3", "Rz", +1, x, y, z, rx, ry, rz+1)
        elif k == Qt.Key_E:
            self.move("E", "Rz", -1, x, y, z, rx, ry, rz-1)
        elif k == Qt.Key_R:
            self.home = self.pose
            print(f"[R] 홈 등록")
        elif k == Qt.Key_H and self.home:
            self.move("H", "Home", 0, *self.home)

    def move(self, key, axis, delta, x, y, z, rx, ry, rz):
        self.moving = True

        # 이동 전
        before = self.ar_rot.copy() if self.ar_rot is not None else None
        print(f"\n{'='*50}")
        print(f"[{key}] {axis} {delta:+d}도")
        if before is not None:
            print(f"  전: Rx={before[0]:.1f}, Ry={before[1]:.1f}, Rz={before[2]:.1f}")

        # 이동
        self.robot.send_move_to_pose(x, y, z, rx, ry, rz, wait=False)

        # 대기
        for _ in range(50):
            QApplication.processEvents()
            s = self.robot.read_status()
            if s in (0, 2):
                break
            time.sleep(0.1)

        time.sleep(0.5)

        # AR 갱신
        for _ in range(5):
            QApplication.processEvents()
            frame = self.camera.get_frame()
            if frame:
                m = self.aruco.detect_and_estimate_pose(frame, self.intrinsics)
                if m:
                    self.ar_rot = R.from_matrix(m[0]['rotation_matrix']).as_euler('xyz', degrees=True)
            time.sleep(0.1)

        # 이동 후
        if self.ar_rot is not None:
            print(f"  후: Rx={self.ar_rot[0]:.1f}, Ry={self.ar_rot[1]:.1f}, Rz={self.ar_rot[2]:.1f}")
            if before is not None:
                d = self.ar_rot - before
                print(f"  변화: dRx={d[0]:.1f}, dRy={d[1]:.1f}, dRz={d[2]:.1f}")
        print(f"{'='*50}")

        self.moving = False

    def closeEvent(self, e):
        self.timer.stop()
        self.camera.stop()
        self.robot.disconnect()
        e.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Viewer()
    w.show()
    sys.exit(app.exec_())
