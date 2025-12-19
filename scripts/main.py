#!/usr/bin/env python3
"""
Charging Robot Task Manager - Main Entry Point
"""

import sys
import os

# 현재 디렉토리를 path에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

from main_window import MainWindow


def main():
    # High DPI 지원
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setApplicationName("Charging Robot Task Manager")
    app.setOrganizationName("KAIST")

    # 메인 윈도우 생성 및 표시
    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
