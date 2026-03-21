"""
pytest conftest: sys.path에 scripts 경로를 추가하여
Sensor, services, utils 패키지를 import 가능하게 한다.
PyQt5 QApplication fixture도 여기서 제공한다.
"""
import sys
import os

# scripts 디렉토리를 sys.path 맨 앞에 추가
SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), "..")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, os.path.abspath(SCRIPTS_DIR))

import pytest


@pytest.fixture(scope="session")
def qapp():
    """PyQt5 QApplication 세션 픽스처.

    QObject 기반 서비스(CameraCalibrationService, HandEyeCalibrationService)
    인스턴스 생성에 필요하다. 세션 스코프로 한 번만 생성한다.
    """
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app
