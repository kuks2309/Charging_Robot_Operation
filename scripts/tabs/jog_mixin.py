#!/usr/bin/env python3
"""
JogMixin - 조그 이동 기능을 제공하는 재사용 가능한 믹스인 클래스

사용 요건:
- Host 위젯에 .ui 파일에서 로드된 다음 UI 요소가 필요:
  - 버튼: btnJog{X,Y,Z,Rx,Ry,Rz}{Plus,Minus}
  - 스핀박스: spinJogStep{X,Y,Z,Rx,Ry,Rz}
- Host 클래스에 다음 시그널이 정의되어야 함:
  - jog_move_requested = pyqtSignal(str, float)
  - jog_rotate_requested = pyqtSignal(str, float)
- Host 클래스에 _log(message) 메서드가 있어야 함
- _setup_ui() 메서드에서 _connect_jog_buttons()를 호출할 것
"""


class JogMixin:
    """조그 이동 시그널 연결 및 핸들러를 제공하는 믹스인.

    NOTE: pyqtSignal은 믹스인에서 정의할 수 없음 (PyQt5 제한).
    반드시 구체적인 QWidget 서브클래스에서 선언해야 함.
    이 믹스인은 로직만 제공함.
    """

    def _connect_jog_buttons(self):
        """조그 버튼을 핸들러에 연결. _setup_ui()에서 호출할 것."""
        # 선형 축
        self.btnJogXMinus.clicked.connect(lambda: self._on_jog_move('x', -1))
        self.btnJogXPlus.clicked.connect(lambda: self._on_jog_move('x', 1))
        self.btnJogYMinus.clicked.connect(lambda: self._on_jog_move('y', -1))
        self.btnJogYPlus.clicked.connect(lambda: self._on_jog_move('y', 1))
        self.btnJogZMinus.clicked.connect(lambda: self._on_jog_move('z', -1))
        self.btnJogZPlus.clicked.connect(lambda: self._on_jog_move('z', 1))
        # 회전 축
        self.btnJogRxMinus.clicked.connect(lambda: self._on_jog_rotate('rx', -1))
        self.btnJogRxPlus.clicked.connect(lambda: self._on_jog_rotate('rx', 1))
        self.btnJogRyMinus.clicked.connect(lambda: self._on_jog_rotate('ry', -1))
        self.btnJogRyPlus.clicked.connect(lambda: self._on_jog_rotate('ry', 1))
        self.btnJogRzMinus.clicked.connect(lambda: self._on_jog_rotate('rz', -1))
        self.btnJogRzPlus.clicked.connect(lambda: self._on_jog_rotate('rz', 1))

    def _on_jog_move(self, axis: str, direction: int):
        """베이스 좌표계 조그 이동 핸들러."""
        step_map = {
            'x': self.spinJogStepX,
            'y': self.spinJogStepY,
            'z': self.spinJogStepZ
        }
        spin = step_map.get(axis)
        if spin is None:
            return
        distance = spin.value() * direction
        self._log(f"조그 이동: {axis.upper()} {'+' if direction > 0 else ''}{distance}mm")
        self.jog_move_requested.emit(axis, distance)

    def _on_jog_rotate(self, axis: str, direction: int):
        """베이스 좌표계 조그 회전 핸들러."""
        step_map = {
            'rx': self.spinJogStepRx,
            'ry': self.spinJogStepRy,
            'rz': self.spinJogStepRz
        }
        spin = step_map.get(axis)
        if spin is None:
            return
        angle = spin.value() * direction
        self._log(f"조그 회전: {axis.upper()} {'+' if direction > 0 else ''}{angle}°")
        self.jog_rotate_requested.emit(axis, angle)
