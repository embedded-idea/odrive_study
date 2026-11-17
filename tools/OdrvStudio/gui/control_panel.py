"""OdrvStudio - 电机控制面板。

参考 SimpleFOCStudio 的 deviceJoggingControl / generalControls:
模式选择 + 设定值 + 点动 + 急停。
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QButtonGroup, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QSlider, QVBoxLayout, QWidget,
)

MODES = [("速度模式", 2), ("位置模式", 3), ("力矩模式", 1)]
STATE_CLOSED_LOOP = 8
STATE_IDLE = 1


class ControlPanel(QWidget):
    """所有写操作都通过 worker 命令队列,不在 GUI 线程直接碰设备。"""

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        lay = QVBoxLayout(self)
        lay.addWidget(self._build_mode_box())
        lay.addWidget(self._build_vel_box())
        lay.addWidget(self._build_pos_box())
        lay.addWidget(self._build_estop())
        lay.addStretch(1)

    # ---- 构建控件 ----

    def _build_mode_box(self):
        box = QGroupBox("闭环控制")
        g = QGridLayout(box)
        self.mode_group = QButtonGroup(self)
        for i, (name, num) in enumerate(MODES):
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.setProperty("mode", num)
            self.mode_group.addButton(btn, num)
            g.addWidget(btn, 0, i)
        self.btn_start = QPushButton("进入闭环")
        self.btn_stop = QPushButton("回 IDLE")
        g.addWidget(self.btn_start, 1, 0)
        g.addWidget(self.btn_stop, 1, 1)
        self.mode_group.button(2).setChecked(True)
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(lambda: self.worker.action_req("stop"))
        self.mode_group.buttonClicked[int].connect(self._on_mode)
        return box

    def _build_vel_box(self):
        box = QGroupBox("速度 / 点动 [turn/s]")
        g = QGridLayout(box)
        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(-100.0, 100.0)
        self.vel_spin.setDecimals(3)
        self.vel_spin.setSingleStep(0.05)
        g.addWidget(self.vel_spin, 0, 0, 1, 2)
        self.vel_slider = QSlider(Qt.Horizontal)
        self.vel_slider.setRange(-1000, 1000)
        g.addWidget(self.vel_slider, 1, 0, 1, 2)
        for col, v in enumerate((-1.0, -0.5, 0.0, 0.5, 1.0)):
            btn = QPushButton(("%g" % v))
            btn.clicked.connect(lambda _, x=v: self.send_vel(x))
            g.addWidget(btn, 2, col)
        self.btn_jog_ccw = QPushButton("◀ 点动")
        self.btn_jog_cw = QPushButton("点动 ▶")
        self.btn_vel_send = QPushButton("发送")
        g.addWidget(self.btn_jog_ccw, 3, 0, 1, 2)
        g.addWidget(self.btn_vel_send, 3, 2)
        g.addWidget(self.btn_jog_cw, 3, 3, 1, 2)
        self.vel_slider.valueChanged.connect(
            lambda v: self.vel_spin.setValue(v / 100.0))
        self.vel_spin.valueChanged.connect(
            lambda v: self.vel_slider.setValue(int(v * 100)))
        self.btn_vel_send.clicked.connect(lambda: self.send_vel(self.vel_spin.value()))
        self.btn_jog_ccw.pressed.connect(lambda: self.send_vel(-self._jog_speed()))
        self.btn_jog_cw.pressed.connect(lambda: self.send_vel(self._jog_speed()))
        for b in (self.btn_jog_ccw, self.btn_jog_cw):
            b.released.connect(lambda: self.send_vel(0.0))
        return box

    def _build_pos_box(self):
        box = QGroupBox("位置 [turn]")
        g = QGridLayout(box)
        self.pos_spin = QDoubleSpinBox()
        self.pos_spin.setRange(-1e6, 1e6)
        self.pos_spin.setDecimals(3)
        self.pos_spin.setSingleStep(0.1)
        g.addWidget(self.pos_spin, 0, 0, 1, 2)
        self.btn_pos_abs = QPushButton("绝对")
        self.btn_pos_rel = QPushButton("相对 +")
        self.btn_pos_rel_neg = QPushButton("相对 -")
        g.addWidget(self.btn_pos_abs, 1, 0)
        g.addWidget(self.btn_pos_rel, 1, 1)
        g.addWidget(self.btn_pos_rel_neg, 1, 2)
        self.btn_pos_abs.clicked.connect(
            lambda: self.worker.write_req("axis0.controller.input_pos",
                                          self.pos_spin.value()))
        self.btn_pos_rel.clicked.connect(lambda: self._pos_rel(+1))
        self.btn_pos_rel_neg.clicked.connect(lambda: self._pos_rel(-1))
        return box

    def _build_estop(self):
        box = QWidget()
        lay = QHBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        self.btn_estop = QPushButton("⛔ 急停 (IDLE)")
        self.btn_estop.setStyleSheet(
            "QPushButton{background:#b71c1c;color:white;font-weight:bold;"
            "padding:10px;border-radius:4px;}"
            "QPushButton:hover{background:#d32f2f;}")
        self.btn_estop.clicked.connect(lambda: self.worker.action_req("estop"))
        lay.addWidget(self.btn_estop)
        return box

    # ---- 行为 ----

    def _jog_speed(self):
        return max(0.05, abs(self.vel_spin.value()) or 0.2)

    def _on_mode(self, num):
        self.worker.write_req("axis0.controller.config.control_mode", num)

    def _on_start(self):
        num = self.mode_group.checkedId()
        if num > 0:
            self.worker.write_req("axis0.controller.config.control_mode", num)
        self.worker.action_req("start")

    def send_vel(self, v):
        self.worker.write_req("axis0.controller.input_vel", float(v))

    def _pos_rel(self, sign):
        req = self.worker.read_req(["axis0.encoder.pos_estimate"])
        self._rel_pending = (req, sign, self.pos_spin.value())

    # ---- 事件 / 状态刷新 ----

    def on_event(self, ev):
        if ev[0] == "rd_result" and getattr(self, "_rel_pending", None):
            req, sign, delta = self._rel_pending
            if req == ev[1]:
                cur = ev[2].get("axis0.encoder.pos_estimate")
                if isinstance(cur, (int, float)):
                    self.worker.write_req("axis0.controller.input_pos",
                                          cur + sign * delta)
                self._rel_pending = None

    def update_state(self, snap):
        state = snap.get("axis0.current_state")
        closed = state == STATE_CLOSED_LOOP
        idle = state == STATE_IDLE
        self.btn_start.setEnabled(idle or state is None or not closed)
        self.btn_stop.setEnabled(state is not None and not idle)
        mode = snap.get("axis0.controller.config.control_mode")
        if isinstance(mode, int) and self.mode_group.checkedId() != mode:
            btn = self.mode_group.button(mode)
            if btn is not None:
                btn.setChecked(True)
