"""OdrvStudio - 控制相关 GroupBox(按 SimpleFOCStudio-zh 分组)。

对应关系:
  ControlLoopGroupBox    → ControlLoopBox   (控制模式 + 进闭环/回IDLE)
  TorqueGroupBox         → TorqueBox        (力矩设定值 / 电流限幅)
  GeneralControls        → GeneralControlsBox (标定/保存/清错/急停)
  DeviceJoggingControl   → JogControlBox    (速度/位置设定值 + 点动)

所有写操作都通过 worker 命令队列,不在 GUI 线程直接碰设备。
"""

from PySide6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QPushButton,
)

from .theme import tag

MODES = [("速度模式", 2), ("位置模式", 3), ("力矩模式", 1)]
STATE_CLOSED_LOOP = 8
STATE_IDLE = 1


class _AxisBox(QGroupBox):
    """带轴切换的 GroupBox 基类。"""

    def __init__(self, worker, title, parent=None):
        super().__init__(title, parent)
        self.worker = worker
        self.axis = 0

    def _p(self, sub):
        return "axis%d.%s" % (self.axis, sub)

    def set_axis(self, axis):
        self.axis = axis


class ControlLoopBox(_AxisBox):
    """控制环模式:下拉框选择 速度/位置/力矩(写 control_mode)。"""

    def __init__(self, worker, parent=None):
        super().__init__(worker, "控制环模式", parent)
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        self.mode_combo = QComboBox()
        for name, num in MODES:
            self.mode_combo.addItem(name, num)
        g.addWidget(self.mode_combo, 0, 0)
        self.mode_combo.currentIndexChanged.connect(self._on_mode)

    def _on_mode(self, idx):
        num = self.mode_combo.itemData(idx)
        if num is not None:
            self.worker.write_req(self._p("controller.config.control_mode"), num)

    def update_state(self, snap):
        mode = snap.get(self._p("controller.config.control_mode"))
        if isinstance(mode, int):
            idx = self.mode_combo.findData(mode)
            if idx >= 0 and idx != self.mode_combo.currentIndex():
                self.mode_combo.blockSignals(True)
                self.mode_combo.setCurrentIndex(idx)
                self.mode_combo.blockSignals(False)


class GeneralControlsBox(_AxisBox):
    """通用控制:进入闭环 / 回 IDLE / 全轴急停(对照 SFS 三按钮)。"""

    def __init__(self, worker, parent=None):
        super().__init__(worker, "通用控制", parent)
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        g.setSpacing(3)
        self.btn_start = QPushButton("进入闭环")
        tag(self.btn_start, "success")
        self.btn_stop = QPushButton("回 IDLE")
        self.btn_estop = QPushButton("⛔ 急停")
        self.btn_estop.setToolTip("全部轴立即回 IDLE")
        self.btn_estop.setStyleSheet(
            "QPushButton{background:#b71c1c;color:white;font-weight:bold;}"
            "QPushButton:hover{background:#d32f2f;}")
        g.addWidget(self.btn_start, 0, 0)
        g.addWidget(self.btn_stop, 0, 1)
        g.addWidget(self.btn_estop, 0, 2)
        self.btn_cal_motor = QPushButton("电机标定")
        self.btn_cal_enc = QPushButton("编码器标定")
        self.btn_save = QPushButton("保存配置")
        self.btn_clear = QPushButton("清除错误")
        g.addWidget(self.btn_cal_motor, 1, 0)
        g.addWidget(self.btn_cal_enc, 1, 1)
        g.addWidget(self.btn_save, 2, 0)
        g.addWidget(self.btn_clear, 2, 1)
        self.btn_cal_motor.clicked.connect(
            lambda: self.worker.action_req("cal_motor", self.axis))
        self.btn_cal_enc.clicked.connect(
            lambda: self.worker.action_req("cal_encoder", self.axis))
        self.btn_save.clicked.connect(lambda: self.worker.action_req("save"))
        self.btn_clear.clicked.connect(
            lambda: self.worker.action_req("clear_errors"))
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_estop.clicked.connect(lambda: self.worker.estop_req())

    def _on_start(self):
        num = self.worker.snapshot.get(
            self._p("controller.config.control_mode"))
        if isinstance(num, int) and num > 0:
            self.worker.write_req(self._p("controller.config.control_mode"), num)
        self.worker.action_req("start", self.axis)

    def _on_stop(self):
        self.worker.stop_req(self.axis)

    def update_state(self, snap):
        state = snap.get(self._p("current_state"))
        closed = state == STATE_CLOSED_LOOP
        idle = state == STATE_IDLE
        self.btn_start.setEnabled(idle or state is None or not closed)
        self.btn_stop.setEnabled(state is not None and not idle)


class TorqueBox(_AxisBox):
    """力矩控制:input_torque 设定值(单行,同 SFS 的 Torque Mode)。"""

    def __init__(self, worker, parent=None):
        super().__init__(worker, "力矩控制", parent)
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        self.torque_spin = QDoubleSpinBox()
        self.torque_spin.setRange(-50.0, 50.0)
        self.torque_spin.setDecimals(3)
        self.torque_spin.setSingleStep(0.05)
        g.addWidget(self.torque_spin, 0, 0)
        self.btn_torque = QPushButton("发送")
        tag(self.btn_torque, "primary")
        self.btn_torque.clicked.connect(self._send_torque)
        g.addWidget(self.btn_torque, 0, 1)

    def _send_torque(self):
        self.worker.write_req(self._p("controller.input_torque"),
                              self.torque_spin.value())


class JogControlBox(_AxisBox):
    """点动 / 设定值:速度设定 + 快捷档位 + 点动,位置绝对/相对(紧凑两行)。"""

    def __init__(self, worker, parent=None):
        super().__init__(worker, "点动 / 设定值", parent)
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        g.setVerticalSpacing(3)
        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(-100.0, 100.0)
        self.vel_spin.setDecimals(3)
        self.vel_spin.setSingleStep(0.05)
        g.addWidget(self.vel_spin, 0, 0, 1, 2)
        self.btn_vel_send = QPushButton("发送")
        tag(self.btn_vel_send, "primary")
        self.btn_vel_send.clicked.connect(
            lambda: self.send_vel(self.vel_spin.value()))
        g.addWidget(self.btn_vel_send, 0, 2)
        self.btn_jog_ccw = QPushButton("◀ 点动")
        self.btn_jog_cw = QPushButton("点动 ▶")
        tag(self.btn_jog_ccw, "tonal")
        tag(self.btn_jog_cw, "tonal")
        g.addWidget(self.btn_jog_ccw, 0, 3)
        g.addWidget(self.btn_jog_cw, 0, 4)
        self.btn_jog_ccw.pressed.connect(
            lambda: self.send_vel(-self._jog_speed()))
        self.btn_jog_cw.pressed.connect(
            lambda: self.send_vel(self._jog_speed()))
        for b in (self.btn_jog_ccw, self.btn_jog_cw):
            b.released.connect(lambda: self.send_vel(0.0))
        for col, v in enumerate((-1.0, -0.5, 0.0, 0.5, 1.0)):
            btn = QPushButton(("%g" % v))
            btn.setToolTip("input_vel = %g" % v)
            btn.clicked.connect(lambda _, x=v: self.send_vel(x))
            g.addWidget(btn, 1, col)
        self.pos_spin = QDoubleSpinBox()
        self.pos_spin.setRange(-1e6, 1e6)
        self.pos_spin.setDecimals(3)
        self.pos_spin.setSingleStep(0.1)
        g.addWidget(self.pos_spin, 2, 0, 1, 2)
        self.btn_pos_abs = QPushButton("绝对")
        self.btn_pos_rel = QPushButton("相对 +")
        self.btn_pos_rel_neg = QPushButton("相对 -")
        g.addWidget(self.btn_pos_abs, 2, 2)
        g.addWidget(self.btn_pos_rel, 2, 3)
        g.addWidget(self.btn_pos_rel_neg, 2, 4)
        self.btn_pos_abs.clicked.connect(self._pos_abs)
        self.btn_pos_rel.clicked.connect(lambda: self._pos_rel(+1))
        self.btn_pos_rel_neg.clicked.connect(lambda: self._pos_rel(-1))

    # ---- 行为 ----

    def _jog_speed(self):
        return max(0.05, abs(self.vel_spin.value()) or 0.2)

    def send_vel(self, v):
        self.worker.write_req(self._p("controller.input_vel"), float(v))

    def _pos_abs(self):
        self.worker.write_req(self._p("controller.input_pos"),
                              self.pos_spin.value())

    def _pos_rel(self, sign):
        req = self.worker.read_req([self._p("encoder.pos_estimate")])
        self._rel_pending = (req, sign, self.pos_spin.value())

    # ---- 事件 / 状态 ----

    def set_axis(self, axis):
        super().set_axis(axis)
        self.setTitle("点动 / 设定值 — axis%d" % axis)

    def on_event(self, ev):
        if ev[0] == "rd_result" and getattr(self, "_rel_pending", None):
            req, sign, delta = self._rel_pending
            if req == ev[1]:
                cur = ev[2].get(self._p("encoder.pos_estimate"))
                if isinstance(cur, (int, float)):
                    self.worker.write_req(self._p("controller.input_pos"),
                                          cur + sign * delta)
                self._rel_pending = None
