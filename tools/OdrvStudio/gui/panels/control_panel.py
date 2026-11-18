"""OdrvStudio - 控制相关 GroupBox(按 SimpleFOCStudio-zh 分组)。

对应关系:
  ControlLoopGroupBox    → ControlLoopBox   (控制模式 + 进闭环/回IDLE)
  GeneralControls        → GeneralControlsBox (标定/保存/清错/急停)
  TorqueGroupBox + DeviceJoggingControl → SetpointBox
      (速度 / 位置 / 力矩设定值集中在同一面板,按模式启用对应行)

所有写操作都通过 worker 命令队列,不在 GUI 线程直接碰设备。
"""

import time

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QLabel, QPushButton,
)

from ..theme import tag

# (显示名, control_mode 值, 该模式下可设置的输入)
MODES = [
    ("速度模式", 2, "点动/设定值 → input_vel [turn/s]"),
    ("位置模式", 3, "点动/设定值 → input_pos [turn]"),
    ("力矩模式", 1, "力矩控制 → input_torque [Nm]"),
]
MODE_VELOCITY, MODE_TORQUE, MODE_POSITION = 2, 1, 3
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
    """控制环模式:下拉框选择 速度/位置/力矩(写 control_mode)。

    切换模式时先把新模式的输入置为安全值(速度=0、力矩=0、位置=当前位置),
    并把 input_mode 设为 PASSTHROUGH,避免带着上一次的设定值直接跳变;
    同时发出 modeChanged,由工作区启用对应的设定值控件。
    """

    modeChanged = Signal(int)

    def __init__(self, worker, parent=None):
        super().__init__(worker, "控制环模式", parent)
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        g.setVerticalSpacing(2)
        self.mode_combo = QComboBox()
        for name, num, _hint in MODES:
            self.mode_combo.addItem(name, num)
        g.addWidget(self.mode_combo, 0, 0)
        self.hint = QLabel("")
        self.hint.setStyleSheet("color:#607d8b; font-size:10px;")
        g.addWidget(self.hint, 1, 0)
        self._pending_mode = None
        self._pending_t = 0.0
        self.mode_combo.currentIndexChanged.connect(self._on_mode)
        self._show_hint(self.current_mode())

    def current_mode(self):
        num = self.mode_combo.currentData()
        return num if num is not None else MODE_VELOCITY

    def _show_hint(self, mode):
        for _name, num, hint in MODES:
            if num == mode:
                self.hint.setText(hint)
                return
        self.hint.setText("")

    def _prepare_input(self, mode):
        """切模式前把目标输入写到安全值(命令队列保证先写输入再写模式)。"""
        if mode == MODE_VELOCITY:
            self.worker.write_req(self._p("controller.input_vel"), 0.0)
        elif mode == MODE_TORQUE:
            self.worker.write_req(self._p("controller.input_torque"), 0.0)
        elif mode == MODE_POSITION:
            pos = self.worker.snapshot.get(self._p("encoder.pos_estimate"))
            if isinstance(pos, (int, float)):
                self.worker.write_req(self._p("controller.input_pos"), float(pos))

    def _on_mode(self, idx):
        num = self.mode_combo.itemData(idx)
        if num is None:
            return
        self._pending_mode = num
        self._pending_t = time.monotonic()
        self._show_hint(num)
        self._prepare_input(num)
        self.worker.write_req(
            self._p("controller.config.input_mode"), 1)  # PASSTHROUGH
        self.worker.write_req(self._p("controller.config.control_mode"), num)
        self.modeChanged.emit(num)

    def update_state(self, snap):
        mode = snap.get(self._p("controller.config.control_mode"))
        if not isinstance(mode, int):
            return
        # 刚切换的 1 s 内以 UI 为准,避免轮询用旧值把下拉框弹回去
        if self._pending_mode is not None:
            if mode == self._pending_mode:
                self._pending_mode = None
            elif time.monotonic() - self._pending_t < 1.0:
                return
            else:
                self._pending_mode = None
        idx = self.mode_combo.findData(mode)
        if idx >= 0 and idx != self.mode_combo.currentIndex():
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentIndex(idx)
            self.mode_combo.blockSignals(False)
            self._show_hint(mode)


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


class SetpointBox(_AxisBox):
    """设定值(速度 / 位置 / 力矩)集中在一个面板,按控制模式启用对应行。"""

    def __init__(self, worker, parent=None):
        super().__init__(worker, "设定值 — 速度 / 位置 / 力矩", parent)
        g = QGridLayout(self)
        g.setContentsMargins(6, 2, 6, 4)
        g.setVerticalSpacing(3)

        # ---- 速度 ----
        lab_vel = QLabel("速度 input_vel [turn/s]")
        lab_vel.setStyleSheet("color:#607d8b; font-size:10px;")
        g.addWidget(lab_vel, 0, 0, 1, 5)
        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(-1e9, 1e9)   # 工具侧不限幅(固件保护仍在)
        self.vel_spin.setDecimals(3)
        self.vel_spin.setSingleStep(0.05)
        self.vel_spin.setToolTip("input_vel [turn/s]")
        g.addWidget(self.vel_spin, 1, 0, 1, 2)
        self.btn_vel_send = QPushButton("发送")
        tag(self.btn_vel_send, "primary")
        self.btn_vel_send.clicked.connect(
            lambda: self.send_vel(self.vel_spin.value()))
        g.addWidget(self.btn_vel_send, 1, 2)
        self.btn_jog_ccw = QPushButton("◀ 点动")
        self.btn_jog_cw = QPushButton("点动 ▶")
        tag(self.btn_jog_ccw, "tonal")
        tag(self.btn_jog_cw, "tonal")
        g.addWidget(self.btn_jog_ccw, 1, 3)
        g.addWidget(self.btn_jog_cw, 1, 4)
        self.btn_jog_ccw.pressed.connect(
            lambda: self.send_vel(-self._jog_speed()))
        self.btn_jog_cw.pressed.connect(
            lambda: self.send_vel(self._jog_speed()))
        for b in (self.btn_jog_ccw, self.btn_jog_cw):
            b.released.connect(lambda: self.send_vel(0.0))
        self._vel_widgets = [self.vel_spin, self.btn_vel_send,
                             self.btn_jog_ccw, self.btn_jog_cw]
        for col, v in enumerate((-1.0, -0.5, 0.0, 0.5, 1.0)):
            btn = QPushButton(("%g" % v))
            btn.setToolTip("input_vel = %g turn/s" % v)
            btn.clicked.connect(lambda _, x=v: self.send_vel(x))
            g.addWidget(btn, 2, col)
            self._vel_widgets.append(btn)

        # ---- 位置 ----
        lab_pos = QLabel("位置 input_pos")
        lab_pos.setStyleSheet("color:#607d8b; font-size:10px;")
        g.addWidget(lab_pos, 3, 0, 1, 4)
        self.pos_unit = QComboBox()
        self.pos_unit.addItem("turn", 1.0)
        self.pos_unit.addItem("度 °", 360.0)
        self.pos_unit.setToolTip("位置输入单位:圈(turn)或角度(度)")
        g.addWidget(self.pos_unit, 3, 4, 1, 2)
        self.pos_spin = QDoubleSpinBox()
        self.pos_spin.setRange(-1e9, 1e9)
        self.pos_spin.setDecimals(3)
        self.pos_spin.setSingleStep(0.1)
        self.pos_spin.setToolTip(
            "input_pos [turn](绝对)或相对增量;1 turn = 360°,角度 ÷ 360 = turn")
        g.addWidget(self.pos_spin, 4, 0, 1, 2)
        self.btn_pos_abs = QPushButton("绝对")
        self.btn_pos_rel = QPushButton("相对 +")
        self.btn_pos_rel_neg = QPushButton("相对 -")
        self.btn_pos_abs.setToolTip("input_pos = 设定值 [turn]")
        self.btn_pos_rel.setToolTip("当前位置 + 设定值")
        self.btn_pos_rel_neg.setToolTip("当前位置 - 设定值")
        g.addWidget(self.btn_pos_abs, 4, 2)
        g.addWidget(self.btn_pos_rel, 4, 3)
        g.addWidget(self.btn_pos_rel_neg, 4, 4)
        self.btn_pos_cur = QPushButton("取当前")
        self.btn_pos_cur.setToolTip("把当前位置填入设定值,避免绝对定位跑很远")
        self.btn_pos_cur.clicked.connect(self._load_current_pos)
        g.addWidget(self.btn_pos_cur, 4, 5)
        self.btn_pos_abs.clicked.connect(self._pos_abs)
        self.btn_pos_rel.clicked.connect(lambda: self._pos_rel(+1))
        self.btn_pos_rel_neg.clicked.connect(lambda: self._pos_rel(-1))
        self._pos_widgets = [self.pos_spin, self.btn_pos_abs,
                             self.btn_pos_rel, self.btn_pos_rel_neg,
                             self.btn_pos_cur]
        self._pos_cur = None
        self._pos_per_turn = 1.0
        self.lab_pos_deg = QLabel("= 0.000 turn / 0.0°")
        self.lab_pos_deg.setStyleSheet("color:#607d8b; font-size:10px;")
        g.addWidget(self.lab_pos_deg, 5, 0, 1, 6)
        self.pos_spin.valueChanged.connect(self._update_pos_deg)
        self.pos_unit.currentIndexChanged.connect(self._on_pos_unit)
        self._update_pos_deg(self.pos_spin.value())

        # ---- 力矩 ----
        lab_tq = QLabel("力矩 input_torque [Nm]")
        lab_tq.setStyleSheet("color:#607d8b; font-size:10px;")
        g.addWidget(lab_tq, 6, 0, 1, 3)
        self.lab_tq_hint = QLabel("力矩模式专用")
        self.lab_tq_hint.setStyleSheet("color:#607d8b; font-size:10px;")
        g.addWidget(self.lab_tq_hint, 6, 3, 1, 2)
        self.torque_spin = QDoubleSpinBox()
        self.torque_spin.setRange(-1e9, 1e9)
        self.torque_spin.setDecimals(3)
        self.torque_spin.setSingleStep(0.05)
        self.torque_spin.setToolTip(
            "input_torque [Nm];云台(电压控制)模式下内部换算成 Vq")
        g.addWidget(self.torque_spin, 7, 0, 1, 2)
        self.btn_torque = QPushButton("发送")
        tag(self.btn_torque, "primary")
        self.btn_torque.clicked.connect(self._send_torque)
        g.addWidget(self.btn_torque, 7, 2)
        self._tq_widgets = [self.torque_spin, self.btn_torque]

        self.lab_mode = QLabel("")
        self.lab_mode.setStyleSheet("color:#607d8b; font-size:10px;")
        g.addWidget(self.lab_mode, 8, 0, 1, 6)
        self._mode = MODE_VELOCITY
        self.set_mode(MODE_VELOCITY)

    def _update_pos_deg(self, value):
        """value 是当前单位下的设定值;换算成 turn 与角度一起显示。"""
        per = self._pos_per_turn or 1.0
        turns = value / per
        txt = "= %.3f turn / %.1f°" % (turns, turns * 360.0)
        if isinstance(self._pos_cur, (int, float)):
            txt += "  |  当前 %.3f turn" % self._pos_cur
        self.lab_pos_deg.setText(txt)

    def _on_pos_unit(self, idx):
        """切换 turn / 度:保持物理目标不变,只换算显示值。"""
        per = float(self.pos_unit.itemData(idx) or 1.0)
        old = self._pos_per_turn or 1.0
        turns = self.pos_spin.value() / old
        self._pos_per_turn = per
        self.pos_spin.blockSignals(True)
        self.pos_spin.setDecimals(3 if per == 1.0 else 1)
        self.pos_spin.setSingleStep(0.1 if per == 1.0 else 10.0)
        self.pos_spin.setValue(turns * per)
        self.pos_spin.blockSignals(False)
        self._update_pos_deg(self.pos_spin.value())

    def _load_current_pos(self):
        """把当前位置填进 pos_spin(绝对定位前先对齐,避免几千圈的长距离移动)。"""
        pos = self.worker.snapshot.get(self._p("encoder.pos_estimate"))
        if isinstance(pos, (int, float)):
            self._pos_cur = float(pos)
            self.pos_spin.setValue(float(pos) * self._pos_per_turn)

    # ---- 行为 ----

    def _jog_speed(self):
        return max(0.05, abs(self.vel_spin.value()) or 0.2)

    def send_vel(self, v):
        self.worker.write_req(self._p("controller.input_vel"), float(v))

    def _send_torque(self):
        self.worker.write_req(self._p("controller.input_torque"),
                              self.torque_spin.value())

    def update_state(self, snap):
        # 设定值不在工具侧限幅;仅更新“当前位置”显示(用于换算角度)
        pos = snap.get(self._p("encoder.pos_estimate"))
        if isinstance(pos, (int, float)):
            self._pos_cur = float(pos)
            self._update_pos_deg(self.pos_spin.value())

    def _pos_abs(self):
        self.worker.write_req(
            self._p("controller.input_pos"),
            self.pos_spin.value() / (self._pos_per_turn or 1.0))

    def _pos_rel(self, sign):
        req = self.worker.read_req([self._p("encoder.pos_estimate")])
        self._rel_pending = (req, sign,
                             self.pos_spin.value() / (self._pos_per_turn or 1.0))

    # ---- 事件 / 状态 ----

    def set_axis(self, axis):
        super().set_axis(axis)
        self.setTitle("设定值 — axis%d(速度 / 位置 / 力矩)" % axis)
        if getattr(self, "_mode", None) == MODE_POSITION:
            self._load_current_pos()

    def set_mode(self, mode):
        """按控制模式启用对应设定值控件(速度 / 位置 / 力矩)。"""
        prev = getattr(self, "_mode", None)
        self._mode = mode
        vel_on = mode == MODE_VELOCITY
        pos_on = mode == MODE_POSITION
        tq_on = mode == MODE_TORQUE
        for w in self._vel_widgets:
            w.setEnabled(vel_on)
        for w in self._pos_widgets:
            w.setEnabled(pos_on)
        for w in self._tq_widgets:
            w.setEnabled(tq_on)
        self.lab_tq_hint.setText(
            "input_torque [Nm]" if tq_on else "力矩模式专用")
        if pos_on and prev != MODE_POSITION:
            self._load_current_pos()      # 绝对定位默认从当前位置开始
        self.lab_mode.setText({
            MODE_VELOCITY: "当前速度模式:改 input_vel [turn/s] 后点“发送”",
            MODE_POSITION: "当前位置模式:输入 turn 或 度,点“绝对/相对”",
            MODE_TORQUE: "当前力矩模式:改 input_torque [Nm] 后点“发送”",
        }.get(mode, ""))

    def on_event(self, ev):
        if ev[0] == "rd_result" and getattr(self, "_rel_pending", None):
            req, sign, delta = self._rel_pending
            if req == ev[1]:
                cur = ev[2].get(self._p("encoder.pos_estimate"))
                if isinstance(cur, (int, float)):
                    self.worker.write_req(self._p("controller.input_pos"),
                                          cur + sign * delta)
                self._rel_pending = None
