"""OdrvStudio - 电机参数识别(自动标定流程)。

一键完成: 记录当前配置 → 写校准参数(电机类型/校准电流/电阻校准最大电压,
按 R×I 自动建议校准电压)→ 进入 MOTOR_CALIBRATION → 跟踪进度 →
读取实测相电阻/相电感 → 与旧配置对比、给电机类型建议,
可一键"应用实测值"(写 R/L 并置 pre_calibrated=True,跳过下次电机校准)。
"""

import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QLabel, QPushButton,
)

from ..device import AXIS_ERR, MOTOR_ERR, decode_error, state_name
from .theme import tag

MOTOR_TYPES = (("高电流 (0)", 0), ("云台 (2)", 2), ("交流感应 (3)", 3))

# 校准期间轴会进入的状态
CAL_STATE_NAMES = {3: "全流程校准", 4: "电机参数测量"}

OK_COLOR = "#2e7d32"
ERR_COLOR = "#c62828"
HINT_COLOR = "#607d8b"


class MotorParamPanel(QGroupBox):
    TIMEOUT_S = 40.0     # 校准超时(云台电机约 5-10 s,含停转等待)

    def __init__(self, worker, parent=None):
        super().__init__("电机参数识别", parent)
        self.worker = worker
        self.axis = 0
        self._phase = None       # None | "arming" | "running" | "reading"
        self._t0 = None
        self._seen_cal = False
        self._pre = {}           # 识别前的配置值 {"R":…, "L":…, "type":…}
        self._meas = None        # 识别结果 {"R":…, "L":…}
        self._pending = {}       # read_req id -> 标签

        g = QGridLayout(self)
        g.setVerticalSpacing(3)
        g.setHorizontalSpacing(4)

        lab = QLabel("电机类型")
        lab.setStyleSheet("color:#8a93a5;")
        self.type_combo = QComboBox()
        for name, val in MOTOR_TYPES:
            self.type_combo.addItem(name, val)
        self.type_combo.setToolTip(
            "云台电机(相电阻 > 1 Ω)选 云台(2);普通 hobby 电机选 高电流(0)")
        g.addWidget(lab, 0, 0)
        g.addWidget(self.type_combo, 0, 1, 1, 3)

        lab = QLabel("校准电流 [A]")
        lab.setStyleSheet("color:#8a93a5;")
        self.cal_i = QDoubleSpinBox()
        self.cal_i.setRange(0.1, 40.0)
        self.cal_i.setDecimals(2)
        self.cal_i.setValue(1.0)
        self.cal_i.setToolTip("电阻测量所用电流;云台电机建议 0.2–1 A\n"
                              "(固件默认 10 A,会烧云台电机)")
        lab2 = QLabel("最大电压 [V]")
        lab2.setStyleSheet("color:#8a93a5;")
        self.cal_v = QDoubleSpinBox()
        self.cal_v.setRange(0.1, 56.0)
        self.cal_v.setDecimals(2)
        self.cal_v.setValue(2.0)
        self.cal_v.setToolTip("需 > 校准电流×相电阻 且 < 0.5×Vbus;\n"
                              "点\"开始识别\"时会按 R×I×1.3 自动建议(不超过 0.45×Vbus)")
        g.addWidget(lab, 1, 0)
        g.addWidget(self.cal_i, 1, 1)
        g.addWidget(lab2, 1, 2)
        g.addWidget(self.cal_v, 1, 3)

        self.btn_start = QPushButton("开始识别")
        tag(self.btn_start, "primary")
        self.btn_start.setToolTip(
            "写校准参数 → 电机标定(若在闭环会先停)→ 跟踪进度 → 读实测 R/L。\n"
            "期间急停/停止会中止;完成后与旧配置对比并给出类型建议")
        self.btn_apply = QPushButton("应用实测值")
        tag(self.btn_apply, "success")
        self.btn_apply.setEnabled(False)
        self.btn_apply.setToolTip(
            "写入实测 R/L 并置 pre_calibrated=True(下次可跳过电机校准);\n"
            "掉电保存还需 通用控制→保存配置(会重启设备)")
        self.lab_state = QLabel("")
        self.lab_state.setStyleSheet("color:%s;" % HINT_COLOR)
        g.addWidget(self.btn_start, 2, 0)
        g.addWidget(self.btn_apply, 2, 1)
        g.addWidget(self.lab_state, 2, 2, 1, 2)

        self.lab_info = QLabel("—")
        self.lab_info.setStyleSheet("color:#8a93a5; font-size:10px;")
        self.lab_info.setWordWrap(True)
        g.addWidget(self.lab_info, 3, 0, 1, 4)

        self.lab_result = QLabel("")
        self.lab_result.setWordWrap(True)
        g.addWidget(self.lab_result, 4, 0, 1, 4)
        g.setColumnStretch(3, 1)

        self.btn_start.clicked.connect(self._start)
        self.btn_apply.clicked.connect(self._apply)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(150)

    # ---- 路径 / 读写 ----

    def _paths(self, keys):
        return ["axis%d.%s" % (self.axis, k) for k in keys]

    def _read(self, tag, keys):
        req = self.worker.read_req(self._paths(keys))
        self._pending[req] = tag

    def set_axis(self, axis):
        self.axis = axis
        self._reset("已切换到 axis%d" % axis)
        self._read("__info__", (
            "motor.config.motor_type", "motor.config.calibration_current",
            "motor.config.resistance_calib_max_voltage",
            "motor.config.phase_resistance", "motor.config.phase_inductance",
            "motor.config.pole_pairs", "motor.config.torque_constant",
            "motor.config.pre_calibrated"))

    def _reset(self, msg):
        self._phase = None
        self._seen_cal = False
        self._meas = None
        self.btn_start.setEnabled(True)
        self.btn_apply.setEnabled(False)
        self.lab_state.setText(msg)

    # ---- 开始识别(两步: 读旧配置 → 写参数+启动) ----

    def _start(self):
        if self._phase is not None:
            return
        self._pre = {}
        self._meas = None
        self.btn_start.setEnabled(False)
        self.btn_apply.setEnabled(False)
        self._phase = "arming"
        self.lab_state.setText("读取当前配置…")
        self._read("__pre__", (
            "motor.config.phase_resistance", "motor.config.phase_inductance",
            "motor.config.motor_type", "motor.config.calibration_current",
            "motor.config.resistance_calib_max_voltage"))

    def _fill_pre(self, d):
        old_r = d.get(self._paths(("motor.config.phase_resistance",))[0])
        old_l = d.get(self._paths(("motor.config.phase_inductance",))[0])
        old_t = d.get(self._paths(("motor.config.motor_type",))[0])
        self._pre = {
            "R": old_r if isinstance(old_r, (int, float)) else None,
            "L": old_l if isinstance(old_l, (int, float)) else None,
            "type": old_t if isinstance(old_t, int) else None,
        }
        # 校准电压自动建议: R×I×1.3,上限 0.45×Vbus
        r_est = self._pre["R"]
        if r_est and r_est > 0:
            vbus = self.worker.snapshot.get("vbus_voltage")
            cap = 0.45 * vbus if isinstance(vbus, (int, float)) and vbus > 0 else 56.0
            sug = min(max(r_est * self.cal_i.value() * 1.3, 0.5), cap)
            if sug > self.cal_v.value() + 0.05:
                self.cal_v.setValue(round(sug, 2))
                self.lab_state.setText("已按 R×I 自动提高校准电压至 %.1f V" % sug)
        a = self.axis
        self.worker.write_req("axis%d.motor.config.motor_type" % a,
                              self.type_combo.currentData())
        self.worker.write_req("axis%d.motor.config.calibration_current" % a,
                              self.cal_i.value())
        self.worker.write_req(
            "axis%d.motor.config.resistance_calib_max_voltage" % a,
            self.cal_v.value())
        self.worker.action_req("cal_motor", a)
        self._phase = "running"
        self._t0 = time.time()
        self._seen_cal = False
        self.lab_state.setText("启动识别…")

    # ---- 进度跟踪 ----

    def _on_tick(self):
        if self._phase != "running":
            return
        el = time.time() - self._t0
        st = self.worker.snapshot.get("axis%d.current_state" % self.axis)
        if st == 4:
            self._seen_cal = True
            self.lab_state.setText("识别中(电机参数测量)… %.1f s" % el)
        elif st == 8:
            self.lab_state.setText("等待电机停止… %.1f s" % el)
        if st == 1 and (self._seen_cal or el > 5.0):
            self._finish()
        elif el > self.TIMEOUT_S:
            self.lab_state.setText("识别超时(%s),读取错误信息…"
                                   % state_name(st if isinstance(st, int) else -1))
            self._finish()

    def _finish(self):
        self._phase = "reading"
        self._read("__post__", (
            "motor.error", "error",
            "motor.config.phase_resistance", "motor.config.phase_inductance",
            "motor.config.pre_calibrated"))

    # ---- 结果 ----

    def _fill_post(self, d):
        self._phase = None
        self.btn_start.setEnabled(True)
        err_m = d.get(self._paths(("motor.error",))[0])
        err_a = d.get(self._paths(("error",))[0])
        if not self._seen_cal:
            self._show_result("未进入校准状态(轴当前 %s)。\n"
                              "可能原因: 校准被拒绝或连接中断;检查状态后重试。"
                              % state_name(
                                  self.worker.snapshot.get(
                                      "axis%d.current_state" % self.axis, -1)),
                              ERR_COLOR)
            return
        if err_m or err_a:
            msg = "识别失败:\n  motor: %s\n  axis: %s" % (
                decode_error(err_m if isinstance(err_m, int) else 0, MOTOR_ERR),
                decode_error(err_a if isinstance(err_a, int) else 0, AXIS_ERR))
            if isinstance(err_m, int) and err_m & 0x03:
                msg += ("\n提示: 电阻/电感超出量程时,增大 校准电流 或 最大电压\n"
                        "(最大电压需 < 0.5×Vbus)")
            self._show_result(msg, ERR_COLOR)
            return
        r = d.get(self._paths(("motor.config.phase_resistance",))[0])
        l = d.get(self._paths(("motor.config.phase_inductance",))[0])
        if not isinstance(r, (int, float)) or r <= 0 or \
                not isinstance(l, (int, float)) or l <= 0:
            self._show_result("识别结果异常(R=%s, L=%s),请重试。" % (r, l),
                              ERR_COLOR)
            return
        self._meas = {"R": float(r), "L": float(l)}
        old_r = self._pre.get("R")
        old_l = self._pre.get("L")
        lines = ["实测相电阻 R = %.4g Ω" % r]
        if old_r:
            lines[-1] += "  (旧配置 %.4g,变化 %+.0f%%)" % (old_r, (r / old_r - 1) * 100)
        lines.append("实测相电感 L = %.4g H (%.0f µH)" % (l, l * 1e6))
        if old_l:
            lines[-1] += "  (旧配置 %.0f µH)" % (old_l * 1e6)
        lines.append("电气时间常数 τ = L/R = %.0f µs" % (l / r * 1e6))
        r_meas = self._meas["R"]
        t_cur = self.type_combo.currentData()
        if r_meas > 1.0 and t_cur != 2:
            lines.append("建议: R > 1 Ω,云台电机应选 电机类型=云台(2)")
        elif r_meas <= 1.0 and t_cur == 2:
            lines.append("建议: R ≤ 1 Ω,普通电机应选 电机类型=高电流(0)")
        lines.append("识别完成 — 点\"应用实测值\"写入并置 pre_calibrated。")
        self._show_result("\n".join(lines), OK_COLOR)
        self.btn_apply.setEnabled(True)

    def _apply(self):
        if not self._meas:
            return
        a = self.axis
        self.worker.write_req("axis%d.motor.config.phase_resistance" % a,
                              self._meas["R"])
        self.worker.write_req("axis%d.motor.config.phase_inductance" % a,
                              self._meas["L"])
        self.worker.write_req("axis%d.motor.config.pre_calibrated" % a, True)
        self.btn_apply.setEnabled(False)
        self.lab_state.setText("已写入实测值;掉电保存需 保存配置(会重启设备)")
        self._read("__info__", (
            "motor.config.phase_resistance", "motor.config.phase_inductance",
            "motor.config.pre_calibrated"))

    # ---- 显示 / 事件 ----

    def _show_result(self, text, color):
        self.lab_result.setText(text)
        self.lab_result.setStyleSheet("color:%s;" % color)

    def _fill_info(self, d):
        g = lambda k: d.get(self._paths((k,))[0])
        r = g("motor.config.phase_resistance")
        l = g("motor.config.phase_inductance")
        pp = g("motor.config.pole_pairs")
        kt = g("motor.config.torque_constant")
        pre = g("motor.config.pre_calibrated")
        parts = ["当前配置: "]
        if isinstance(r, (int, float)) and r > 0:
            parts.append("R=%.4gΩ" % r)
        if isinstance(l, (int, float)) and l > 0:
            parts.append("L=%.0fµH" % (l * 1e6))
        if isinstance(pp, int) and pp > 0:
            parts.append("极对数=%d" % pp)
        if isinstance(kt, (int, float)) and kt > 0:
            parts.append("Kt=%.3gN·m/A(KV≈%.0f)" % (kt, 8.27 / kt))
        if pre is True:
            parts.append("已 pre_calibrated")
        self.lab_info.setText(" ".join(parts))
        t = g("motor.config.motor_type")
        if isinstance(t, int) and self.type_combo.findData(t) >= 0:
            self.type_combo.setCurrentIndex(self.type_combo.findData(t))
        ci = g("motor.config.calibration_current")
        if isinstance(ci, (int, float)) and 0.1 <= ci <= 40.0:
            self.cal_i.setValue(float(ci))
        cv = g("motor.config.resistance_calib_max_voltage")
        if isinstance(cv, (int, float)) and 0.1 <= cv <= 56.0:
            self.cal_v.setValue(float(cv))

    def on_event(self, ev):
        kind = ev[0]
        if kind == "disconnected":
            if self._phase is not None:
                self._reset("连接已断开,识别中止")
            return
        if kind != "rd_result":
            return
        tag = self._pending.pop(ev[1], None)
        if tag is None:
            return
        if tag == "__info__":
            self._fill_info(ev[2])
        elif tag == "__pre__":
            self._fill_pre(ev[2])
        elif tag == "__post__":
            self._fill_post(ev[2])
