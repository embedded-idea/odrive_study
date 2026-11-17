"""OdrvStudio - 命令终端。

参考 SimpleFOCStudio 的 commandLineInterface:
直接读写任意属性 + 常用调试命令。
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QLineEdit, QPlainTextEdit, QPushButton, QHBoxLayout, QVBoxLayout, QWidget,
)

from ..device import (
    AXIS_ERR, CTRL_ERR, ENC_ERR, MOTOR_ERR, ODRV_ERR, decode_error,
    friendly_error,
)
from .theme import mono_font, tag

ERR_TABLES = {
    "error": ODRV_ERR,
    "axis0.error": AXIS_ERR,
    "axis0.motor.error": MOTOR_ERR,
    "axis0.encoder.error": ENC_ERR,
    "axis0.controller.error": CTRL_ERR,
}

HELP = """可用命令:
  help                        显示本帮助
  status                      摘要状态(全部轴)
  errors                      各级错误码(已翻译)
  clear                       清除全部错误
  start | stop                当前选中轴 进入闭环 / 回 IDLE
  estop                       全部轴急停
  cal motor | cal encoder     电机标定 / 编码器标定(当前轴)
  save                        保存配置并重启
  reboot                      重启
  get <path> [path…]          读属性,如 get axis0.controller.config.vel_gain
  set <path> <value>          写属性,如 set axis1.controller.input_vel 0.5
  <path>                      等价于 get <path>
  <path> = <value>            等价于 set <path> <value>
提示: axis1 的路径在双轴板卡上可用(如 axis1.encoder.pos_estimate)。
"""


class Terminal(QWidget):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self._hist = []
        self._hist_i = 0
        self._pending = {}

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setFont(mono_font(12))
        self.out.setStyleSheet("QPlainTextEdit{font-size:12px;}")
        lay.addWidget(self.out, 1)

        bar = QHBoxLayout()
        self.prompt = QLineEdit()
        self.prompt.setFont(mono_font(12))
        self.prompt.setPlaceholderText("输入命令,回车执行(help 查看帮助)")
        btn = QPushButton("发送")
        tag(btn, "primary")
        btn_clear = QPushButton("清空")
        bar.addWidget(self.prompt, 1)
        bar.addWidget(btn)
        bar.addWidget(btn_clear)
        lay.addLayout(bar)

        self.prompt.returnPressed.connect(self._run)
        btn.clicked.connect(self._run)
        btn_clear.clicked.connect(self.out.clear)
        self.prompt.installEventFilter(self)
        self.out.appendPlainText(HELP)

    # ---- 输出 ----

    def print(self, text):
        self.out.appendPlainText(text)
        self.out.moveCursor(QTextCursor.End)

    # ---- 命令解析 ----

    def _run(self):
        line = self.prompt.text().strip()
        if not line:
            return
        self.prompt.clear()
        self._hist.append(line)
        self._hist_i = len(self._hist)
        self.print(">>> " + line)
        try:
            self._exec(line)
        except Exception as e:
            self.print("[err] %s" % friendly_error(e))

    def _exec(self, line):
        low = line.lower()
        if low in ("help", "?", "h"):
            self.print(HELP)
        elif low == "status":
            req = self.worker.read_req([
                "vbus_voltage", "axis0.current_state",
                "axis0.encoder.vel_estimate", "axis0.encoder.pos_estimate"])
            self._pending[req] = "status"
        elif low == "errors":
            req = self.worker.read_req(list(ERR_TABLES))
            self._pending[req] = "errors"
        elif low == "clear":
            self.worker.action_req("clear_errors")
        elif low == "start":
            self.worker.action_req("start")
        elif low in ("stop", "estop"):
            self.worker.stop_req()
        elif low.startswith("cal"):
            parts = low.split()
            if len(parts) == 2 and parts[1] in ("motor", "encoder"):
                self.worker.action_req("cal_" + parts[1])
            else:
                self.print("用法: cal motor | cal encoder")
        elif low == "save":
            self.worker.action_req("save")
        elif low == "reboot":
            self.worker.action_req("reboot")
        elif low.startswith("get "):
            paths = [p for p in line[4:].split() if p]
            if paths:
                req = self.worker.read_req(paths)
                self._pending[req] = "get"
        elif low.startswith("set "):
            parts = line[4:].split(None, 1)
            if len(parts) != 2:
                self.print("用法: set <path> <value>")
                return
            self._write(parts[0], parts[1])
        elif " = " in line:
            path, val = line.split(" = ", 1)
            self._write(path.strip(), val.strip())
        elif "." in line and " " not in line:
            req = self.worker.read_req([line])
            self._pending[req] = "get"
        else:
            self.print("无法识别的命令,输入 help 查看帮助")

    def _write(self, path, value_text):
        v = value_text.strip()
        if v.lower() in ("true", "false"):
            value = v.lower() == "true"
        else:
            try:
                value = int(v, 0)
            except ValueError:
                try:
                    value = float(v)
                except ValueError:
                    value = v
        self.worker.write_req(path, value)

    # ---- 事件 ----

    def on_event(self, ev):
        if ev[0] == "log":
            self.print(ev[1])
        elif ev[0] == "wr_done":
            _, path, ok, err = ev
            self.print("[ok] %s 已写入" % path if ok
                       else "[err] 写入 %s 失败: %s" % (path, err))
        elif ev[0] == "rd_result":
            mode = self._pending.pop(ev[1], None)
            if mode is None:
                return
            vals = ev[2]
            if mode == "status":
                for p, v in vals.items():
                    self.print("  %s = %s" % (p, v))
            elif mode == "errors":
                for p, table in ERR_TABLES.items():
                    v = vals.get(p)
                    if isinstance(v, int):
                        self.print("  %-24s %s" % (p, decode_error(v, table)))
                    else:
                        self.print("  %-24s <读取失败>" % p)
            elif mode == "get":
                for p, v in vals.items():
                    self.print("  %s = %s" % (p, v))

    def eventFilter(self, obj, e):
        if obj is self.prompt and self._hist:
            if e.key() == Qt.Key_Up:
                self._hist_i = max(0, self._hist_i - 1)
                self.prompt.setText(self._hist[self._hist_i])
                return True
            if e.key() == Qt.Key_Down:
                self._hist_i = min(len(self._hist), self._hist_i + 1)
                self.prompt.setText(
                    self._hist[self._hist_i] if self._hist_i < len(self._hist) else "")
                return True
        return super().eventFilter(obj, e)
