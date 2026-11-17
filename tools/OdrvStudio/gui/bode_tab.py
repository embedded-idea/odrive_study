"""OdrvStudio - Bode 响应标签页。

显示速度扫频的锁相分析结果: 幅频(dB) 与 相频(°) 曲线,X 轴对数。
"""

from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

import pyqtgraph as pg

pg.setConfigOptions(antialias=False, background="#000000", foreground="#c8ccd4")


class BodeTab(QWidget):
    """扫频结果: 上下两块黑图(增益/相位)。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(2)

        self.lab_info = QLabel("尚未扫频 — 在设备页 PID 面板设置参数后点“开始扫频”。")
        self.lab_info.setStyleSheet("color:#607d8b;")
        lay.addWidget(self.lab_info)

        self.gain_plot = pg.PlotWidget()
        self.gain_plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.gain_plot.setLabel("bottom", "频率 [Hz]")
        self.gain_plot.setLabel("left", "增益 [dB]")
        self.gain_plot.setLogMode(x=True, y=False)
        self.gain_plot.showGrid(x=True, y=True, alpha=0.3)
        lay.addWidget(self.gain_plot, 1)

        self.phase_plot = pg.PlotWidget()
        self.phase_plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.phase_plot.setLabel("bottom", "频率 [Hz]")
        self.phase_plot.setLabel("left", "相位 [°]")
        self.phase_plot.setLogMode(x=True, y=False)
        self.phase_plot.showGrid(x=True, y=True, alpha=0.3)
        lay.addWidget(self.phase_plot, 1)

        bar = QHBoxLayout()
        self.btn_csv = QPushButton("导出 CSV")
        self.btn_clear = QPushButton("清空")
        bar.addStretch(1)
        bar.addWidget(self.btn_csv)
        bar.addWidget(self.btn_clear)
        lay.addLayout(bar)

        self._data = None
        self.gain_curve = None
        self.phase_curve = None
        self.btn_clear.clicked.connect(self.clear)
        self.btn_csv.clicked.connect(self._export_csv)

    def set_data(self, data):
        self._data = data
        axis = data.get("axis", 0)
        name = "axis%d  幅度 %.3g turn/s" % (axis, data.get("amp", 0))
        self.lab_info.setText(
            "扫频结果: %s | %d 个频点,每点 %s 周期"
            % (name, len(data.get("freqs", [])), data.get("cycles", "-")))
        self.gain_plot.clear()
        self.phase_plot.clear()
        self.gain_curve = self.gain_plot.plot(
            data["freqs"], data["gain_db"], pen=pg.mkPen("#26c6da", width=1.6),
            symbol="o", symbolSize=4, symbolBrush="#26c6da", name="增益")
        self.phase_curve = self.phase_plot.plot(
            data["freqs"], data["phase_deg"], pen=pg.mkPen("#ef5350", width=1.6),
            symbol="o", symbolSize=4, symbolBrush="#ef5350", name="相位")

    def clear(self):
        self._data = None
        self.gain_plot.clear()
        self.phase_plot.clear()
        self.lab_info.setText("已清空 — 重新扫频生成新数据。")

    def _export_csv(self):
        from PyQt5.QtWidgets import QFileDialog
        if not self._data:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "导出 Bode 数据", "odrv_bode_a%d.csv" % self._data.get("axis", 0),
            "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            f.write("freq_hz,gain_db,phase_deg\n")
            for fr, g, p in zip(self._data["freqs"], self._data["gain_db"],
                                self._data["phase_deg"]):
                f.write("%.6g,%.6g,%.6g\n" % (fr, g, p))
