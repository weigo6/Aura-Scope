import sys
import os
import csv
import datetime
import json
import numpy as np
import serial
import serial.tools.list_ports
import struct
from collections import deque

# 核心 UI 库导入
from PySide6.QtCore import Qt, QThread, Signal, Slot, QPointF, QTimer
from PySide6.QtGui import QColor, QPalette, QIcon, QPixmap, QPainter
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QComboBox, QPushButton, QLabel, 
                             QCheckBox, QRadioButton, QGroupBox, QSlider, QButtonGroup,
                             QFileDialog, QMessageBox, QScrollArea, QFrame,
                             QSplitter, QDoubleSpinBox)
import pyqtgraph as pg
from scipy.fft import rfft, rfftfreq
from scipy.signal import find_peaks

# --- 采样率对应表 ---
FS_TABLE = [500000, 200000, 100000, 50000, 20000, 10000, 5000, 2000, 1000]

# --- 样式表 (Dark Theme) ---
DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #1e1e1e;
    color: #e0e0e0;
    font-family: "Segoe UI", "Microsoft YaHei";
    font-size: 9pt;
}

QGroupBox {
    border: 1px solid #3d3d3d;
    border-radius: 6px;
    margin-top: 15px;
    padding-top: 4px;
    font-weight: bold;
    background-color: #252526;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 3px;
    color: #4db6ac; 
    background-color: #252526; 
}

QPushButton {
    background-color: #3c3c3c;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 6px 12px;
    color: #ffffff;
}
QPushButton:hover {
    background-color: #505050;
    border-color: #4db6ac;
}
QPushButton:pressed {
    background-color: #252526;
}
QPushButton:disabled {
    background-color: #2d2d2d;
    color: #777;
    border-color: #333;
}

QComboBox {
    background-color: #3c3c3c;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 4px;
    color: #ffffff;
}
QComboBox::drop-down {
    border: none;
    width: 20px;
}
QComboBox QAbstractItemView {
    background-color: #3c3c3c;
    color: #ffffff;
    selection-background-color: #4db6ac;
}

QLineEdit {
    background-color: #3c3c3c;
    border: 1px solid #555;
    border-radius: 4px;
    color: #fff;
    padding: 4px;
}

QCheckBox {
    spacing: 5px;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #555;
    border-radius: 3px;
    background: #3c3c3c;
}
QCheckBox::indicator:checked {
    background-color: rgba(77, 182, 172, 0.15);
    border-color: #4db6ac;
    image: url(check.png);
}

QRadioButton {
    spacing: 5px;
}
QRadioButton::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #555;
    border-radius: 8px;
    background: #3c3c3c;
}
QRadioButton::indicator:checked {
    border: 1px solid #4db6ac;
    border-radius: 8px;
    background-color: qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:0.5, fx:0.5, fy:0.5, stop:0 #4db6ac, stop:0.5 #4db6ac, stop:0.6 transparent, stop:1 transparent);
    image: none;
}

QSlider::groove:horizontal {
    border: 1px solid #3d3d3d;
    height: 6px;
    background: #2d2d2d;
    margin: 2px 0;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #4db6ac;
    border: 1px solid #4db6ac;
    width: 10px;
    height: 10px;
    margin: -3px 0;
    border-radius: 5px;
}

QSplitter::handle {
    background-color: #2d2d2d;
}
QSplitter::handle:horizontal {
    width: 4px;
}

QScrollBar:vertical {
    border: none;
    background: #1e1e1e;
    width: 10px;
    margin: 0px 0px 0px 0px;
}
QScrollBar::handle:vertical {
    background: #444;
    min-height: 20px;
    border-radius: 5px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

QLabel#Dashboard {
    background-color: #252526;
    border: 1px solid #3d3d3d;
    border-radius: 6px;
    padding: 8px;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 10pt;
}
"""

class SignalProcessor:
    @staticmethod
    def get_vpp(data):
        return np.ptp(data) if len(data) > 0 else 0

    @staticmethod
    def phase_delay_xcorr(data1, data2, fs, freq_hint=None):
        if len(data1) < 10 or len(data2) < 10:
            return 0
            
        # 1. 频率估算
        if freq_hint is None or freq_hint <= 0:
            f0 = SignalProcessor.freq_acf(data1, fs)
        else:
            f0 = freq_hint
            
        if f0 <= 0:
            return 0
            
        # 2. Reference Signal Windowing (关键: 避免窗函数效应)
        # 不截断原始数据，而是定义一个“有效搜索范围”
        # 我们只需要搜索 +/- 0.6 个周期即可覆盖 +/- 180 度相位差
        n = min(len(data1), len(data2))
        x = data1[:n] - np.mean(data1[:n])
        y = data2[:n] - np.mean(data2[:n])
        
        period_pts = fs / f0
        # 搜索范围: +/- 0.6 周期 (留点余量)
        max_lag = int(np.ceil(0.6 * period_pts))
        
        # 确保数据足够长以支持 Valid 模式卷积
        # Template 长度 = n - 2 * max_lag
        # 我们希望 Template 长度至少有一些点
        if n > 2 * max_lag + 2:
            # 使用 Valid 模式: 
            # Template 取自 x 的中心，长度较短
            # y 保持全长
            # 这样卷积过程中，重叠长度始终等于 Template 长度 (常数)
            # 因此不需要归一化，且没有三角形窗效应
            start = max_lag
            end = n - max_lag
            template = x[start:end]
            
            # correlate(y, template, 'valid') 
            # Output length = len(y) - len(template) + 1 = 2 * max_lag + 1
            # Lags 对应从 -max_lag 到 +max_lag
            corr = np.correlate(y, template, mode='valid')
            lags = np.arange(-max_lag, -max_lag + len(corr))
        else:
            # 数据太短，退回到 Full 模式 (不推荐，但作为 fallback)
            corr = np.correlate(x, y, mode='full')
            lags = np.arange(-n + 1, n)
            # 简单的中心加权抑制边缘
            # 或者直接相信中心峰值
            
        # 3. 找峰值
        peak_idx = np.argmax(corr)
        lag = lags[peak_idx]
        
        # 4. 抛物线插值 (提升精度到亚采样点)
        if 0 < peak_idx < len(corr) - 1:
            y0 = corr[peak_idx]
            ym1 = corr[peak_idx - 1]
            yp1 = corr[peak_idx + 1]
            # y(x) = a(x-x0)^2 + b(x-x0) + c
            # delta = -b / 2a
            denom = 2 * (ym1 - 2 * y0 + yp1)
            if denom != 0:
                delta = (ym1 - yp1) / denom
                if -0.5 <= delta <= 0.5:
                    lag += delta

        # 注意: lag > 0 表示 y 滞后于 x (delay), 对应相位差为负
        # lag < 0 表示 y 超前于 x (lead), 对应相位差为正
        # 因此需要取反
        phase = -(lag * f0 / fs) * 360.0
        phase = (phase + 180.0) % 360.0 - 180.0
        return phase

    @staticmethod
    def phase_fft(data1, data2, fs, freq_hint=None):
        if len(data1) < 10 or len(data2) < 10:
            return 0
        n = min(len(data1), len(data2))
        x = data1[:n] - np.mean(data1[:n])
        y = data2[:n] - np.mean(data2[:n])
        
        # 改进: 如果已知频率，使用 Single Bin DFT 计算精确相位
        if freq_hint is not None and freq_hint > 0:
            t = np.arange(n) / fs
            exp_vec = np.exp(-1j * 2 * np.pi * freq_hint * t)
            # 计算指定频率点的 DFT 分量
            X = np.sum(x * exp_vec)
            Y = np.sum(y * exp_vec)
            p1 = np.angle(X)
            p2 = np.angle(Y)
        else:
            yf1 = rfft(x)
            yf2 = rfft(y)
            xf = rfftfreq(n, 1/fs)
            if freq_hint is not None and freq_hint > 0:
                idx = np.argmin(np.abs(xf - freq_hint))
            else:
                m1 = np.abs(yf1)
                if len(m1) == 0:
                    return 0
                idx = np.argmax(m1)
            if idx <= 0 or idx >= len(yf1) or idx >= len(yf2):
                return 0
            p1 = np.angle(yf1[idx])
            p2 = np.angle(yf2[idx])

        phase = (p2 - p1) * 180.0 / np.pi
        phase = (phase + 180.0) % 360.0 - 180.0
        return phase

    @staticmethod
    def freq_schmitt(data, fs):
        if len(data) < 10: return 0
        norm = data - np.mean(data)
        vpp = np.ptp(norm)
        
        # 增加幅度检测门限，太小的信号视为噪声
        if vpp < 0.1: return 0
        
        # 改进为双阈值施密特触发器 (Hysteresis)
        # High Threshold: +20% Vpp
        # Low Threshold:  -20% Vpp
        # 这样能有效抵抗过零点附近的噪声干扰
        high_thresh = vpp * 0.2
        low_thresh = -vpp * 0.2
        
        # 1: > high, -1: < low, 0: hold
        threshold_signals = np.zeros_like(norm, dtype=int)
        threshold_signals[norm > high_thresh] = 1
        threshold_signals[norm < low_thresh] = -1
        
        # 提取有效状态翻转点
        mask = threshold_signals != 0
        if not np.any(mask):
            return 0
            
        indices = np.where(mask)[0]
        values = threshold_signals[indices]
        
        # 寻找上升沿: 状态从 -1 变为 1
        # values[:-1] == -1 且 values[1:] == 1
        rising_edge_mask = (values[:-1] == -1) & (values[1:] == 1)
        # indices[1:] 对应发生 1 状态的位置（即翻转后的第一时间点）
        edges = indices[1:][rising_edge_mask]
        
        if len(edges) < 2: return 0
        
        return fs / np.mean(np.diff(edges))

    @staticmethod
    def freq_acf(data, fs):
        if len(data) < 10: return 0
        n = len(data)
        norm = data - np.mean(data)
        
        # Use Biased ACF (more robust against noise)
        acf = np.correlate(norm, norm, mode='full')
        acf = acf[len(acf)//2:]
        
        # Search range parameters
        # 1. Dynamic start: Find first zero crossing to skip main lobe
        # Fixed 50 was too large for high freq signals (e.g. 2400Hz @ 100kSps -> Period 41)
        # We look for the first point where correlation drops to zero or below.
        zero_crossings = np.where(acf <= 0)[0]
        if len(zero_crossings) > 0:
            search_start = zero_crossings[0]
        else:
            # If never crosses zero, use a small default or return 0
            # But let's use a safe small number to allow search
            search_start = 10
            
        # Ensure we don't start too late or too early (avoid lag 0)
        search_start = max(search_start, 5)
        # 2. End at 70% to avoid tail noise (where overlap is small)
        search_limit = int(n * 0.7)
        
        if search_start >= len(acf): return 0
        
        # Threshold: 30% of max energy (lag 0)
        threshold = acf[0] * 0.3
        
        # Find peaks in the valid range
        peaks, _ = find_peaks(acf[search_start:search_limit], height=threshold)
        
        if len(peaks) < 1: return 0
        
        # Adjust indices
        peaks = peaks + search_start
        
        # Take the FIRST significant peak as the fundamental period
        peak_idx = peaks[0]
        true_idx = float(peak_idx)
        
        # Parabolic interpolation on Biased ACF for sub-sample precision
        if 0 < peak_idx < len(acf) - 1:
            y0 = acf[peak_idx]
            ym1 = acf[peak_idx - 1]
            yp1 = acf[peak_idx + 1]
            denom = 2 * (ym1 - 2 * y0 + yp1)
            if denom != 0:
                delta = (ym1 - yp1) / denom
                # Only accept reasonable delta
                if -1.0 <= delta <= 1.0:
                    true_idx += delta
                    
        return fs / true_idx

class DataWorker(QThread):
    """串口接收线程"""
    packet_ready = Signal(np.ndarray, np.ndarray, int)
    
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, 2000000, timeout=0.1)
            try:
                if hasattr(ser, "set_buffer_size"):
                    try:
                        ser.set_buffer_size(rx_size=65536, tx_size=65536)
                    except TypeError:
                        try: ser.set_buffer_size(65536, 65536)
                        except TypeError: pass
            except Exception: pass
            
            while self.running:
                if ser.in_waiting >= 5:
                    if ser.read(1) == b'\xAA':
                        if ser.read(1) == b'\x55':
                            header = ser.read(3)
                            if len(header) < 3: continue
                            
                            fs_idx = header[0]
                            data_len = struct.unpack('<H', header[1:3])[0]

                            if fs_idx >= len(FS_TABLE): continue
                            if data_len != 4000: continue
                            
                            raw = ser.read(data_len)
                            actual_len = len(raw)
                            if actual_len != data_len or actual_len % 4 != 0: continue
                            
                            try:
                                data_32 = np.frombuffer(raw, dtype=np.uint32)
                                ch1 = data_32 & 0x0FFF
                                ch2 = (data_32 >> 16) & 0x0FFF
                                self.packet_ready.emit(ch1, ch2, fs_idx)
                            except Exception: continue 
                                
        except Exception as e:
            print(f"Serial worker error: {e}")
        finally:
            if 'ser' in locals(): ser.close()

    def stop(self):
        self.running = False
        self.wait()

class AuraScope(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AuraScope Pro - STM32 Signal Analyzer")
        self.resize(1280, 800)
        
        # 处理资源路径和样式表
        base_dir = os.path.dirname(__file__).replace('\\', '/')
        
        # 1. 修正 check.png 路径
        style = DARK_STYLESHEET.replace("url(check.png)", f"url({base_dir}/check.png)")
        
        # 2. 生成并添加 arrow.png 样式
        self.ensure_arrow_icon()
        style += f"""
        QComboBox::down-arrow {{
            image: url({base_dir}/arrow.png);
            width: 12px;
            height: 12px;
            padding-right: 2px;
        }}
        """
        self.setStyleSheet(style)
        
        # 设置窗口图标
        icon_path = os.path.join(os.path.dirname(__file__), "oscilloscope.png")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
        
        self.double_buf_ch1 = []
        self.double_buf_ch2 = []
        self.time_mode_buf_ch1 = []
        self.time_mode_buf_ch2 = []
        self.rolling_ch1 = deque(maxlen=5000)
        self.rolling_ch2 = deque(maxlen=5000)
        self.mode_points = [1000, 2000, 5000]
        self.normal_volt_range = (-6.0, 6.0)
        self.atten_volt_range = (-300.0, 300.0)
        self.last_vpp1 = None
        self.last_vpp2 = None
        self.enable_vpp_filter = True
        self.enable_smooth = False
        self.show_ch1 = True
        self.show_ch2 = True
        self.paused = False
        
        # 数据导出缓存
        self.last_view_ch1 = None
        self.last_view_ch2 = None
        self.last_fs = 0

        self.is_connected = False
        
        # 校准相关变量
        self.cal_data = {}  # 存储所有档位的校准数据
        self.current_scale_key = "x1"
        self.ch1_offset = 0.0
        self.ch1_gain = 1.0
        self.ch2_offset = 0.0
        self.ch2_gain = 1.0
        self.is_calibrating = False
        self.is_calibrating_gain = False
        self.cal_active_ch1 = True
        self.cal_active_ch2 = True
        self.cal_samples_ch1 = []
        self.cal_samples_ch2 = []
        self.load_config()
        
        # 光标相关
        self.cursor_t1 = None
        self.cursor_t2 = None
        
        self.init_ui()
        self.setup_plots()
        self.refresh_ports()

    def init_ui(self):
        # 主布局容器
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)

        # === 左侧侧边栏 (Control Panel) ===
        scroll_area = QScrollArea()
        scroll_area.setMinimumWidth(300)
        scroll_area.setMaximumWidth(480)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(5, 5, 10, 5)
        scroll_layout.setSpacing(6)

        # 1. 设备连接
        gp_conn = QGroupBox("1. 连接设置")
        l_conn = QVBoxLayout()
        h_port = QHBoxLayout()
        self.cb_port = QComboBox()
        self.btn_refresh = QPushButton()
        self.btn_refresh.setFixedWidth(30)
        
        refresh_icon_path = os.path.join(os.path.dirname(__file__), "refresh.png")
        if os.path.exists(refresh_icon_path):
            self.btn_refresh.setIcon(QIcon(refresh_icon_path))
        else:
            self.btn_refresh.setText("R")
            
        self.btn_refresh.setToolTip("刷新串口列表")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        h_port.addWidget(self.cb_port)
        h_port.addWidget(self.btn_refresh)
        self.btn_conn = QPushButton("打开串口")
        self.btn_conn.setStyleSheet("font-weight: bold; background-color: #2e7d32;")
        self.btn_conn.clicked.connect(self.toggle_serial)
        l_conn.addLayout(h_port)
        l_conn.addWidget(self.btn_conn)
        gp_conn.setLayout(l_conn)
        scroll_layout.addWidget(gp_conn)

        # 2. 采集设置
        gp_mode = QGroupBox("2. 采集模式")
        l_mode = QVBoxLayout()
        self.cb_display_mode = QComboBox()
        self.cb_display_mode.addItems(["单包 (1k pts)", "双包 (2k pts)", "滚动 (5k pts)", "按时间采集"])
        self.cb_display_mode.currentIndexChanged.connect(self.on_mode_changed)

        # 时间模式控制容器
        self.container_time_mode = QWidget()
        h_time_layout = QHBoxLayout(self.container_time_mode)
        h_time_layout.setContentsMargins(0, 0, 0, 0)
        
        self.spin_time_duration = QDoubleSpinBox()
        self.spin_time_duration.setPrefix("时长: ")
        self.spin_time_duration.setSuffix(" s")
        self.spin_time_duration.setRange(0.1, 10.0)
        self.spin_time_duration.setSingleStep(0.1)
        self.spin_time_duration.setValue(1.0)
        
        self.btn_time_refresh = QPushButton("刷新")
        self.btn_time_refresh.setFixedWidth(50)
        self.btn_time_refresh.clicked.connect(self.on_time_mode_refresh)
        self.btn_time_refresh.setStyleSheet("padding: 4px;")
        
        h_time_layout.addWidget(self.spin_time_duration)
        h_time_layout.addWidget(self.btn_time_refresh)
        
        self.container_time_mode.setVisible(False)

        h_scale = QHBoxLayout()
        h_scale.addWidget(QLabel("探头倍率:"))
        self.cb_probe_scale = QComboBox()
        self.cb_probe_scale.addItems(["x1", "x50", "x100"])
        self.cb_probe_scale.currentTextChanged.connect(self.on_scale_changed)
        h_scale.addWidget(self.cb_probe_scale)

        l_mode.addWidget(self.cb_display_mode)
        l_mode.addWidget(self.container_time_mode)
        l_mode.addLayout(h_scale)
        gp_mode.setLayout(l_mode)
        scroll_layout.addWidget(gp_mode)

        # 3. 触发控制
        gp_trig = QGroupBox("3. 软件触发")
        l_trig = QVBoxLayout()
        h_trig_en = QHBoxLayout()
        self.chk_trig = QCheckBox("启用触发")
        h_trig_en.addWidget(self.chk_trig)
        l_trig.addLayout(h_trig_en)
        
        self.sl_trig = QSlider(Qt.Horizontal)
        self.sl_trig.setRange(-500, 500)
        self.sl_trig.valueChanged.connect(self.on_trigger_slider_move)
        l_trig.addWidget(self.sl_trig)
        
        self.lbl_trig = QLabel("电平: 0.00 V")
        self.lbl_trig.setAlignment(Qt.AlignCenter)
        l_trig.addWidget(self.lbl_trig)
        gp_trig.setLayout(l_trig)
        scroll_layout.addWidget(gp_trig)

        # 4. 显示通道
        gp_ch = QGroupBox("4. 通道管理")
        l_ch = QVBoxLayout()
        h_ch_sw = QHBoxLayout()
        self.chk_show_ch1 = QCheckBox("CH1 (黄)")
        self.chk_show_ch1.setChecked(True)
        self.chk_show_ch1.setStyleSheet("color: #ffeb3b; font-weight: bold;")
        self.chk_show_ch1.toggled.connect(self.on_channel_toggled)
        self.chk_show_ch2 = QCheckBox("CH2 (青)")
        self.chk_show_ch2.setChecked(True)
        self.chk_show_ch2.setStyleSheet("color: #00bcd4; font-weight: bold;")
        self.chk_show_ch2.toggled.connect(self.on_channel_toggled)
        h_ch_sw.addWidget(self.chk_show_ch1)
        h_ch_sw.addWidget(self.chk_show_ch2)
        l_ch.addLayout(h_ch_sw)
        
        h_cursors = QHBoxLayout()
        self.chk_cursors_x = QCheckBox("X轴光标")
        self.chk_cursors_x.toggled.connect(self.on_cursors_x_toggled)
        self.chk_cursors_y = QCheckBox("Y轴光标")
        self.chk_cursors_y.toggled.connect(self.on_cursors_y_toggled)
        h_cursors.addWidget(self.chk_cursors_x)
        h_cursors.addWidget(self.chk_cursors_y)
        l_ch.addLayout(h_cursors)
        gp_ch.setLayout(l_ch)
        scroll_layout.addWidget(gp_ch)

        # 5. 算法设置
        gp_algo = QGroupBox("5. 算法与 FFT")
        l_algo = QVBoxLayout()
        
        l_freq = QHBoxLayout()
        self.rad_schmitt = QRadioButton("施密特测频")
        self.rad_acf = QRadioButton("自相关测频")
        self.rad_schmitt.setChecked(True)
        self.algo_group = QButtonGroup(self)
        self.algo_group.addButton(self.rad_schmitt)
        self.algo_group.addButton(self.rad_acf)
        self.algo_group.buttonClicked.connect(self.reprocess_view)
        l_freq.addWidget(self.rad_schmitt)
        l_freq.addWidget(self.rad_acf)
        l_algo.addLayout(l_freq)

        l_fft = QHBoxLayout()
        self.rad_fft_lin = QRadioButton("线性坐标")
        self.rad_fft_log = QRadioButton("对数(dB)坐标")
        self.rad_fft_lin.setChecked(True)
        self.fft_group = QButtonGroup(self)
        self.fft_group.addButton(self.rad_fft_lin)
        self.fft_group.addButton(self.rad_fft_log)
        self.fft_group.buttonClicked.connect(self.reprocess_view)
        l_fft.addWidget(self.rad_fft_lin)
        l_fft.addWidget(self.rad_fft_log)
        l_algo.addLayout(l_fft)

        l_phase = QHBoxLayout()
        self.rad_phase_xcorr = QRadioButton("XCorr相位测量")
        self.rad_phase_fft = QRadioButton("FFT相位测量")
        self.rad_phase_xcorr.setChecked(True)
        self.phase_group = QButtonGroup(self)
        self.phase_group.addButton(self.rad_phase_xcorr)
        self.phase_group.addButton(self.rad_phase_fft)
        self.phase_group.buttonClicked.connect(self.reprocess_view)
        l_phase.addWidget(self.rad_phase_xcorr)
        l_phase.addWidget(self.rad_phase_fft)
        l_algo.addLayout(l_phase)

        gp_algo.setLayout(l_algo)
        scroll_layout.addWidget(gp_algo)

        # 6. 视图控制
        gp_view = QGroupBox("6. 视图优化")
        l_view = QVBoxLayout()
        
        self.btn_auto_range = QPushButton("自动全轴 (Auto Range)")
        self.btn_auto_range.clicked.connect(self.on_auto_range_clicked)
        l_view.addWidget(self.btn_auto_range)

        h_lock = QHBoxLayout()
        self.chk_lock_x = QCheckBox("锁定X轴")
        self.chk_lock_y = QCheckBox("锁定Y轴")
        self.chk_lock_x.toggled.connect(self.on_axis_lock_changed)
        self.chk_lock_y.toggled.connect(self.on_axis_lock_changed)
        h_lock.addWidget(self.chk_lock_x)
        h_lock.addWidget(self.chk_lock_y)
        l_view.addLayout(h_lock)

        self.chk_vpp_filter = QCheckBox("Vpp 异常过滤")
        self.chk_vpp_filter.setChecked(True)
        self.chk_vpp_filter.toggled.connect(self.on_vpp_filter_toggled)
        l_view.addWidget(self.chk_vpp_filter)
        
        self.chk_smooth = QCheckBox("波形平滑 (3点)")
        self.chk_smooth.toggled.connect(self.on_smooth_toggled)
        l_view.addWidget(self.chk_smooth)
        
        gp_view.setLayout(l_view)
        scroll_layout.addWidget(gp_view)

        # 7. 导出与控制
        gp_act = QGroupBox("7. 操作")
        l_act = QVBoxLayout()
        
        self.btn_pause = QPushButton("暂停 / 继续")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self.on_pause_toggled)
        l_act.addWidget(self.btn_pause)

        h_io = QHBoxLayout()
        self.btn_export = QPushButton("导出")
        self.btn_export.clicked.connect(self.export_csv)
        self.btn_import = QPushButton("导入")
        self.btn_import.clicked.connect(self.import_csv)
        h_io.addWidget(self.btn_export)
        h_io.addWidget(self.btn_import)
        l_act.addLayout(h_io)

        self.btn_help = QPushButton("帮助与说明")
        self.btn_help.clicked.connect(self.show_help)
        self.btn_help.setStyleSheet("background-color: #0277bd; font-weight: bold;")
        l_act.addWidget(self.btn_help)

        gp_act.setLayout(l_act)
        scroll_layout.addWidget(gp_act)

        # 8. 信号校准
        gp_cal = QGroupBox("8. 信号校准")
        l_cal = QVBoxLayout()
        
        # CH1 校准
        h_cal1 = QHBoxLayout()
        self.spin_ch1_off = QDoubleSpinBox()
        self.spin_ch1_off.setRange(-5.0, 5.0)
        self.spin_ch1_off.setSingleStep(0.001)
        self.spin_ch1_off.setDecimals(3)
        self.spin_ch1_off.setValue(self.ch1_offset)
        self.spin_ch1_off.setPrefix("C1 Off: ")
        self.spin_ch1_off.setSuffix(" V")
        self.spin_ch1_off.valueChanged.connect(self.on_cal_params_changed)
        
        self.spin_ch1_gain = QDoubleSpinBox()
        self.spin_ch1_gain.setRange(0.5, 2.0)
        self.spin_ch1_gain.setSingleStep(0.01)
        self.spin_ch1_gain.setValue(self.ch1_gain)
        self.spin_ch1_gain.setPrefix("Gain: ")
        self.spin_ch1_gain.valueChanged.connect(self.on_cal_params_changed)
        
        h_cal1.addWidget(self.spin_ch1_off)
        h_cal1.addWidget(self.spin_ch1_gain)
        l_cal.addLayout(h_cal1)
        
        # CH2 校准
        h_cal2 = QHBoxLayout()
        self.spin_ch2_off = QDoubleSpinBox()
        self.spin_ch2_off.setRange(-5.0, 5.0)
        self.spin_ch2_off.setSingleStep(0.001)
        self.spin_ch2_off.setDecimals(3)
        self.spin_ch2_off.setValue(self.ch2_offset)
        self.spin_ch2_off.setPrefix("C2 Off: ")
        self.spin_ch2_off.setSuffix(" V")
        self.spin_ch2_off.valueChanged.connect(self.on_cal_params_changed)
        
        self.spin_ch2_gain = QDoubleSpinBox()
        self.spin_ch2_gain.setRange(0.5, 2.0)
        self.spin_ch2_gain.setSingleStep(0.01)
        self.spin_ch2_gain.setValue(self.ch2_gain)
        self.spin_ch2_gain.setPrefix("Gain: ")
        self.spin_ch2_gain.valueChanged.connect(self.on_cal_params_changed)
        
        h_cal2.addWidget(self.spin_ch2_off)
        h_cal2.addWidget(self.spin_ch2_gain)
        l_cal.addLayout(h_cal2)
        
        self.btn_auto_zero = QPushButton("自动归零 (Auto Zero)")
        self.btn_auto_zero.clicked.connect(self.on_auto_zero_clicked)
        self.btn_auto_zero.setStyleSheet("background-color: #5d4037;")
        l_cal.addWidget(self.btn_auto_zero)

        # 增益校准 UI
        h_gain_cal = QHBoxLayout()
        self.spin_ref_vpp = QDoubleSpinBox()
        self.spin_ref_vpp.setRange(0.1, 400.0)
        self.spin_ref_vpp.setValue(3.3)
        self.spin_ref_vpp.setDecimals(3)
        self.spin_ref_vpp.setPrefix("Ref: ")
        self.spin_ref_vpp.setSuffix(" V")
        self.spin_ref_vpp.setToolTip("输入参考电压源的实际精确值")
        self.spin_ref_vpp.setFocusPolicy(Qt.ClickFocus) # 防止自动聚焦导致的高亮
        
        self.btn_auto_gain = QPushButton("自动增益 (Auto Gain)")
        self.btn_auto_gain.clicked.connect(self.on_auto_gain_clicked)
        self.btn_auto_gain.setStyleSheet("background-color: #455a64;")
        
        h_gain_cal.addWidget(self.spin_ref_vpp)
        h_gain_cal.addWidget(self.btn_auto_gain)
        l_cal.addLayout(h_gain_cal)
        
        self.btn_save_cal = QPushButton("保存校准配置")
        self.btn_save_cal.clicked.connect(self.save_config)
        l_cal.addWidget(self.btn_save_cal)
        
        gp_cal.setLayout(l_cal)
        scroll_layout.addWidget(gp_cal)

        # 填充底部空白
        scroll_layout.addStretch()
        scroll_area.setWidget(scroll_content)
        
        # === 右侧绘图区 ===
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(5)

        # 绘图控件
        self.win = pg.GraphicsLayoutWidget()
        self.win.setBackground('#1e1e1e')
        right_layout.addWidget(self.win)

        # 仪表盘信息
        self.lbl_dash = QLabel()
        self.lbl_dash.setObjectName("Dashboard")
        self.lbl_dash.setFixedHeight(150)
        self.lbl_dash.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        # 初始占位显示
        self.lbl_dash.setText("""
            <table width='100%'>
            <tr><td align='center' style='padding-top:40px; color:#666; font-size:14px'>
                <b>AuraScope Pro</b><br>
                <span style='font-size:11px'>No Device Connected</span>
            </td></tr>
            </table>
        """)
        
        # 使用布局包裹以添加边距 (Left, Top, Right, Bottom)
        # 左侧留出 70px 以对齐上方绘图区的 Y 轴 (60px 轴宽 + 间隙)
        dash_container = QHBoxLayout()
        dash_container.setContentsMargins(70, 0, 12, 10)
        dash_container.addWidget(self.lbl_dash)
        right_layout.addLayout(dash_container)

        # 主分割器
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(scroll_area)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(1, 4)
        
        main_layout.addWidget(splitter)

    def setup_plots(self):
        # 1. 时域图
        self.p_t = self.win.addPlot(row=0, col=0, title="Time Domain")
        # 固定 Y 轴宽度以确保上下图表对齐
        self.p_t.getAxis('left').setWidth(60)
        self.p_t.showGrid(x=True, y=True, alpha=0.2)
        self.p_t.setLabel('left', 'Voltage', units='V')
        self.p_t.setLabel('bottom', 'Time', units='s')
        
        # 曲线定义
        self.cur1 = self.p_t.plot(pen=pg.mkPen('#ffeb3b', width=1.5))
        self.cur2 = self.p_t.plot(pen=pg.mkPen('#00bcd4', width=1.5))
        
        # 触发线
        self.trig_line = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('#e0e0e0', style=Qt.DashLine, width=1))
        self.p_t.addItem(self.trig_line)

        # 2. 频域图
        self.p_f = self.win.addPlot(row=1, col=0, title="Frequency Domain")
        # 固定 Y 轴宽度以确保上下图表对齐
        self.p_f.getAxis('left').setWidth(60)
        self.p_f.showGrid(x=True, y=True, alpha=0.2)
        self.p_f.setLabel('left', 'Magnitude')
        self.p_f.setLabel('bottom', 'Frequency', units='Hz')
        
        self.f_cur1 = self.p_f.plot(pen=pg.mkPen('#ffeb3b', width=1.2))
        self.f_cur2 = self.p_f.plot(pen=pg.mkPen('#00bcd4', width=1.2))

        # 3. 测量光标 (默认隐藏)
        # X轴光标 (Time)
        self.v_line1 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('#ff00ff', width=1, style=Qt.DashDotLine))
        self.v_line2 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('#ff00ff', width=1, style=Qt.DashDotLine))
        self.v_line1.label = pg.InfLineLabel(self.v_line1, text="T1", position=0.95, rotateAxis=(1,0), anchor=(1, 1), color='#ff00ff')
        self.v_line2.label = pg.InfLineLabel(self.v_line2, text="T2", position=0.95, rotateAxis=(1,0), anchor=(1, 1), color='#ff00ff')
        
        # Y轴光标 (Voltage)
        self.h_line1 = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen('#00ff00', width=1, style=Qt.DashDotLine))
        self.h_line2 = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen('#00ff00', width=1, style=Qt.DashDotLine))
        self.h_line1.label = pg.InfLineLabel(self.h_line1, text="V1", position=0.95, anchor=(1, 1), color='#00ff00')
        self.h_line2.label = pg.InfLineLabel(self.h_line2, text="V2", position=0.95, anchor=(1, 1), color='#00ff00')

        # 光标移动回调
        self.v_line1.sigPositionChanged.connect(self.update_cursor_readout)
        self.v_line2.sigPositionChanged.connect(self.update_cursor_readout)
        self.h_line1.sigPositionChanged.connect(self.update_cursor_readout)
        self.h_line2.sigPositionChanged.connect(self.update_cursor_readout)

    def load_config(self):
        path = os.path.join(os.path.dirname(__file__), "calibration.json")
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                    # 兼容旧版本配置 (Flat structure -> Nested)
                    if "ch1_offset" in data:
                        self.cal_data = {"x1": data, "x50": {}, "x100": {}}
                    else:
                        self.cal_data = data
            except Exception as e:
                print(f"Load config error: {e}")
        
        # 确保当前档位有数据
        key = self.current_scale_key
        if key not in self.cal_data:
            self.cal_data[key] = {}
            
        params = self.cal_data[key]
        self.ch1_offset = params.get("ch1_offset", 0.0)
        self.ch1_gain = params.get("ch1_gain", 1.0)
        self.ch2_offset = params.get("ch2_offset", 0.0)
        self.ch2_gain = params.get("ch2_gain", 1.0)

    def save_config(self):
        # 保存当前档位参数到内存字典
        self.cal_data[self.current_scale_key] = {
            "ch1_offset": self.ch1_offset,
            "ch1_gain": self.ch1_gain,
            "ch2_offset": self.ch2_offset,
            "ch2_gain": self.ch2_gain
        }
        
        path = os.path.join(os.path.dirname(__file__), "calibration.json")
        try:
            with open(path, 'w') as f:
                json.dump(self.cal_data, f, indent=4)
            QMessageBox.information(self, "保存成功", f"当前档位 ({self.current_scale_key}) 的校准参数已保存。")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", str(e))

    def on_scale_changed(self, new_text):
        # 1. 保存旧档位参数 (只更新内存，不写文件，避免频繁IO)
        self.cal_data[self.current_scale_key] = {
            "ch1_offset": self.ch1_offset,
            "ch1_gain": self.ch1_gain,
            "ch2_offset": self.ch2_offset,
            "ch2_gain": self.ch2_gain
        }
        
        # 2. 切换到新档位
        self.current_scale_key = new_text
        
        # 3. 加载新档位参数
        if new_text not in self.cal_data:
            self.cal_data[new_text] = {}
        
        params = self.cal_data[new_text]
        self.ch1_offset = params.get("ch1_offset", 0.0)
        self.ch1_gain = params.get("ch1_gain", 1.0)
        self.ch2_offset = params.get("ch2_offset", 0.0)
        self.ch2_gain = params.get("ch2_gain", 1.0)
        
        # 4. 更新 UI (阻断信号防止递归调用处理)
        self.spin_ch1_off.blockSignals(True)
        self.spin_ch1_gain.blockSignals(True)
        self.spin_ch2_off.blockSignals(True)
        self.spin_ch2_gain.blockSignals(True)
        
        self.spin_ch1_off.setValue(self.ch1_offset)
        self.spin_ch1_gain.setValue(self.ch1_gain)
        self.spin_ch2_off.setValue(self.ch2_offset)
        self.spin_ch2_gain.setValue(self.ch2_gain)
        
        self.spin_ch1_off.blockSignals(False)
        self.spin_ch1_gain.blockSignals(False)
        self.spin_ch2_off.blockSignals(False)
        self.spin_ch2_gain.blockSignals(False)
        
        # 5. 更新触发电平和视图
        self.update_trigger_range()

    def refresh_ports(self):
        current = self.cb_port.currentText()
        self.cb_port.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_port.addItems(ports)
        if current in ports: self.cb_port.setCurrentText(current)

    def update_trigger_range(self):
        self.on_trigger_slider_move(self.sl_trig.value())
        self.reprocess_view()

    def on_trigger_slider_move(self, value):
        try:
            scale_str = self.cb_probe_scale.currentText()
            scale = float(scale_str[1:])
        except:
            scale = 1.0
            
        real_volts = (value / 100.0) * scale
        self.lbl_trig.setText(f"电平: {real_volts:.2f} V")
        self.trig_line.setValue(real_volts)

    def on_mode_changed(self, idx):
        self.double_buf_ch1.clear()
        self.double_buf_ch2.clear()
        self.time_mode_buf_ch1.clear()
        self.time_mode_buf_ch2.clear()
        self.rolling_ch1.clear()
        self.rolling_ch2.clear()
        self.container_time_mode.setVisible(idx == 3)
        if idx == 3:
            self.on_time_mode_refresh()

    def on_time_mode_refresh(self):
        """刷新按时间采集的数据缓冲区"""
        # 如果处于暂停状态（包括导入模式），则点击无效，避免清屏
        if self.paused:
            return

        self.time_mode_buf_ch1.clear()
        self.time_mode_buf_ch2.clear()
        
        # 视觉清空，表示重新开始
        self.cur1.clear()
        self.cur2.clear()
        self.f_cur1.clear()
        self.f_cur2.clear()
        if hasattr(self, 'env_cur'): self.env_cur.clear()
        if hasattr(self, 'inst_f_cur'): self.inst_f_cur.clear()
        
        self.last_view_ch1 = None
        self.last_view_ch2 = None
        
        # 更新状态提示
        if not self.paused:
             self.lbl_dash.setText("<br><br><center><span style='color:#4db6ac; font-size:11pt'>Waiting for data...</span></center>")

    def on_auto_range_clicked(self):
        self.chk_lock_x.setChecked(False)
        self.chk_lock_y.setChecked(False)
        self.p_t.enableAutoRange(x=True, y=True)
        self.p_f.enableAutoRange(x=True, y=True)

    def on_axis_lock_changed(self):
        lock_x = self.chk_lock_x.isChecked()
        lock_y = self.chk_lock_y.isChecked()
        x_range, y_range = self.p_t.viewRange()
        fx_range, fy_range = self.p_f.viewRange()
        if lock_x:
            self.p_t.setXRange(x_range[0], x_range[1], padding=0)
            self.p_f.setXRange(fx_range[0], fx_range[1], padding=0)
        if lock_y:
            self.p_t.setYRange(y_range[0], y_range[1], padding=0)
            self.p_f.setYRange(fy_range[0], fy_range[1], padding=0)
        self.p_t.enableAutoRange(x=not lock_x, y=not lock_y)
        self.p_f.enableAutoRange(x=not lock_x, y=not lock_y)

    def on_vpp_filter_toggled(self, checked):
        self.enable_vpp_filter = checked

    def on_smooth_toggled(self, checked):
        self.enable_smooth = checked
        self.reprocess_view()

    def on_cal_params_changed(self):
        self.ch1_offset = self.spin_ch1_off.value()
        self.ch1_gain = self.spin_ch1_gain.value()
        self.ch2_offset = self.spin_ch2_off.value()
        self.ch2_gain = self.spin_ch2_gain.value()
        self.reprocess_view()

    def on_auto_zero_clicked(self):
        self.btn_auto_zero.clearFocus() # 清除焦点避免弹窗时其他控件被选中
        if not self.is_connected:
            QMessageBox.warning(self, "自动归零", "请先连接设备并开启数据采集。")
            return
        
        self.cal_active_ch1 = self.chk_show_ch1.isChecked()
        self.cal_active_ch2 = self.chk_show_ch2.isChecked()
        
        if not (self.cal_active_ch1 or self.cal_active_ch2):
            QMessageBox.warning(self, "自动归零", "请至少开启一个要校准的通道 (CH1/CH2)。")
            return

        target_str = "、".join([c for c, a in zip(["CH1", "CH2"], [self.cal_active_ch1, self.cal_active_ch2]) if a])
        
        reply = QMessageBox.question(self, "自动归零", 
            f"准备开始自动归零校准 [{target_str}]：\n"
            f"1. 请确保所选探头已接地(GND)\n"
            f"2. 过程中请保持信号稳定\n\n"
            f"是否开始？",
            QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.cal_samples_ch1.clear()
            self.cal_samples_ch2.clear()
            self.is_calibrating = True
            self.btn_auto_zero.setText("校准中 (Sampling...)")
            self.btn_auto_zero.setEnabled(False)

    def on_auto_gain_clicked(self):
        self.btn_auto_gain.clearFocus() # 清除焦点避免弹窗时其他控件被选中
        if not self.is_connected:
            QMessageBox.warning(self, "自动增益", "请先连接设备并开启数据采集。")
            return
        
        self.cal_active_ch1 = self.chk_show_ch1.isChecked()
        self.cal_active_ch2 = self.chk_show_ch2.isChecked()
        
        if not (self.cal_active_ch1 or self.cal_active_ch2):
            QMessageBox.warning(self, "自动增益", "请至少开启一个要校准的通道 (CH1/CH2)。")
            return

        target_str = "、".join([c for c, a in zip(["CH1", "CH2"], [self.cal_active_ch1, self.cal_active_ch2]) if a])
        ref_v = self.spin_ref_vpp.value()
        
        reply = QMessageBox.question(self, "自动增益校准", 
            f"准备开始自动增益校准 [{target_str}]：\n"
            f"1. 请确保所选探头已连接到 {ref_v:.3f}V 参考电压源\n"
            f"2. 请确保已完成“自动归零”校准\n"
            f"3. 过程中请保持信号稳定\n\n"
            f"是否开始？",
            QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.cal_samples_ch1.clear()
            self.cal_samples_ch2.clear()
            self.is_calibrating_gain = True
            self.btn_auto_gain.setText("校准中 (Sampling...)")
            self.btn_auto_gain.setEnabled(False)
        
    def on_cursors_x_toggled(self, checked):
        if checked:
            self.p_t.addItem(self.v_line1)
            self.p_t.addItem(self.v_line2)
            # 初始化光标位置到视图中间
            xr = self.p_t.viewRange()[0]
            mid = (xr[0] + xr[1]) / 2
            width = (xr[1] - xr[0]) / 4
            self.v_line1.setValue(mid - width)
            self.v_line2.setValue(mid + width)
            self.update_cursor_readout()
        else:
            self.p_t.removeItem(self.v_line1)
            self.p_t.removeItem(self.v_line2)
            self.reprocess_view()

    def on_cursors_y_toggled(self, checked):
        if checked:
            self.p_t.addItem(self.h_line1)
            self.p_t.addItem(self.h_line2)
            # 初始化光标位置到视图中间
            yr = self.p_t.viewRange()[1]
            mid = (yr[0] + yr[1]) / 2
            height = (yr[1] - yr[0]) / 4
            self.h_line1.setValue(mid - height)
            self.h_line2.setValue(mid + height)
            self.update_cursor_readout()
        else:
            self.p_t.removeItem(self.h_line1)
            self.p_t.removeItem(self.h_line2)
            self.reprocess_view()

    def update_cursor_readout(self):
        # 仅在需要时刷新文字，避免过度刷新。实际上 Dashboard 的刷新主要在 process_and_plot
        # 这里仅在暂停时或者需要即时反馈时调用
        if self.paused and (self.chk_cursors_x.isChecked() or self.chk_cursors_y.isChecked()):
            self.reprocess_view()

    def on_channel_toggled(self):
        self.show_ch1 = self.chk_show_ch1.isChecked()
        self.show_ch2 = self.chk_show_ch2.isChecked()
        if not self.show_ch1:
            self.cur1.clear()
            self.f_cur1.clear()
        if not self.show_ch2:
            self.cur2.clear()
            self.f_cur2.clear()
        self.reprocess_view()

    def reprocess_view(self, _=None):
        if self.paused and self.last_view_ch1 is not None:
            # 确保即使在暂停状态下应用了新 Gain/Offset，重新绘图也能反映出来
            self.process_and_plot(self.last_view_ch1, self.last_view_ch2, self.last_fs)

    def on_pause_toggled(self, checked):
        self.paused = checked
        if checked:
            self.btn_pause.setText("继续 (RESUME)")
            self.btn_pause.setStyleSheet("background-color: #c62828; font-weight: bold;")
        else:
            self.btn_pause.setText("暂停 (PAUSE)")
            self.btn_pause.setStyleSheet("")

    def toggle_serial(self):
        if not self.is_connected:
            port = self.cb_port.currentText()
            if not port: return
            self.worker = DataWorker(port)
            self.worker.packet_ready.connect(self.handle_packet)
            self.worker.start()
            self.btn_conn.setText("断开连接")
            self.btn_conn.setStyleSheet("font-weight: bold; background-color: #c62828;")
            self.cb_port.setEnabled(False)
            self.btn_refresh.setEnabled(False)
            self.is_connected = True
            self.lbl_dash.setText("<span style='color:#4db6ac'>Connected. Waiting for data...</span>")
        else:
            self.worker.stop()
            self.btn_conn.setText("打开串口")
            self.btn_conn.setStyleSheet("font-weight: bold; background-color: #2e7d32;")
            self.cb_port.setEnabled(True)
            self.btn_refresh.setEnabled(True)
            self.is_connected = False
            self.lbl_dash.setText("<span style='color:#777'>Offline</span>")

    @Slot(np.ndarray, np.ndarray, int)
    def handle_packet(self, ch1_raw, ch2_raw, tb_idx):
        fs = FS_TABLE[tb_idx] if tb_idx < len(FS_TABLE) else 1000

        if self.paused:
            return
        
        # 1x基准电压: 0~3.3V -> 1.7V中值 -> ±3V范围 (按公式)
        # v1 = 5.0 - 2.0 * (adc / 4095 * 3.3)
        v1_base = (5.0 - 2.0 * (ch1_raw / 4095.0 * 3.3))
        v2_base = (5.0 - 2.0 * (ch2_raw / 4095.0 * 3.3))

        # 应用校准系数
        v1_base = v1_base * self.ch1_gain + self.ch1_offset
        v2_base = v2_base * self.ch2_gain + self.ch2_offset

        # 自动校准逻辑 (Auto Zero)
        if self.is_calibrating:
            vpp1 = np.ptp(v1_base)
            vpp2 = np.ptp(v2_base)
            
            # 过滤异常包 (校准时要求信号稳定，且不触发 Vpp 过滤)
            is_valid = True
            if self.enable_vpp_filter:
                try:
                    is_atten = float(self.cb_probe_scale.currentText()[1:]) >= 50
                except: is_atten = False
                v_range = self.atten_volt_range if is_atten else self.normal_volt_range
                max_vpp = (v_range[1] - v_range[0]) * 1.2
                if vpp1 > max_vpp or vpp2 > max_vpp:
                    is_valid = False
            
            if is_valid:
                if self.cal_active_ch1: self.cal_samples_ch1.append(np.mean(v1_base))
                if self.cal_active_ch2: self.cal_samples_ch2.append(np.mean(v2_base))
                
                # 以样本数最多的为准（通常同步）
                count = max(len(self.cal_samples_ch1), len(self.cal_samples_ch2))
                if count >= 20: 
                    diff1, diff2 = 0, 0
                    msg_details = ""
                    
                    if self.cal_active_ch1 and self.cal_samples_ch1:
                        diff1 = np.mean(self.cal_samples_ch1)
                        self.ch1_offset -= diff1
                        v1_base -= diff1
                        msg_details += f"CH1 修正: {-diff1*1000:.1f} mV\n"
                        
                    if self.cal_active_ch2 and self.cal_samples_ch2:
                        diff2 = np.mean(self.cal_samples_ch2)
                        self.ch2_offset -= diff2
                        v2_base -= diff2
                        msg_details += f"CH2 修正: {-diff2*1000:.1f} mV\n"
                    
                    self.is_calibrating = False
                    
                    # 更新 UI
                    self.spin_ch1_off.blockSignals(True)
                    self.spin_ch2_off.blockSignals(True)
                    self.spin_ch1_off.setValue(self.ch1_offset)
                    self.spin_ch2_off.setValue(self.ch2_offset)
                    self.spin_ch1_off.blockSignals(False)
                    self.spin_ch2_off.blockSignals(False)

                    self.btn_auto_zero.setText("自动归零 (Auto Zero)")
                    self.btn_auto_zero.setEnabled(True)
                    self.save_config()
                    
                    msg = (f"已完成 {self.current_scale_key} 档位归零校准\n\n"
                           f"{msg_details}"
                           f"当前 Offset: C1={self.ch1_offset:.3f}V, C2={self.ch2_offset:.3f}V")
                    QTimer.singleShot(50, lambda: QMessageBox.information(self, "自动归零完成", msg))

        # 自动增益校准逻辑 (Auto Gain)
        if self.is_calibrating_gain:
            if self.cal_active_ch1: self.cal_samples_ch1.append(np.mean(v1_base))
            if self.cal_active_ch2: self.cal_samples_ch2.append(np.mean(v2_base))
            
            count = max(len(self.cal_samples_ch1), len(self.cal_samples_ch2))
            if count >= 20:
                target_v = self.spin_ref_vpp.value()
                try:
                    scale = float(self.cb_probe_scale.currentText()[1:])
                except: scale = 1.0
                
                msg_details = f"参考电压: {target_v:.3f} V\n"
                
                if self.cal_active_ch1 and self.cal_samples_ch1:
                    measured1 = np.mean(self.cal_samples_ch1)
                    if abs(measured1) > 0.01:
                        ratio1 = target_v / (measured1 * scale)
                        self.ch1_gain *= ratio1
                        msg_details += f"CH1 实测: {measured1*scale:.3f}V -> 新 Gain: {self.ch1_gain:.3f}\n"
                
                if self.cal_active_ch2 and self.cal_samples_ch2:
                    measured2 = np.mean(self.cal_samples_ch2)
                    if abs(measured2) > 0.01:
                        ratio2 = target_v / (measured2 * scale)
                        self.ch2_gain *= ratio2
                        msg_details += f"CH2 实测: {measured2*scale:.3f}V -> 新 Gain: {self.ch2_gain:.3f}\n"
                
                self.is_calibrating_gain = False
                
                # 更新 UI
                self.spin_ch1_gain.blockSignals(True)
                self.spin_ch2_gain.blockSignals(True)
                self.spin_ch1_gain.setValue(self.ch1_gain)
                self.spin_ch2_gain.setValue(self.ch2_gain)
                self.spin_ch1_gain.blockSignals(False)
                self.spin_ch2_gain.blockSignals(False)
                
                self.btn_auto_gain.setText("自动增益 (Auto Gain)")
                self.btn_auto_gain.setEnabled(True)
                self.save_config()
                
                msg = f"已完成 {self.current_scale_key} 档位增益校准\n\n{msg_details}"
                QTimer.singleShot(50, lambda: QMessageBox.information(self, "增益校准完成", msg))

        mode = self.cb_display_mode.currentIndex()
        f1, f2 = None, None

        if mode == 0: 
            f1, f2 = v1_base, v2_base
        elif mode == 1:
            self.double_buf_ch1.extend(v1_base)
            self.double_buf_ch2.extend(v2_base)
            if len(self.double_buf_ch1) >= 2000:
                f1 = np.array(self.double_buf_ch1[:2000])
                f2 = np.array(self.double_buf_ch2[:2000])
                self.double_buf_ch1.clear(); self.double_buf_ch2.clear()
            else: return
        elif mode == 2:
            self.rolling_ch1.extend(v1_base); self.rolling_ch2.extend(v2_base)
            f1, f2 = np.array(self.rolling_ch1), np.array(self.rolling_ch2)
        elif mode == 3:
            # 异常包过滤 (Vpp Check)
            if self.enable_vpp_filter:
                try:
                    is_atten = float(self.cb_probe_scale.currentText()[1:]) >= 50
                except: is_atten = False
                v_range = self.atten_volt_range if is_atten else self.normal_volt_range
                max_vpp = (v_range[1] - v_range[0]) * 1.2
                if np.ptp(v1_base) > max_vpp or np.ptp(v2_base) > max_vpp:
                    return

            self.time_mode_buf_ch1.extend(v1_base)
            self.time_mode_buf_ch2.extend(v2_base)
            
            target_pts = int(fs * self.spin_time_duration.value())
            if len(self.time_mode_buf_ch1) >= target_pts:
                f1 = np.array(self.time_mode_buf_ch1[:target_pts])
                f2 = np.array(self.time_mode_buf_ch2[:target_pts])
                self.time_mode_buf_ch1.clear()
                self.time_mode_buf_ch2.clear()
            else:
                return

        if f1 is None: return

        trig_v_base = (self.sl_trig.value() / 100.0)
        if self.chk_trig.isChecked() and mode < 2:
            indices = np.where((f1[:-1] < trig_v_base) & (f1[1:] >= trig_v_base))[0]
            if len(indices) > 0:
                s = indices[0]
                f1 = f1[s:s+1000]; f2 = f2[s:s+1000]

        self.process_and_plot(f1, f2, fs)

    def process_and_plot(self, v1_base, v2_base, fs):
        self.last_view_ch1 = v1_base
        self.last_view_ch2 = v2_base
        self.last_fs = fs

        try:
            scale_str = self.cb_probe_scale.currentText()
            scale = float(scale_str[1:])
        except:
            scale = 1.0
            
        is_atten = scale >= 50
        v1_p = v1_base * scale
        v2_p = v2_base * scale

        algo = SignalProcessor.freq_acf if self.rad_acf.isChecked() else SignalProcessor.freq_schmitt
        ft1, ft2 = algo(v1_p, fs), algo(v2_p, fs)
        vpp1, vpp2 = SignalProcessor.get_vpp(v1_p), SignalProcessor.get_vpp(v2_p)
        
        if self.enable_vpp_filter and not self.paused:
            v_range = self.atten_volt_range if is_atten else self.normal_volt_range
            max_vpp = (v_range[1] - v_range[0]) * 1.2
            if vpp1 > max_vpp or vpp2 > max_vpp:
                return
            if self.last_vpp1 is not None and vpp1 > self.last_vpp1 * 4 and vpp1 > max_vpp * 0.25:
                return
            if self.last_vpp2 is not None and vpp2 > self.last_vpp2 * 4 and vpp2 > max_vpp * 0.25:
                return

        def get_fft(d, fs):
            n = len(d)
            yf = rfft(d - np.mean(d))
            xf = rfftfreq(n, 1/fs)
            m = np.abs(yf)
            return xf, m, xf[np.argmax(m)] if len(m)>0 else 0

        xf1, mf1, ff1 = get_fft(v1_p, fs)
        xf2, mf2, ff2 = get_fft(v2_p, fs)

        freq_hint = 0.0
        if ft1 > 0 and ft2 > 0: freq_hint = 0.5 * (ft1 + ft2)
        elif ft1 > 0: freq_hint = ft1
        elif ft2 > 0: freq_hint = ft2
        if freq_hint == 0.0:
            if ff1 > 0: freq_hint = ff1
            elif ff2 > 0: freq_hint = ff2

        phase = None
        if self.show_ch1 and self.show_ch2:
            if self.rad_phase_fft.isChecked():
                phase = SignalProcessor.phase_fft(v1_p, v2_p, fs, freq_hint if freq_hint > 0 else None)
            else:
                phase = SignalProcessor.phase_delay_xcorr(v1_p, v2_p, fs, freq_hint if freq_hint > 0 else None)

        # 视觉平滑
        f1_disp = v1_p
        f2_disp = v2_p
        if self.enable_smooth:
            def smooth(d):
                if len(d) < 3: return d
                return np.convolve(d, [1/3, 1/3, 1/3], mode='same')
            f1_disp = smooth(f1_disp)
            f2_disp = smooth(f2_disp)

        self.p_f.setLogMode(y=self.rad_fft_log.isChecked())
        
        # X轴时间向量
        t_vec = np.arange(len(f1_disp)) / fs
        
        if self.show_ch1:
            self.cur1.setData(t_vec, f1_disp)
            self.f_cur1.setData(xf1, mf1)
        if self.show_ch2:
            self.cur2.setData(t_vec, f2_disp)
            self.f_cur2.setData(xf2, mf2)

        # 构建 Dashboard
        span_s = len(f1_disp) / fs if fs > 0 else 0
        if span_s >= 1.0: span_str = f"{span_s:.2f}s"
        elif span_s >= 1e-3: span_str = f"{span_s*1000:.2f}ms"
        else: span_str = f"{span_s*1e6:.2f}us"

        style_y = "color:#ffeb3b; font-weight:bold"
        style_c = "color:#00bcd4; font-weight:bold"
        style_w = "color:#e0e0e0"
        style_g = "color:#4db6ac"
        
        # 基础表格
        html = f"""
        <table width="100%" cellspacing="0" cellpadding="1" style="font-size:9pt; line-height:120%">
        <tr>
            <td colspan="4" style="{style_g}; border-bottom:1px solid #444; padding-bottom:2px">
                FS: {int(fs/1000)}kS/s | Span: {span_str}
            </td>
        </tr>
        <tr>
            <td style="{style_y}; width:30px">CH1</td>
            <td style="{style_w}">Vpp:{vpp1:.1f}V</td>
            <td style="{style_w}">F:{ft1:.0f}Hz</td>
            <td style="{style_w}">FFT:{ff1:.0f}Hz</td>
        </tr>
        <tr>
            <td style="{style_c}">CH2</td>
            <td style="{style_w}">Vpp:{vpp2:.1f}V</td>
            <td style="{style_w}">F:{ft2:.0f}Hz</td>
            <td style="{style_w}">FFT:{ff2:.0f}Hz</td>
        </tr>
        <tr>
            <td colspan="4" style="{style_w}; padding-bottom:2px">
                Phase(2-1): {f'{phase:.1f}°' if phase is not None else '--'} 
            </td>
        </tr>
        """
        
        # 光标部分
        has_cursor = False
        
        # X轴光标
        if self.chk_cursors_x.isChecked():
            if not has_cursor:
                html += f"<tr><td colspan='4' style='border-top:1px solid #444; height:4px'></td></tr>"
                has_cursor = True
                
            t1 = self.v_line1.value()
            t2 = self.v_line2.value()
            dt = abs(t2 - t1)
            freq_dt = 1.0/dt if dt > 0 else 0
            
            html += f"""
            <tr>
                <td colspan="4" style="color:#ff00ff; white-space:nowrap; font-size:10pt">
                    <b>[X]</b> T1:{t1*1000:.1f}ms | T2:{t2*1000:.1f}ms | <b>ΔT:{dt*1000:.1f}ms</b> | f:{freq_dt:.1f}Hz
                </td>
            </tr>
            """

        # Y轴光标
        if self.chk_cursors_y.isChecked():
            if not has_cursor:
                html += f"<tr><td colspan='4' style='border-top:1px solid #444; height:4px'></td></tr>"
                has_cursor = True
                
            v1_c = self.h_line1.value()
            v2_c = self.h_line2.value()
            dv = abs(v2_c - v1_c)
            
            html += f"""
            <tr>
                <td colspan="4" style="color:#00ff00; white-space:nowrap; font-size:10pt">
                    <b>[Y]</b> V1:{v1_c:.2f}V | V2:{v2_c:.2f}V | <b>ΔV:{dv:.2f}V</b>
                </td>
            </tr>
            """
        
        html += "</table>"
        self.lbl_dash.setText(html)

        self.last_vpp1 = vpp1
        self.last_vpp2 = vpp2
        self.p_t.update(); self.p_f.update()

    def export_csv(self):
        if self.last_view_ch1 is None or len(self.last_view_ch1) == 0:
            QMessageBox.warning(self, "导出失败", "当前无有效数据可导出。")
            return

        path, _ = QFileDialog.getSaveFileName(self, "导出 CSV", "", "CSV Files (*.csv)")
        if not path:
            return

        try:
            scale_str = self.cb_probe_scale.currentText()
            try: scale = float(scale_str[1:])
            except: scale = 1.0

            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 写入元数据
                writer.writerow([f"# AuraScope Data Export"])
                writer.writerow([f"# Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"])
                writer.writerow([f"# SampleRate: {self.last_fs}"])
                writer.writerow([f"# Attenuation: {scale_str}"])
                writer.writerow(["Time_s", "CH1_V", "CH2_V"])
                
                # 写入数据 (保存缩放后的真实电压值)
                t_step = 1.0 / self.last_fs if self.last_fs > 0 else 0
                rows = []
                for i in range(len(self.last_view_ch1)):
                    t = i * t_step
                    v1 = self.last_view_ch1[i] * scale
                    v2 = (self.last_view_ch2[i] if i < len(self.last_view_ch2) else 0) * scale
                    rows.append([f"{t:.6f}", f"{v1:.4f}", f"{v2:.4f}"])
                writer.writerows(rows)
            QMessageBox.information(self, "导出成功", f"数据已保存至: {path}")
        except Exception as e:
            QMessageBox.critical(self, "导出错误", str(e))

    def ensure_arrow_icon(self):
        """确保目录下存在 arrow.png 图标，不存在则绘制"""
        arrow_path = os.path.join(os.path.dirname(__file__), "arrow.png")
        if not os.path.exists(arrow_path):
            try:
                pix = QPixmap(16, 16)
                pix.fill(Qt.transparent)
                p = QPainter(pix)
                p.setRenderHint(QPainter.Antialiasing)
                p.setBrush(QColor(224, 224, 224))
                p.setPen(Qt.NoPen)
                # 绘制倒三角
                triangle = [QPointF(4, 6), QPointF(12, 6), QPointF(8, 11)]
                p.drawPolygon(triangle)
                p.end()
                pix.save(arrow_path)
            except Exception as e:
                print(f"Failed to generate arrow icon: {e}")

    def import_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入 CSV", "", "CSV Files (*.csv)")
        if not path:
            return

        try:
            # 进入暂停/导入模式
            if not self.paused:
                self.btn_pause.click()
            
            # 读取文件
            with open(path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                data_rows = []
                fs = 200000 
                file_scale = 1.0
                
                for row in reader:
                    if not row: continue
                    if row[0].startswith('#'):
                        line = row[0].lower()
                        if "samplerate" in line:
                            try: fs = float(line.split(':')[-1].strip())
                            except: pass
                        if "attenuation" in line:
                            if "50x" in line: file_scale = 50.0
                    elif row[0].lower().startswith('time'):
                        continue 
                    else:
                        try:
                            # 数据行：Time, CH1, CH2
                            data_rows.append([float(row[1]), float(row[2])])
                        except: pass
                
                if not data_rows:
                    raise ValueError("未找到有效数据行")

                data_np = np.array(data_rows)
                # 反归一化：将文件中的缩放值还原为基准 1x 电压
                v1_base = data_np[:, 0] / file_scale
                v2_base = data_np[:, 1] / file_scale
                
                # 自动同步 UI 状态
                self.cb_probe_scale.blockSignals(True)
                if file_scale == 50.0: self.cb_probe_scale.setCurrentText("x50")
                elif file_scale == 100.0: self.cb_probe_scale.setCurrentText("x100")
                else: self.cb_probe_scale.setCurrentText("x1")
                self.cb_probe_scale.blockSignals(False)
                
                # 更新触发线等 UI 元素 (手动触发一次不带 reprocess_view 的 UI 更新)
                self.on_trigger_slider_move(self.sl_trig.value())
                
                self.process_and_plot(v1_base, v2_base, fs)
                self.setWindowTitle(f"AuraScope Viewer - [Loaded: {path}]")
                QMessageBox.information(self, "导入成功", 
                    f"已加载 {len(v1_base)} 点数据\\n采样率: {fs} Hz\\n探头倍率: x{int(file_scale)}")

        except Exception as e:
            QMessageBox.critical(self, "导入错误", str(e))

    def show_help(self):
        help_text = """
        <h3 style='color:#4db6ac'>AuraScope 功能说明</h3>
        <p><b>1. 连接设置</b>: 选择串口并点击“打开串口”以连接设备。R 按钮用于刷新串口列表。</p>
        <p><b>2. 采集模式</b>: 
           <ul>
           <li><b>单包/双包</b>: 适合触发观测，波形稳定。</li>
           <li><b>滚动模式</b>: 适合观察低频或变化信号。</li>
           <li><b>50x 探头衰减</b>: 使用高压探头时勾选，量程扩大至 ±300V。</li>
           </ul>
        </p>
        <p><b>3. 软件触发</b>: 启用后，波形将在设定的电平处稳定显示（仅限单/双包模式）。</p>
        <p><b>4. 通道管理</b>: 开启/关闭通道显示。勾选光标可进行时间和电压测量。</p>
        <p><b>5. 算法与 FFT</b>:
           <ul>
           <li><b>频率检测</b>: 施密特触发器适合干净波形，自相关适合噪声波形。</li>
           <li><b>FFT</b>: 切换线性或对数坐标显示频域。</li>
           <li><b>相位</b>: XCorr 适合同频信号，FFT 适合复杂信号。</li>
           </ul>
        </p>
        <p><b>6. 视图优化</b>:
           <ul>
           <li><b>Auto Range</b>: 自动调整坐标轴范围。</li>
           <li><b>锁轴</b>: 防止坐标轴自动缩放。</li>
           <li><b>Vpp 过滤</b>: 滤除异常的尖峰脉冲。</li>
           <li><b>波形平滑</b>: 简单的 3 点平均平滑。</li>
           </ul>
        </p>
        <p><b>7. 操作</b>: 暂停波形刷新，导出/导入 CSV 数据以供分析。</p>
        """
        QMessageBox.about(self, "帮助与说明", help_text)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # 调色板微调
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(30, 30, 30))
    palette.setColor(QPalette.WindowText, QColor(224, 224, 224))
    app.setPalette(palette)
    
    win = AuraScope()
    win.show()
    sys.exit(app.exec())
