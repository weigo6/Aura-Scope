import sys
import os
import csv
import datetime
import numpy as np
import serial
import serial.tools.list_ports
import struct
from collections import deque

# 核心 UI 库导入
from PySide6.QtCore import Qt, QThread, Signal, Slot, QPointF
from PySide6.QtGui import QColor, QPalette, QIcon, QPixmap, QPainter
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QComboBox, QPushButton, QLabel, 
                             QCheckBox, QRadioButton, QGroupBox, QSlider, QButtonGroup,
                             QFileDialog, QMessageBox, QScrollArea, QFrame,
                             QSplitter, QDoubleSpinBox)
import pyqtgraph as pg
from scipy.fft import rfft, rfftfreq
from scipy.signal import find_peaks, hilbert

# --- 采样率对应表 ---
FS_TABLE = [500000, 200000, 100000, 50000, 20000, 10000, 5000, 2000, 1000]

# --- ZPW-2000A 标准参数表 ---
ZPW_STD_FC = {
    1700: "下行",
    2000: "上行",
    2300: "下行",
    2600: "上行"
}

ZPW_STD_FM = {
    10.3: ("L5", "五级绿灯(最高速度)"),
    11.4: ("L4", "四级绿灯"),
    12.5: ("L3", "三级绿灯"),
    13.6: ("L2", "二级绿灯"),
    14.7: ("L", "一级绿灯(正常运行)"),
    15.8: ("LU", "绿黄灯(预告减速)"),
    16.9: ("U2", "黄灯+2(限速)"),
    18.0: ("U", "黄灯(准备停车)"),
    19.1: ("UU", "双黄灯(侧线进站)"),
    20.2: ("UUS", "双黄闪(高速侧线)"),
    21.3: ("HU", "红黄灯(立即停车)"),
    22.4: ("HB", "红白灯(引导信号)"),
    23.5: ("K1", "预留"),
    24.6: ("K2", "预留"),
    25.7: ("K3", "预留"),
    26.8: ("H", "红灯(绝对停车)"),
    27.9: ("JC", "检测码"),
    29.0: ("SP", "特殊用途")
}

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

class SignalSimulator:
    """
    轨道信号模拟器 (内置版)
    """

    @staticmethod
    def generate_zpw2000a(fc=1700, fm=29, df=11, duration=1.0, fs=200000, amplitude=1.0):
        """
        生成 ZPW-2000A 移频信号 (纯净版)
        """
        t = np.linspace(0, duration, int(fs * duration), endpoint=False)
        
        # 1. 生成调制方波: sin(2*pi*fm*t) > 0 时为 +1, < 0 时为 -1
        mod_signal = np.sign(np.sin(2 * np.pi * fm * t))
        
        # 2. 计算瞬时频率
        inst_freq = fc + mod_signal * df
        
        # 3. 计算相位 (频率的积分)
        phase = 2 * np.pi * np.cumsum(inst_freq) / fs
        
        # 4. 生成调频信号
        signal = amplitude * np.cos(phase)
        
        return t, signal, mod_signal

    @staticmethod
    def generate_25hz_phase(freq=25.0, phase_diff=90.0, duration=1.0, fs=200000, amp1=110.0, amp2=110.0):
        """
        生成 25Hz 相敏轨道电路信号 (纯净版)
        """
        t = np.linspace(0, duration, int(fs * duration), endpoint=False)
        
        # CH1: 参考信号 (局部电压)
        sig1 = amp1 * np.sin(2 * np.pi * freq * t)
        
        # CH2: 轨道信号 (带相位差)
        # phase_diff is in degrees
        phi = np.deg2rad(phase_diff)
        sig2 = amp2 * np.sin(2 * np.pi * freq * t + phi)
        
        return t, sig1, sig2

class SignalProcessor:
    @staticmethod
    def get_vpp(data):
        return np.ptp(data) if len(data) > 0 else 0

    @staticmethod
    def find_nearest(value, std_map, tolerance=None):
        """
        在标准映射表中查找最接近的值
        :param value: 测量值
        :param std_map: 标准值字典 {std_val: label}
        :param tolerance: 允许的最大偏差 (绝对值)，None表示无限制
        :return: (std_val, label, diff) or (None, None, diff)
        """
        if value <= 0:
            return None, "未知", 0
            
        std_keys = np.array(list(std_map.keys()))
        idx = (np.abs(std_keys - value)).argmin()
        nearest = std_keys[idx]
        diff = abs(value - nearest)
        
        if tolerance is not None and diff > tolerance:
            return None, "未知", diff
            
        return nearest, std_map[nearest], diff

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

    @staticmethod
    def decode_zpw2000a(signal, fs):
        """
        ZPW-2000A 解码算法
        :param signal: 输入信号数组
        :param fs: 采样率
        :return: (detected_fc, detected_fm, envelope, inst_freq_seq)
        """
        if len(signal) == 0:
            return 0, 0, np.array([]), np.array([])

        # a. 提取载频 (通过FFT)
        fft_vals = np.abs(np.fft.rfft(signal))
        freqs = np.fft.rfftfreq(len(signal), 1/fs)
        if len(fft_vals) > 0:
            detected_fc = freqs[np.argmax(fft_vals)]
        else:
            detected_fc = 0
        
        # b. 提取包络线 (希尔伯特变换)
        try:
            analytic_signal = hilbert(signal)
            envelope = np.abs(analytic_signal)
            
            # c. 频率解调 (瞬时频率计算)
            # 计算瞬时相位
            instantaneous_phase = np.unwrap(np.angle(analytic_signal))
            # 频率是相位的导数: f = (1/2pi) * d(phi)/dt
            inst_freq_seq = np.diff(instantaneous_phase) / (2.0 * np.pi) * fs
            
            # d. 提取低频调制数据
            if len(inst_freq_seq) > 10:
                # 对解调出的瞬时频率序列进行FFT
                fm_raw = inst_freq_seq - np.mean(inst_freq_seq)
                
                # 加窗减少频谱泄露
                window = np.hanning(len(fm_raw))
                fm_fft = np.abs(np.fft.rfft(fm_raw * window))
                fm_freqs = np.fft.rfftfreq(len(fm_raw), 1/fs)
                
                # 锁定低频范围 (5Hz - 40Hz)
                mask = (fm_freqs > 5) & (fm_freqs < 40)
                
                if np.any(mask):
                    # 使用频谱重心法提高精度
                    idx_masked = np.argmax(fm_fft[mask])
                    full_indices = np.where(mask)[0]
                    peak_idx = full_indices[idx_masked]
                    
                    if 0 < peak_idx < len(fm_fft) - 1:
                        y = fm_fft[peak_idx-1 : peak_idx+2]
                        x = fm_freqs[peak_idx-1 : peak_idx+2]
                        if np.sum(y) != 0:
                            detected_fm = np.sum(x * y) / np.sum(y)
                        else:
                            detected_fm = fm_freqs[peak_idx]
                    else:
                        detected_fm = fm_freqs[peak_idx]
                else:
                    detected_fm = 0
            else:
                detected_fm = 0
                
        except Exception:
            envelope = np.zeros_like(signal)
            inst_freq_seq = np.zeros_like(signal)
            detected_fm = 0

        return detected_fc, detected_fm, envelope, inst_freq_seq

    @staticmethod
    def detect_power_interference(data, fs):
        """
        检测 50Hz 工频干扰
        :return: (volts_50hz, ratio_percent, harmonics_str)
        """
        if len(data) < 10: return 0.0, 0.0, ""
        
        n = len(data)
        # 去直流
        yf = rfft(data - np.mean(data))
        xf = rfftfreq(n, 1/fs)
        m = np.abs(yf)
        
        if len(m) == 0 or np.max(m) == 0:
            return 0.0, 0.0, ""

        def get_mag_at(target_freq):
            # 寻找最接近的频率分量
            if len(xf) == 0: return 0.0
            idx = np.argmin(np.abs(xf - target_freq))
            # 如果频率分辨率不够(偏差太大)，则认为无法检测
            if abs(xf[idx] - target_freq) < 5.0: # 允许 5Hz 偏差
                return m[idx]
            return 0.0

        mag_50 = get_mag_at(50.0)
        mag_100 = get_mag_at(100.0) # 2nd harmonic
        mag_150 = get_mag_at(150.0) # 3rd harmonic
        
        # Calculate Volts (Approx Amplitude)
        # FFT mag = A * N / 2  => A = mag * 2 / N
        volts_50 = mag_50 * 2 / n
        
        # Calculate Ratio (Relative to Max Peak)
        max_mag = np.max(m)
        ratio = (mag_50 / max_mag) * 100.0 if max_mag > 0 else 0
        
        harmonics = []
        if mag_100 > max_mag * 0.1: harmonics.append("100Hz")
        if mag_150 > max_mag * 0.1: harmonics.append("150Hz")
        
        h_str = ",".join(harmonics) if harmonics else "None"
        
        return volts_50, ratio, h_str

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

        self.chk_atten = QCheckBox("50x 探头衰减 (±300V)")
        self.chk_atten.toggled.connect(self.update_trigger_range)
        l_mode.addWidget(self.cb_display_mode)
        l_mode.addWidget(self.container_time_mode)
        l_mode.addWidget(self.chk_atten)
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
        self.rad_schmitt = QRadioButton("施密特")
        self.rad_acf = QRadioButton("自相关")
        self.rad_schmitt.setChecked(True)
        self.algo_group = QButtonGroup(self)
        self.algo_group.addButton(self.rad_schmitt)
        self.algo_group.addButton(self.rad_acf)
        self.algo_group.buttonClicked.connect(self.reprocess_view)
        l_freq.addWidget(self.rad_schmitt)
        l_freq.addWidget(self.rad_acf)
        l_algo.addLayout(l_freq)

        l_fft = QHBoxLayout()
        self.rad_fft_lin = QRadioButton("线性")
        self.rad_fft_log = QRadioButton("对数(dB)")
        self.rad_fft_lin.setChecked(True)
        self.fft_group = QButtonGroup(self)
        self.fft_group.addButton(self.rad_fft_lin)
        self.fft_group.addButton(self.rad_fft_log)
        self.fft_group.buttonClicked.connect(self.reprocess_view)
        l_fft.addWidget(self.rad_fft_lin)
        l_fft.addWidget(self.rad_fft_log)
        l_algo.addLayout(l_fft)

        l_phase = QHBoxLayout()
        self.rad_phase_xcorr = QRadioButton("XCorr相位")
        self.rad_phase_fft = QRadioButton("FFT相位")
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
        self.chk_lock_x = QCheckBox("锁X轴")
        self.chk_lock_y = QCheckBox("锁Y轴")
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

        # 8. 轨道信号与仿真
        gp_track = QGroupBox("8. 轨道信号与仿真")
        l_track = QVBoxLayout()
        
        self.chk_zpw_mode = QCheckBox("启用 ZPW-2000A 分析")
        self.chk_zpw_mode.toggled.connect(self.on_zpw_mode_toggled)
        l_track.addWidget(self.chk_zpw_mode)
        
        # 仿真信号选择
        h_sim = QHBoxLayout()
        self.cb_sim_type = QComboBox()
        self.cb_sim_type.addItems(["ZPW-2000A 移频信号", "25Hz 相敏轨道电路"])
        h_sim.addWidget(self.cb_sim_type)
        
        self.btn_load_sim = QPushButton("加载仿真")
        self.btn_load_sim.clicked.connect(self.load_simulation)
        self.btn_load_sim.setStyleSheet("background-color: #512da8; font-weight: bold;")
        h_sim.addWidget(self.btn_load_sim)
        
        l_track.addLayout(h_sim)

        # 50Hz 工频干扰检测
        self.btn_detect_50hz = QPushButton("50Hz 工频干扰检测")
        self.btn_detect_50hz.setCheckable(True)
        self.btn_detect_50hz.clicked.connect(self.on_detect_50hz_toggled)
        self.btn_detect_50hz.setStyleSheet("background-color: #00796b;") 
        l_track.addWidget(self.btn_detect_50hz)
        
        gp_track.setLayout(l_track)
        scroll_layout.addWidget(gp_track)

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
        self.env_cur = self.p_t.plot(pen=pg.mkPen('#ff5252', width=2)) # 包络线
        
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

        # 3. 频率解调曲线 (默认隐藏，仅 ZPW 模式显示)
        self.p_demod = self.win.addPlot(row=2, col=0, title="Demodulated Frequency")
        self.p_demod.getAxis('left').setWidth(60)
        self.p_demod.showGrid(x=True, y=True, alpha=0.2)
        self.p_demod.setLabel('left', 'Freq', units='Hz')
        self.p_demod.setLabel('bottom', 'Time', units='s')
        self.p_demod.setVisible(False)
        self.inst_f_cur = self.p_demod.plot(pen=pg.mkPen('#448aff', width=1.5))

        # 4. 测量光标 (默认隐藏)
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
        scale = 50 if self.chk_atten.isChecked() else 1
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
        self.env_cur.clear()
        self.inst_f_cur.clear()
        
        self.last_view_ch1 = None
        self.last_view_ch2 = None
        
        # 更新状态提示
        if not self.paused:
             self.lbl_dash.setText("<br><br><center><span style='color:#4db6ac; font-size:11pt'>Waiting for data...</span></center>")

    def on_zpw_mode_toggled(self, checked):
        """切换 ZPW 模式时提供 UI 反馈"""
        if checked:
            # 互斥：关闭 50Hz 检测
            if self.btn_detect_50hz.isChecked():
                 self.btn_detect_50hz.setChecked(False)

            self.lbl_dash.setText("<br><br><center><span style='color:#ff9800; font-size:12pt'><b>Running ZPW Analysis...</b></span><br><span style='color:#888'>This may take a moment for large datasets.</span></center>")
            self.lbl_dash.repaint()
            QApplication.processEvents()
            
        self.reprocess_view()

    def on_detect_50hz_toggled(self, checked):
        if checked:
            # 互斥：关闭 ZPW 模式
            if self.chk_zpw_mode.isChecked():
                self.chk_zpw_mode.setChecked(False)
            
            self.lbl_dash.setText("<br><br><center><span style='color:#00796b; font-size:12pt'><b>Analyzing 50Hz Interference...</b></span></center>")
            self.lbl_dash.repaint()
            QApplication.processEvents()
        self.reprocess_view()

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
            self.process_and_plot(self.last_view_ch1, self.last_view_ch2, self.last_fs)

    def on_pause_toggled(self, checked):
        self.paused = checked
        if hasattr(self, 'btn_time_refresh'):
            self.btn_time_refresh.setEnabled(not checked)
            
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
                v_range = self.atten_volt_range if self.chk_atten.isChecked() else self.normal_volt_range
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

        is_atten = self.chk_atten.isChecked()
        scale = 50.0 if is_atten else 1.0
        v1_p = v1_base * scale
        v2_p = v2_base * scale

        algo = SignalProcessor.freq_acf if self.rad_acf.isChecked() else SignalProcessor.freq_schmitt
        ft1, ft2 = algo(v1_p, fs), algo(v2_p, fs)
        vpp1, vpp2 = SignalProcessor.get_vpp(v1_p), SignalProcessor.get_vpp(v2_p)
        
        # 计算极值用于空闲显示
        v_max1, v_min1 = (np.max(v1_p), np.min(v1_p)) if len(v1_p) > 0 else (0, 0)
        v_max2, v_min2 = (np.max(v2_p), np.min(v2_p)) if len(v2_p) > 0 else (0, 0)
        
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

        # ZPW-2000A 处理
        zpw_fc, zpw_fm = 0, 0
        is_zpw = self.chk_zpw_mode.isChecked()
        is_50hz = self.btn_detect_50hz.isChecked()
        
        self.p_demod.setVisible(is_zpw)
        self.env_cur.setVisible(is_zpw)

        if is_zpw:
            zpw_fc, zpw_fm, env, inst_freq = SignalProcessor.decode_zpw2000a(v1_p, fs)
            # 更新包络线显示
            t_axis = np.arange(len(env)) / fs
            self.env_cur.setData(t_axis, env)
            
            # 更新解调频率曲线
            if len(inst_freq) > 0:
                t_freq = t_axis[1:] if len(t_axis) > len(inst_freq) else t_axis[:len(inst_freq)]
                self.inst_f_cur.setData(t_freq, inst_freq)
        else:
            self.env_cur.clear()
            self.inst_f_cur.clear()

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
        
        # 动态调整布局：如果有光标，使用紧凑模式；否则使用舒适模式
        has_cursors = self.chk_cursors_x.isChecked() or self.chk_cursors_y.isChecked()
        line_height = "115%" if has_cursors else "130%"
        
        # 基础表格
        html = f"""
        <table width="100%" cellspacing="0" cellpadding="0" style="font-size:9pt; line-height:{line_height}">
        <tr>
            <td colspan="4" style="{style_g}; border-bottom:1px solid #444; padding-bottom:1px">
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
            <td colspan="4" style="{style_w}; padding-bottom:1px">
                Phase(2-1): {f'{phase:.1f}°' if phase is not None else '--'} 
            </td>
        </tr>
        """
        
        # ZPW 信息
        if is_zpw:
            # 模糊匹配载频 (允许 ±50Hz 偏差)
            std_fc, label_fc, diff_fc = SignalProcessor.find_nearest(zpw_fc, ZPW_STD_FC, tolerance=50)
            
            # 模糊匹配低频 (允许 ±0.6Hz 偏差)
            # find_nearest 返回的 label_fm 现在是 tuple (code, desc)
            std_fm, info_fm, diff_fm = SignalProcessor.find_nearest(zpw_fm, ZPW_STD_FM, tolerance=0.6)
            
            # 准备显示数据
            # 载频部分
            if std_fc:
                disp_fc = f"{label_fc} {std_fc}"
                color_fc = "#448aff"
            else:
                disp_fc = "未知"
                color_fc = "#888"
            
            # 低频部分
            if std_fm:
                code, desc = info_fm
                disp_fm = f"{code} {std_fm}"
                disp_desc = desc
                color_fm = "#ff9800"
            else:
                disp_fm = "未知"
                disp_desc = ""
                color_fm = "#888"

            # 格式化显示: ZPW [Carrier] ([Meas]) | [Code] [Freq] ([Meas]) [Desc]
            html += f"""
            <tr>
                <td colspan="4" style="border-top:1px solid #444; padding-top:4px; font-size:10pt">
                    <span style="color:{color_fc}"><b>ZPW {disp_fc}</b></span> 
                    <span style="color:#aaa; font-size:9pt">({zpw_fc:.1f})</span>
                    <span style="color:#666"> | </span>
                    <span style="color:{color_fm}"><b>{disp_fm}</b></span>
                    <span style="color:#aaa; font-size:9pt">({zpw_fm:.2f})</span>
                    <span style="color:#ddd; font-size:9pt"> {disp_desc}</span>
                </td>
            </tr>
            """

        # 50Hz 干扰检测显示
        if is_50hz:
            v50_1, r50_1, h_1 = SignalProcessor.detect_power_interference(v1_p, fs)
            v50_2, r50_2, h_2 = SignalProcessor.detect_power_interference(v2_p, fs)
            
            # 定义警告颜色 (如果占比超过 5% 则变红)
            c1 = "#ff5252" if r50_1 > 5.0 else "#888"
            c2 = "#ff5252" if r50_2 > 5.0 else "#888"
            
            html += f"""
            <tr>
                <td colspan="4" style="border-top:1px solid #444; padding-top:2px; font-size:9pt">
                    <b style="color:#00796b">50Hz Check:</b><br>
                    <span style="color:#ffeb3b">CH1:</span> <span style="color:{c1}">{v50_1:.2f}V ({r50_1:.1f}%)</span> 
                    <span style="color:#666">Harm: {h_1}</span><br>
                    <span style="color:#00bcd4">CH2:</span> <span style="color:{c2}">{v50_2:.2f}V ({r50_2:.1f}%)</span>
                    <span style="color:#666">Harm: {h_2}</span>
                </td>
            </tr>
            """

        # 光标部分
        has_cursor = False
        
        # X轴光标
        if self.chk_cursors_x.isChecked():
            if not has_cursor:
                # 移除额外高度，仅保留分割线
                html += f"<tr><td colspan='4' style='border-top:1px solid #444;'></td></tr>"
                has_cursor = True
                
            t1 = self.v_line1.value()
            t2 = self.v_line2.value()
            dt = abs(t2 - t1)
            
            html += f"""
            <tr>
                <td colspan="4" style="color:#ff00ff; white-space:nowrap; font-size:9pt">
                    <b>[X]</b> T1:{t1*1000:.1f}m | T2:{t2*1000:.1f}m | <b>Δ:{dt*1000:.1f}ms</b>
                </td>
            </tr>
            """

        # Y轴光标
        if self.chk_cursors_y.isChecked():
            if not has_cursor:
                html += f"<tr><td colspan='4' style='border-top:1px solid #444;'></td></tr>"
                has_cursor = True
                
            v1_c = self.h_line1.value()
            v2_c = self.h_line2.value()
            dv = abs(v2_c - v1_c)
            
            html += f"""
            <tr>
                <td colspan="4" style="color:#00ff00; white-space:nowrap; font-size:9pt">
                    <b>[Y]</b> V1:{v1_c:.1f}V | V2:{v2_c:.1f}V | <b>Δ:{dv:.2f}V</b>
                </td>
            </tr>
            """

        # 如果没有开启任何光标且不是 50Hz 模式，显示极值统计填补空白
        if not has_cursor and not is_50hz:
             html += f"""
            <tr><td colspan='4' style='border-top:1px solid #444;'></td></tr>
            <tr>
                <td style="{style_y}">CH1</td>
                <td style="{style_w}">Max:{v_max1:.1f}V</td>
                <td colspan="2" style="{style_w}">Min:{v_min1:.1f}V</td>
            </tr>
            <tr>
                <td style="{style_c}">CH2</td>
                <td style="{style_w}">Max:{v_max2:.1f}V</td>
                <td colspan="2" style="{style_w}">Min:{v_min2:.1f}V</td>
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
            is_atten = self.chk_atten.isChecked()
            scale = 50.0 if is_atten else 1.0

            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 写入元数据
                writer.writerow([f"# AuraScope Data Export"])
                writer.writerow([f"# Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"])
                writer.writerow([f"# SampleRate: {self.last_fs}"])
                writer.writerow([f"# Attenuation: {'50x' if is_atten else '1x'}"])
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

    def load_zpw_simulation(self):
        """加载仿真信号"""
        if not self.paused:
            self.btn_pause.click()
        
        # 随机选择一个标准配置进行仿真
        import random
        sim_fc = random.choice(list(ZPW_STD_FC.keys()))
        sim_fm = random.choice(list(ZPW_STD_FM.keys()))
        
        # 1700Hz 载频, 29Hz 低频
        fs = 50000 
        duration = 1.0 # 增加时长以提高频率分辨率 (1s数据 -> 1Hz分辨率)
        
        # ZPW-2000A 典型轨面电压 (幅度约 1.5V, 即 3Vpp)
        actual_amp = 1.5
        t, sig, _ = SignalSimulator.generate_zpw2000a(fc=sim_fc, fm=sim_fm, duration=duration, fs=fs, amplitude=actual_amp)
        
        v1 = sig
        v2 = np.zeros_like(sig)
        
        self.chk_zpw_mode.setChecked(True)
        self.process_and_plot(v1, v2, fs)
        
        label_fc = ZPW_STD_FC[sim_fc]
        # ZPW_STD_FM values are now tuples (code, desc)
        code, desc = ZPW_STD_FM[sim_fm]
        self.setWindowTitle(f"AuraScope Viewer - [Simulation: {label_fc} {sim_fc}Hz | Code {code} ({sim_fm}Hz)]")

    def load_25hz_simulation(self):
        """加载 25Hz 相敏轨道电路仿真信号"""
        if not self.paused:
            self.btn_pause.click()
        
        # 25Hz 相敏轨道电路参数
        fs = 50000
        duration = 1.0
        phase = 90.0 # 标准相位差
        
        # 生成纯净信号 (无噪声)
        # CH1: 局部电压 110V, CH2: 轨道电压 18V (模拟值)
        t, sig1, sig2 = SignalSimulator.generate_25hz_phase(
            freq=25.0, phase_diff=phase, duration=duration, fs=fs,
            amp1=110.0, amp2=18.0
        )
        
        # 自动切换到 50x 衰减模式以显示高压信号
        self.chk_atten.setChecked(True)
        
        # 禁用 ZPW 模式
        self.chk_zpw_mode.setChecked(False)
        
        # 模拟 50x 衰减后的输入电压 (Sig 是实际物理电压)
        scale = 50.0
        v1_base = sig1 / scale
        v2_base = sig2 / scale
        
        self.process_and_plot(v1_base, v2_base, fs)
        
        self.setWindowTitle(f"AuraScope Viewer - [Simulation: 25Hz Phase Sensitive | Phase {phase}°]")

    def load_simulation(self):
        idx = self.cb_sim_type.currentIndex()
        if idx == 0:
            self.load_zpw_simulation()
        elif idx == 1:
            self.load_25hz_simulation()

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
                self.chk_atten.blockSignals(True)
                self.chk_atten.setChecked(file_scale == 50.0)
                self.chk_atten.blockSignals(False)
                # 更新触发线等 UI 元素 (手动触发一次不带 reprocess_view 的 UI 更新)
                self.on_trigger_slider_move(self.sl_trig.value())
                
                self.process_and_plot(v1_base, v2_base, fs)
                self.setWindowTitle(f"AuraScope Viewer - [Loaded: {path}]")
                QMessageBox.information(self, "导入成功", 
                    f"已加载 {len(v1_base)} 点数据\\n采样率: {fs} Hz\\n衰减模式: {'50x' if file_scale==50 else '1x'}")

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
