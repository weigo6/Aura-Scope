import sys
import csv
import datetime
import numpy as np
import serial
import serial.tools.list_ports
import struct
from collections import deque

# 核心 UI 库导入
from PySide6.QtCore import Qt, QThread, Signal, Slot
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QComboBox, QPushButton, QLabel, 
                             QCheckBox, QRadioButton, QGroupBox, QSlider, QButtonGroup,
                             QSizePolicy, QFileDialog, QMessageBox)
import pyqtgraph as pg
from scipy.fft import rfft, rfftfreq
from scipy.signal import find_peaks

# --- 采样率对应表 ---
FS_TABLE = [500000, 200000, 100000, 50000, 20000, 10000, 5000, 2000, 1000]

class SignalProcessor:
    @staticmethod
    def get_vpp(data):
        return np.ptp(data) if len(data) > 0 else 0

    @staticmethod
    def phase_delay_xcorr(data1, data2, fs, freq_hint=None):
        if len(data1) < 10 or len(data2) < 10:
            return 0
        n = min(len(data1), len(data2))
        x = data1[:n] - np.mean(data1[:n])
        y = data2[:n] - np.mean(data2[:n])
        corr = np.correlate(x, y, mode='full')
        lags = np.arange(-n + 1, n)
        lag = lags[np.argmax(corr)]
        if freq_hint is None:
            f0 = SignalProcessor.freq_acf(data1, fs)
        else:
            f0 = freq_hint
        if f0 <= 0:
            return 0
        phase = (lag * f0 / fs) * 360.0
        phase = (phase + 180.0) % 360.0 - 180.0
        return phase

    @staticmethod
    def phase_fft(data1, data2, fs, freq_hint=None):
        if len(data1) < 10 or len(data2) < 10:
            return 0
        n = min(len(data1), len(data2))
        x = data1[:n] - np.mean(data1[:n])
        y = data2[:n] - np.mean(data2[:n])
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
        threshold = vpp * 0.1 if vpp > 0.1 else 0.05
        pos = np.where(norm > threshold, 1, 0)
        edges = np.where(np.diff(pos) > 0)[0]
        if len(edges) < 2: return 0
        return fs / np.mean(np.diff(edges))

    @staticmethod
    def freq_acf(data, fs):
        if len(data) < 10: return 0
        norm = data - np.mean(data)
        acf = np.correlate(norm, norm, mode='full')
        acf = acf[len(acf)//2:]
        peaks, _ = find_peaks(acf, height=np.max(acf)*0.3)
        if len(peaks) < 1: return 0
        return fs / peaks[0]

class DataWorker(QThread):
    """串口接收线程 - 包含健壮性校验"""
    packet_ready = Signal(np.ndarray, np.ndarray, int)
    
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        try:
            # 尝试开启串口，波特率 2M
            ser = serial.Serial(self.port, 2000000, timeout=0.1)
            try:
                if hasattr(ser, "set_buffer_size"):
                    try:
                        ser.set_buffer_size(rx_size=65536, tx_size=65536)
                    except TypeError:
                        try:
                            ser.set_buffer_size(65536, 65536)
                        except TypeError:
                            pass
            except Exception:
                pass
            
            while self.running:
                # 至少读到帧头和长度 (5字节)
                if ser.in_waiting >= 5:
                    if ser.read(1) == b'\xAA':
                        if ser.read(1) == b'\x55':
                            header = ser.read(3)
                            if len(header) < 3: continue
                            
                            fs_idx = header[0]
                            data_len = struct.unpack('<H', header[1:3])[0]

                            # --- 协议头严格校验 ---
                            # 1. 采样率索引校验 (严格匹配 FS_TABLE 长度，目前为9，即索引 0~8)
                            # FS_TABLE = [500k, 200k, ..., 1k] 共9项，>=9 即非法
                            if fs_idx >= len(FS_TABLE): 
                                continue

                            # 2. 长度强校验：固件目前固定发送 1000点 (4000字节)
                            if data_len != 4000:
                                continue
                            
                            # 读取数据负载
                            raw = ser.read(data_len)
                            
                            # --- 关键修复：长度对齐校验 ---
                            actual_len = len(raw)
                            if actual_len != data_len or actual_len % 4 != 0:
                                # 如果长度不对或不能被4整除（uint32对齐要求），舍弃
                                continue
                            
                            try:
                                # 转换为 32 位数组
                                data_32 = np.frombuffer(raw, dtype=np.uint32)
                                # CH1: bits [11:0], CH2: bits [27:16]
                                ch1 = data_32 & 0x0FFF
                                ch2 = (data_32 >> 16) & 0x0FFF
                                self.packet_ready.emit(ch1, ch2, fs_idx)
                            except Exception:
                                continue 
                                
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
        self.setWindowTitle("AuraScope Viewer - V2.2 (Stability Fixed)")
        self.resize(1450, 950)
        
        self.double_buf_ch1 = []
        self.double_buf_ch2 = []
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
        self.init_ui()
        self.setup_plots()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        ctrl_panel = QVBoxLayout()
        
        # 1. 串口配置
        gp_conn = QGroupBox("1. 设备连接")
        l_conn = QVBoxLayout()
        h_port = QHBoxLayout()
        self.cb_port = QComboBox()
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.setFixedWidth(50)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        h_port.addWidget(self.cb_port)
        h_port.addWidget(self.btn_refresh)
        self.btn_conn = QPushButton("打开串口")
        self.btn_conn.clicked.connect(self.toggle_serial)
        l_conn.addLayout(h_port)
        l_conn.addWidget(self.btn_conn)
        gp_conn.setLayout(l_conn)
        ctrl_panel.addWidget(gp_conn)

        # 2. 模式与衰减
        gp_mode = QGroupBox("2. 采集模式")
        l_mode = QVBoxLayout()
        self.cb_display_mode = QComboBox()
        self.cb_display_mode.addItems(["单包模式 (1000点)", "双包模式 (2000点)", "多包滚动 (5000点)"])
        self.cb_display_mode.currentIndexChanged.connect(self.on_mode_changed)
        self.chk_atten = QCheckBox("50x 衰减模式 (±250V)")
        self.chk_atten.toggled.connect(self.update_trigger_range)
        l_mode.addWidget(self.cb_display_mode)
        l_mode.addWidget(self.chk_atten)
        gp_mode.setLayout(l_mode)
        ctrl_panel.addWidget(gp_mode)

        # 3. 软件触发
        gp_trig = QGroupBox("3. 软件触发")
        l_trig = QVBoxLayout()
        self.chk_trig = QCheckBox("启用触发")
        self.sl_trig = QSlider(Qt.Horizontal)
        self.sl_trig.setRange(-500, 500)
        self.lbl_trig = QLabel("触发电平: 0.00 V")
        self.sl_trig.valueChanged.connect(self.on_trigger_slider_move)
        l_trig.addWidget(self.chk_trig)
        l_trig.addWidget(self.lbl_trig)
        l_trig.addWidget(self.sl_trig)
        gp_trig.setLayout(l_trig)
        ctrl_panel.addWidget(gp_trig)

        # 4. 频率算法 (互斥组)
        gp_algo = QGroupBox("4. 频率检测算法")
        l_algo = QHBoxLayout()
        self.rad_schmitt = QRadioButton("施密特触发")
        self.rad_acf = QRadioButton("自相关 (ACF)")
        self.rad_schmitt.setChecked(True)
        self.algo_group = QButtonGroup(self)
        self.algo_group.addButton(self.rad_schmitt)
        self.algo_group.addButton(self.rad_acf)
        self.algo_group.buttonClicked.connect(self.reprocess_view)
        l_algo.addWidget(self.rad_schmitt)
        l_algo.addWidget(self.rad_acf)
        gp_algo.setLayout(l_algo)
        ctrl_panel.addWidget(gp_algo)

        # 5. FFT 模式 (互斥组)
        gp_fft = QGroupBox("5. FFT 显示模式")
        l_fft = QHBoxLayout()
        self.rad_fft_lin = QRadioButton("线性坐标")
        self.rad_fft_log = QRadioButton("对数坐标 (dB)")
        self.rad_fft_lin.setChecked(True)
        self.fft_group = QButtonGroup(self)
        self.fft_group.addButton(self.rad_fft_lin)
        self.fft_group.addButton(self.rad_fft_log)
        self.fft_group.buttonClicked.connect(self.reprocess_view)
        l_fft.addWidget(self.rad_fft_lin)
        l_fft.addWidget(self.rad_fft_log)
        gp_fft.setLayout(l_fft)
        ctrl_panel.addWidget(gp_fft)

        gp_phase = QGroupBox("6. 相位差算法")
        l_phase = QHBoxLayout()
        self.rad_phase_xcorr = QRadioButton("时间延迟交叉相关")
        self.rad_phase_fft = QRadioButton("FFT 相位")
        self.rad_phase_xcorr.setChecked(True)
        self.phase_group = QButtonGroup(self)
        self.phase_group.addButton(self.rad_phase_xcorr)
        self.phase_group.addButton(self.rad_phase_fft)
        self.phase_group.buttonClicked.connect(self.reprocess_view)
        l_phase.addWidget(self.rad_phase_xcorr)
        l_phase.addWidget(self.rad_phase_fft)
        gp_phase.setLayout(l_phase)
        ctrl_panel.addWidget(gp_phase)

        gp_lock = QGroupBox("7. 坐标轴锁定")
        l_lock = QHBoxLayout()
        self.chk_lock_x = QCheckBox("锁定 X 轴标尺")
        self.chk_lock_y = QCheckBox("锁定 Y 轴标尺")
        self.chk_lock_x.toggled.connect(self.on_axis_lock_changed)
        self.chk_lock_y.toggled.connect(self.on_axis_lock_changed)
        l_lock.addWidget(self.chk_lock_x)
        l_lock.addWidget(self.chk_lock_y)
        gp_lock.setLayout(l_lock)
        ctrl_panel.addWidget(gp_lock)

        gp_vpp = QGroupBox("高级过滤")
        l_vpp = QVBoxLayout()
        self.chk_vpp_filter = QCheckBox("启用 Vpp 过滤（抑制异常帧闪烁）")
        self.chk_vpp_filter.setChecked(True)
        self.chk_vpp_filter.toggled.connect(self.on_vpp_filter_toggled)
        
        self.chk_smooth = QCheckBox("启用波形平滑（视觉降噪）")
        self.chk_smooth.setChecked(False)
        self.chk_smooth.toggled.connect(self.on_smooth_toggled)

        l_vpp.addWidget(self.chk_vpp_filter)
        l_vpp.addWidget(self.chk_smooth)
        gp_vpp.setLayout(l_vpp)
        ctrl_panel.addWidget(gp_vpp)

        gp_ch = QGroupBox("通道控制")
        l_ch = QHBoxLayout()
        self.chk_show_ch1 = QCheckBox("显示 CH1")
        self.chk_show_ch1.setChecked(True)
        self.chk_show_ch1.setStyleSheet("color: yellow; font-weight: bold;")
        self.chk_show_ch1.toggled.connect(self.on_channel_toggled)
        self.chk_show_ch2 = QCheckBox("显示 CH2")
        self.chk_show_ch2.setChecked(True)
        self.chk_show_ch2.setStyleSheet("color: cyan; font-weight: bold;")
        self.chk_show_ch2.toggled.connect(self.on_channel_toggled)
        l_ch.addWidget(self.chk_show_ch1)
        l_ch.addWidget(self.chk_show_ch2)
        gp_ch.setLayout(l_ch)
        ctrl_panel.addWidget(gp_ch)

        # 8. 数据管理
        gp_data = QGroupBox("8. 数据管理")
        l_data = QHBoxLayout()
        self.btn_export = QPushButton("导出 CSV")
        self.btn_export.clicked.connect(self.export_csv)
        self.btn_import = QPushButton("导入 CSV")
        self.btn_import.clicked.connect(self.import_csv)
        l_data.addWidget(self.btn_export)
        l_data.addWidget(self.btn_import)
        gp_data.setLayout(l_data)
        ctrl_panel.addWidget(gp_data)

        self.btn_pause = QPushButton("暂停波形")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self.on_pause_toggled)
        ctrl_panel.addWidget(self.btn_pause)

        # 7. 信息仪表盘
        self.lbl_dash = QLabel("离线")
        self.lbl_dash.setStyleSheet("background: #000; color: #0F0; font-family: Consolas; padding: 10px; border: 1px solid #444;")
        ctrl_panel.addWidget(self.lbl_dash)

        ctrl_panel.addStretch()
        layout.addLayout(ctrl_panel, 1)
        
        self.win = pg.GraphicsLayoutWidget()
        self.win.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.win.setMinimumSize(900, 600)
        layout.addWidget(self.win, 4)
        self.refresh_ports()

    def setup_plots(self):
        self.p_t = self.win.addPlot(title="时域波形")
        self.p_t.showGrid(x=True, y=True, alpha=0.3)
        self.p_t.setMouseEnabled(x=True, y=True)
        self.cur1 = self.p_t.plot(pen=pg.mkPen('y', width=1.2))
        self.cur2 = self.p_t.plot(pen=pg.mkPen('c', width=1.2))
        self.trig_line = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('w', style=Qt.DashLine))
        self.p_t.addItem(self.trig_line)

        self.win.nextRow()
        self.p_f = self.win.addPlot(title="频谱分析")
        self.p_f.showGrid(x=True, y=True, alpha=0.3)
        self.f_cur1 = self.p_f.plot(pen='y')
        self.f_cur2 = self.p_f.plot(pen='c')

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
        self.lbl_trig.setText(f"触发电平: {real_volts:.2f} V")
        self.trig_line.setValue(real_volts)

    def on_mode_changed(self, idx):
        # 切换模式时清空所有缓冲区，避免旧数据残留导致波形衔接异常
        self.double_buf_ch1.clear()
        self.double_buf_ch2.clear()
        self.rolling_ch1.clear()
        self.rolling_ch2.clear()

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
        """重新处理当前视图数据（响应 UI 设置变更）"""
        if self.paused and self.last_view_ch1 is not None:
            self.process_and_plot(self.last_view_ch1, self.last_view_ch2, self.last_fs)

    def on_pause_toggled(self, checked):
        self.paused = checked
        self.btn_pause.setText("继续波形 / 退出导入模式" if checked else "暂停波形")

    def toggle_serial(self):
        if not self.is_connected:
            port = self.cb_port.currentText()
            if not port: return
            self.worker = DataWorker(port)
            self.worker.packet_ready.connect(self.handle_packet)
            self.worker.start()
            self.btn_conn.setText("断开连接")
            self.cb_port.setEnabled(False)
            self.btn_refresh.setEnabled(False)
            self.is_connected = True
        else:
            self.worker.stop()
            self.btn_conn.setText("打开串口")
            self.cb_port.setEnabled(True)
            self.btn_refresh.setEnabled(True)
            self.is_connected = False

    @Slot(np.ndarray, np.ndarray, int)
    def handle_packet(self, ch1_raw, ch2_raw, tb_idx):
        fs = FS_TABLE[tb_idx] if tb_idx < len(FS_TABLE) else 1000

        if self.paused:
            return
        
        # 电压转换 (基准 1x 电压)
        v1_base = (5.0 - 2.0 * (ch1_raw / 4095.0 * 3.3))
        v2_base = (5.0 - 2.0 * (ch2_raw / 4095.0 * 3.3))
        
        mode = self.cb_display_mode.currentIndex()
        f1, f2 = None, None

        # 模式切换逻辑
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

        if f1 is None: return

        # 软件触发 (使用 1x 基准电平)
        trig_v_base = (self.sl_trig.value() / 100.0)
        if self.chk_trig.isChecked() and mode < 2:
            indices = np.where((f1[:-1] < trig_v_base) & (f1[1:] >= trig_v_base))[0]
            if len(indices) > 0:
                s = indices[0]
                f1 = f1[s:s+1000]; f2 = f2[s:s+1000]

        self.process_and_plot(f1, f2, fs)

    def process_and_plot(self, v1_base, v2_base, fs):
        # 缓存基准数据（用于切换倍率时重新处理）
        self.last_view_ch1 = v1_base
        self.last_view_ch2 = v2_base
        self.last_fs = fs

        # 应用当前倍率
        is_atten = self.chk_atten.isChecked()
        scale = 50.0 if is_atten else 1.0
        v1_p = v1_base * scale
        v2_p = v2_base * scale

        # 计算频率
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

        # FFT
        def get_fft(d, fs):
            n = len(d)
            yf = rfft(d - np.mean(d))
            xf = rfftfreq(n, 1/fs)
            m = np.abs(yf)
            return xf, m, xf[np.argmax(m)] if len(m)>0 else 0

        xf1, mf1, ff1 = get_fft(v1_p, fs)
        xf2, mf2, ff2 = get_fft(v2_p, fs)

        freq_hint = 0.0
        if ft1 > 0 and ft2 > 0:
            freq_hint = 0.5 * (ft1 + ft2)
        elif ft1 > 0 or ft2 > 0:
            freq_hint = max(ft1, ft2)
        if freq_hint == 0.0:
            if ff1 > 0 and ff2 > 0:
                freq_hint = 0.5 * (ff1 + ff2)
            elif ff1 > 0 or ff2 > 0:
                freq_hint = max(ff1, ff2)

        phase = None
        if self.show_ch1 and self.show_ch2:
            if self.rad_phase_fft.isChecked():
                phase = SignalProcessor.phase_fft(v1_p, v2_p, fs, freq_hint if freq_hint > 0 else None)
            else:
                phase = SignalProcessor.phase_delay_xcorr(v1_p, v2_p, fs, freq_hint if freq_hint > 0 else None)

        # 可选：视觉平滑 (3点移动平均)
        f1_disp = v1_p
        f2_disp = v2_p
        if self.enable_smooth:
            def smooth(d):
                if len(d) < 3: return d
                return np.convolve(d, [1/3, 1/3, 1/3], mode='same')
            f1_disp = smooth(f1_disp)
            f2_disp = smooth(f2_disp)

        # 更新绘图
        self.p_f.setLogMode(y=self.rad_fft_log.isChecked())
        
        if self.show_ch1:
            self.cur1.setData(f1_disp)
            self.f_cur1.setData(xf1, mf1)
        
        if self.show_ch2:
            self.cur2.setData(f2_disp)
            self.f_cur2.setData(xf2, mf2)

        # 采样时间（当前时域帧的时间跨度）
        span_s = len(f1_disp) / fs if fs > 0 else 0
        if span_s >= 1.0:
            span_str = f"{span_s:.2f} s"
        elif span_s >= 1e-3:
            span_str = f"{span_s*1000:.2f} ms"
        else:
            span_str = f"{span_s*1e6:.2f} us"

        # 轴锁定状态
        lock_x = self.chk_lock_x.isChecked()
        lock_y = self.chk_lock_y.isChecked()
        if lock_x and lock_y:
            lock_str = " | 轴锁定: X,Y"
        elif lock_x:
            lock_str = " | 轴锁定: X"
        elif lock_y:
            lock_str = " | 轴锁定: Y"
        else:
            lock_str = ""

        # 相位差显示
        if self.show_ch1 and self.show_ch2:
            if phase is None:
                phase_line = "相位差(CH2-CH1): --"
            else:
                phase_line = f"相位差(CH2-CH1): {phase:.1f}°"
        else:
            phase_line = "相位差(CH2-CH1): --"

        # 通道信息：标题与细节分行
        if self.show_ch1:
            ch1_title = "CH1(黄色)"
            ch1_detail = f"Vpp: {vpp1:.2f}V | F_T: {ft1:.1f}Hz | F_F: {ff1:.1f}Hz"
        else:
            ch1_title = "CH1(黄色)"
            ch1_detail = "OFF"

        if self.show_ch2:
            ch2_title = "CH2(青色)"
            ch2_detail = f"Vpp: {vpp2:.2f}V | F_T: {ft2:.1f}Hz | F_F: {ff2:.1f}Hz"
        else:
            ch2_title = "CH2(青色)"
            ch2_detail = "OFF"

        fs_k = int(fs / 1000)
        self.lbl_dash.setText(
            f"采样率: {fs_k}k | 采样时间: {span_str}{lock_str}<br>"  # 第1行
            f"{phase_line}<br>"                                 # 第2行：相位差
            f"────────────<br>"                                 # 第3行：横线
            f"{ch1_title}<br>"                                  # 第4行：CH1 标题
            f"{ch1_detail}<br>"                                 # 第5行：CH1 信息
            f"{ch2_title}<br>"                                  # 第6行：CH2 标题
            f"{ch2_detail}"                                     # 第7行：CH2 信息
        )
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
                    f"已加载 {len(v1_base)} 点数据\n采样率: {fs} Hz\n衰减模式: {'50x' if file_scale==50 else '1x'}")

        except Exception as e:
            QMessageBox.critical(self, "导入错误", str(e))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = AuraScope()
    win.show()
    sys.exit(app.exec())
