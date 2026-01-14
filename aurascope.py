import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
from collections import deque

# 核心 UI 库导入
from PySide6.QtCore import Qt, QThread, Signal, Slot
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QComboBox, QPushButton, QLabel, 
                             QCheckBox, QRadioButton, QGroupBox, QSlider, QButtonGroup)
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
        
        # 数据缓冲区
        self.double_buf_ch1 = []
        self.double_buf_ch2 = []
        self.rolling_ch1 = deque(maxlen=5000)
        self.rolling_ch2 = deque(maxlen=5000)
        
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
        l_algo = QVBoxLayout()
        self.rad_schmitt = QRadioButton("施密特触发")
        self.rad_acf = QRadioButton("自相关 (ACF)")
        self.rad_schmitt.setChecked(True)
        self.algo_group = QButtonGroup(self)
        self.algo_group.addButton(self.rad_schmitt)
        self.algo_group.addButton(self.rad_acf)
        l_algo.addWidget(self.rad_schmitt)
        l_algo.addWidget(self.rad_acf)
        gp_algo.setLayout(l_algo)
        ctrl_panel.addWidget(gp_algo)

        # 5. FFT 模式 (互斥组)
        gp_fft = QGroupBox("5. FFT 显示模式")
        l_fft = QVBoxLayout()
        self.rad_fft_lin = QRadioButton("线性坐标")
        self.rad_fft_log = QRadioButton("对数坐标 (dB)")
        self.rad_fft_lin.setChecked(True)
        self.fft_group = QButtonGroup(self)
        self.fft_group.addButton(self.rad_fft_lin)
        self.fft_group.addButton(self.rad_fft_log)
        l_fft.addWidget(self.rad_fft_lin)
        l_fft.addWidget(self.rad_fft_log)
        gp_fft.setLayout(l_fft)
        ctrl_panel.addWidget(gp_fft)

        # 6. 信息仪表盘
        self.lbl_dash = QLabel("离线")
        self.lbl_dash.setStyleSheet("background: #000; color: #0F0; font-family: Consolas; padding: 10px; border: 1px solid #444;")
        ctrl_panel.addWidget(self.lbl_dash)

        ctrl_panel.addStretch()
        layout.addLayout(ctrl_panel, 1)

        self.win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.win, 4)
        self.refresh_ports()

    def setup_plots(self):
        self.p_t = self.win.addPlot(title="时域波形")
        self.p_t.showGrid(x=True, y=True, alpha=0.3)
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

    def on_trigger_slider_move(self, value):
        scale = 50 if self.chk_atten.isChecked() else 1
        real_volts = (value / 100.0) * scale
        self.lbl_trig.setText(f"触发电平: {real_volts:.2f} V")
        self.trig_line.setValue(real_volts)

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
        is_atten = self.chk_atten.isChecked()
        v_mult = 50.0 if is_atten else 1.0
        
        # 电压转换
        v1_p = (5.0 - 2.0 * (ch1_raw / 4095.0 * 3.3)) * v_mult
        v2_p = (5.0 - 2.0 * (ch2_raw / 4095.0 * 3.3)) * v_mult
        
        mode = self.cb_display_mode.currentIndex()
        f1, f2 = None, None

        # 模式切换逻辑
        if mode == 0: 
            f1, f2 = v1_p, v2_p
        elif mode == 1:
            self.double_buf_ch1.extend(v1_p)
            self.double_buf_ch2.extend(v2_p)
            if len(self.double_buf_ch1) >= 2000:
                f1 = np.array(self.double_buf_ch1[:2000])
                f2 = np.array(self.double_buf_ch2[:2000])
                self.double_buf_ch1.clear(); self.double_buf_ch2.clear()
            else: return
        elif mode == 2:
            self.rolling_ch1.extend(v1_p); self.rolling_ch2.extend(v2_p)
            f1, f2 = np.array(self.rolling_ch1), np.array(self.rolling_ch2)

        if f1 is None: return

        # 触发
        trig_v = (self.sl_trig.value() / 100.0) * v_mult
        if self.chk_trig.isChecked() and mode < 2:
            indices = np.where((f1[:-1] < trig_v) & (f1[1:] >= trig_v))[0]
            if len(indices) > 0:
                s = indices[0]
                f1 = f1[s:s+1000]; f2 = f2[s:s+1000]

        # 计算频率
        algo = SignalProcessor.freq_acf if self.rad_acf.isChecked() else SignalProcessor.freq_schmitt
        ft1, ft2 = algo(v1_p, fs), algo(v2_p, fs)
        vpp1, vpp2 = SignalProcessor.get_vpp(v1_p), SignalProcessor.get_vpp(v2_p)

        # FFT
        def get_fft(d, fs):
            n = len(d)
            yf = rfft(d - np.mean(d))
            xf = rfftfreq(n, 1/fs)
            m = np.abs(yf)
            return xf, m, xf[np.argmax(m)] if len(m)>0 else 0

        xf1, mf1, ff1 = get_fft(v1_p, fs)
        xf2, mf2, ff2 = get_fft(v2_p, fs)

        # 更新绘图
        self.p_f.setLogMode(y=self.rad_fft_log.isChecked())
        self.cur1.setData(f1); self.f_cur1.setData(xf1, mf1)
        self.cur2.setData(f2); self.f_cur2.setData(xf2, mf2)

        # 看板刷新
        self.lbl_dash.setText(
            f"<b>采样率:</b> {fs/1000:.1f} kHz<br><hr>"
            f"<b>CH1:</b> Vpp:{vpp1:.2f}V | F_T:{ft1:.1f}Hz | F_F:{ff1:.1f}Hz<br>"
            f"<b>CH2:</b> Vpp:{vpp2:.2f}V | F_T:{ft2:.1f}Hz | F_F:{ff2:.1f}Hz"
        )
        self.p_t.update(); self.p_f.update()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = AuraScope()
    win.show()
    sys.exit(app.exec())
