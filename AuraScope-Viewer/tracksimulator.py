import numpy as np
import sys
import csv
import datetime
import matplotlib.pyplot as plt
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                               QLabel, QComboBox, QDoubleSpinBox, QPushButton, 
                               QFileDialog, QMessageBox, QCheckBox, QSpinBox, QGroupBox)
from PySide6.QtCore import Qt

class SignalSimulator:
    """
    轨道信号模拟器
    用于生成 ZPW-2000A 等轨道电路的仿真信号
    """

    # ZPW-2000A 标准频率定义
    CARRIERS = [1700, 2000, 2300, 2600]
    LOW_FREQS = [
        10.3, 11.4, 12.5, 13.6, 14.7, 15.8, 16.9, 18.0, 19.1,
        20.2, 21.3, 22.4, 23.5, 24.6, 25.7, 26.8, 27.9, 29.0
    ]

    @staticmethod
    def generate_zpw2000a(fc=1700, fm=29, df=11, duration=1.0, fs=200000, amplitude=1.0, noise_std=0.0, interference_50hz=0.0):
        """
        生成 ZPW-2000A 移频信号
        
        参数:
        fc : 载频 (Hz)
        fm : 低频调制频率 (Hz)
        df : 频偏 (Hz), 通常为 11 Hz
        duration : 信号时长 (s)
        fs : 采样率 (Hz)
        amplitude : 信号幅值 (V)
        noise_std : 高斯噪声标准差 (V)
        interference_50hz : 50Hz 工频干扰幅值 (V)
        
        返回:
        t : 时间轴数组
        signal : 调制后的信号数组 (幅度 scaled)
        mod_signal : 调制波形 (方波)
        """
        t = np.linspace(0, duration, int(fs * duration), endpoint=False)
        
        # 1. 生成调制方波: 控制频率在 fc+df 和 fc-df 之间切换
        # sin(2*pi*fm*t) > 0 时为 +1, < 0 时为 -1
        mod_signal = np.sign(np.sin(2 * np.pi * fm * t))
        
        # 2. 计算瞬时频率
        inst_freq = fc + mod_signal * df
        
        # 3. 计算相位 (频率的积分)
        # 累加瞬时频率，乘以 2*pi/fs 得到相位增量
        phase = 2 * np.pi * np.cumsum(inst_freq) / fs
        
        # 4. 生成调频信号
        signal = amplitude * np.cos(phase)
        
        # 5. 添加干扰
        if interference_50hz > 0:
            # 50Hz 工频干扰
            signal += interference_50hz * np.sin(2 * np.pi * 50 * t)
            
        if noise_std > 0:
            # 高斯白噪声
            signal += np.random.normal(0, noise_std, size=len(t))
        
        return t, signal, mod_signal

    @staticmethod
    def generate_25hz_phase(freq=25.0, phase_diff=90.0, duration=1.0, fs=200000, amp1=110.0, amp2=110.0, 
                            noise_std1=0.0, interference_50hz1=0.0,
                            noise_std2=0.0, interference_50hz2=0.0):
        """
        生成 25Hz 相敏轨道电路信号
        
        参数:
        freq : 频率 (Hz), 通常为 25Hz (或 50Hz 工频干扰)
        phase_diff : 相位差 (度), CH2 滞后于 CH1 的角度
        duration : 信号时长 (s)
        fs : 采样率 (Hz)
        amp1 : CH1 幅值 (V) - 局部电压
        amp2 : CH2 幅值 (V) - 轨道电压
        noise_std1 : CH1 高斯噪声标准差 (V)
        interference_50hz1 : CH1 50Hz 工频干扰幅值 (V)
        noise_std2 : CH2 高斯噪声标准差 (V)
        interference_50hz2 : CH2 50Hz 工频干扰幅值 (V)
        
        返回:
        t : 时间轴数组
        sig1 : CH1 信号
        sig2 : CH2 信号
        """
        t = np.linspace(0, duration, int(fs * duration), endpoint=False)
        
        # CH1: 参考信号 (局部电压)
        sig1 = amp1 * np.sin(2 * np.pi * freq * t)
        
        # CH2: 轨道信号 (带相位差)
        # phase_diff is in degrees
        phi = np.deg2rad(phase_diff)
        sig2 = amp2 * np.sin(2 * np.pi * freq * t + phi)
        
        # CH1 干扰
        if interference_50hz1 > 0:
            sig1 += interference_50hz1 * np.sin(2 * np.pi * 50 * t)
        if noise_std1 > 0:
            sig1 += np.random.normal(0, noise_std1, size=len(t))

        # CH2 干扰
        if interference_50hz2 > 0:
            sig2 += interference_50hz2 * np.sin(2 * np.pi * 50 * t)
        if noise_std2 > 0:
            sig2 += np.random.normal(0, noise_std2, size=len(t))
        
        return t, sig1, sig2

class SimulatorApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AuraScope Signal Generator")
        self.resize(450, 500)
        self.setup_ui()
        self.update_ui_state()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # 0. 模式选择
        gp_mode = QGroupBox("模式选择")
        l_mode = QHBoxLayout()
        l_mode.addWidget(QLabel("信号类型:"))
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["ZPW-2000A (移频)", "25Hz 相敏轨道电路"])
        self.cb_mode.currentIndexChanged.connect(self.update_ui_state)
        l_mode.addWidget(self.cb_mode)
        gp_mode.setLayout(l_mode)
        layout.addWidget(gp_mode)

        # 1. ZPW-2000A 参数
        self.gp_zpw = QGroupBox("ZPW-2000A 参数")
        l_zpw = QVBoxLayout()
        
        h_fc = QHBoxLayout()
        h_fc.addWidget(QLabel("载频 (Hz):"))
        self.cb_fc = QComboBox()
        self.cb_fc.addItems([str(f) for f in SignalSimulator.CARRIERS])
        h_fc.addWidget(self.cb_fc)
        
        h_fc.addWidget(QLabel("频偏:"))
        self.cb_offset = QComboBox()
        self.cb_offset.addItems(["无 (Standard)", "-1 (+1.4Hz)", "-2 (-1.3Hz)"])
        h_fc.addWidget(self.cb_offset)

        l_zpw.addLayout(h_fc)
        
        h_fm = QHBoxLayout()
        h_fm.addWidget(QLabel("低频 (Hz):"))
        self.cb_fm = QComboBox()
        self.cb_fm.addItems([str(f) for f in SignalSimulator.LOW_FREQS])
        self.cb_fm.setCurrentText("29.0")
        h_fm.addWidget(self.cb_fm)
        l_zpw.addLayout(h_fm)
        
        self.gp_zpw.setLayout(l_zpw)
        layout.addWidget(self.gp_zpw)

        # 2. 25Hz 参数
        self.gp_25hz = QGroupBox("25Hz 相敏参数")
        l_25hz = QVBoxLayout()
        
        h_phase = QHBoxLayout()
        h_phase.addWidget(QLabel("相位差 (°):"))
        self.sb_phase = QDoubleSpinBox()
        self.sb_phase.setRange(-180.0, 180.0)
        self.sb_phase.setValue(90.0)
        self.sb_phase.setSingleStep(5.0)
        h_phase.addWidget(self.sb_phase)
        l_25hz.addLayout(h_phase)

        h_amp2 = QHBoxLayout()
        h_amp2.addWidget(QLabel("CH2 幅值 (V):"))
        self.sb_amp2 = QDoubleSpinBox()
        self.sb_amp2.setRange(0.1, 1000.0)
        self.sb_amp2.setValue(110.0)
        h_amp2.addWidget(self.sb_amp2)
        l_25hz.addLayout(h_amp2)
        
        self.gp_25hz.setLayout(l_25hz)
        layout.addWidget(self.gp_25hz)

        # 3. 干扰设置 (分通道)
        gp_noise = QGroupBox("干扰与噪声设置")
        l_noise = QVBoxLayout()
        
        # CH1 干扰设置
        self.gp_noise_ch1 = QGroupBox("CH1 (ZPW信号 / 25Hz局部)")
        l_noise_ch1 = QVBoxLayout()
        
        h_n1 = QHBoxLayout()
        h_n1.addWidget(QLabel("CH1 白噪声 (mV):"))
        self.sb_noise_ch1 = QDoubleSpinBox()
        self.sb_noise_ch1.setRange(0.0, 5000.0)
        self.sb_noise_ch1.setValue(10.0) 
        self.sb_noise_ch1.setSingleStep(10.0)
        h_n1.addWidget(self.sb_noise_ch1)
        
        h_n1.addWidget(QLabel("CH1 50Hz干扰 (V):"))
        self.sb_int50_ch1 = QDoubleSpinBox()
        self.sb_int50_ch1.setRange(0.0, 200.0)
        self.sb_int50_ch1.setValue(0.0)
        self.sb_int50_ch1.setSingleStep(0.1)
        h_n1.addWidget(self.sb_int50_ch1)
        
        l_noise_ch1.addLayout(h_n1)
        self.gp_noise_ch1.setLayout(l_noise_ch1)
        l_noise.addWidget(self.gp_noise_ch1)

        # CH2 干扰设置
        self.gp_noise_ch2 = QGroupBox("CH2 (25Hz轨道)")
        l_noise_ch2 = QVBoxLayout()
        
        h_n2 = QHBoxLayout()
        h_n2.addWidget(QLabel("CH2 白噪声 (mV):"))
        self.sb_noise_ch2 = QDoubleSpinBox()
        self.sb_noise_ch2.setRange(0.0, 5000.0)
        self.sb_noise_ch2.setValue(50.0)
        self.sb_noise_ch2.setSingleStep(10.0)
        h_n2.addWidget(self.sb_noise_ch2)
        
        h_n2.addWidget(QLabel("CH2 50Hz干扰 (V):"))
        self.sb_int50_ch2 = QDoubleSpinBox()
        self.sb_int50_ch2.setRange(0.0, 200.0)
        self.sb_int50_ch2.setValue(0.5)
        self.sb_int50_ch2.setSingleStep(0.1)
        h_n2.addWidget(self.sb_int50_ch2)
        
        l_noise_ch2.addLayout(h_n2)
        self.gp_noise_ch2.setLayout(l_noise_ch2)
        l_noise.addWidget(self.gp_noise_ch2)
        
        gp_noise.setLayout(l_noise)
        layout.addWidget(gp_noise)

        # 4. 公共参数
        gp_common = QGroupBox("公共参数")
        l_common = QVBoxLayout()
        
        h_fs = QHBoxLayout()
        h_fs.addWidget(QLabel("采样率 (Hz):"))
        self.sb_fs = QSpinBox()
        self.sb_fs.setRange(1000, 1000000)
        self.sb_fs.setValue(50000)
        self.sb_fs.setSingleStep(10000)
        h_fs.addWidget(self.sb_fs)
        l_common.addLayout(h_fs)

        h_dur = QHBoxLayout()
        h_dur.addWidget(QLabel("时长 (s):"))
        self.sb_dur = QDoubleSpinBox()
        self.sb_dur.setRange(0.1, 10.0)
        self.sb_dur.setValue(1.0)
        self.sb_dur.setSingleStep(0.1)
        h_dur.addWidget(self.sb_dur)
        l_common.addLayout(h_dur)

        h_amp = QHBoxLayout()
        self.lbl_amp = QLabel("幅值 (V):")
        h_amp.addWidget(self.lbl_amp)
        self.sb_amp = QDoubleSpinBox()
        self.sb_amp.setRange(0.1, 1000.0)
        self.sb_amp.setValue(18.0)  # Default changed to 18.0V (Typical for occupied/unoccupied transition or ZPW rail voltage is lower)
        h_amp.addWidget(self.sb_amp)
        l_common.addLayout(h_amp)
        
        self.chk_atten = QCheckBox("标记为 50x 衰减 (模拟高压输入)")
        self.chk_atten.setChecked(False) # Default unchecked as requested
        l_common.addWidget(self.chk_atten)
        
        gp_common.setLayout(l_common)
        layout.addWidget(gp_common)

        # 5. 操作按钮
        h_btns = QHBoxLayout()
        
        self.btn_preview = QPushButton("预览波形 (Preview)")
        self.btn_preview.setStyleSheet("background-color: #1976d2; color: white; font-weight: bold; padding: 10px;")
        self.btn_preview.clicked.connect(self.preview_waveform)
        h_btns.addWidget(self.btn_preview)
        
        self.btn_export = QPushButton("生成并导出 CSV")
        self.btn_export.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 10px;")
        self.btn_export.clicked.connect(self.export_csv)
        h_btns.addWidget(self.btn_export)
        
        layout.addLayout(h_btns)
        
        layout.addStretch()

    def update_ui_state(self):
        mode = self.cb_mode.currentIndex()
        if mode == 0: # ZPW-2000A
            self.gp_zpw.setVisible(True)
            self.gp_25hz.setVisible(False)
            self.lbl_amp.setText("幅值 (V):")
            self.gp_noise_ch2.setVisible(False) # ZPW 主要是单通道
            self.gp_noise_ch1.setTitle("信号干扰设置")
            
            # Set sensible defaults for ZPW (轨面电压通常较低, 约 0.5V - 3V)
            if self.sb_amp.value() > 10.0:
                self.sb_amp.setValue(1.5)
            # ZPW 模式下 50Hz 干扰可能较小
            self.sb_int50_ch1.setValue(0.0)
            
        else: # 25Hz
            self.gp_zpw.setVisible(False)
            self.gp_25hz.setVisible(True)
            self.lbl_amp.setText("CH1 幅值 (V):")
            self.gp_noise_ch2.setVisible(True)
            self.gp_noise_ch1.setTitle("CH1 (局部) 干扰")
            self.gp_noise_ch2.setTitle("CH2 (轨道) 干扰")

            # Set sensible defaults for 25Hz
            # CH1 局部电压 110V
            if self.sb_amp.value() < 10.0:
                self.sb_amp.setValue(110.0)
            # CH2 轨道电压: 调整期约 18-24V，分路时 < 2.7V
            self.sb_amp2.setValue(18.0) 
            # 25Hz 模式下，50Hz 干扰是主要检测对象 (牵引电流谐波)
            self.sb_int50_ch2.setValue(2.0)

    def preview_waveform(self):
        try:
            mode = self.cb_mode.currentIndex()
            fs = self.sb_fs.value()
            duration = self.sb_dur.value()
            amp1 = self.sb_amp.value()
            
            # 获取干扰参数
            noise_v1 = self.sb_noise_ch1.value() / 1000.0
            int50_v1 = self.sb_int50_ch1.value()
            noise_v2 = self.sb_noise_ch2.value() / 1000.0
            int50_v2 = self.sb_int50_ch2.value()
            
            t = None
            ch1_sig = None
            ch2_sig = None
            title = ""
            
            if mode == 0: # ZPW-2000A
                fc = float(self.cb_fc.currentText())
                
                # Apply frequency offset
                offset_idx = self.cb_offset.currentIndex()
                if offset_idx == 1:
                    fc += 1.4
                elif offset_idx == 2:
                    fc -= 1.3
                    
                fm = float(self.cb_fm.currentText())
                
                # ZPW-2000A 使用 CH1 的干扰设置
                t, sig, mod = SignalSimulator.generate_zpw2000a(
                    fc=fc, fm=fm, duration=duration, fs=fs, amplitude=amp1,
                    noise_std=noise_v1, interference_50hz=int50_v1
                )
                ch1_sig = sig
                ch2_sig = mod * (amp1 / 2) # Show modulation signal as CH2
                title = f"ZPW-2000A Preview (Fc={fc}, Fm={fm})"
                
            else: # 25Hz Phase
                phase = self.sb_phase.value()
                amp2 = self.sb_amp2.value()
                
                t, ch1_sig, ch2_sig = SignalSimulator.generate_25hz_phase(
                    freq=25.0, phase_diff=phase, duration=duration, fs=fs, amp1=amp1, amp2=amp2,
                    noise_std1=noise_v1, interference_50hz1=int50_v1,
                    noise_std2=noise_v2, interference_50hz2=int50_v2
                )
                title = f"25Hz Phase Preview (Phase={phase}°)"

            # Plotting
            plt.figure(figsize=(10, 6))
            
            # Limit points for performance if duration is long
            limit = min(len(t), 2000) # Show first 2000 points or less
            t_view = t[:limit]
            
            plt.subplot(2, 1, 1)
            plt.plot(t_view, ch1_sig[:limit], 'b', label='CH1 (Main/Local)')
            plt.title(f"{title} - First {limit} samples")
            plt.grid(True)
            plt.legend()
            
            plt.subplot(2, 1, 2)
            plt.plot(t_view, ch2_sig[:limit], 'r', label='CH2 (Ref/Track)')
            plt.grid(True)
            plt.legend()
            
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            QMessageBox.critical(self, "Preview Error", str(e))

    def export_csv(self):
        try:
            mode = self.cb_mode.currentIndex()
            fs = self.sb_fs.value()
            duration = self.sb_dur.value()
            amp1 = self.sb_amp.value()
            is_50x = self.chk_atten.isChecked()
            
            # 获取干扰参数
            noise_v1 = self.sb_noise_ch1.value() / 1000.0 # Convert mV to V
            int50_v1 = self.sb_int50_ch1.value()
            noise_v2 = self.sb_noise_ch2.value() / 1000.0
            int50_v2 = self.sb_int50_ch2.value()
            
            if mode == 0: # ZPW-2000A
                fc = float(self.cb_fc.currentText())
                
                # Apply frequency offset
                offset_idx = self.cb_offset.currentIndex()
                if offset_idx == 1:
                    fc += 1.4
                elif offset_idx == 2:
                    fc -= 1.3
                    
                fm = float(self.cb_fm.currentText())
                
                t, sig, mod = SignalSimulator.generate_zpw2000a(
                    fc=fc, fm=fm, duration=duration, fs=fs, amplitude=amp1,
                    noise_std=noise_v1, interference_50hz=int50_v1
                )
                ch1_sig = sig
                ch2_sig = mod * (amp1 / 2)
                
                filename = f"ZPW2000A_Fc{int(fc)}_Fm{fm}_N{int(noise_v1*1000)}mV.csv"
                param_str = f"Fc={fc:.1f}Hz, Fm={fm}Hz, Noise={noise_v1*1000}mV, Int50Hz={int50_v1}V"
                
            else: # 25Hz Phase
                phase = self.sb_phase.value()
                amp2 = self.sb_amp2.value()
                
                t, ch1_sig, ch2_sig = SignalSimulator.generate_25hz_phase(
                    freq=25.0, phase_diff=phase, duration=duration, fs=fs, amp1=amp1, amp2=amp2,
                    noise_std1=noise_v1, interference_50hz1=int50_v1,
                    noise_std2=noise_v2, interference_50hz2=int50_v2
                )
                
                filename = f"25Hz_Phase{int(phase)}_N{int(noise_v2*1000)}mV.csv"
                param_str = f"Freq=25Hz, Phase={phase}deg, CH1={amp1}V, CH2={amp2}V, N1={noise_v1*1000}mV, I1={int50_v1}V, N2={noise_v2*1000}mV, I2={int50_v2}V"
            
            # 准备导出
            path, _ = QFileDialog.getSaveFileName(self, "保存 CSV 文件", filename, "CSV Files (*.csv)")
            
            if not path:
                return
                
            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 写入头部元数据
                writer.writerow([f"# AuraScope Simulated Data"])
                writer.writerow([f"# Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"])
                writer.writerow([f"# SampleRate: {fs}"])
                writer.writerow([f"# Attenuation: {'50x' if is_50x else '1x'}"])
                writer.writerow([f"# Parameters: {param_str}"])
                writer.writerow(["Time_s", "CH1_V", "CH2_V"])
                
                rows = zip(t, ch1_sig, ch2_sig)
                for t_val, v1, v2 in rows:
                    writer.writerow([f"{t_val:.6f}", f"{v1:.4f}", f"{v2:.4f}"])
                    
            QMessageBox.information(self, "成功", f"文件已保存:\n{path}")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", str(e))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SimulatorApp()
    window.show()
    sys.exit(app.exec())
