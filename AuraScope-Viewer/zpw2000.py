import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert, butter, lfilter
import matplotlib as mpl

# --- 1. 字体缺失解决方案 ---
def setup_chinese_font():
    # 尝试设置常用的中文字体
    fonts = ['SimHei', 'Microsoft YaHei', 'PingFang SC', 'Arial Unicode MS']
    for font in fonts:
        try:
            mpl.rcParams['font.sans-serif'] = [font]
            mpl.rcParams['axes.unicode_minus'] = False
            # 尝试渲染一个标题测试
            plt.figure()
            plt.close()
            return font
        except:
            continue
    return None

used_font = setup_chinese_font()

# --- 2. ZPW-2000A 核心参数 ---
FS = 10000          # 采样率 (Hz)
DURATION = 10.0     # 信号总时长 (s) (进一步增加时长以提高FFT分辨率)
FC = 2300           # 设定载频 (Hz): 1700, 2000, 2300, 2600
FM = 14.7           # 设定低频调制 (Hz): 10.3Hz - 29Hz
DELTA_F = 11        # 移频常数 (Hz)

# --- 3. 信号产生模块 ---
def generate_zpw2000a(fc, fm, df, duration, fs):
    t = np.linspace(0, duration, int(fs * duration), endpoint=False)
    # 产生连续相位移频信号 (CPFSK)
    # 调制信号为方波，控制频率在 fc+11 和 fc-11 之间切换
    mod_signal = np.sign(np.sin(2 * np.pi * fm * t))
    inst_freq = fc + mod_signal * df
    # 相位是频率的积分
    phase = 2 * np.pi * np.cumsum(inst_freq) / fs
    signal = np.cos(phase)
    return t, signal, mod_signal

# --- 4. 信号解析模块 ---
def decode_zpw2000a(signal, fs):
    # a. 提取载频 (通过功率谱密度或FFT)
    fft_vals = np.abs(np.fft.rfft(signal))
    freqs = np.fft.rfftfreq(len(signal), 1/fs)
    detected_fc = freqs[np.argmax(fft_vals)]
    
    # b. 提取包络线 (希尔伯特变换)
    analytic_signal = hilbert(signal)
    envelope = np.abs(analytic_signal)
    
    # c. 频率解调 (瞬时频率计算)
    # 计算瞬时相位
    instantaneous_phase = np.unwrap(np.angle(analytic_signal))
    # 频率是相位的导数: f = (1/2pi) * d(phi)/dt
    inst_freq_seq = np.diff(instantaneous_phase) / (2.0 * np.pi) * fs
    
    # d. 提取低频调制数据
    # 对解调出的瞬时频率序列进行FFT，找到跳变的周期
    fm_raw = inst_freq_seq - np.mean(inst_freq_seq)
    fm_fft = np.abs(np.fft.rfft(fm_raw))
    fm_freqs = np.fft.rfftfreq(len(fm_raw), 1/fs)
    
    # 锁定低频范围 (10Hz - 30Hz)
    mask = (fm_freqs > 5) & (fm_freqs < 40)
    
    # 使用频谱重心法提高精度
    idx = np.argmax(fm_fft[mask])
    
    # 提取峰值及其相邻点
    # 注意：这里的索引是 mask 后的索引，需要转换回原 fm_fft 索引
    full_indices = np.where(mask)[0]
    peak_idx = full_indices[idx]
    
    if 0 < peak_idx < len(fm_fft) - 1:
        # 频谱重心法 (Center of Gravity)
        y = fm_fft[peak_idx-1 : peak_idx+2]
        x = fm_freqs[peak_idx-1 : peak_idx+2]
        detected_fm = np.sum(x * y) / np.sum(y)
    else:
        detected_fm = fm_freqs[peak_idx]
    
    return detected_fc, detected_fm, envelope, inst_freq_seq

# --- 5. 执行与可视化 ---
# 生成信号
t, sig, original_mod = generate_zpw2000a(FC, FM, DELTA_F, DURATION, FS)

# 解析信号
dec_fc, dec_fm, env, inst_f = decode_zpw2000a(sig, FS)

print(f"--- 仿真结果 ---")
print(f"【发送端】载频: {FC} Hz, 低频调制: {FM} Hz")
print(f"【接收端】还原载频: {dec_fc:.1f} Hz, 还原低频: {dec_fm:.2f} Hz")

# 绘图
fig, axes = plt.subplots(3, 1, figsize=(12, 10))

# 图1: 时域信号与包络
axes[0].plot(t[:1000], sig[:1000], color='silver', label='高频信号')
axes[0].plot(t[:1000], env[:1000], color='red', linewidth=2, label='包络线')
axes[0].set_title(f"1. ZPW-2000A 原始信号与包络 (局部)\n载频:{FC}Hz, 偏频:±{DELTA_F}Hz")
axes[0].legend(loc='upper right')
axes[0].set_ylabel("幅度")

# 图2: 频率解调过程
# 注意：diff操作导致长度少1，绘图时对应t[1:]
axes[1].plot(t[1:5000], inst_f[:4999], color='blue', label='解调瞬时频率')
axes[1].axhline(y=FC+DELTA_F, color='orange', linestyle='--', label='上偏频')
axes[1].axhline(y=FC-DELTA_F, color='green', linestyle='--', label='下偏频')
axes[1].set_title("2. 频率解调 (瞬时频率随时间跳变曲线)")
axes[1].set_ylabel("频率 (Hz)")
axes[1].legend(loc='upper right')

# 图3: 低频频谱解析
axes[2].set_title("3. 低频调制数据解析 (对频率跳变序列做FFT)")
# 只显示0-50Hz范围以便观察低频
f_mask = (np.fft.rfftfreq(len(inst_f)-1, 1/FS) < 50)
freq_axis = np.fft.rfftfreq(len(inst_f)-1, 1/FS)[f_mask]
fft_axis = np.abs(np.fft.rfft(inst_f - np.mean(inst_f)))[f_mask]
axes[2].plot(freq_axis, fft_axis, color='purple')
axes[2].axvline(x=dec_fm, color='red', alpha=0.5, linestyle=':')
axes[2].text(dec_fm + 1, np.max(fft_axis)*0.8, f'识别低频: {dec_fm:.2f}Hz', color='red')
axes[2].set_xlabel("频率 (Hz)")
axes[2].set_ylabel("能量")

plt.tight_layout()
plt.show()
