import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------
# 1. 输入数据
# ----------------------------------------

# 激励频率 f (Hz)
freq = np.array([25, 50.5, 75.2, 100, 125, 151, 175])
# 相位差 V2 相对于 V1 (度)
phase_diff = np.array([71.82, 85.45, 106.9, 106.2, 114.7, 118.2, 125.9])
# 幅值比 |V2|/|V1|
amplitude_ratio = np.array([7.47, 3.91, 2.35, 1.4, 1.03, 0.8, 0.676])

# ----------------------------------------
# 2. 绘图
# ----------------------------------------

# 创建一个包含两个子图的 Figure 对象
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
#fig.suptitle('电路频率响应分析 (V2 相对于 V1)', fontsize=16)

# --- 2.1 幅频特性 (Amplitude Response) ---
# Y 轴：幅值比 |V2|/|V1|
ax1.plot(freq, amplitude_ratio, marker='o', linestyle='-', color='blue', label='$|V_2| / |V_1|$')
ax1.set_ylabel('amplitude ratio $|V_2| / |V_1|$', fontsize=12)
ax1.set_title('amplitude vs frequency', fontsize=14)
ax1.grid(True, which='both', linestyle='--', alpha=0.7)
ax1.legend(loc='upper right')

# (可选) 如果幅值变化范围大，可以使用对数 y 轴
# ax1.set_yscale('log')
# ax1.set_ylabel('幅值比 $|V_2| / |V_1|$ (对数)', fontsize=12)


# --- 2.2 相频特性 (Phase Response) ---
# Y 轴：相位差 (度)
ax2.plot(freq, phase_diff, marker='s', linestyle='-', color='red', label='Phase Difference $\\Delta\\phi$')
ax2.set_xlabel('frequency $f$ (Hz)', fontsize=12)
ax2.set_ylabel('phase differ  (deg)', fontsize=12)
ax2.set_title('phase vs frequency', fontsize=14)
ax2.grid(True, which='both', linestyle='--', alpha=0.7)
ax2.legend(loc='lower right')

# 设置 X 轴刻度更易读
ax2.set_xticks(freq)
ax2.tick_params(axis='x', rotation=45)

# 自动调整布局，防止标签重叠
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
