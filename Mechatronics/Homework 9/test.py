import numpy as np
import matplotlib.pyplot as plt

# --- 1. 定义时间段 ---
# 第一段：0 到 0.1s
t1 = np.linspace(0, 0.1, 500)
# 第二段：0.1 到 0.2s
t2 = np.linspace(0.1, 0.2, 500)

# --- 2. 定义函数表达式 ---
# User Request Part 1: 5 * sin(10*pi*t)
y1 = 5 * np.sin(10 * np.pi * t1)

# User Request Part 2: 5 * e^-(10t - 0.5)
# 注意：这个公式其实等价于 5 * e^(-(t - 0.05)/0.1)
# 意味着它描述的是从 t=0.05s 开始的衰减
y2 = 5 * np.exp(-(10 * t2 - 0.5))

# --- 3. 绘图检验 ---
plt.figure(figsize=(10, 6))

# 画第一段 (蓝色)
plt.plot(t1, y1, 'b-', linewidth=2, label=r'0-0.1s: $5\sin(10\pi t)$')

# 画第二段 (红色)
plt.plot(t2, y2, 'r-', linewidth=2, label=r'0.1-0.2s: $5e^{-(10t-0.5)}$')

# 为了对比，画出真实的物理修正曲线（虚线）
# 真实情况：在 t=0.05s (峰值) 就应该切换到指数衰减
t_correct = np.linspace(0.05, 0.2, 500)
y_correct = 5 * np.exp(-(10 * t_correct - 0.5))
plt.plot(t_correct, y_correct, 'g--', alpha=0.6, label='Correct Physics Path (Switch at 0.05s)')

# --- 4. 标注与美化 ---
plt.title('Waveform Verification')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

# 标出 t=0.1s 处的断点
plt.scatter([0.1], [0], color='blue', s=50, zorder=5)       # 正弦波终点 (0V)
plt.scatter([0.1], [y2[0]], color='red', s=50, zorder=5)    # 指数波起点 (~3.03V)
plt.annotate('Discontinuity!\n(Gap)', xy=(0.1, 1.5), xytext=(0.12, 1.5),
             arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=.2'))

plt.show()