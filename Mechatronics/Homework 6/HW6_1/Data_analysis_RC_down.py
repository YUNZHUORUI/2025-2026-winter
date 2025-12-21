import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- 1. 电路参数（RC 电路）---
R_int = 30  # Arduino内部电阻
R_pot = 10000  # 假设的电位器电阻值 (请根据你的实验数据调整)
R_total = R_int + R_pot  # RC电路的总电阻
C = 0.1e-7  # 电容值 0.1 uF

# 时间常数 tau
tau = R_total * C

# 数据读取和处理参数
dt = 2e-6  # 根据你的数据采集频率设置

# 假设放电数据的文件和起始点
file_name = "10000d.csv"
discharge_start = 545  # 假设这是输入信号从HIGH变LOW的时刻（放电开始）
time_last = 600  # 采样点数

# --- 2. 数据读取 ---
try:
    data = pd.read_csv(f"/Users/yunzhuorui/Arduino/Homework 6/HW6_1/{file_name}")
except FileNotFoundError:
    print(f"Error: File {file_name} not found. Please check the path.")
    exit()

# 获取时间序列和电压序列，使用放电的起始点
temp = data['Sequence'].to_numpy()[discharge_start:discharge_start + time_last] * dt
t = temp - temp[0]
voltage = data['Volt'].to_numpy()[discharge_start:discharge_start + time_last]

# 确定放电起始时的初始电压 V_initial
# 由于电容可能没有充满，我们使用数据中的第一个电压值作为 V_initial
if len(voltage) > 0:
    V_initial = voltage[0]
else:
    V_initial = 5.0  # 如果数据有问题，假设从5V开始放电

print(f"RC 电路时间常数 tau = {tau * 1e6:.2f} us")
print(f"放电初始电压 V_initial = {V_initial:.2f} V")

# --- 3. 理论模型（RC 一阶系统 - 放电）---
# 放电响应公式: Vc(t) = V_initial * exp(-t/tau)
response = V_initial * np.exp(-t / tau)

# --- 4. 绘图 ---
plt.figure(figsize=[7, 5])
plt.plot(t * 1000, voltage, 'b', label='Experimental (Exp.)')
plt.plot(t * 1000, response, 'r--', label=f'RC Discharge model ($\tau={tau * 1e6:.2f} \mu s$)')

plt.xlabel('Time [ms]')
plt.ylabel('Voltage [V]')

# 设置合理的纵轴范围
plt.ylim([-0.3, V_initial + 0.5])

# 设置合理的横轴范围（如果 tau 很大，需要放大）
# 例如，我们绘制 5 个 tau 的时间长度
plot_limit_ms = (5 * tau) * 1000
plt.xlim([0, plot_limit_ms])

plt.title(f'RC discharge response (R={R_total} $\Omega$)')
plt.legend(loc='upper right')
plt.grid(True)
plt.tight_layout()

# 保存文件
plt.savefig(f'RC_discharge_{int(R_pot)}ohm_response.pdf')
plt.show()
