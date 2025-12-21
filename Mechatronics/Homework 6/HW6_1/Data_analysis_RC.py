import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- 1. 电路参数（RC 电路）---
R_int = 30  # Arduino内部电阻
R_pot = 100  # 假设的电位器电阻值 (请根据你的实验数据调整)
R_total = R_int + R_pot  # RC电路的总电阻
C = 1e-8  # 电容值 1 uF

# RLC相关的参数L在这里被忽略或设置为0，但我们使用一阶RC公式
# L = 2.5e-3 # 移除电感 L

# 时间常数 tau
tau = R_total * C

# 数据读取和处理参数
dt = 1e-7  # 根据你的数据采集频率设置
file_name = "100u.csv" # 假设你的文件名为10kohm.csv
time_start = 260 # 假设的起始点
time_last = 500  # 采样点数

# --- 2. 数据读取 ---
try:
    data = pd.read_csv(f"/Users/yunzhuorui/Arduino/Homework 6/HW6_1/{file_name}")
except FileNotFoundError:
    print(f"Error: File {file_name} not found. Please check the path.")
    exit()

# 获取时间序列和电压序列
temp = data['Sequence'].to_numpy()[time_start:time_start+time_last]*dt
t = temp - temp[0]
voltage = data['Volt'].to_numpy()[time_start:time_start+time_last]

# --- 3. 理论模型（RC 一阶系统）---
print(f"RC 电路时间常数 tau = {tau*1e6:.2f} us")

# 充电响应公式: Vc(t) = V_final * (1 - exp(-t/tau))
V_final = 5.0  # 最终稳态电压
response = V_final * (1 - np.exp(-t / tau))


# --- 4. 绘图 ---
plt.figure(figsize=[7, 5])
plt.plot(t*1000, voltage, 'b', label='Experimental')
plt.plot(t*1000, response, 'r--', label=f'RC Model ($\ tau={tau*1e6:.2f} \mu s$)')

plt.xlabel('time [ms]')
plt.ylabel('voltage [V]')

# 设置合理的纵轴范围，通常从0到5V多一点
plt.ylim([-0.3, 5.5])

# 为了清晰观察，我们可能只需要绘制前几个tau的时间
# 1003 us = 1.003 ms，绘制 5 ms 足够观察5个tau
#plt.xlim([0, 5])

plt.title(f'RC response (R={R_total} $\Omega$)')
plt.legend(loc='lower right')
plt.grid(True)
plt.tight_layout()

# 保存文件，使用RC和电阻值命名
plt.savefig(f'RC_{int(R_pot)}ohm_response.pdf')
plt.show()
