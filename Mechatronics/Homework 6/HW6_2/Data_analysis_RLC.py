import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 更新电阻值
R = 130 # 1030Ω
C = 1e-7
L = 2.5e-3
dt = 5e-7

# 数据读取（修复警告）
data = pd.read_csv("/Users/yunzhuorui/Arduino/Homework 6/HW6_2/10.csv")
time_start = 440
time_last = 500

# 使用 to_numpy() 替代 ravel()
temp = data['Sequence'].to_numpy()[time_start:time_start+time_last]*dt
t = temp - temp[0]
voltage = data['Volt'].to_numpy()[time_start:time_start+time_last]

# 系统参数
omega_n = 1/np.sqrt(L*C)
zeta = R/2/L*np.sqrt(L*C)

print(f"阻尼比 zeta = {zeta}")  # 调试信息

# 根据阻尼比选择正确的响应公式
if zeta < 1:
    # 欠阻尼情况
    omega_d = omega_n * np.sqrt(1 - zeta**2)
    phi = np.arctan(np.sqrt(1 - zeta**2)/zeta)
    response = (5 - 5/np.sqrt(1 - zeta**2) * np.exp(-zeta*omega_n*t)
               * np.sin(omega_d*t + phi))
elif zeta == 1:
    # 临界阻尼情况
    response = 5 * (1 - (1 + omega_n*t) * np.exp(-omega_n*t))
else:
    # 过阻尼情况 (zeta > 1)
    s1 = -zeta*omega_n + omega_n*np.sqrt(zeta**2 - 1)
    s2 = -zeta*omega_n - omega_n*np.sqrt(zeta**2 - 1)
    response = 5 * (1 - (s2*np.exp(s1*t) - s1*np.exp(s2*t))/(s2 - s1))

# 绘图
plt.figure(figsize=[5, 3])
plt.plot(t*1000, voltage, 'b', label='Exp.')
plt.plot(t*1000, response, 'r--', label='Model')
plt.xlabel('Time [ms]')
plt.ylabel('Voltage [V]')
plt.ylim([-0.3, 8])
plt.legend(loc='upper right')
plt.tight_layout()
plt.savefig('10ohm.pdf')
plt.show()