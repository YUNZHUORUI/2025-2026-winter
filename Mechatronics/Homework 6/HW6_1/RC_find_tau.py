import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

dt = 5e-6
# 读取和处理数据
data = pd.read_csv("/Users/yunzhuorui/Arduino/Homework 6/HW6_1/10000u.csv")
time_start = 219
time_last = 500
temp = data['Sequence'].to_numpy()[time_start:time_start+time_last]*dt
t = temp - temp[0]
voltage = data['Volt'].to_numpy()[time_start:time_start+time_last]

# 充电过程：电压从低到高
V_initial = np.min(voltage)  # 充电起始电压
V_final = np.max(voltage)    # 充电结束电压

print(f"充电过程检测:")
print(f"起始电压: {V_initial:.3f} V")
print(f"最终电压: {V_final:.3f} V")

# 方法1：63.2%法（充电过程）
# 充电公式: V(t) = V_initial + (V_final - V_initial) * (1 - exp(-t/τ))
# 当达到63.2%的充电量时：V(t) = V_initial + 0.632*(V_final - V_initial)
V_target = V_initial + 0.632 * (V_final - V_initial)

# 找到最接近目标电压的时间点
idx = np.argmin(np.abs(voltage - V_target))
tau_63 = t[idx]

print(f"\n方法1 - 63.2%法 (充电过程):")
print(f"目标电压: {V_target:.3f} V (63.2%充电点)")
print(f"实验时间常数: {tau_63*1000:.4f} ms")