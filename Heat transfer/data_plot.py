import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. 读取实验数据 (Your Data)
# ==========================================
df_exp = pd.read_csv('Lab_data.csv')

# 计算温差 (X轴)
df_exp['delta_T'] = df_exp['T_heater'] - df_exp['T_liquid']
df_ref = pd.read_csv('Reference Dataset.csv', header=None, names=['delta_T_ref', 'Y_ref'])

plt.figure(figsize=(10, 7))

# 画你的实验数据 (蓝色圆点)
plt.scatter(df_exp['delta_T'], df_exp['Power'],
            color='blue', marker='o', s=50, label='Experimental Data')

# 画参考数据 (红色虚线 或 方块)
# 如果参考点很密集，可以用 plot 画线；如果稀疏，用 scatter 画点
plt.scatter(df_ref['delta_T_ref'], df_ref['Y_ref'],
         color='red', marker='o', s=50, edgecolor='black', linewidth=1 ,label='Reference Data (Moodle)')
# 如果只想画点，把上面这行换成:
# plt.scatter(df_ref['delta_T_ref'], df_ref['Y_ref'], color='red', marker='s', label='Reference Data')

# ==========================================
# 5. 格式美化
# ==========================================
plt.title('Pool Boiling Curve: Comparison', fontsize=14)
plt.xlabel(r'Excess Temperature $\Delta T_e = T_{wall} - T_{sat}$ ($^\circ C$)', fontsize=12)
plt.ylabel('Power (W)', fontsize=12) # 如果换成了热通量，记得改成 Heat Flux (W/m^2)

plt.grid(True, which="both", ls="--", alpha=0.5)
plt.legend(loc='best', fontsize=12)

# 可选：使用对数坐标 (Log Scale)
# 沸腾曲线通常在对数坐标下看得很清楚 (Log-Log)
# 如果需要，取消下面两行的注释:
# plt.xscale('log')
# plt.yscale('log')

plt.tight_layout()
plt.savefig('Boiling_Curve_Comparison.png', dpi=300)
plt.show()