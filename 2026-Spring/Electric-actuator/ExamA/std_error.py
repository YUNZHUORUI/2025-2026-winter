import numpy as np

# ============================================================
# 参数定义
# ============================================================
# 均值
K1_mean = 4        # kΩ
y_mean  = 400      # kPa
K2_mean = 2        # V
b1_mean = 0.5      # V
K3_mean = 80       # kPa/V
b2_mean = -50      # kPa

# 标准差
sigma_K1 = 0.3e-3
sigma_y  = 2
sigma_K2 = 0.05
sigma_b1 = 0.02
sigma_K3 = 2
sigma_b2 = 1.5

# ============================================================
# 中间函数定义
# ============================================================
def calc_Rp(K1, y, p):
    return K1 * np.exp(y / p)

def calc_VA(K2, b1, Rp):
    return K2 * (2 / (1 + 2.5 / Rp)) - b1

def calc_PM(K3, b2, VA):
    return K3 * VA + b2

# ============================================================
# a) 均值计算 p = 200 kPa
# ============================================================
p_a = 200  # kPa

Rp_mean = calc_Rp(K1_mean, y_mean, p_a)
VA_mean = calc_VA(K2_mean, b1_mean, Rp_mean)
PM_mean = calc_PM(K3_mean, b2_mean, VA_mean)
error_mean = PM_mean - p_a

print("=" * 50)
print(f"a) p = {p_a} kPa")
print(f"   Rp     = {Rp_mean:.4f} kΩ")
print(f"   VA     = {VA_mean:.4f} V")
print(f"   PM     = {PM_mean:.4f} kPa")
print(f"   误差均值 = {error_mean:.4f} kPa")

# ============================================================
# b) 标准差计算 p = 50 kPa（数值偏导）
# ============================================================
p_b = 50  # kPa
h = 1e-6  # 微小扰动，用于数值求导

# 先算均值处的 Rp, VA, PM
Rp0 = calc_Rp(K1_mean, y_mean, p_b)
VA0 = calc_VA(K2_mean, b1_mean, Rp0)
PM0 = calc_PM(K3_mean, b2_mean, VA0)

print("\n" + "=" * 50)
print(f"b) p = {p_b} kPa")
print(f"   Rp0 = {Rp0:.2e} kΩ")
print(f"   VA0 = {VA0:.4f} V")
print(f"   PM0 = {PM0:.4f} kPa")

# ---- 数值偏导：对每个变量扰动 ----
def PM_full(K1, y, K2, b1, K3, b2, p):
    Rp = calc_Rp(K1, y, p)
    VA = calc_VA(K2, b1, Rp)
    return calc_PM(K3, b2, VA)

# 用中心差分求偏导
def partial(var_name, h=1e-6):
    args = dict(K1=K1_mean, y=y_mean, K2=K2_mean,
                b1=b1_mean, K3=K3_mean, b2=b2_mean, p=p_b)
    args_plus  = dict(args); args_plus[var_name]  += h
    args_minus = dict(args); args_minus[var_name] -= h
    return (PM_full(**args_plus) - PM_full(**args_minus)) / (2*h)

dPM_dK1 = partial('K1')
dPM_dy  = partial('y')
dPM_dK2 = partial('K2')
dPM_db1 = partial('b1')
dPM_dK3 = partial('K3')
dPM_db2 = partial('b2')

print("\n   各偏导数：")
print(f"   ∂PM/∂K1 = {dPM_dK1:.4f}")
print(f"   ∂PM/∂y  = {dPM_dy:.4f}")
print(f"   ∂PM/∂K2 = {dPM_dK2:.4f}")
print(f"   ∂PM/∂b1 = {dPM_db1:.4f}")
print(f"   ∂PM/∂K3 = {dPM_dK3:.4f}")
print(f"   ∂PM/∂b2 = {dPM_db2:.4f}")

# ---- 误差传播 ----
contributions = {
    'K1': (dPM_dK1 * sigma_K1)**2,
    'y' : (dPM_dy  * sigma_y )**2,
    'K2': (dPM_dK2 * sigma_K2)**2,
    'b1': (dPM_db1 * sigma_b1)**2,
    'K3': (dPM_dK3 * sigma_K3)**2,
    'b2': (dPM_db2 * sigma_b2)**2,
}

variance = sum(contributions.values())
sigma_PM = np.sqrt(variance)

print("\n   各变量方差贡献：")
for name, val in contributions.items():
    print(f"   {name}: {val:.4f} kPa²  ({100*val/variance:.1f}%)")

print(f"\n   总方差   = {variance:.4f} kPa²")
print(f"   标准差   = {sigma_PM:.4f} kPa")
print(f"   误差均值 = {PM0 - p_b:.4f} kPa")