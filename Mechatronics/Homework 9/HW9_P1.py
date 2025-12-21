from numpy import pi, linalg, linspace, zeros_like, sin, exp
import matplotlib.pyplot as plt

R_L = 1e6        # 负载电阻 1 MΩ
C = 0.1e-6       # 滤波电容 0.1 μF
tau = R_L * C    # 时间常数 = 0.1 s
V_peak = 5       # 峰值电压 5V
omega = 10 * pi
T = 2* pi / omega

# --- 2. 时间设置 (0 到 0.4s) ---
t = linspace(0, 0.4, 1000)  # 生成时间轴，1000个点
dt = t[1] - t[0]

# --- 3. 生成输入电压波形 ---
v_s = V_peak * sin(omega * t)

# --- 4. 模拟输出电压 v_L (逐点计算) ---
v_l = zeros_like(t)
v_l[0] = max(0, v_s[0])  # 初始条件

for i in range(1, len(t)):
    # 计算如果二极管截止，电容自然放电后的电压
    v_decay = v_l[i-1] * exp(-dt / tau)
    
    # 判断二极管状态
    # 如果输入电压 > 电容当前电压，二极管导通 (Ideal Diode)
    if v_s[i] > v_decay:
        v_l[i] = v_s[i]  # 充电：输出跟随输入
    else:
        v_l[i] = v_decay # 截止：输出按指数衰减

# --- 5. 绘图 ---
plt.figure(figsize=(10, 6))

# 画输入电压 (虚线)
plt.plot(t, v_s, 'b--', label='$v_S(t)$ Input', alpha=0.5)

# 画输出电压 (实线)
plt.plot(t, v_l, 'r-', label='$v_L(t)$ Load Voltage', linewidth=2)

plt.title(f'Voltage on Load Resistor $R_L$ (Time Domain)\n$C=0.1\mu F, R_L=1M\Omega$')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.axhline(0, color='black', linewidth=0.5)

# 显示
plt.show()