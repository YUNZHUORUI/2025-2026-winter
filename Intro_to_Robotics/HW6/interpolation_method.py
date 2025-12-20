import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import imageio
import math

# ==============================================================
# 1. ROBOT PARAMETERS
# ==============================================================
L1 = 1.2
L2 = 0.8

A = np.array([-1.2, 0.9])
B = np.array([1.1, 0.8])

vmax = 1.5  # rad/s (原始最大速度限制)
amax = 3.0  # rad/s^2 (最大加速度限制)

# 离散时间步长
dt = 0.01


# ==============================================================
# 2. INVERSE KINEMATICS
# ==============================================================
def IK(x, y):
    """
    计算二连杆机械臂的逆运动学。
    选择满足条件 (x, y) 且 |theta1| 较小的解。
    """
    D = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    D = np.clip(D, -1, 1)  # 防止浮点数误差导致超出 [-1, 1]

    # theta2 的两个解
    theta2_1 = np.arctan2(np.sqrt(1 - D * D), D)
    theta2_2 = np.arctan2(-np.sqrt(1 - D * D), D)

    def solve(theta2):
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        return theta1, theta2

    sol1 = solve(theta2_1)
    sol2 = solve(theta2_2)

    # 挑出 |theta1| 更小的解
    return sol1 if abs(sol1[0]) < abs(sol2[0]) else sol2


theta1_A, theta2_A = IK(A[0], A[1])
theta1_B, theta2_B = IK(B[0], B[1])

print(f"--- 机械臂初始/目标角度 ---")
print(f"初始角度 (A): \u03B81={theta1_A:.4f}, \u03B82={theta2_A:.4f} (rad)")
print(f"目标角度 (B): \u03B81={theta1_B:.4f}, \u03B82={theta2_B:.4f} (rad)")


# ==============================================================
# 3. TIME-OPTIMAL TRAJECTORY (原始函数，用于计算最小时间)
# ==============================================================
def trapezoidal_traj(q0, qf, vmax, amax):
    """
    计算在给定 vmax 和 amax 下，从 q0 到 qf 的时间最优梯形轨迹的总时间。
    """
    dq = abs(qf - q0)

    # 纯加速/减速所需的距离 (三角形剖面)
    d_acc = vmax ** 2 / amax

    if dq < d_acc:
        # 三角形速度剖面：无法达到 vmax
        T = 2 * np.sqrt(dq / amax)
    else:
        # 梯形速度剖面：达到 vmax
        t_acc = vmax / amax
        d_flat = dq - d_acc
        t_flat = d_flat / vmax
        T = 2 * t_acc + t_flat

    return T


# 独立计算两个关节所需的最短时间
T1_min = trapezoidal_traj(theta1_A, theta1_B, vmax, amax)
T2_min = trapezoidal_traj(theta2_A, theta2_B, vmax, amax)

# 确定最长的运动时间 T，这是同步时间
T_sync = max(T1_min, T2_min)

print(f"--- 轨迹规划结果 ---")
print(f"\u03B81 最小时间 (T1_min): {T1_min:.4f} s")
print(f"\u03B82 最小时间 (T2_min): {T2_min:.4f} s")
print(f"同步总时间 (T_sync): {T_sync:.4f} s")


# ==============================================================
# 4. TIME-SYNCHRONIZED TRAJECTORY (同步函数)
# ==============================================================
def sync_trapezoidal_traj(q0, qf, T, amax, dt):
    """
    在给定的总时间 T (T >= T_min) 和加速度 amax 下，生成同步轨迹。
    """
    dq = qf - q0
    sgn = np.sign(dq)
    dq_abs = abs(dq)

    # 1. 计算新的加速时间 t_acc 和虚拟最大速度 v_prime

    # 从运动距离公式 dq_abs = 2 * (1/2 * amax * t_acc^2) + v_prime * t_flat
    # 和时间公式 T = 2 * t_acc + t_flat
    # 导出关于 t_acc 的二次方程: amax * t_acc^2 - amax * T * t_acc + dq_abs = 0

    # 二次方程判别式： Delta = (a*T)^2 - 4*a*dq_abs
    delta = (amax * T) ** 2 - 4 * amax * dq_abs

    if delta < 0:
        # 理论上不应该发生，除非输入的 T 比实际最短时间 T_min_acc 还短
        t_acc = T / 2.0
    else:
        # 选择较小的解 (对应于最快加速/减速，使梯形底边最长)
        t_acc = (amax * T - np.sqrt(delta)) / (2 * amax)

    # 检查是否为三角形剖面 (t_acc > T/2)
    if t_acc > T / 2.0:
        # 如果方程解出的 t_acc 超过 T/2, 则说明应该用三角形剖面 (T_min_acc)
        # 此时应该使用 T_min_acc 作为总时间，或者强制 t_acc = T/2
        t_acc = T / 2.0

    v_prime = amax * t_acc
    t_flat = T - 2 * t_acc

    # 2. 生成轨迹
    t = np.arange(0, T, dt)
    # 确保最后一个点是 T，便于插值
    if t[-1] < T:
        t = np.append(t, T)

    q = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti <= t_acc:  # 加速段
            q[i] = q0 + sgn * 0.5 * amax * ti ** 2
        elif ti <= t_acc + t_flat:  # 匀速段
            q[i] = q0 + sgn * (0.5 * amax * t_acc ** 2 + v_prime * (ti - t_acc))
        else:  # 减速段
            # 使用对称性: 从 qf 往回计算
            t_remaining = T - ti
            q[i] = qf - sgn * 0.5 * amax * t_remaining ** 2

    # 确保起点和终点准确
    q[0] = q0
    q[-1] = qf

    return t, q, T, v_prime


# 重新计算同步轨迹
t, q1, T, v1_prime = sync_trapezoidal_traj(theta1_A, theta1_B, T_sync, amax, dt)
t, q2, T, v2_prime = sync_trapezoidal_traj(theta2_A, theta2_B, T_sync, amax, dt)

print(f"\u03B81 新最大速度 (v1'): {v1_prime:.4f} rad/s (原 v_max={vmax} rad/s)")
print(f"\u03B82 新最大速度 (v2'): {v2_prime:.4f} rad/s (原 v_max={vmax} rad/s)")
print("两个关节现在将在相同的时间 T 内完成运动，且保持梯形速度剖面。")

# ==============================================================
# 5. Plot θ1, θ2 vs time
# ==============================================================
# 计算角速度
# 使用数值微分计算角速度
dt = t[1] - t[0]  # 时间步长
dq1_dt = np.gradient(q1, dt)  # 关节1角速度
dq2_dt = np.gradient(q2, dt)  # 关节2角速度

# 创建包含两个子图的图形
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# 第一个子图：角度变化
ax1.plot(t, q1, label=r'$\theta_1$', color='blue', linewidth=2)
ax1.plot(t, q2, label=r'$\theta_2$', color='red', linewidth=2)
ax1.set_ylabel("Angle (rad)")
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_title("Joint Angles vs Time (Synchronized Trajectory)")

# 第二个子图：角速度变化
# 绘制速度曲线
ax2.plot(t, dq1_dt, label=r'$\dot{\theta}_1$', color='blue', linestyle='-', linewidth=2)
ax2.plot(t, dq2_dt, label=r'$\dot{\theta}_2$', color='red', linestyle='-', linewidth=2)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Angular Velocity (rad/s)")
ax2.grid(True, alpha=0.3)
ax2.set_title("Joint Angular Velocities vs Time (Synchronized Trajectory)")

# 在角速度图上添加原始和同步后的最大速度限制线
ax2.axhline(y=vmax, color='green', linestyle=':', alpha=0.7, label=f'Original $v_{{max}}$ = {vmax} rad/s')
ax2.axhline(y=-vmax, color='green', linestyle=':', alpha=0.7)

ax2.axhline(y=v1_prime, color='blue', linestyle='--', alpha=0.7, label=f'$v\'_{{max,1}}$ = {v1_prime:.3f} rad/s')
ax2.axhline(y=-v1_prime, color='blue', linestyle='--', alpha=0.7)
ax2.axhline(y=v2_prime, color='red', linestyle='--', alpha=0.7, label=f'$v\'_{{max,2}}$ = {v2_prime:.3f} rad/s')
ax2.axhline(y=-v2_prime, color='red', linestyle='--', alpha=0.7)

ax2.legend(loc='lower left')

plt.tight_layout()
plt.savefig('synchronous_trajectory_curve.pdf')
plt.show()

# ==============================================================
# 6. Animation with imageio saving
# ==============================================================

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-2.2, 2.2)
ax.set_ylim(-0.5, 2.2)
ax.set_aspect("equal")
ax.grid()
ax.set_title("Synchronized 2-DOF Robot Motion")

# 绘制初始点和目标点
ax.plot(A[0], A[1], 'o', color='purple', markersize=8, label='Start Point (A)')
ax.plot(B[0], B[1], 'x', color='green', markersize=8, label='End Point (B)')
ax.legend(loc='upper right')

line, = ax.plot([], [], 'o-', lw=3, color='blue', label='Robot Arm')
time_text = ax.text(-2.0, 2.0, "", fontsize=12)

frames = []

for i in range(len(t)):
    th1 = q1[i]
    th2 = q2[i]

    # 正运动学
    x1 = L1 * np.cos(th1)
    y1 = L1 * np.sin(th1)

    x2 = x1 + L2 * np.cos(th1 + th2)
    y2 = y1 + L2 * np.sin(th1 + th2)

    # 设置机械臂关节位置
    line.set_data([0, x1, x2], [0, y1, y2])

    # 绘制关节 O, J1, J2 (连杆末端)
    ax.plot([0, x1, x2], [0, y1, y2], 'o', color='red', markersize=5)

    time_text.set_text(f"Time = {t[i]:.2f} s / {T:.2f} s")

    fig.canvas.draw()

    # ---- 捕获帧 ----
    rgba = np.asarray(fig.canvas.buffer_rgba()).copy()
    rgb = rgba[:, :, :3]
    frames.append(rgb)

plt.close(fig)

# Save GIF
# 帧率使用 1/dt
imageio.mimsave("robot_motion_synchronized.gif", frames, fps=int(1 / dt))
print("\nSaved plot to synchronous_trajectory_curve.pdf")
print("Saved animation to robot_motion_synchronized.gif")