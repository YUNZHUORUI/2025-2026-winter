import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 机械臂参数
L1 = 1.2  # 第一连杆长度
L2 = 0.8  # 第二连杆长度
m = 1.0  # 末端质量
g = 9.81  # 重力加速度
t_max = 0.7  # 仿真时间
dt = 0.005  # 减小时间步长以提高精度和流畅性
frames_per_sec = 50  # 提高帧率
 
# 初始关节角度 (度转弧度)
theta1_0 = np.radians(45)
theta2_0 = np.radians(80)

# 计算初始末端位置
x0 = L1 * np.cos(theta1_0) + L2 * np.cos(theta1_0 + theta2_0)
y0 = L1 * np.sin(theta1_0) + L2 * np.sin(theta1_0 + theta2_0)

print(f"初始位置: x={x0:.3f}, y={y0:.3f}")

# 全局变量：存储上次选择的解索引 (1 或 2)
# 这对于奇异点处的解稳定至关重要。
last_solution_index = 1
SINGULARITY_TOLERANCE = 0.05  # 奇异点切换的容忍度


def angle_diff(a1, a2):
    """计算两个角度之间的最小差异，考虑环绕 [-pi, pi]"""
    diff = a1 - a2
    return (diff + np.pi) % (2 * np.pi) - np.pi


def inverse_kinematics_continuous(x, y, prev_theta1, prev_theta2):
    """
    连续 IK 求解器，优化了奇异点切换逻辑。
    """
    global last_solution_index

    d_sq = x ** 2 + y ** 2

    # 距离约束检查
    R_max_sq = (L1 + L2) ** 2
    R_min_sq = (L1 - L2) ** 2
    if d_sq > R_max_sq + 1e-6 or d_sq < R_min_sq - 1e-6:
        return prev_theta1, prev_theta2

    # 肘关节角 (theta2)
    cos_theta2 = (d_sq - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2_base = np.arccos(cos_theta2)

    theta2_p = theta2_base
    theta2_n = -theta2_base

    beta = np.arctan2(y, x)

    def calc_theta1(t2):
        phi = np.arctan2(L2 * np.sin(t2), L1 + L2 * np.cos(t2))
        return beta - phi

    sol1 = (calc_theta1(theta2_p), theta2_p)  # 肘朝上解 (theta2 正)
    sol2 = (calc_theta1(theta2_n), theta2_n)  # 肘朝下解 (theta2 负)

    def find_nearest_angle(new_angle, prev_angle):
        """找到最接近 prev_angle 的 new_angle + 2*n*pi"""
        n = np.round((prev_angle - new_angle) / (2 * np.pi))
        return new_angle + n * 2 * np.pi

    # 1. 解 1 的连续角度
    th1_1_cont = find_nearest_angle(sol1[0], prev_theta1)
    th2_1_cont = find_nearest_angle(sol1[1], prev_theta2)

    # 2. 解 2 的连续角度
    th1_2_cont = find_nearest_angle(sol2[0], prev_theta1)
    th2_2_cont = find_nearest_angle(sol2[1], prev_theta2)

    # 3. 比较角速度的平滑性 (最小化角度变化平方)
    cost1 = angle_diff(th1_1_cont, prev_theta1) ** 2 + angle_diff(th2_1_cont, prev_theta2) ** 2
    cost2 = angle_diff(th1_2_cont, prev_theta1) ** 2 + angle_diff(th2_2_cont, prev_theta2) ** 2

    # ----------------------------------------------------------------
    # 改进的切换逻辑: 引入容忍度

    # 如果 cost 相差很小 (在奇异点附近)，我们更倾向于保持上一次的选择，
    # 除非新的选择明显更好 (超过 SINGULARITY_TOLERANCE)

    if abs(cost1 - cost2) < 1e-6:  # 极度接近奇异点
        # 保持上一次的选择
        chosen_index = last_solution_index
    elif cost1 < cost2:
        if last_solution_index == 2 and (cost2 - cost1) < SINGULARITY_TOLERANCE:
            # 如果当前解1更好，但改善不够明显，且上次是解2，我们继续保持解2，避免微小抖动
            chosen_index = 2
        else:
            chosen_index = 1
    else:  # cost2 < cost1
        if last_solution_index == 1 and (cost1 - cost2) < SINGULARITY_TOLERANCE:
            # 如果当前解2更好，但改善不够明显，且上次是解1，我们继续保持解1
            chosen_index = 1
        else:
            chosen_index = 2

    last_solution_index = chosen_index

    if chosen_index == 1:
        return th1_1_cont, th2_1_cont
    else:
        return th1_2_cont, th2_2_cont


def smooth_trajectory_continuous(t_max, dt):
    """计算平滑轨迹，使用连续角度跟踪"""
    global last_solution_index

    t_points = np.arange(0, t_max + dt, dt)

    y_traj = y0 - 0.5 * g * t_points ** 2
    x_traj = np.full_like(y_traj, x0)

    y_traj = np.maximum(y_traj, -1.0)

    theta1_traj = [theta1_0]
    theta2_traj = [theta2_0]

    # 初始化 last_solution_index，根据第一个非零步的 IK 解来确定
    # 由于初始速度为0，第一步的IK选择就是我们初始的解趋势

    # 角度平滑系数
    alpha = 0.5  # 提高平滑系数，让角度更快速跟随IK解

    for i in range(1, len(t_points)):
        x, y = x_traj[i], y_traj[i]
        prev_theta1, prev_theta2 = theta1_traj[-1], theta2_traj[-1]

        # 计算IK解，同时更新 last_solution_index
        theta1_ik, theta2_ik = inverse_kinematics_continuous(x, y, prev_theta1, prev_theta2)

        # 角度平滑滤波 (确保最终轨迹平滑)
        # 使用更大的 alpha，让平滑效果降低，但轨迹更贴近 IK 结果
        theta1 = prev_theta1 + alpha * angle_diff(theta1_ik, prev_theta1)
        theta2 = prev_theta2 + alpha * angle_diff(theta2_ik, prev_theta2)

        theta1_traj.append(theta1)
        theta2_traj.append(theta2)

    return t_points, np.array(theta1_traj), np.array(theta2_traj), x_traj, y_traj


# --- 运行计算 ---
# 必须在每次调用前重置全局变量
last_solution_index = 1
t, theta1_traj, theta2_traj, x_traj, y_traj = smooth_trajectory_continuous(t_max, dt)

# --- 结果绘图和动画 (保持与上一次代码一致的绘图逻辑，确保连续性) ---

# 创建动画
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

# ----------------- 子图 1: 机械臂动画 -----------------
ax1.set_xlim(-L1 - L2 - 0.1, L1 + L2 + 0.1)
ax1.set_ylim(-L1 - L2 - 0.1, L1 + L2 + 0.1)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.set_title('Robotic Arm Animation', fontsize=14, fontweight='bold')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')

ax1.axvline(x=x0, color='r', linestyle='--', alpha=0.5, label=f'Constraint X={x0:.2f}m')
ax1.legend(loc='lower left')

arm_line, = ax1.plot([], [], 'bo-', linewidth=2, markersize=4, markerfacecolor='blue')
mass_point, = ax1.plot([], [], 'ro', markersize=6, markerfacecolor='red')
trajectory_line, = ax1.plot(x_traj, y_traj, 'k--', alpha=0.6, linewidth=0.5, label='Mass Trajectory')

# ----------------- 子图 2: 关节角度随时间变化 -----------------
ax2.set_xlim(0, t_max)
ax2.set_ylim(-50, 300)  # 设置Y轴范围为-50到300度
ax2.grid(True, alpha=0.3)
ax2.set_title('Joint Angles vs Time (Continuous Tracking)', fontsize=14, fontweight='bold')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle (degrees)')

# 设置Y轴刻度为度数，间隔50度
yticks_deg = np.arange(-50, 301, 50)  # 从-50到300度，间隔50度
ax2.set_yticks(yticks_deg)
ax2.set_yticklabels([f'{int(y)}°' for y in yticks_deg])

# 绘制虚线零位
ax2.axhline(0, color='k', linestyle='--', linewidth=1.5, alpha=0.8)

# 将弧度转换为度数用于绘图
theta1_traj_deg = np.degrees(theta1_traj)
theta2_traj_deg = np.degrees(theta2_traj)

theta1_line, = ax2.plot([], [], 'b-', label=r'$\theta_1$', linewidth=3)
theta2_line, = ax2.plot([], [], 'r-', label=r'$\theta_2$', linewidth=3)
current_time_line = ax2.axvline(x=0, color='gray', linestyle='--')
ax2.legend(fontsize=12)


# 初始化函数
def init():
    arm_line.set_data([], [])
    mass_point.set_data([], [])
    theta1_line.set_data([], [])
    theta2_line.set_data([], [])
    current_time_line.set_xdata([0])
    return arm_line, mass_point, theta1_line, theta2_line, current_time_line


# 动画更新函数
def update(frame):
    theta1 = theta1_traj[frame]
    theta2 = theta2_traj[frame]
    t_current = t[frame]

    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    arm_line.set_data([0, x1, x2], [0, y1, y2])
    mass_point.set_data([x2], [y2])

    # 使用度数数据绘图
    theta1_line.set_data(t[:frame + 1], theta1_traj_deg[:frame + 1])
    theta2_line.set_data(t[:frame + 1], theta2_traj_deg[:frame + 1])

    current_time_line.set_xdata([t_current])

    ax1.set_title(f'Robotic Arm Animation - Time: {t_current:.3f}s', fontsize=14, fontweight='bold')

    return arm_line, mass_point, theta1_line, theta2_line, current_time_line


# 创建动画
interval_ms = 1000 / frames_per_sec

ani = FuncAnimation(fig, update, frames=len(t),
                    init_func=init, blit=True, interval=interval_ms)

plt.tight_layout()
plt.show()

# --- 保存为 GIF ---
try:
    print("Saving GIF...")
    # 增加 DPI 和减少 interval 确保流畅度
    ani.save('constrained_arm_motion_smooth.gif', writer='pillow', fps=frames_per_sec, dpi=150)
    print("GIF saved successfully as 'constrained_arm_motion_smooth.gif'")
except Exception as e:
    print(f"Could not save GIF (ensure 'pillow' or 'ffmpeg' is installed and working): {e}")