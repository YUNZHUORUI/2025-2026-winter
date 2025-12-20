import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- 1. 参数设置 ---
L1 = 1.2  # 链路1长度
L2 = 0.8  # 链路2长度

# 运动学约束
ALPHA_MAX = 3.0  # 最大加速度 (rad/s^2)
OMEGA_MAX = 1.5  # 最大速度 (rad/s)

# 固定运动时间 (必需条件)
T_FINAL = 2.0  # seconds

# 起点和终点 (笛卡尔坐标)
A_cartesian = np.array([-1.2, 0.9])
B_cartesian = np.array([1.1, 0.8])

# 时间分辨率
dt = 0.005


# --- 2. 逆运动学函数 (与之前相同) ---

def inverse_kinematics(x, y, L1, L2):
    r_sq = x ** 2 + y ** 2
    r = np.sqrt(r_sq)

    if r > (L1 + L2 + 1e-6) or r < abs(L1 - L2 - 1e-6):
        return None, None

    cos_theta2 = (r_sq - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    sin_theta2_pos = np.sqrt(1 - cos_theta2 ** 2)

    theta2_1 = np.arctan2(sin_theta2_pos, cos_theta2)
    theta2_2 = np.arctan2(-sin_theta2_pos, cos_theta2)

    gamma = np.arctan2(y, x)
    alpha1 = np.arctan2(L2 * np.sin(theta2_1), L1 + L2 * np.cos(theta2_1))
    alpha2 = np.arctan2(L2 * np.sin(theta2_2), L1 + L2 * np.cos(theta2_2))

    theta1_1 = gamma - alpha1
    theta1_2 = gamma - alpha2

    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    sol1 = np.array([normalize_angle(theta1_1), normalize_angle(theta2_1)])
    sol2 = np.array([normalize_angle(theta1_2), normalize_angle(theta2_2)])

    return sol1, sol2


# --- 3. 确定起始和结束关节角度 ---

solA1, solA2 = inverse_kinematics(A_cartesian[0], A_cartesian[1], L1, L2)
if solA1[0] < solA2[0]:
    theta_start = solA1
else:
    theta_start = solA2

solB1, solB2 = inverse_kinematics(B_cartesian[0], B_cartesian[1], L1, L2)
delta1 = np.sum(np.abs(solB1 - theta_start))
delta2 = np.sum(np.abs(solB2 - theta_start))
theta_end = solB1 if delta1 < delta2 else solB2

print(f"起始角度: {np.degrees(theta_start).round(2)} deg")
print(f"目标角度: {np.degrees(theta_end).round(2)} deg")
print(f"总运动时间 T_f = {T_FINAL} s")

# --- 4. 三次多项式轨迹生成 ---

time_array = np.arange(0, T_FINAL + dt, dt)
N_steps = len(time_array)


def generate_cubic_trajectory(start_angle, end_angle, T_f, time_array):
    delta_theta = end_angle - start_angle

    if abs(delta_theta) < 1e-6:
        return np.full(len(time_array), start_angle), np.zeros(len(time_array)), np.zeros(len(time_array))

    # 计算系数
    # a0 = start_angle
    # a1 = 0
    a2 = 3 * delta_theta / (T_f ** 2)
    a3 = -2 * delta_theta / (T_f ** 3)

    theta_traj = np.zeros(len(time_array))
    omega_traj = np.zeros(len(time_array))
    alpha_traj = np.zeros(len(time_array))

    for i, t in enumerate(time_array):
        t_norm = min(t, T_f)

        # 角度: theta(t) = a0 + a2*t^2 + a3*t^3
        theta = start_angle + a2 * t_norm ** 2 + a3 * t_norm ** 3

        # 速度: dtheta/dt = 2*a2*t + 3*a3*t^2
        omega = 2 * a2 * t_norm + 3 * a3 * t_norm ** 2

        # 加速度: d2theta/dt2 = 2*a2 + 6*a3*t
        alpha = 2 * a2 + 6 * a3 * t_norm

        theta_traj[i] = theta
        omega_traj[i] = omega
        alpha_traj[i] = alpha

    # 边界校正
    theta_traj[-1] = end_angle
    omega_traj[-1] = 0.0
    alpha_traj[-1] = 2 * a2 + 6 * a3 * T_f  # 应该为0

    return theta_traj, omega_traj, alpha_traj


# 生成轨迹
theta1_traj, omega1_traj, alpha1_traj = generate_cubic_trajectory(theta_start[0], theta_end[0], T_FINAL, time_array)
theta2_traj, omega2_traj, alpha2_traj = generate_cubic_trajectory(theta_start[1], theta_end[1], T_FINAL, time_array)

# --- 5. 约束检查 ---

max_omega_1 = np.max(np.abs(omega1_traj))
max_alpha_1 = np.max(np.abs(alpha1_traj))
max_omega_2 = np.max(np.abs(omega2_traj))
max_alpha_2 = np.max(np.abs(alpha2_traj))

print("\n--- 约束检查 ---")
print(
    f"Theta 1 Max Speed: {max_omega_1:.3f} rad/s (Limit: {OMEGA_MAX}) -> {'OK' if max_omega_1 <= OMEGA_MAX else 'FAIL'}")
print(
    f"Theta 1 Max Accel: {max_alpha_1:.3f} rad/s^2 (Limit: {ALPHA_MAX}) -> {'OK' if max_alpha_1 <= ALPHA_MAX else 'FAIL'}")
print(
    f"Theta 2 Max Speed: {max_omega_2:.3f} rad/s (Limit: {OMEGA_MAX}) -> {'OK' if max_omega_2 <= OMEGA_MAX else 'FAIL'}")
print(
    f"Theta 2 Max Accel: {max_alpha_2:.3f} rad/s^2 (Limit: {ALPHA_MAX}) -> {'OK' if max_alpha_2 <= ALPHA_MAX else 'FAIL'}")


# --- 6. 绘图和动画设置 ---

def forward_kinematics(theta1, theta2, L1, L2):
    x0, y0 = 0, 0
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return (x0, y0, x1, y1, x2, y2)


# 设置子图布局：1行3列
fig = plt.figure(figsize=(18, 6))

ax_anim = fig.add_subplot(1, 3, 1)
ax_theta = fig.add_subplot(1, 3, 2)
ax_omega = fig.add_subplot(1, 3, 3)

# --- 6.1 动画图设置 ---
max_range = L1 + L2 + 0.2
ax_anim.set_xlim([-max_range, max_range])
ax_anim.set_ylim([-max_range, max_range])
ax_anim.set_aspect('equal', adjustable='box')
ax_anim.set_title(f"Robot Arm Animation (T_f={T_FINAL} s)")
ax_anim.set_xlabel("X (m)")
ax_anim.set_ylabel("Y (m)")
ax_anim.grid(True)


x0, y0, x1_init, y1_init, x2_init, y2_init = forward_kinematics(theta1_traj[0], theta2_traj[0], L1, L2)
line, = ax_anim.plot([x0, x1_init, x2_init], [y0, y1_init, y2_init], 'b-', lw=3, marker='o', markersize=4)
path, = ax_anim.plot([], [], 'c--', alpha=0.5, label='End Effector Path')

# --- 6.2 角度和速度图设置 ---

# 角度图
ax_theta.plot(time_array, np.degrees(theta1_traj), label=r'$\theta_1$', color='b')
ax_theta.plot(time_array, np.degrees(theta2_traj), label=r'$\theta_2$', color='r')
ax_theta.set_xlabel("Time (s)")
ax_theta.set_ylabel("Angle (degrees)")
ax_theta.set_title(r"Joint Angles $\theta$ vs. Time")
ax_theta.grid(True)
ax_theta.legend()
ax_theta.axvline(T_FINAL, color='k', linestyle='--', linewidth=0.8)

# 速度图
ax_omega.plot(time_array, omega1_traj, label=r'$\dot{\theta}_1$', color='b')
ax_omega.plot(time_array, omega2_traj, label=r'$\dot{\theta}_2$', color='r')
ax_omega.axhline(OMEGA_MAX, color='g', linestyle='--', linewidth=1)
ax_omega.axhline(-OMEGA_MAX, color='g', linestyle='--', linewidth=1, label=r'$\pm \omega_{max}$')
ax_omega.set_xlabel("Time (s)")
ax_omega.set_ylabel("Angular Velocity (rad/s)")
ax_omega.set_title(r"Joint Velocity $\dot{\theta}$ vs. Time")
ax_omega.grid(True)
ax_omega.legend()
ax_omega.axvline(T_FINAL, color='k', linestyle='--', linewidth=0.8)

time_text = ax_anim.text(0.05, 0.95, '', transform=ax_anim.transAxes,
                         fontsize=12, bbox=dict(facecolor='white', alpha=0.7))

# --- 7. 动画函数 ---
def animate(i):
    t1 = theta1_traj[i]
    t2 = theta2_traj[i]
    current_time = time_array[i]

    x0, y0, x1, y1, x2, y2 = forward_kinematics(t1, t2, L1, L2)

    # 更新机械臂
    line.set_data([x0, x1, x2], [y0, y1, y2])


    # 更新路径
    # 为了简化，只取当前帧之前的所有末端执行器位置
    current_path_x = [forward_kinematics(theta1_traj[j], theta2_traj[j], L1, L2)[4] for j in range(i + 1)]
    current_path_y = [forward_kinematics(theta1_traj[j], theta2_traj[j], L1, L2)[5] for j in range(i + 1)]
    path.set_data(current_path_x, current_path_y)

    time_text.set_text(f"Time = {current_time:.2f} s / {T_FINAL:.2f} s")
    return line, path, time_text


# 创建动画
frame_rate = 1.0 / dt  # 200 FPS
interval_ms = dt * 1000

ani = animation.FuncAnimation(
    fig, animate, frames=N_steps, interval=interval_ms, blit=True
)

plt.tight_layout()


# --- 保存 GIF 部分 ---

# 目标 GIF 帧率 (通常 30-60 FPS 即可)
target_fps = 30

# 计算跳帧步长
skip_frames = max(1, int(frame_rate / target_fps))

# 调整 FuncAnimation 来跳过帧
ani_save = animation.FuncAnimation(
    fig, animate,
    frames=np.arange(0, N_steps, skip_frames),
    interval=1000/target_fps, # 保持与 target_fps 对应的播放间隔
    blit=True
)

# 使用 pillow writer 保存 GIF
print(f"Saving GIF at {target_fps} FPS. Total time: {T_FINAL} s.")
ani_save.save('robot_arm_cubic_trajectory.gif', writer='pillow', fps=target_fps)

plt.show()


