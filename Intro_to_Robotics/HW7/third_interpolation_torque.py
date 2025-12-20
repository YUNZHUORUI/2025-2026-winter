import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import imageio

# ==============================================================
# 1. ROBOT AND PHYSICAL PARAMETERS
# ==============================================================
L1 = 1.2  # Link 1 length (m)
L2 = 0.8  # Link 2 length (m)
m1 = 1.0  # Mass of link 1 (kg), assumed at L1 end
m2 = 1.0  # Mass of link 2 (kg), assumed at L2 end
g = 9.81  # Gravity (m/s^2)

A = np.array([-1.2, 0.9])
B = np.array([1.1, 0.8])
T_TOTAL = 2.0  # Required motion time (s)

# Simulation parameters
dt = 0.005
t = np.arange(0, T_TOTAL + dt, dt)
N_steps = len(t)


# ==============================================================
# 2. INVERSE KINEMATICS (same as before)
# ==============================================================
def IK(x, y):
    D = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    D = np.clip(D, -1, 1)

    theta2_1 = np.arctan2(np.sqrt(1 - D * D), D)
    theta2_2 = np.arctan2(-np.sqrt(1 - D * D), D)

    def solve(theta2):
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        return theta1, theta2

    sol1 = solve(theta2_1)
    sol2 = solve(theta2_2)

    return sol1 if abs(sol1[0]) < abs(sol2[0]) else sol2


theta1_A, theta2_A = IK(A[0], A[1])
theta1_B, theta2_B = IK(B[0], B[1])

print(f"Initial angles (deg): {np.degrees(theta1_A):.2f}, {np.degrees(theta2_A):.2f}")
print(f"Target  angles (deg): {np.degrees(theta1_B):.2f}, {np.degrees(theta2_B):.2f}")


# ==============================================================
# 3. CUBIC POLYNOMIAL TRAJECTORY GENERATION
# ==============================================================

def cubic_trajectory(q0, qf, v0, vf, T, t_arr):
    """Generates position, velocity, and acceleration for a cubic trajectory."""
    a0 = q0
    a1 = v0

    A = np.array([
        [T ** 2, T ** 3],
        [2 * T, 3 * T ** 2]
    ])
    B = np.array([
        qf - q0 - v0 * T,
        vf - v0
    ])

    X = np.linalg.solve(A, B)
    a2, a3 = X[0], X[1]

    q_traj = a0 + a1 * t_arr + a2 * t_arr ** 2 + a3 * t_arr ** 3
    v_traj = a1 + 2 * a2 * t_arr + 3 * a3 * t_arr ** 2
    a_traj = 2 * a2 + 6 * a3 * t_arr

    # Correction for last step due to floating point
    q_traj[-1] = qf
    v_traj[-1] = vf
    a_traj[-1] = 2 * a2 + 6 * a3 * T  # Final acceleration

    return q_traj, v_traj, a_traj


# Boundary conditions: Start and End velocity are zero
v0 = 0.0
vf = 0.0

q1, dq1, ddq1 = cubic_trajectory(theta1_A, theta1_B, v0, vf, T_TOTAL, t)
q2, dq2, ddq2 = cubic_trajectory(theta2_A, theta2_B, v0, vf, T_TOTAL, t)

# Combine into trajectory arrays
theta_traj = np.array([q1, q2])
omega_traj = np.array([dq1, dq2])
alpha_traj = np.array([ddq1, ddq2])


# ==============================================================
# 4. ROBOT DYNAMICS MODEL (Lagrangian)
# ==============================================================

def M_matrix(th2):
    """Inertia Matrix M(theta)"""
    c2 = np.cos(th2)

    M11 = m1 * L1 ** 2 + m2 * (L1 ** 2 + L2 ** 2 + 2 * L1 * L2 * c2)
    M12 = m2 * (L2 ** 2 + L1 * L2 * c2)
    M22 = m2 * L2 ** 2

    return np.array([
        [M11, M12],
        [M12, M22]
    ])


def C_vector(th2, dth1, dth2):
    """Coriolis and Centrifugal vector C(theta, dtheta)*dtheta"""
    s2 = np.sin(th2)

    C1 = -m2 * L1 * L2 * s2 * (2 * dth1 * dth2 + dth2 ** 2)
    C2 = m2 * L1 * L2 * s2 * dth1 ** 2

    return np.array([C1, C2])


def G_vector(th1, th2):
    """Gravity vector G(theta)"""
    c1 = np.cos(th1)
    c12 = np.cos(th1 + th2)

    G1 = (m1 * L1 * c1 + m2 * (L1 * c1 + L2 * c12)) * g
    G2 = m2 * L2 * c12 * g

    return np.array([G1, G2])


def compute_torque(th, dth, ddth):
    """Compute required joint torque tau = M*ddth + C*dth + G"""
    th1, th2 = th[0], th[1]
    dth1, dth2 = dth[0], dth[1]
    ddth1, ddth2 = ddth[0], ddth[1]

    M = M_matrix(th2)
    C = C_vector(th2, dth1, dth2)
    G = G_vector(th1, th2)

    # tau = M * ddth + C + G
    tau = np.dot(M, np.array([ddth1, ddth2])) + C + G
    return tau


# ==============================================================
# 5. TORQUE CALCULATION
# ==============================================================
tau1_traj = np.zeros(N_steps)
tau2_traj = np.zeros(N_steps)

# Calculate torque at each time step
for i in range(N_steps):
    th = theta_traj[:, i]
    dth = omega_traj[:, i]
    ddth = alpha_traj[:, i]

    tau = compute_torque(th, dth, ddth)
    tau1_traj[i] = tau[0]
    tau2_traj[i] = tau[1]

# ==============================================================
# 6. PLOTTING RESULTS
# ==============================================================

fig = plt.figure(figsize=(15, 10))

# Subplot 1: Angles (Theta)
ax_theta = fig.add_subplot(3, 1, 1)
ax_theta.plot(t, np.degrees(q1), label=r'$\theta_1$', color='b')
ax_theta.plot(t, np.degrees(q2), label=r'$\theta_2$', color='r')
ax_theta.set_title(f"Joint Angles $\\theta$ (Cubic Polynomial, T={T_TOTAL:.1f}s)")
ax_theta.set_ylabel("Angle (degrees)")
ax_theta.grid(True, alpha=0.5)
ax_theta.legend()

# Subplot 2: Velocities (Omega)
ax_omega = fig.add_subplot(3, 1, 2)
ax_omega.plot(t, dq1, label=r'$\dot{\theta}_1$', color='b')
ax_omega.plot(t, dq2, label=r'$\dot{\theta}_2$', color='r')
ax_omega.set_title(r"Joint Angular Velocities $\dot{\theta}$")
ax_omega.set_ylabel("Velocity (rad/s)")
ax_omega.grid(True, alpha=0.5)
ax_omega.legend()

# Subplot 3: Torques (Tau)
ax_tau = fig.add_subplot(3, 1, 3)
ax_tau.plot(t, tau1_traj, label=r'$\tau_1$', color='darkgreen', linewidth=2)
ax_tau.plot(t, tau2_traj, label=r'$\tau_2$', color='orange', linewidth=2)
ax_tau.set_title(r"Required Joint Torques $\tau$ (Calculated by Dynamics)")
ax_tau.set_xlabel("Time (s)")
ax_tau.set_ylabel("Torque (Nm)")
ax_tau.grid(True, alpha=0.5)
ax_tau.legend()

plt.tight_layout()
plt.savefig('cubic_trajectory_and_torques.pdf')
plt.show()

# ==============================================================
# 7. Animation (Simplified)
# ==============================================================

fig_anim, ax_anim = plt.subplots(figsize=(6, 6))
max_range = L1 + L2 + 0.2
ax_anim.set_xlim(-max_range, max_range)
ax_anim.set_ylim(-0.5, max_range)
ax_anim.set_aspect("equal")
ax_anim.grid()
ax_anim.set_title(f"Robot Arm Motion (Cubic, T={T_TOTAL:.1f}s)")


def forward_kinematics_plot(th1, th2):
    x1 = L1 * np.cos(th1)
    y1 = L1 * np.sin(th1)
    x2 = x1 + L2 * np.cos(th1 + th2)
    y2 = y1 + L2 * np.sin(th1 + th2)
    return x1, y1, x2, y2


x1_init, y1_init, x2_init, y2_init = forward_kinematics_plot(q1[0], q2[0])
line, = ax_anim.plot([0, x1_init, x2_init], [0, y1_init, y2_init], 'o-', lw=4, color='k')
path, = ax_anim.plot([], [], 'c--', alpha=0.6, label='End Effector Path')
time_text = ax_anim.text(-max_range + 0.2, max_range - 0.2, "", fontsize=12)

# Path data
path_x, path_y = [], []
fps_anim = 30
N_skip = max(1, int((1 / fps_anim) / dt))
frames_to_save = []

for i in range(N_steps):
    if i % N_skip != 0:
        continue

    th1 = q1[i]
    th2 = q2[i]

    x1, y1, x2, y2 = forward_kinematics_plot(th1, th2)

    line.set_data([0, x1, x2], [0, y1, y2])

    path_x.append(x2)
    path_y.append(y2)
    path.set_data(path_x, path_y)

    time_text.set_text(f"Time = {t[i]:.2f} s / {T_TOTAL:.2f} s")

    fig_anim.canvas.draw()

    rgba = np.asarray(fig_anim.canvas.buffer_rgba()).copy()
    frames_to_save.append(rgba[:, :, :3])

plt.close(fig_anim)

imageio.mimsave("cubic_robot_motion_with_torque_analysis.gif", frames_to_save, fps=fps_anim)
print("Saved animation and torque analysis plot.")
