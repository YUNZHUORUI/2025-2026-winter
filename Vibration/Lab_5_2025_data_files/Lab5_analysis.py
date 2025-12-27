from pandas import read_excel
import numpy as np
from printarray import printarray
import matplotlib.pyplot as plt

KDF = read_excel('stiffness_matrix.xlsx')
MDF = read_excel('mass_matrix.xlsx')
CDF = read_excel('damping_matrix.xlsx')


K = KDF.values
M = MDF.values
C = CDF.values
F_amp_df = read_excel('Excitation_force_amplt.xlsx')
F_amp = F_amp_df.values.flatten()


num_dof = 5
I = np.eye(num_dof)           # identical matrix
Z = np.zeros((num_dof, num_dof)) # zero matrix
Minv = np.linalg.inv(M)
# A = [[0     , I      ],
#     [-M^-1*K, -M^-1*C]]
A_sys = np.vstack([
    np.hstack([Z, I]),
    np.hstack([-Minv @ K, -Minv @ C])
])


def get_force_vector(t):
    freq = 4.0
    duration = 1.0 / (2.0 * freq)  # 0.125s

    if 0 <= t <= duration:
        # 半正弦波公式: Amplitude * sin(2*pi*f*t)
        pulse_value = np.sin(2 * np.pi * freq * t)
        return F_amp * pulse_value
    else:
        return np.zeros(num_dof)


def model_derivative(t, z):
    # F(t)
    Ft = get_force_vector(t)

    # input B*F(t)
    # : M^-1 * F(t)
    input_term = np.concatenate([np.zeros(num_dof), Minv @ Ft])

    # 计算 dz/dt = A * z + Input
    dzdt = A_sys @ z + input_term
    return dzdt


t_start = 0.0
t_end = 5.0
dt = 0.001
time_steps = np.arange(t_start, t_end, dt)

# 初始化状态存储数组
results = np.zeros((len(time_steps), 2 * num_dof))

# 初始条件: 位移和速度都为0
z_current = np.zeros(2 * num_dof)

print("开始 RK4 计算...")

for i, t in enumerate(time_steps):
    # 存储当前状态
    results[i, :] = z_current

    # RK4 经典四步法
    k1 = model_derivative(t, z_current)
    k2 = model_derivative(t + 0.5 * dt, z_current + 0.5 * dt * k1)
    k3 = model_derivative(t + 0.5 * dt, z_current + 0.5 * dt * k2)
    k4 = model_derivative(t + dt, z_current + dt * k3)

    z_current = z_current + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

print("计算完成。")

# ----------------------
# Plot
# ----------------------
plt.figure(figsize=(10, 6))

# 提取位移结果 (前5列)
displacements = results[:, :num_dof]

labels = [f'DOF {j + 1}' for j in range(num_dof)]
plt.plot(time_steps, displacements)

plt.title('Displacement Response of 5-DOF System (RK4 Method)')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (m)')
plt.legend(labels)
plt.grid(True)
plt.show()
