from numpy import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


# DH Matrix
def DH_matrix(alpha, a, d, theta):
    return array([
        [cos(theta), -sin(theta), 0, a],
        [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
        [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d],
        [0, 0, 0, 1]
    ])


# Parameters
L0 = 1.2
L1 = L2 = 0.5
L3 = L4 = L5 = 1
t_total = 3.0  # total time (s)
N = 100  # number of frames
t = linspace(0, t_total, N)


# Forward Kinematics
def forward_kinematics(theta1, theta2, theta3, d):
    T01 = DH_matrix(0, 0, L0, theta1)
    T12 = DH_matrix(0, sqrt(L1 ** 2 + L2 ** 2), d, -arctan(L1 / L2))
    T23 = DH_matrix(0, L3, 0, theta2)
    T34 = DH_matrix(0, L4, 0, theta3)
    T45 = DH_matrix(0, L5, 0, 0)

    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45

    p0 = array([0, 0, 0])
    p1 = T01[0:3, 3]
    p2 = T02[0:3, 3]
    p3 = T03[0:3, 3]
    p4 = T04[0:3, 3]
    p5 = T05[0:3, 3]

    return [p0, p1, p2, p3, p4, p5]


# Animation Setup
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([0, 3])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Manipulator Motion Animation")

line, = ax.plot([], [], [], 'o-', lw=2, color='b', markersize=6)
time_text = ax.text2D(0.02, 0.92, "", transform=ax.transAxes, fontsize=12,
                      bbox=dict(boxstyle="round", fc="wheat", alpha=0.8))


def init():
    line.set_data([], [])
    line.set_3d_properties([])
    time_text.set_text("")
    return line, time_text


def update(i):
    current_t = t[i]  # 真正使用时间 t

    # 根据时间线性插值关节变量
    d = 1.2 * (current_t / t_total)  # d: 0 -> 1.2
    theta1 = deg2rad(90) * (current_t / t_total)  # theta1: 0 -> 90°
    theta2 = deg2rad(150) * (current_t / t_total)  # theta2: 0 -> 150°
    theta3 = deg2rad(90) + deg2rad(-180) * (current_t / t_total)  # 90° -> -90°

    points = forward_kinematics(theta1, theta2, theta3, d)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    line.set_data(xs, ys)
    line.set_3d_properties(zs)

    # 更新时间显示
    time_text.set_text(f"t = {current_t:.2f} s")

    return line, time_text


# Create animation
ani = FuncAnimation(fig, update, frames=N, init_func=init, blit=True, interval=1000 * t_total / N)

# Save as GIF
ani.save('manipulator_motion_with_time.gif', writer=PillowWriter(fps=30))

plt.show()