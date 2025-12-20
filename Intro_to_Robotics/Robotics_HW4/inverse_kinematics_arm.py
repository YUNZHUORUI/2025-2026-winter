import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# 1. PARAMETERS AND CONSTANTS
# Link lengths
L1 = 1.2  # Length of link 1 (from joint 0 to joint 1)
L2 = 0.8  # Length of link 2 (from joint 1 to joint 2)
L3 = 0.2  # Length of link 3 (from joint 2 to end-effector)

# Drawing parameters
a = 0.3  # Side length of the rectangle (m)
start_point = np.array([1.2, 1.2])  # Start at left-top of the rectangle
phi_L3 = np.deg2rad(45)  # Fixed absolute orientation of L3 (radians)

# Animation parameters
t_total = 1.0  # Total time for drawing the rectangle (s)
N = 100  # Number of frames in the animation
dt = t_total / N

# Global list to store IK results
thetas_data = {'theta1': [], 'theta2': [], 'theta3': []}


# 2. KINEMATICS FUNCTIONS (DH and FK kept as you provided)
def DH_matrix(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(theta1, theta2, theta3, d):
    # Frame 0 to 1
    T01 = DH_matrix(0, 0, 0, theta1)
    # Frame 1 to 2
    T12 = DH_matrix(0, L1, 0, theta2)
    # Frame 2 to 3
    T23 = DH_matrix(0, L2, 0, theta3)
    # Frame 3 to end-effector
    T34 = DH_matrix(0, L3, 0, 0)

    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34

    p0 = np.array([0.0, 0.0, 0.0])
    p1 = T01[0:3, 3]
    p2 = T02[0:3, 3]
    p3 = T03[0:3, 3]  # joint3 position (between L2 and L3)
    p4 = T04[0:3, 3]  # end-effector position

    return p0, p1, p2, p3, p4


# 3. ANALYTICAL 2-LINK INVERSE KINEMATICS FOR JOINT3 (p3)
def ik_2link_planar(x, y, L1, L2, prev_solution=None):
    """
    Analytic inverse kinematics for planar 2-link manipulator to reach (x,y).
    Returns (theta1, theta2) selected to be closest to prev_solution (if given).
    If unreachable, returns None.
    """
    r = np.hypot(x, y)
    # reachability check
    if r > (L1 + L2) + 1e-8 or r < abs(L1 - L2) - 1e-8:
        return None

    # cos of theta2
    cos_q2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)
    q2_pos = np.arccos(cos_q2)  # elbow-down (by sign convention)
    q2_neg = -q2_pos  # elbow-up

    # compute corresponding q1
    def compute_q1(q2):
        k1 = L1 + L2 * np.cos(q2)
        k2 = L2 * np.sin(q2)
        q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        return q1

    q1a = compute_q1(q2_pos)
    q1b = compute_q1(q2_neg)

    sol_a = (q1a, q2_pos)
    sol_b = (q1b, q2_neg)

    if prev_solution is None:
        # choose a default (prefer elbow-down q2_pos)
        return sol_a

    # choose solution closer to previous (ensures continuity)
    def angdiff(a, b):
        d = a - b
        return (d + np.pi) % (2 * np.pi) - np.pi

    prev1, prev2 = prev_solution
    da = np.hypot(angdiff(sol_a[0], prev1), angdiff(sol_a[1], prev2))
    db = np.hypot(angdiff(sol_b[0], prev1), angdiff(sol_b[1], prev2))
    return sol_a if da <= db else sol_b


# 4. SQUARE TRAJECTORY (joint3 path) - clockwise starting at start_point
def square_trajectory(t_norm):
    """
    Parametric mapping t_norm in [0,1] -> (x,y) on a clockwise square.
    start_point is left-top corner; we move right -> down -> left -> up (clockwise).
    """
    # define corners clockwise
    c0 = start_point  # left-top
    c1 = start_point + np.array([a, 0.0])  # right-top
    c2 = start_point + np.array([a, -a])  # right-bottom
    c3 = start_point + np.array([0.0, -a])  # left-bottom
    corners = [c0, c1, c2, c3, c0]

    # map t_norm to 4 equal segments
    s = t_norm * 4.0
    seg = int(np.floor(s)) if t_norm < 1.0 else 3
    frac = s - seg
    P0 = corners[seg]
    P1 = corners[seg + 1]
    return (1 - frac) * P0 + frac * P1


# 5. GENERATE IK FOR ALL FRAMES (joint3 follows square; L3 orientation fixed)
def generate_thetas_for_frames(N):
    prev_sol = None
    for i in range(N):
        t_norm = i / (N - 1) if N > 1 else 0.0
        p3_target = square_trajectory(t_norm)
        x, y = p3_target[0], p3_target[1]

        two_link_sol = ik_2link_planar(x, y, L1, L2, prev_solution=prev_sol)
        if two_link_sol is None:
            raise RuntimeError(f"Unreachable point for joint3: {p3_target}")

        th1, th2 = two_link_sol
        # compute theta3 so that absolute orientation of L3 equals phi_L3
        th3 = phi_L3 - (th1 + th2)

        thetas_data['theta1'].append(th1)
        thetas_data['theta2'].append(th2)
        thetas_data['theta3'].append(th3)

        prev_sol = (th1, th2)

    # convert lists to numpy arrays for convenience
    for k in thetas_data:
        thetas_data[k] = np.array(thetas_data[k])

    return thetas_data


# Generate angles
print("Computing analytical IK for joint3 square trajectory...")
generate_thetas_for_frames(N)

# 6. ANIMATION SETUP AND RUN
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-0.5, 2)
ax.set_ylim(-0.5, 2)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("2D Robotic Arm: joint3 draws square, L3 fixed at 45°")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

# link line and trace of joint3
line, = ax.plot([], [], 'o-', lw=3, color='blue', markersize=8)
trace, = ax.plot([], [], '-', lw=1.5, color='green', alpha=0.8)
trace_x, trace_y = [], []

# draw target square for reference
sq = np.array([start_point,
               start_point + np.array([a, 0.0]),
               start_point + np.array([a, -a]),
               start_point + np.array([0.0, -a]),
               start_point])
ax.plot(sq[:, 0], sq[:, 1], 'k--', linewidth=1)


def update(frame):
    t1 = thetas_data['theta1'][frame]
    t2 = thetas_data['theta2'][frame]
    t3 = thetas_data['theta3'][frame]

    # Use FK to get joint positions; d param unused but keep passing something
    p0, p1, p2, p3, p4 = forward_kinematics(t1, t2, t3, d=1.2)

    # update robot links (show p0->p1->p2->p3->p4)
    xs = [p0[0], p1[0], p2[0], p3[0], p4[0]]
    ys = [p0[1], p1[1], p2[1], p3[1], p4[1]]
    line.set_data(xs, ys)

    # update trace for joint3 (p3)
    trace_x.append(p3[0])
    trace_y.append(p3[1])
    trace.set_data(trace_x, trace_y)

    return line, trace


ani = FuncAnimation(fig, update, frames=N, interval=dt * 1000, blit=True)

# Optionally save (commented out to avoid long save times)
# writer = PillowWriter(fps=25)
# ani.save('robot_drawing.gif', writer=writer)

plt.show()


# 7. PLOT JOINT ANGLES AFTER ANIMATION
def plot_joint_angles(thetas_data, N, t_total):
    """Plot the joint angles over time"""
    fig, ax = plt.subplots(figsize=(10, 6))

    # Time array
    time = np.linspace(0, t_total, N)

    # Convert angles to degrees for better readability
    theta1_deg = np.rad2deg(thetas_data['theta1'])
    theta2_deg = np.rad2deg(thetas_data['theta2'])
    theta3_deg = np.rad2deg(thetas_data['theta3'])

    # Plot all three angles
    ax.plot(time, theta1_deg, 'b-', linewidth=2, label='Theta1 (Joint 1)')
    ax.plot(time, theta2_deg, 'r-', linewidth=2, label='Theta2 (Joint 2)')
    ax.plot(time, theta3_deg, 'g-', linewidth=2, label='Theta3 (Joint 3)')

    # Add vertical lines to show square segments
    segment_times = [0, 0.25, 0.5, 0.75, 1.0]
    segment_labels = ['Start', 'Right', 'Down', 'Left', 'End']
    for t, label in zip(segment_times, segment_labels):
        ax.axvline(x=t, color='gray', linestyle='--', alpha=0.5)
        ax.text(t, ax.get_ylim()[0] + 5, label, rotation=90, verticalalignment='bottom')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Joint Angle (degrees)')
    ax.set_title('Joint Angles vs Time for Square Trajectory')
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Add some statistics
    stats_text = f"Angle Ranges:\nTheta1: {theta1_deg.min():.1f}° to {theta1_deg.max():.1f}°\n" \
                 f"Theta2: {theta2_deg.min():.1f}° to {theta2_deg.max():.1f}°\n" \
                 f"Theta3: {theta3_deg.min():.1f}° to {theta3_deg.max():.1f}°"
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()
    plt.show()

    return fig


# Plot the joint angles after the animation
print("Plotting joint angles...")
plot_joint_angles(thetas_data, N, t_total)