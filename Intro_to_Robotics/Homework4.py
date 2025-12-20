import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# ---------------- DH / forward (numpy) ----------------
def DH_matrix(alpha, a, d, theta):
    c = np.cos(theta); s = np.sin(theta)
    ca = np.cos(alpha); sa = np.sin(alpha)
    return np.array([
        [c, -s, 0, a],
        [s*ca, c*ca, -sa, -sa*d],
        [s*sa, c*sa, ca, ca*d],
        [0,0,0,1]
    ])

def forward_kinematics(theta1, theta2, theta3, L1, L2, L3):
    # planar 3R: we construct transforms with rotations about z and translations along x
    T01 = DH_matrix(0, L1, 0, theta1)
    T12 = DH_matrix(0, L2, 0, theta2)
    T23 = DH_matrix(0, L3, 0, theta3)
    T02 = T01 @ T12
    T03 = T02 @ T23
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([0.0, 0.0, 0.0])  # base at origin, if you want base offset change code
    # But for plotting we want joint positions in plane:
    # compute transforms from base to each joint explicitly:
    T0 = np.eye(4)
    T1 = DH_matrix(0, L1, 0, theta1)      # base->joint1
    T2 = T1 @ DH_matrix(0, L2, 0, theta2) # base->joint2
    T3 = T2 @ DH_matrix(0, L3, 0, theta3) # base->end
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = T1[0:3, 3]
    p2 = T2[0:3, 3]
    p3 = T3[0:3, 3]
    return [p0, p1, p2, p3]

# ---------------- Parameters & trajectory ----------------
L1 = 1.2   # link1
L2 = 0.8   # link2
L3 = 0.2   # end tool length (wrist->end)
a_side = 0.3   # side length of square (你写的是矩形，这里按正方形处理)

start = np.array([1.2, 1.5])   # start point (x,y)
phi0 = np.deg2rad(45.0)        # start orientation in radians (CCW)
t_total = 1.0
N = 120                        # frames
t = np.linspace(0, t_total, N)

# Build square corners in clockwise order.
# We'll take first edge along heading phi0, then turn right (clockwise) each corner.
def corner(i):
    # i = 0..3
    if i == 0:
        return start
    elif i == 1:
        return start + a_side * np.array([np.cos(phi0), np.sin(phi0)])
    elif i == 2:
        return corner(1) + a_side * np.array([np.cos(phi0 - np.pi/2), np.sin(phi0 - np.pi/2)])
    elif i == 3:
        return corner(2) + a_side * np.array([np.cos(phi0 - np.pi), np.sin(phi0 - np.pi)])
    else:
        return start

corners = [corner(i) for i in range(4)]
# ensure closure
corners.append(corners[0])

# build piecewise linear path at constant speed
# compute distances for each segment and allocate frames proportionally
seg_len = [np.linalg.norm(corners[i+1]-corners[i]) for i in range(4)]
total_len = sum(seg_len)
frames_per_seg = [max(1, int(round(N * (L/total_len)))) for L in seg_len]
# adjust to sum N
sumf = sum(frames_per_seg)
if sumf < N:
    frames_per_seg[0] += (N - sumf)
elif sumf > N:
    frames_per_seg[0] -= (sumf - N)

traj_pts = []
traj_phi = []  # keep orientation constant phi0 (你可以改为切线方向)
for i in range(4):
    p0 = corners[i]; p1 = corners[i+1]
    k = frames_per_seg[i]
    for j in range(k):
        s = j / k
        traj_pts.append(p0*(1-s) + p1*s)
        traj_phi.append(phi0)  # constant orientation; 若要沿切线： use atan2 of segment direction

# ensure length N
traj_pts = np.array(traj_pts)
if traj_pts.shape[0] > N:
    traj_pts = traj_pts[:N]
elif traj_pts.shape[0] < N:
    # pad with last point
    last = traj_pts[-1]
    needed = N - traj_pts.shape[0]
    traj_pts = np.vstack([traj_pts, np.tile(last, (needed,1))])
    traj_phi = traj_phi + [phi0]*needed

# ---------------- Inverse kinematics (planar 3R analytic) ----------------
def ik_planar_3R(x, y, phi, L1, L2, L3):
    """
    Solve for theta1, theta2, theta3 for planar 3R to reach end-effector pose (x,y,phi)
    Returns (theta1, theta2, theta3) in radians or None if unreachable.
    We compute wrist position = end - L3 * [cos(phi), sin(phi)]
    Then solve 2R IK for wrist with link lengths L1, L2.
    Choose elbow-down solution (you can flip sign for elbow-up).
    """
    # wrist position
    wx = x - L3 * np.cos(phi)
    wy = y - L3 * np.sin(phi)
    r2 = wx**2 + wy**2
    # law of cosines for theta2
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    # numerical safety
    if cos_theta2 < -1 - 1e-9 or cos_theta2 > 1 + 1e-9:
        return None
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    # choose elbow-down: theta2 = -arccos(...)  (elbow-down vs elbow-up)
    theta2 = -np.arccos(cos_theta2)
    sin_theta2 = np.sin(theta2)
    # theta1
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2
    theta1 = np.arctan2(wy, wx) - np.arctan2(k2, k1)
    # theta3 to match end orientation
    theta3 = phi - theta1 - theta2
    return theta1, theta2, theta3

# compute joint trajectories
thetas = []
for (x,y), phi in zip(traj_pts, traj_phi):
    sol = ik_planar_3R(x, y, phi, L1, L2, L3)
    if sol is None:
        # unreachable: clip wrist distance to reachable radius and recompute (or set NaN)
        # here we clip to the nearest reachable point
        # maximum reach for wrist: L1+L2, minimum reach: |L1-L2|
        wx = x - L3*np.cos(phi); wy = y - L3*np.sin(phi)
        r = np.hypot(wx, wy)
        maxr = L1 + L2 - 1e-6
        minr = abs(L1 - L2) + 1e-6
        if r > maxr:
            scale = maxr / r
            wx *= scale; wy *= scale
        elif r < minr:
            scale = minr / r if r>1e-12 else 1.0
            wx *= scale; wy *= scale
        # recompute using adjusted wrist
        x_adj = wx + L3*np.cos(phi); y_adj = wy + L3*np.sin(phi)
        sol = ik_planar_3R(x_adj, y_adj, phi, L1, L2, L3)
        if sol is None:
            thetas.append((np.nan, np.nan, np.nan))
        else:
            thetas.append(sol)
    else:
        thetas.append(sol)

thetas = np.array(thetas)  # shape (N,3)

# ---------------- Animation ----------------
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111)
ax.set_aspect('equal')
ax.set_xlim(-0.5, 2.5)
ax.set_ylim(-0.5, 2.5)
ax.set_title("Manipulator following square (clockwise)")

line, = ax.plot([], [], '-o', lw=2)
path_line, = ax.plot(traj_pts[:,0], traj_pts[:,1], '--', lw=1, alpha=0.6)  # desired path

def init():
    line.set_data([], [])
    return line,

def update(i):
    th1, th2, th3 = thetas[i]
    if np.isnan(th1):
        xs = [0,0,0,0]; ys=[0,0,0,0]
    else:
        pts = forward_kinematics(th1, th2, th3, L1, L2, L3)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
    line.set_data(xs, ys)
    return line,

ani = FuncAnimation(fig, update, frames=N, init_func=init, blit=True, interval=1000*(t_total/N))
# save gif
ani.save('rectangle.gif', writer=PillowWriter(fps=30))
plt.show()
