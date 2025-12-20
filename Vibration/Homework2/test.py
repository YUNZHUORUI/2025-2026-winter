"""
Double pendulum: symbolic derivation -> numeric simulation -> plots + animation.

Usage:
    - As a standalone script: `python double_pendulum.py`
        The animation window will open (make sure your matplotlib backend supports interactive windows).
        The animation object is stored in `ani` and plt.show() is called at the end.
        To save the animation to MP4, uncomment the ani.save(...) line (requires ffmpeg).


Dependencies:
    - numpy, sympy, matplotlib
"""

import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -----------------------------
# 1) Symbolic derivation (2DOF double pendulum)
#    Angles are measured from vertical (downwards). Pivot at origin.
# -----------------------------
t = sp.symbols('t', real=True)
m1, m2, L1, L2, g = sp.symbols('m1 m2 L1 L2 g', positive=True, real=True)

# generalized coordinates theta1(t), theta2(t)
theta1 = sp.Function('theta1')(t)
theta2 = sp.Function('theta2')(t)

# Coords of bobs (point masses at link ends)
x1 = L1 * sp.sin(theta1)
y1 = -L1 * sp.cos(theta1)

x2 = x1 + L2 * sp.sin(theta2)
y2 = y1 - L2 * sp.cos(theta2)

# velocities
x1d = sp.diff(x1, t)
y1d = sp.diff(y1, t)
x2d = sp.diff(x2, t)
y2d = sp.diff(y2, t)

# energies
T = sp.simplify(sp.Rational(1, 2) * m1 * (x1d ** 2 + y1d ** 2)
                + sp.Rational(1, 2) * m2 * (x2d ** 2 + y2d ** 2))
V = sp.simplify(m1 * g * y1 + m2 * g * y2)
Lagr = sp.simplify(T - V)

# generalized velocities and Lagrange equations
th1d = sp.diff(theta1, t)
th2d = sp.diff(theta2, t)
dL_dth1 = sp.diff(Lagr, theta1)
dL_dth1d = sp.diff(Lagr, th1d)
ddt_dL_dth1d = sp.diff(dL_dth1d, t)
eq1 = sp.simplify(ddt_dL_dth1d - dL_dth1)

dL_dth2 = sp.diff(Lagr, theta2)
dL_dth2d = sp.diff(Lagr, th2d)
ddt_dL_dth2d = sp.diff(dL_dth2d, t)
eq2 = sp.simplify(ddt_dL_dth2d - dL_dth2)

# second derivatives
th1dd = sp.diff(theta1, (t, 2))
th2dd = sp.diff(theta2, (t, 2))


# Solve for accelerations
sol = sp.solve([sp.Eq(eq1, 0), sp.Eq(eq2, 0)], (th1dd, th2dd), simplify=True, rational=False)
sol_th1dd = sp.simplify(sol[th1dd])
sol_th2dd = sp.simplify(sol[th2dd])

# Replace theta(t), theta'(t) with symbols for lambdify
th1_sym, th2_sym, th1d_sym, th2d_sym = sp.symbols('th1 th2 th1d th2d', real=True)
subs_map = {theta1: th1_sym, theta2: th2_sym,
            sp.diff(theta1, t): th1d_sym, sp.diff(theta2, t): th2d_sym}
sol_th1dd_sym = sp.simplify(sol_th1dd.subs(subs_map))
sol_th2dd_sym = sp.simplify(sol_th2dd.subs(subs_map))
print(sol_th1dd_sym)
print(sol_th2dd_sym)
# lambdify numeric RHS (returns tuple (ddth1, ddth2))
rhs_func = sp.lambdify((th1_sym, th2_sym, th1d_sym, th2d_sym, m1, m2, L1, L2, g),
                       (sol_th1dd_sym, sol_th2dd_sym), 'numpy')


# -----------------------------
# 2) Numeric wrapper and RK4 integrator
# -----------------------------
def double_pendulum_rhs(state, params):
    """state: [th1, th2, dth1, dth2]"""
    th1, th2, th1d, th2d = state
    a1, a2 = rhs_func(th1, th2, th1d, th2d,
                      params['m1'], params['m2'], params['L1'], params['L2'], params['g'])
    return np.array([th1d, th2d, float(a1), float(a2)], dtype=float)


def rk4_step(f, y, dt, params):
    k1 = f(y, params)
    k2 = f(y + 0.5 * dt * k1, params)
    k3 = f(y + 0.5 * dt * k2, params)
    k4 = f(y + dt * k3, params)
    return y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(y0, params, t_max=10.0, dt=0.01):
    n = int(np.round(t_max / dt)) + 1
    t = np.linspace(0, t_max, n)
    Y = np.zeros((n, 4))
    Y[0, :] = y0
    for i in range(n - 1):
        Y[i + 1, :] = rk4_step(double_pendulum_rhs, Y[i, :], dt, params)
    return t, Y


# -----------------------------
# 3) Parameters, initial condition, run
# -----------------------------
params = {'m1': 1.0, 'm2': 1.0, 'L1': 1.0, 'L2': 1.0, 'g': 9.81}

# example initial condition (rad)
theta1_0 = 120.0 * np.pi / 180.0
theta2_0 = -10.0 * np.pi / 180.0
theta1d_0 = 0.0
theta2d_0 = 0.0
y0 = np.array([theta1_0, theta2_0, theta1d_0, theta2d_0])

# simulation settings
t_max = 8.0  # seconds
dt = 0.01  # timestep (smaller dt -> more accurate, but slower)

t_vec, Y = simulate(y0, params, t_max=t_max, dt=dt)
th1 = Y[:, 0]
th2 = Y[:, 1]
th1d = Y[:, 2]
th2d = Y[:, 3]

# -----------------------------
# 4) Time-domain plots
# -----------------------------
plt.figure(figsize=(8, 6))
ax1 = plt.subplot(311)
ax1.plot(t_vec, th1, label='theta1')
ax1.plot(t_vec, th2, label='theta2', alpha=0.8)
ax1.set_ylabel('Angle [rad]')
ax1.legend()
ax1.grid(True)

ax2 = plt.subplot(312, sharex=ax1)
ax2.plot(t_vec, th1d, label='theta1_dot')
ax2.plot(t_vec, th2d, label='theta2_dot', alpha=0.8)
ax2.set_ylabel('Angular vel [rad/s]')
ax2.legend()
ax2.grid(True)

ax3 = plt.subplot(313, sharex=ax1)
ax3.plot(th2, th2d)
ax3.set_xlabel('Time [s] / angle')
ax3.set_ylabel('theta2_dot [rad/s]')
ax3.grid(True)

plt.suptitle('Double pendulum time-domain results')
plt.tight_layout(rect=[0, 0, 1, 0.96])

# -----------------------------
# 5) Animation (keeps reference to `ani`)
# -----------------------------
# coordinates of the two bobs
x1 = params['L1'] * np.sin(th1)
y1 = -params['L1'] * np.cos(th1)
x2 = x1 + params['L2'] * np.sin(th2)
y2 = y1 - params['L2'] * np.cos(th2)

# subsample frames for reasonable size (user can change)
max_frames = 400
step = max(1, int(np.ceil(len(t_vec) / max_frames)))
t_anim = t_vec[::step]
x1_anim = x1[::step]
y1_anim = y1[::step]
x2_anim = x2[::step]
y2_anim = y2[::step]
th1_anim = th1[::step]
th2_anim = th2[::step]

fig, (axA, axB) = plt.subplots(1, 2, figsize=(12, 5))
axA.set_aspect('equal')
R = params['L1'] + params['L2']
axA.set_xlim(-1.1 * R, 1.1 * R)
axA.set_ylim(-1.1 * R, 0.3 * R)
axA.set_xlabel('x [m]')
axA.set_ylabel('y [m]')
axA.grid(True)
line, = axA.plot([], [], 'o-', lw=2)  # rods + bobs
trace, = axA.plot([], [], '-', lw=1, alpha=0.6)  # trace of second bob
pivot = axA.plot(0, 0, 'ko')[0]

axB.set_xlim(t_anim[0], t_anim[-1])
axB.set_xlabel('Time [s]')
axB.set_ylabel('Angle [rad]')
axB.set_ylim(-3, 3)
axB.grid(True)
theta1_line, = axB.plot([], [], label='theta1')
theta2_line, = axB.plot([], [], label='theta2')
markerB, = axB.plot([], [], 'ro')
axB.legend()

trace_x, trace_y = [], []


def init():
    line.set_data([], [])
    trace.set_data([], [])
    theta1_line.set_data([], [])
    theta2_line.set_data([], [])
    markerB.set_data([], [])
    return line, trace, theta1_line, theta2_line, markerB


def update(i):
    xs = [0, x1_anim[i], x2_anim[i]]
    ys = [0, y1_anim[i], y2_anim[i]]
    line.set_data(xs, ys)
    trace_x.append(x2_anim[i])
    trace_y.append(y2_anim[i])
    trace.set_data(trace_x, trace_y)
    theta1_line.set_data(t_anim[:i + 1], th1_anim[:i + 1])
    theta2_line.set_data(t_anim[:i + 1], th2_anim[:i + 1])
    markerB.set_data(t_anim[i], th2_anim[i])
    return line, trace, theta1_line, theta2_line, markerB


ani = FuncAnimation(fig, update, frames=len(t_anim), init_func=init, interval=30, blit=False)

# Display/save behavior:
#  - If running as script: use plt.show() to pop up the window.
#  - If running in Jupyter: use HTML(ani.to_jshtml()) to embed (may be large for long sims).
#  - To save to file (MP4), uncomment the line below (requires ffmpeg):
ani.save('double_pendulum.mp4', fps=30, dpi=200)

# keep reference to ani to prevent GC
plt.show()
#plt.savefig('temp.pdf')
