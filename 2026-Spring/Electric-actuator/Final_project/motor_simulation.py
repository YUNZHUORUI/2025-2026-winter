"""
Permanent-Magnet Brushed DC Motor Simulation
=============================================
Usage:
    python motor_simulation.py
    Then enter parameters in one line when prompted:
    material_number, axial_depth_mm, core_diameter_mm, turns, wire_dia_mm, lam_thickness_mm, voltage

    Example (DC 3V toy motor defaults):
    1, 12, 10, 45, 0.15, 0.5, 3.0
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import csv
import os
import sys

# ─────────────────────────────────────────────
#  OUTPUT FOLDER
# ─────────────────────────────────────────────
OUT_DIR = "motor_sim_outputs"
os.makedirs(OUT_DIR, exist_ok=True)

# ─────────────────────────────────────────────
#  MAGNET MATERIAL TABLE
# ─────────────────────────────────────────────
MATERIALS = {
    1: {"name": "NdFeB N52",          "Br": 1.45, "Hc": 955e3,  "mu_r": 1.05},
    2: {"name": "NdFeB N42",          "Br": 1.32, "Hc": 876e3,  "mu_r": 1.05},
    3: {"name": "SmCo",               "Br": 1.05, "Hc": 750e3,  "mu_r": 1.08},
    4: {"name": "Alnico 5",           "Br": 1.25, "Hc":  50e3,  "mu_r": 4.0 },
    5: {"name": "Ferrite Ceramic 8",  "Br": 0.40, "Hc": 240e3,  "mu_r": 1.05},
}

# ─────────────────────────────────────────────
#  LAMINATED ELECTRICAL-STEEL B-H CURVE
#  (typical M19 / 50Hz non-oriented steel)
# ─────────────────────────────────────────────
BH_B = np.array([0.00, 0.20, 0.50, 0.80, 1.00, 1.20,
                 1.35, 1.50, 1.60, 1.70, 1.80, 1.90])
BH_H = np.array([0.0,  40.0, 90.0, 170.0, 300.0,  650.0,
                 1200.0, 2600.0, 5200.0, 12000.0, 30000.0, 80000.0])

def interpolate_H(B_val):
    """Return H [A/m] for a given B [T] using the steel B-H curve."""
    B_val = np.clip(abs(B_val), 0, BH_B[-1])
    return float(np.interp(B_val, BH_B, BH_H))

# ─────────────────────────────────────────────
#  FIXED GEOMETRY  (stator magnet shell)
# ─────────────────────────────────────────────
D_BORE   = 40e-3      # fixed magnet bore diameter  [m]
D_SQUARE = 40e-3      # fixed square outside side   [m]  (= FIXED_MAGNET_OUTER_SIDE)
G_MIN    = 0.5e-3     # minimum air-gap             [m]

MU0 = 4e-7 * np.pi   # permeability of free space

# ─────────────────────────────────────────────
#  FAN / MECHANICAL CONSTANTS
# ─────────────────────────────────────────────
K_FAN  = 1.0e-7       # fan load  τ_fan = k_fan·ω²  [N·m·s²/rad²]  (= DEFAULT_FAN_COEFF)
B_DAMP = 2.0e-5       # viscous damping              [N·m·s/rad]
T_FRIC = 3.0e-4       # Coulomb friction             [N·m]

# Steinmetz hysteresis
K_H    = 80.0         # hysteresis coefficient       [W/(T^α · Hz · m³)]  (= steinmetz_k_W_per_m3_Hz_Talpha)
ALPHA  = 1.7          # Steinmetz exponent  (= steinmetz_alpha)

# ─────────────────────────────────────────────
#  SIMULATION TIME
# ─────────────────────────────────────────────
T_END = 2.0           # total sim time [s]  (= DEFAULT_SIMULATION_TIME)
DT    = 1e-4          # time step      [s]

# ─────────────────────────────────────────────────────────────
#  MAGNETIC CIRCUIT
# ─────────────────────────────────────────────────────────────
def magnetic_circuit(mat, L_axial, D_core):
    """
    Compute reluctances, flux, and B values.
    Returns a dict with all magnetic quantities.
    """
    R_bore   = D_BORE / 2
    R_core   = D_core / 2
    g        = R_bore - R_core          # radial air gap

    if g < G_MIN:
        raise ValueError(
            f"Air gap g = {g*1e3:.2f} mm < minimum {G_MIN*1e3:.1f} mm. "
            f"Increase core diameter or decrease bore.")

    pole_arc_fraction = 0.82            # matches analysis.py

    # Average magnet radial thickness: polar sweep from bore to square outer
    # boundary (analysis.py: average_magnet_length_m property)
    half_side  = D_SQUARE / 2.0
    thetas     = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
    denom      = np.maximum(np.abs(np.cos(thetas)), np.abs(np.sin(thetas)))
    boundary_r = half_side / np.maximum(denom, 1e-12)
    l_m_avg    = float(np.mean(boundary_r - R_bore))   # average magnet length
    r_mag_mean = R_bore + 0.5 * l_m_avg                # mean magnet radius

    # Areas  (matching analysis.py formulas)
    A_gap    = max(1e-12, pole_arc_fraction * 2.0 * np.pi * R_core * L_axial)
    A_mag    = max(1e-12, pole_arc_fraction * 2.0 * np.pi * r_mag_mean * L_axial)
    A_core   = max(1e-12, D_core * L_axial)          # rectangular cross-section
    A_return = max(1e-12, pole_arc_fraction * np.pi * R_bore * L_axial)

    # Actual iron cylinder volume (for eddy/hysteresis loss calculations)
    V_core_iron = np.pi * R_core**2 * L_axial

    # MMF source  (analysis.py: F_pm = Hc * l_m_avg)
    Hc   = mat["Hc"]
    MMF  = Hc * l_m_avg

    # Individual reluctances  R = l / (mu * A)
    mu_mag    = mat["mu_r"] * MU0
    R_mag     = l_m_avg / (mu_mag * A_mag)
    R_gap     = g / (MU0 * A_gap)
    # Steel core reluctance — start with linear guess (mu_r_steel_unsat = 4000)
    l_core    = np.pi * R_core              # half circumference (analysis.py)
    mu_steel0 = 4000 * MU0
    R_core_el = l_core / (mu_steel0 * A_core)
    # Return-path reluctance through air (dominant term, matches analysis.py)
    l_return  = 1.2 * D_SQUARE
    R_return  = l_return / (MU0 * A_return)

    R_total   = R_mag + R_gap + R_core_el + R_return
    phi_ideal = MMF / R_total

    B_gap  = phi_ideal / A_gap
    B_core = phi_ideal / A_core

    # One saturation correction: recalculate core reluctance from B-H curve
    H_core    = interpolate_H(B_core)
    if B_core > 0:
        mu_steel_eff = B_core / (H_core + 1e-9) / MU0   # relative
    else:
        mu_steel_eff = 4000.0
    R_core_el = l_core / (mu_steel_eff * MU0 * A_core)

    R_total   = R_mag + R_gap + R_core_el + R_return
    phi       = MMF / R_total
    B_gap     = phi / A_gap
    B_core    = phi / A_core

    return {
        "g":            g,
        "t_mag":        l_m_avg,
        "A_gap":        A_gap,
        "A_core":       A_core,
        "l_core":       l_core,
        "V_core":       V_core_iron,
        "MMF":          MMF,
        "R_mag":        R_mag,
        "R_gap":        R_gap,
        "R_core":       R_core_el,
        "R_return":     R_return,
        "R_total":      R_total,
        "phi":          phi,
        "B_gap":        B_gap,
        "B_core":       B_core,
        "mu_steel":     mu_steel_eff,
        "pole_arc":     pole_arc_fraction,
    }

# ─────────────────────────────────────────────────────────────
#  MOTOR CONSTANTS
# ─────────────────────────────────────────────────────────────
def motor_constants(mag, N_turns, L_axial, D_core, wire_dia):
    """Compute Kt, Ke, R_coil, L_coil, J_rotor."""
    B_gap   = mag["B_gap"]
    R_core  = D_core / 2

    commutation_factor = 0.85           # matches analysis.py

    # Torque/back-EMF constant: Kt = comm * N * B_gap * A_coil
    A_coil  = D_core * L_axial          # rectangular coil area (analysis.py)
    Kt = commutation_factor * N_turns * B_gap * A_coil

    # Back-EMF constant = Kt in SI
    Ke = Kt

    # Coil resistance
    rho_Cu  = 1.72e-8                   # resistivity of copper [Ω·m]
    A_wire  = np.pi * (wire_dia / 2)**2  # wire cross-section [m²]
    # Mean turn length = 2 * (π * R_core + L_axial)  (analysis.py)
    l_turn  = 2.0 * (np.pi * R_core + L_axial)
    l_total = N_turns * l_turn
    R_coil  = rho_Cu * l_total / A_wire

    # Coil inductance: air-gap based (analysis.py), capped to [1e-6, 0.05] H
    L_airgap = MU0 * N_turns**2 * mag["A_gap"] / max(mag["g"], 1e-6)
    L_coil   = float(np.clip(L_airgap, 1e-6, 0.05))

    # Rotor inertia  J = 0.5 * m * r²  (solid steel cylinder)
    rho_steel = 7850                    # [kg/m³]  (= DENSITY_STEEL)
    m_rotor   = rho_steel * np.pi * R_core**2 * L_axial
    J_rotor   = 0.5 * m_rotor * R_core**2

    return {
        "Kt":     Kt,
        "Ke":     Ke,
        "R_coil": R_coil,
        "L_coil": L_coil,
        "J_rotor":J_rotor,
        "A_wire": A_wire,
        "l_total":l_total,
    }

# ─────────────────────────────────────────────────────────────
#  TIME-DOMAIN SIMULATION  (forward Euler, DT = 0.1 ms)
# ─────────────────────────────────────────────────────────────
def simulate(_, const, V_drive, mag, lam_thickness, N_turns, f1=50.0):
    """
    Integrate the motor ODE:
        L dI/dt = V - R*I - Ke*ω
        J dω/dt = Kt*I - K_fan*ω² - b*ω - T_fric*sign(ω)
    """
    Kt      = const["Kt"]
    Ke      = const["Ke"]
    R_coil  = const["R_coil"]
    L_coil  = const["L_coil"]
    J       = const["J_rotor"]
    B_core  = mag["B_core"]
    A_core  = mag["A_core"]
    l_core  = mag["l_core"]

    # pre-compute eddy-current loss coefficient (per unit volume, per rad/s)
    # P_eddy = (π² * σ * d² * B² * f²) / 6   [W/m³]
    # We evaluate per time-step with instantaneous f = ω * p / (2π), p=1 pair
    rho_steel = 4.7e-7                  # electrical resistivity of steel [Ω·m]  (= RHO_STEEL_ELECTRICAL)
    V_core    = mag["V_core"]           # actual cylinder volume π·r²·L  [m³]

    t_arr   = np.arange(0, T_END, DT)
    n_steps = len(t_arr)

    I   = 0.0   # initial current
    w   = 0.0   # initial speed [rad/s]

    # Arrays for logging
    t_log   = np.empty(n_steps)
    I_log   = np.empty(n_steps)
    w_log   = np.empty(n_steps)
    emf_log = np.empty(n_steps)
    T_log   = np.empty(n_steps)   # electromagnetic torque
    Tf_log  = np.empty(n_steps)   # fan torque
    Pout_log= np.empty(n_steps)   # fan output power
    Pcu_log = np.empty(n_steps)   # copper loss
    Ped_log = np.empty(n_steps)   # eddy-current loss
    Phy_log = np.empty(n_steps)   # hysteresis loss
    Pmec_log= np.empty(n_steps)   # mechanical loss (friction+damping)
    Pin_log = np.empty(n_steps)   # total input power
    eff_log = np.empty(n_steps)   # efficiency

    for i, t in enumerate(t_arr):
        emf   = Ke * w
        # ── electrical ODE ──
        dI_dt = (V_drive - R_coil * I - emf) / (L_coil + 1e-12)
        I    += dI_dt * DT
        I     = max(I, 0.0)         # rectified (brushed commutator)

        T_em  = Kt * I
        T_fan = K_FAN * w**2
        T_damp= B_DAMP * w
        T_fr  = T_FRIC * np.sign(w) if w != 0 else min(abs(T_em), T_FRIC)*np.sign(T_em)

        # ── mechanical ODE ──
        dw_dt = (T_em - T_fan - T_damp - T_fr) / (J + 1e-15)
        w    += dw_dt * DT
        w     = max(w, 0.0)

        # ── losses ──
        P_cu   = R_coil * I**2
        # Eddy-current: electrical frequency from mechanical (1 pole-pair)
        f_el   = w / (2 * np.pi) if w > 0 else 0.0
        P_eddy = (np.pi**2 * (lam_thickness**2) * (f_el**2) * (B_core**2) * V_core) / (6 * rho_steel)
        P_hyst = K_H * abs(B_core)**ALPHA * f_el * V_core
        P_mec  = (T_damp + abs(T_fr)) * abs(w)
        P_fan  = T_fan * w             # useful shaft output
        P_in   = V_drive * I

        eta    = P_fan / (P_in + 1e-9) if P_in > 1e-9 else 0.0
        eta    = np.clip(eta, 0, 1)

        # log
        t_log[i]   = t
        I_log[i]   = I
        w_log[i]   = w
        emf_log[i] = emf
        T_log[i]   = T_em
        Tf_log[i]  = T_fan
        Pout_log[i]= P_fan
        Pcu_log[i] = P_cu
        Ped_log[i] = P_eddy
        Phy_log[i] = P_hyst
        Pmec_log[i]= P_mec
        Pin_log[i] = P_in
        eff_log[i] = eta

    results = {
        "t":     t_log,
        "I":     I_log,
        "w":     w_log,
        "emf":   emf_log,
        "T_em":  T_log,
        "T_fan": Tf_log,
        "P_out": Pout_log,
        "P_cu":  Pcu_log,
        "P_eddy":Ped_log,
        "P_hyst":Phy_log,
        "P_mec": Pmec_log,
        "P_in":  Pin_log,
        "eta":   eff_log,
    }
    return results

# ─────────────────────────────────────────────────────────────
#  STEADY-STATE AVERAGES  (last 20 % of simulation)
# ─────────────────────────────────────────────────────────────
def steady_state(res):
    n   = len(res["t"])
    idx = int(0.8 * n)
    ss  = {}
    for k, v in res.items():
        ss[k] = float(np.mean(v[idx:]))
    return ss

# ─────────────────────────────────────────────────────────────
#  PLOTS
# ─────────────────────────────────────────────────────────────
COLORS = {
    "speed":  "#2196F3",
    "current":"#FF5722",
    "emf":    "#9C27B0",
    "P_in":   "#607D8B",
    "P_out":  "#4CAF50",
    "P_cu":   "#F44336",
    "P_eddy": "#FF9800",
    "P_hyst": "#FFEB3B",
    "P_mec":  "#795548",
    "eta":    "#009688",
}

def _add_steady_line(ax, val, color, fmt="{:.2f}", side="right", units=""):
    """Draw a dashed horizontal line at steady-state value with a label."""
    ax.axhline(val, color=color, linewidth=0.8, linestyle=":", alpha=0.7)
    xlim = ax.get_xlim()
    x = xlim[1] * 0.97 if side == "right" else xlim[0] + (xlim[1]-xlim[0])*0.03
    ha = "right" if side == "right" else "left"
    ax.text(x, val, f" {fmt.format(val)}{units}", color=color,
            fontsize=7, va="bottom", ha=ha)


def plot_dashboard(res, mat_name, out_prefix):
    t   = res["t"]
    rpm = res["w"] * 60 / (2 * np.pi)

    # ── figure: 4 rows × 2 cols  (8 panels total) ──────────────────────────
    fig = plt.figure(figsize=(14, 18))
    fig.patch.set_facecolor("#F8F9FA")
    gs  = fig.add_gridspec(4, 2, hspace=0.45, wspace=0.35,
                           left=0.08, right=0.94, top=0.95, bottom=0.04)
    fig.suptitle(f"PM Brushed DC Motor Simulation — {mat_name}",
                 fontsize=14, fontweight="bold", y=0.97)

    def styled_ax(row, col, title):
        ax = fig.add_subplot(gs[row, col])
        ax.set_facecolor("white")
        ax.set_title(title, fontsize=10, fontweight="semibold", pad=6)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.grid(True, color="#DDDDDD", linewidth=0.6, zorder=0)
        ax.spines[["top","right"]].set_visible(False)
        return ax

    # ── 1. Rotor Speed ──────────────────────────────────────────────────────
    ax = styled_ax(0, 0, "Rotor Speed")
    ax.plot(t, rpm, color=COLORS["speed"], linewidth=1.6, zorder=3)
    ax.set_ylabel("Speed (RPM)", fontsize=8)
    ax.fill_between(t, rpm, alpha=0.08, color=COLORS["speed"])
    ss_rpm = float(np.mean(rpm[int(0.8*len(t)):]))
    ax.axhline(ss_rpm, color=COLORS["speed"], lw=0.8, ls=":", alpha=0.8)
    ax.text(t[-1]*0.97, ss_rpm, f" {ss_rpm:.0f} RPM",
            color=COLORS["speed"], fontsize=7, va="bottom", ha="right")

    # ── 2. Current (left) & Back-EMF (right, separate y-axis) ─────────────
    ax = styled_ax(0, 1, "Current  &  Back-EMF")
    lc, = ax.plot(t, res["I"],  color=COLORS["current"], lw=1.6,
                  label="Current (A)", zorder=3)
    ax.set_ylabel("Current (A)", color=COLORS["current"], fontsize=8)
    ax.tick_params(axis="y", labelcolor=COLORS["current"])

    ax2 = ax.twinx()
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_color(COLORS["emf"])
    le, = ax2.plot(t, res["emf"], color=COLORS["emf"], lw=1.6,
                   linestyle="--", label="Back-EMF (V)", zorder=3)
    ax2.set_ylabel("Back-EMF (V)", color=COLORS["emf"], fontsize=8)
    ax2.tick_params(axis="y", labelcolor=COLORS["emf"])

    # annotate steady-state values
    ss_I   = float(np.mean(res["I"]  [int(0.8*len(t)):]))
    ss_emf = float(np.mean(res["emf"][int(0.8*len(t)):]))
    ax.axhline(ss_I,  color=COLORS["current"], lw=0.7, ls=":", alpha=0.7)
    ax.text(t[-1]*0.03, ss_I, f"{ss_I:.2f} A", color=COLORS["current"],
            fontsize=7, va="bottom")
    ax2.axhline(ss_emf, color=COLORS["emf"], lw=0.7, ls=":", alpha=0.7)
    ax2.text(t[-1]*0.97, ss_emf, f"{ss_emf:.3f} V ", color=COLORS["emf"],
             fontsize=7, va="top", ha="right")

    ax.legend(handles=[lc, le], fontsize=7, loc="center right",
              framealpha=0.85, edgecolor="#CCCCCC")

    # ── 3. Power — large losses (linear scale, dominant terms) ─────────────
    ax = styled_ax(1, 0, "Input Power  &  Major Losses")
    ax.plot(t, res["P_in"],  color=COLORS["P_in"],  lw=1.8, label="P_input",  zorder=4)
    ax.plot(t, res["P_cu"],  color=COLORS["P_cu"],  lw=1.6, label="P_copper", zorder=3)
    ax.plot(t, res["P_mec"], color=COLORS["P_mec"], lw=1.4, label="P_mech loss",
            linestyle="--", zorder=3)
    ax.set_ylabel("Power (W)", fontsize=8)
    ax.legend(fontsize=7, loc="upper right", framealpha=0.85, edgecolor="#CCCCCC")

    # ── 4. Small losses — log scale so eddy & hysteresis are visible ───────
    ax = styled_ax(1, 1, "Small Losses  (log scale)")
    # floor at 1 µW to avoid log(0)
    p_out  = np.maximum(res["P_out"],  1e-6)
    p_eddy = np.maximum(res["P_eddy"], 1e-6)
    p_hyst = np.maximum(res["P_hyst"], 1e-6)
    p_mec  = np.maximum(res["P_mec"],  1e-6)

    ax.semilogy(t, p_out,  color=COLORS["P_out"],  lw=1.6, label="P_fan output", zorder=4)
    ax.semilogy(t, p_eddy, color=COLORS["P_eddy"], lw=1.4,
                linestyle="--", label="P_eddy",        zorder=3)
    ax.semilogy(t, p_hyst, color="#FFC107",         lw=1.4,
                linestyle="-.", label="P_hysteresis",   zorder=3)
    ax.semilogy(t, p_mec,  color=COLORS["P_mec"],  lw=1.4,
                linestyle=":", label="P_mech loss",     zorder=3)
    ax.set_ylabel("Power (W) — log scale", fontsize=8)
    ax.legend(fontsize=7, loc="lower right", framealpha=0.85, edgecolor="#CCCCCC")

    # ── 5. Efficiency ────────────────────────────────────────────────────────
    ax = styled_ax(2, 0, "Efficiency")
    ax.plot(t, res["eta"] * 100, color=COLORS["eta"], lw=1.6, zorder=3)
    ax.fill_between(t, res["eta"]*100, alpha=0.10, color=COLORS["eta"])
    ax.set_ylabel("Efficiency (%)", fontsize=8)
    ax.set_ylim(0, max(res["eta"].max()*110, 5))
    ss_eta = float(np.mean(res["eta"][int(0.8*len(t)):]))*100
    ax.axhline(ss_eta, color=COLORS["eta"], lw=0.8, ls=":", alpha=0.8)
    ax.text(t[-1]*0.97, ss_eta, f" {ss_eta:.2f}%",
            color=COLORS["eta"], fontsize=7, va="bottom", ha="right")

    # ── 6. Torque — separate lines with clear gap ────────────────────────────
    ax = styled_ax(2, 1, "Electromagnetic  &  Fan Torque")
    ax.plot(t, res["T_em"] * 1e3,  color="#3F51B5", lw=1.6,
            label="T_electromagnetic (mN·m)", zorder=4)
    ax.plot(t, res["T_fan"] * 1e3, color="#E91E63", lw=1.4,
            linestyle="--", label="T_fan load (mN·m)", zorder=3)
    ax.set_ylabel("Torque (mN·m)", fontsize=8)
    ax.legend(fontsize=7, loc="upper right", framealpha=0.85, edgecolor="#CCCCCC")

    # ── 7. Power balance bar at steady state ────────────────────────────────
    ax = styled_ax(3, 0, "Steady-State Power Balance")
    idx = int(0.8 * len(t))
    labels_bar = ["P_input", "P_copper", "P_fan\n(output)", "P_mech\nloss",
                  "P_eddy", "P_hysteresis"]
    vals_bar   = [np.mean(res["P_in"][idx:]),
                  np.mean(res["P_cu"][idx:]),
                  np.mean(res["P_out"][idx:]),
                  np.mean(res["P_mec"][idx:]),
                  np.mean(res["P_eddy"][idx:]),
                  np.mean(res["P_hyst"][idx:])]
    colors_bar = [COLORS["P_in"], COLORS["P_cu"], COLORS["P_out"],
                  COLORS["P_mec"], COLORS["P_eddy"], "#FFC107"]
    bars = ax.bar(labels_bar, vals_bar, color=colors_bar,
                  edgecolor="white", linewidth=0.8, zorder=3)
    ax.set_ylabel("Power (W)", fontsize=8)
    ax.set_yscale("log")
    ax.set_xlabel("")
    ax.tick_params(axis="x", labelsize=7)
    for bar, v in zip(bars, vals_bar):
        if v > 1e-5:
            ax.text(bar.get_x() + bar.get_width()/2, v*1.4,
                    f"{v*1e3:.2f}\nmW" if v < 1 else f"{v:.2f}\nW",
                    ha="center", va="bottom", fontsize=6.5, fontweight="bold")

    # ── 8. Fan cumulative energy & equivalent lift ────────────────────────
    ax = styled_ax(3, 1, "Fan Work Output (Cumulative)")
    dt_arr = np.diff(t, prepend=t[0])
    E_fan  = np.cumsum(res["P_out"] * dt_arr)
    h_lift = E_fan / (0.01 * 9.81)

    lE, = ax.plot(t, E_fan * 1e3, color=COLORS["P_out"], lw=1.6,
                  label="Fan energy (mJ)", zorder=4)
    ax.set_ylabel("Cumulative energy (mJ)", color=COLORS["P_out"], fontsize=8)
    ax.tick_params(axis="y", labelcolor=COLORS["P_out"])

    ax2 = ax.twinx()
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_color("#FF5722")
    lH, = ax2.plot(t, h_lift, color="#FF5722", lw=1.4,
                   linestyle="--", label="Equiv. lift height (m)", zorder=3)
    ax2.set_ylabel("Equiv. lift height (m)", color="#FF5722", fontsize=8)
    ax2.tick_params(axis="y", labelcolor="#FF5722")
    ax.legend(handles=[lE, lH], fontsize=7, loc="upper left",
              framealpha=0.85, edgecolor="#CCCCCC")

    path = os.path.join(OUT_DIR, f"{out_prefix}_dashboard.png")
    fig.savefig(path, dpi=160, facecolor=fig.get_facecolor())
    plt.close()
    print(f"  → Saved: {path}")

# ─────────────────────────────────────────────────────────────
#  CSV EXPORTS
# ─────────────────────────────────────────────────────────────
def export_time_csv(res):
    path = os.path.join(OUT_DIR, "motor_time_domain_results.csv")
    headers = ["time_s","speed_rpm","current_A","back_emf_V",
               "T_em_Nm","T_fan_Nm","P_fan_out_W",
               "P_copper_W","P_eddy_W","P_hysteresis_W","P_mec_W",
               "P_in_W","efficiency"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        rpm = res["w"] * 60 / (2*np.pi)
        for i in range(len(res["t"])):
            w.writerow([
                f"{res['t'][i]:.5f}",
                f"{rpm[i]:.2f}",
                f"{res['I'][i]:.5f}",
                f"{res['emf'][i]:.5f}",
                f"{res['T_em'][i]:.6f}",
                f"{res['T_fan'][i]:.6f}",
                f"{res['P_out'][i]:.6f}",
                f"{res['P_cu'][i]:.6f}",
                f"{res['P_eddy'][i]:.6f}",
                f"{res['P_hyst'][i]:.6f}",
                f"{res['P_mec'][i]:.6f}",
                f"{res['P_in'][i]:.6f}",
                f"{res['eta'][i]:.4f}",
            ])
    print(f"  → Saved: {path}")

def export_design_csv(mat, params, mag, const, ss):
    path = os.path.join(OUT_DIR, "motor_design_inputs_and_estimates.csv")
    rows = [
        ["=== MATERIAL ==="],
        ["Material", mat["name"]],
        ["Br (T)", mat["Br"]],
        ["Hc (kA/m)", mat["Hc"]/1e3],
        ["Recoil mu_r", mat["mu_r"]],
        [],
        ["=== GEOMETRY ==="],
        ["Axial depth (mm)", params["L_axial"]*1e3],
        ["Iron-core diameter (mm)", params["D_core"]*1e3],
        ["Bore diameter (mm)", D_BORE*1e3],
        ["Air gap (mm)", mag["g"]*1e3],
        ["Magnet thickness (mm)", mag["t_mag"]*1e3],
        ["Air-gap area (cm²)", mag["A_gap"]*1e4],
        ["Core area (cm²)", mag["A_core"]*1e4],
        [],
        ["=== RELUCTANCES (H⁻¹) ==="],
        ["R_magnet", f"{mag['R_mag']:.2e}"],
        ["R_gap",    f"{mag['R_gap']:.2e}"],
        ["R_core",   f"{mag['R_core']:.2e}"],
        ["R_return", f"{mag['R_return']:.2e}"],
        ["R_total",  f"{mag['R_total']:.2e}"],
        [],
        ["=== MAGNETIC QUANTITIES ==="],
        ["MMF (A)",      f"{mag['MMF']:.2f}"],
        ["Flux phi (mWb)", f"{mag['phi']*1e3:.4f}"],
        ["B_gap (T)",   f"{mag['B_gap']:.4f}"],
        ["B_core (T)",  f"{mag['B_core']:.4f}"],
        ["mu_steel_eff (relative)", f"{mag['mu_steel']:.1f}"],
        [],
        ["=== MOTOR CONSTANTS ==="],
        ["Kt = Ke (N·m/A)",  f"{const['Kt']:.5f}"],
        ["R_coil (Ω)",       f"{const['R_coil']:.4f}"],
        ["L_coil (mH)",      f"{const['L_coil']*1e3:.4f}"],
        ["J_rotor (g·cm²)",  f"{const['J_rotor']*1e7:.4f}"],
        ["Wire total length (m)", f"{const['l_total']:.3f}"],
        [],
        ["=== DRIVING ==="],
        ["Voltage (V)", params["V_drive"]],
        ["Turns", params["N_turns"]],
        ["Wire dia (mm)", params["wire_dia"]*1e3],
        ["Lam thickness (mm)", params["lam_thickness"]*1e3],
        [],
        ["=== STEADY-STATE RESULTS ==="],
        ["Speed (RPM)",        f"{ss['w']*60/(2*np.pi):.1f}"],
        ["Current (A)",        f"{ss['I']:.4f}"],
        ["Back-EMF (V)",       f"{ss['emf']:.4f}"],
        ["Fan output power (mW)", f"{ss['P_out']*1e3:.3f}"],
        ["Copper loss (mW)",   f"{ss['P_cu']*1e3:.3f}"],
        ["Eddy-current loss (mW)", f"{ss['P_eddy']*1e3:.3f}"],
        ["Hysteresis loss (mW)",   f"{ss['P_hyst']*1e3:.3f}"],
        ["Mechanical loss (mW)",   f"{ss['P_mec']*1e3:.3f}"],
        ["Input power (mW)",   f"{ss['P_in']*1e3:.3f}"],
        ["Efficiency (%)",     f"{ss['eta']*100:.2f}"],
    ]
    with open(path, "w", newline="") as f:
        wtr = csv.writer(f)
        for row in rows:
            wtr.writerow(row)
    print(f"  → Saved: {path}")

def export_summary_txt(mat, params, mag, const, ss):
    path = os.path.join(OUT_DIR, "motor_simulation_summary.txt")
    rpm = ss["w"] * 60 / (2*np.pi)
    lines = [
        "=" * 60,
        "  Permanent-Magnet Brushed DC Motor  —  Simulation Summary",
        "=" * 60,
        "",
        f"  Material       : {mat['name']}",
        f"  Axial depth    : {params['L_axial']*1e3:.1f} mm",
        f"  Core diameter  : {params['D_core']*1e3:.1f} mm",
        f"  Air gap        : {mag['g']*1e3:.2f} mm",
        f"  Turns          : {params['N_turns']}",
        f"  Wire diameter  : {params['wire_dia']*1e3:.2f} mm",
        f"  Lam thickness  : {params['lam_thickness']*1e3:.2f} mm",
        f"  Drive voltage  : {params['V_drive']:.1f} V",
        "",
        f"  B_gap          : {mag['B_gap']:.3f} T",
        f"  B_core         : {mag['B_core']:.3f} T",
        f"  mu_steel_eff   : {mag['mu_steel']:.0f}  (vs linear 2000)",
        "",
        f"  Kt = Ke        : {const['Kt']:.5f} N·m/A",
        f"  R_coil         : {const['R_coil']:.3f} Ω",
        f"  L_coil         : {const['L_coil']*1e3:.3f} mH",
        "",
        "-" * 60,
        "  STEADY-STATE PERFORMANCE",
        "-" * 60,
        f"  Speed          : {rpm:.0f} RPM",
        f"  Current        : {ss['I']*1e3:.2f} mA",
        f"  Back-EMF       : {ss['emf']:.3f} V",
        f"  Fan output     : {ss['P_out']*1e3:.3f} mW",
        f"  Copper loss    : {ss['P_cu']*1e3:.3f} mW",
        f"  Eddy loss      : {ss['P_eddy']*1e3:.3f} mW",
        f"  Hysteresis loss: {ss['P_hyst']*1e3:.3f} mW",
        f"  Mec loss       : {ss['P_mec']*1e3:.3f} mW",
        f"  Input power    : {ss['P_in']*1e3:.3f} mW",
        f"  Efficiency     : {ss['eta']*100:.2f} %",
        "",
        "=" * 60,
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"  → Saved: {path}")
    # Also print to console
    print()
    for ln in lines:
        print(ln)

# ─────────────────────────────────────────────────────────────
#  PARAMETER SWEEP HELPER
# ─────────────────────────────────────────────────────────────
def run_one(mat_no, L_axial_mm, D_core_mm, N_turns, wire_dia_mm,
            lam_thickness_mm, V_drive, label="run", verbose=True):
    """Run a single simulation and return (ss, mag, const)."""
    mat          = MATERIALS[mat_no]
    L_axial      = L_axial_mm      * 1e-3
    D_core       = D_core_mm       * 1e-3
    wire_dia     = wire_dia_mm     * 1e-3
    lam_thickness= lam_thickness_mm* 1e-3

    mag   = magnetic_circuit(mat, L_axial, D_core)
    const = motor_constants(mag, N_turns, L_axial, D_core, wire_dia)
    res   = simulate(mag, const, V_drive, mag, lam_thickness, N_turns)
    ss    = steady_state(res)

    params = {
        "L_axial":      L_axial,
        "D_core":       D_core,
        "N_turns":      N_turns,
        "wire_dia":     wire_dia,
        "lam_thickness":lam_thickness,
        "V_drive":      V_drive,
    }

    if verbose:
        plot_dashboard(res, mat["name"], label)
        export_time_csv(res)
        export_design_csv(mat, params, mag, const, ss)
        export_summary_txt(mat, params, mag, const, ss)

    return ss, mag, const, res, params

# ─────────────────────────────────────────────────────────────
#  PARAMETER SENSITIVITY SWEEP  (full-range, per-parameter subplots)
# ─────────────────────────────────────────────────────────────
_TURNS_CHOICES = [10, 20, 40, 60, 80, 100, 150, 200]

def run_sensitivity(base_args):
    """
    Sweep each parameter across its full valid range (12 points each,
    discrete for turns).  Produces sensitivity_sweep.png with 6 subplots
    (one per parameter) and sensitivity_analysis.csv.
    """
    mat_no, L_mm, D_mm, N, wd_mm, lt_mm, V = base_args
    base_ss, *_ = run_one(*base_args, label="base", verbose=False)
    P0 = base_ss["P_out"]
    e0 = base_ss["eta"]

    max_core = (D_BORE - 2 * G_MIN) * 1e3   # mm

    sweep_defs = [
        ("axial_depth_mm",   L_mm,  np.linspace(10,  150, 12), None),
        ("core_diameter_mm", D_mm,  np.linspace(5,   max_core, 12), None),
        ("turns",            N,     None, _TURNS_CHOICES),
        ("wire_dia_mm",      wd_mm, np.linspace(0.1, 5.0, 12), None),
        ("lam_thickness_mm", lt_mm, np.geomspace(0.05, 2.0, 10), None),
        ("voltage",          V,     np.linspace(3,   200, 12), None),
    ]
    param_order = [s[0] for s in sweep_defs]

    print("\n" + "="*72)
    print("  PARAMETER SENSITIVITY  (full-range sweep)")
    print("="*72)
    print(f"  Base: P_fan = {P0*1e3:.3f} mW,  η = {e0*100:.2f} %")
    print("-"*72)

    all_rows = []
    sweep_data = {}   # pname → (values, P_fan_list, eta_list)

    for pname, base_val, linvals, discvals in sweep_defs:
        values  = discvals if discvals is not None else linvals
        P_list, eta_list = [], []
        idx_in_args = param_order.index(pname) + 1  # +1: mat_no is args[0]

        for v in values:
            args = list(base_args)
            args[idx_in_args] = int(v) if pname == "turns" else float(v)
            try:
                ss, *_ = run_one(*args, label="sens", verbose=False)
                P_list.append(ss["P_out"] * 1e3)
                eta_list.append(ss["eta"] * 100)
                all_rows.append([pname, float(v),
                                  ss["P_out"]*1e3, ss["eta"]*100,
                                  (ss["P_out"]-P0)/(P0+1e-12)*100])
            except ValueError:
                P_list.append(float("nan"))
                eta_list.append(float("nan"))

        sweep_data[pname] = (list(values), P_list, eta_list)
        n_ok = len([p for p in P_list if not np.isnan(p)])
        print(f"  {pname:22s}: {n_ok} points OK")

    # Save CSV
    sens_path = os.path.join(OUT_DIR, "sensitivity_analysis.csv")
    with open(sens_path, "w", newline="") as f:
        wtr = csv.writer(f)
        wtr.writerow(["parameter","value","P_fan_mW","efficiency_%","delta_P_%"])
        for r in all_rows:
            wtr.writerow(r)
    print(f"\n  → Sensitivity table saved: {sens_path}")

    _plot_sensitivity_subplots(sweep_data, base_args, P0, e0)


def _plot_sensitivity_subplots(sweep_data, base_args, P0, e0):
    """3×2 grid: one subplot per parameter, dual y-axis (P_fan left, η right)."""
    mat_no, L_mm, D_mm, N, wd_mm, lt_mm, V = base_args
    base_vals = {
        "axial_depth_mm":   L_mm,
        "core_diameter_mm": D_mm,
        "turns":            N,
        "wire_dia_mm":      wd_mm,
        "lam_thickness_mm": lt_mm,
        "voltage":          V,
    }
    titles = {
        "axial_depth_mm":   "Axial Depth (mm)",
        "core_diameter_mm": "Core Diameter (mm)",
        "turns":            "Coil Turns",
        "wire_dia_mm":      "Wire Diameter (mm)",
        "lam_thickness_mm": "Lamination Thickness (mm)",
        "voltage":          "Supply Voltage (V)",
    }

    C_P   = "#2196F3"   # blue  – P_fan
    C_ETA = "#E91E63"   # pink  – efficiency
    C_BASE= "#FF9800"   # orange – base design marker

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.patch.set_facecolor("#F8F9FA")
    fig.suptitle("Parameter Sensitivity — Full Range Sweep",
                 fontsize=13, fontweight="bold", y=0.98)

    for ax, (pname, (vals, P_list, eta_list)) in zip(axes.flat, sweep_data.items()):
        vals     = np.array(vals, dtype=float)
        P_arr    = np.array(P_list, dtype=float)
        eta_arr  = np.array(eta_list, dtype=float)
        base_v   = base_vals[pname]

        ax.set_facecolor("white")
        ax.set_title(titles[pname], fontsize=9, fontweight="semibold", pad=5)
        ax.set_xlabel(titles[pname].split("(")[0].strip(), fontsize=8)
        ax.grid(True, color="#E0E0E0", linewidth=0.5, zorder=0)
        ax.spines[["top", "right"]].set_visible(False)

        # P_fan on left axis
        mask = ~np.isnan(P_arr)
        ax.plot(vals[mask], P_arr[mask], color=C_P, linewidth=2.0,
                marker="o", markersize=4, zorder=3, label="P_fan (mW)")
        ax.set_ylabel("P_fan (mW)", color=C_P, fontsize=8)
        ax.tick_params(axis="y", labelcolor=C_P, labelsize=7)

        # η on right axis
        ax2 = ax.twinx()
        ax2.plot(vals[mask], eta_arr[mask], color=C_ETA, linewidth=1.8,
                 linestyle="--", marker="s", markersize=3, zorder=3, label="η (%)")
        ax2.set_ylabel("Efficiency (%)", color=C_ETA, fontsize=8)
        ax2.tick_params(axis="y", labelcolor=C_ETA, labelsize=7)
        ax2.spines["right"].set_color(C_ETA)

        # Base design marker
        ax.axvline(base_v, color=C_BASE, linewidth=1.4, linestyle=":", zorder=4)
        ylim = ax.get_ylim()
        ax.text(base_v, ylim[1], f" base\n {base_v:.3g}",
                color=C_BASE, fontsize=7, va="top", ha="left")

        # Use log x-scale for lamination thickness
        if pname == "lam_thickness_mm":
            ax.set_xscale("log")

        ax.tick_params(axis="x", labelsize=7)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(OUT_DIR, "sensitivity_sweep.png")
    fig.savefig(path, dpi=160, facecolor=fig.get_facecolor())
    plt.close()
    print(f"  → Sensitivity plot saved: {path}")

# ─────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────
def parse_input(line):
    parts = [x.strip() for x in line.split(",")]
    if len(parts) != 7:
        raise ValueError("Need exactly 7 values: mat, L_mm, D_mm, N, wd_mm, lt_mm, V")
    mat_no       = int(parts[0])
    L_mm         = float(parts[1])
    D_mm         = float(parts[2])
    N_turns      = int(parts[3])
    wire_dia_mm  = float(parts[4])
    lam_thick_mm = float(parts[5])
    V            = float(parts[6])
    if mat_no not in MATERIALS:
        raise ValueError(f"Material must be 1-5, got {mat_no}")
    if D_mm * 1e-3 >= D_BORE:
        raise ValueError(f"Core diameter {D_mm} mm must be < bore {D_BORE*1e3} mm")
    return mat_no, L_mm, D_mm, N_turns, wire_dia_mm, lam_thick_mm, V


def print_material_table():
    print("\n  Available magnet materials:")
    print(f"  {'No.':<5} {'Name':<22} {'Br(T)':<8} {'Hc(kA/m)':<12} {'mu_r'}")
    for k, m in MATERIALS.items():
        print(f"  {k:<5} {m['name']:<22} {m['Br']:<8.2f} {m['Hc']/1e3:<12.0f} {m['mu_r']}")
    print()
    print(f"  Fixed bore diameter : {D_BORE*1e3:.0f} mm")
    print(f"  Minimum air gap     : {G_MIN*1e3:.1f} mm  →  max core dia = {(D_BORE - 2*G_MIN)*1e3:.1f} mm")
    print()
    print("  Input format:  mat_no, axial_depth_mm, core_dia_mm, turns,")
    print("                 wire_dia_mm, lam_thick_mm, voltage")
    print("  DC 3V toy motor example:  1, 12, 10, 45, 0.15, 0.5, 3.0")
    print()


def main():
    print("\n" + "="*60)
    print("  PM Brushed DC Motor Simulation")
    print("="*60)
    print_material_table()

    # Allow command-line argument (useful for scripting sweeps)
    if len(sys.argv) > 1:
        line = " ".join(sys.argv[1:])
    else:
        line = input("  Enter parameters: ").strip()

    try:
        args = parse_input(line)
    except (ValueError, IndexError) as e:
        print(f"\n  ERROR: {e}")
        sys.exit(1)

    mat_no, L_mm, D_mm, N_turns, wd_mm, lt_mm, V = args
    print(f"\n  Running simulation ...")
    try:
        ss, mag, const, res, params = run_one(*args, label="main_design", verbose=True)
    except ValueError as e:
        print(f"\n  ERROR: {e}")
        sys.exit(1)

    # Sensitivity sweep around the user's design
    print("\n  Running sensitivity sweep ...")
    run_sensitivity(args)

    print("\n  All outputs saved to:", os.path.abspath(OUT_DIR))
    print("  Done.\n")


if __name__ == "__main__":
    main()