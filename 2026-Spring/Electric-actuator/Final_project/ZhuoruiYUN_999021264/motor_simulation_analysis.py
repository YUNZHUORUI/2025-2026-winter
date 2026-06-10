"""
Simplified permanent-magnet brushed DC motor simulation.

Geometry:
    - Square outside permanent magnet with a circular inner bore.
    - Round laminated steel iron core/rotor in the bore.
    - Horizontal rotation axis, matching the usual motor shaft direction.
    - Radial magnetization, with each magnet split into N and S pole halves.

The model is intentionally low-order. It is meant for student design-space
analysis, not replacement of finite-element magnetic analysis.

Outputs:
    - 3D geometry illustration with flux arrows
    - speed, current/back-EMF, power/loss/efficiency plots
    - CSV file with all time-domain signals
    - text summary with constants and magnetic reluctance estimates
"""

from __future__ import annotations

import argparse
import math
import os
import tempfile
from dataclasses import dataclass, asdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "motor_sim_matplotlib_cache"))
os.environ.setdefault("XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "motor_sim_xdg_cache"))

import matplotlib
if os.environ.get("MOTOR_SIM_BACKEND"):
    matplotlib.use(os.environ["MOTOR_SIM_BACKEND"])
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import matplotlib

matplotlib.use('MacOSX')
MU0 = 4.0 * math.pi * 1e-7
RHO_COPPER = 1.724e-8              # ohm m, copper at about 20 C
RHO_STEEL_ELECTRICAL = 4.7e-7      # ohm m, representative low-carbon steel
DENSITY_STEEL = 7850.0             # kg/m^3
DENSITY_COPPER = 8960.0            # kg/m^3
DEFAULT_AIR_GAP = 0.5e-3           # m, manufacturing lower bound requested
LAMINATION_THICKNESS = 1.0e-3      # m
FIXED_MAGNET_OUTER_SIDE = 40.0e-3  # m, fixed square outside magnet side length
FIXED_MAGNET_BORE_DIAMETER = FIXED_MAGNET_OUTER_SIDE  # m, fixed circular inner magnet bore limit
DEFAULT_SIMULATION_TIME = 2.0      # s
DEFAULT_FAN_COEFF = 1.0e-7         # N m/(rad/s)^2


# Approximate B-H curve for non-oriented laminated electrical steel. The values
# are representative teaching parameters, not vendor certification data.
STEEL_BH_CURVE = np.array([
    [0.00, 0.0],
    [0.20, 40.0],
    [0.50, 90.0],
    [0.80, 170.0],
    [1.00, 300.0],
    [1.20, 650.0],
    [1.35, 1200.0],
    [1.50, 2600.0],
    [1.60, 5200.0],
    [1.70, 12000.0],
    [1.80, 30000.0],
    [1.90, 80000.0],
], dtype=float)


MAGNET_MATERIALS = {
    "1": {
        "name": "NdFeB N52",
        "Br_T": 1.45,
        "Hc_A_per_m": 955e3,
        "mu_r": 1.05,
        "note": "Very high energy product; temperature margin must be checked.",
    },
    "2": {
        "name": "NdFeB N42",
        "Br_T": 1.32,
        "Hc_A_per_m": 876e3,
        "mu_r": 1.05,
        "note": "Common high-performance neodymium grade.",
    },
    "3": {
        "name": "SmCo",
        "Br_T": 1.05,
        "Hc_A_per_m": 750e3,
        "mu_r": 1.08,
        "note": "Good temperature stability and corrosion resistance.",
    },
    "4": {
        "name": "Alnico 5",
        "Br_T": 1.25,
        "Hc_A_per_m": 50e3,
        "mu_r": 4.0,
        "note": "High remanence but low coercivity; easy to demagnetize.",
    },
    "5": {
        "name": "Ferrite Ceramic 8",
        "Br_T": 0.40,
        "Hc_A_per_m": 240e3,
        "mu_r": 1.05,
        "note": "Low cost and robust, but lower flux density.",
    },
}


COIL_TURN_CHOICES = [10, 20, 40, 60, 80, 100, 150, 200]
PLOT_COLOR = "#111827"


@dataclass
class Geometry:
    magnet_thickness_m: float
    magnet_depth_m: float
    hub_diameter_m: float
    core_diameter_m: float
    air_gap_m: float
    outer_side_m_value: float = FIXED_MAGNET_OUTER_SIDE

    @property
    def hub_radius_m(self) -> float:
        return 0.5 * self.hub_diameter_m

    @property
    def core_radius_m(self) -> float:
        return 0.5 * self.core_diameter_m

    @property
    def outer_side_m(self) -> float:
        return self.outer_side_m_value

    @property
    def outer_half_side_m(self) -> float:
        return 0.5 * self.outer_side_m

    @property
    def outer_radius_m(self) -> float:
        return self.outer_half_side_m

    @property
    def outer_diameter_m(self) -> float:
        return self.outer_side_m

    @property
    def average_magnet_length_m(self) -> float:
        angles = np.linspace(0.0, 2.0 * math.pi, 720, endpoint=False)
        boundary_r = self.outer_square_radius_at_angles(angles)
        return float(np.mean(boundary_r - self.hub_radius_m))

    @property
    def mean_magnet_radius_m(self) -> float:
        return self.hub_radius_m + 0.5 * self.average_magnet_length_m

    def outer_square_radius_at_angles(self, angles: np.ndarray) -> np.ndarray:
        denom = np.maximum(np.abs(np.cos(angles)), np.abs(np.sin(angles)))
        return self.outer_half_side_m / np.maximum(denom, 1e-12)


@dataclass
class MotorConstants:
    voltage_V: float = 12.0
    wire_diameter_m: float = 0.50e-3
    mu_r_steel_unsat: float = 4000.0
    steel_saturation_T: float = 1.6
    lamination_thickness_m: float = LAMINATION_THICKNESS
    viscous_damping_Nm_s: float = 2.0e-5
    coulomb_friction_Nm: float = 3.0e-4
    fan_load_coefficient_Nm_per_rad_s_sq: float = DEFAULT_FAN_COEFF
    current_density_limit_A_per_m2: float = 6.0e6
    current_limit_softness: float = 2.0
    pole_pairs: int = 1
    steinmetz_alpha: float = 1.7
    steinmetz_k_W_per_m3_Hz_Talpha: float = 80.0
    commutation_factor: float = 0.85
    simulation_time_s: float = 2.0
    dt_s: float = 1.0e-4


def prompt_float(prompt: str, default: float, min_value: float | None = None,
                 max_value: float | None = None) -> float:
    while True:
        bounds = []
        if min_value is not None:
            bounds.append(f"min {min_value:g}")
        if max_value is not None:
            bounds.append(f"max {max_value:g}")
        bound_text = f" ({', '.join(bounds)})" if bounds else ""
        raw = input(f"{prompt}{bound_text} [default {default:g}]: ").strip()
        if raw == "":
            value = default
        else:
            try:
                value = float(raw)
            except ValueError:
                print("Please enter a number.")
                continue
        if min_value is not None and value < min_value:
            print(f"Value must be at least {min_value:g}.")
            continue
        if max_value is not None and value > max_value:
            print(f"Value must be at most {max_value:g}.")
            continue
        return value


def prompt_yes_no(prompt: str, default: bool = True) -> bool:
    default_text = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{prompt} [{default_text}]: ").strip().lower()
        if raw == "":
            return default
        if raw in {"y", "yes"}:
            return True
        if raw in {"n", "no"}:
            return False
        print("Please enter y or n.")


def choose_material() -> dict:
    print("\nChoose permanent magnet material:")
    for key, mat in MAGNET_MATERIALS.items():
        print(
            f"  {key}. {mat['name']}: Br={mat['Br_T']:.2f} T, "
            f"Hc={mat['Hc_A_per_m']/1000:.0f} kA/m, mu_r={mat['mu_r']:.2f}"
        )
    while True:
        raw = input("Material number [default 1]: ").strip() or "1"
        if raw in MAGNET_MATERIALS:
            return MAGNET_MATERIALS[raw]
        print("Please choose a number from 1 to 5.")


def choose_turns() -> int:
    print("\nAvailable coil-turn choices:")
    print("  " + ", ".join(str(n) for n in COIL_TURN_CHOICES))
    while True:
        raw = input("Number of turns [default 10]: ").strip() or "10"
        try:
            turns = int(raw)
        except ValueError:
            print("Please enter an integer from the listed choices.")
            continue
        if turns in COIL_TURN_CHOICES:
            return turns
        print("Please choose one of the listed values.")


def parse_single_line_inputs(raw: str) -> tuple[dict, Geometry, int, MotorConstants]:
    parts = [p.strip() for p in raw.replace(";", ",").split(",") if p.strip()]
    if len(parts) != 7:
        raise ValueError(
            "Please enter exactly 7 values: material, axial depth, core diameter, "
            "turns, copper wire diameter, lamination thickness, voltage."
        )
    material_key = parts[0]
    if material_key not in MAGNET_MATERIALS:
        raise ValueError("Material must be a number from 1 to 5.")
    material = MAGNET_MATERIALS[material_key]
    depth_mm = float(parts[1])
    core_mm = float(parts[2])
    turns = int(float(parts[3]))
    wire_mm = float(parts[4])
    lamination_mm = float(parts[5])
    voltage = float(parts[6])

    if not 2.0 <= depth_mm <= 150.0:
        raise ValueError("Magnet/rotor axial depth must be between 2 and 150 mm.")
    max_core_mm = (FIXED_MAGNET_BORE_DIAMETER - 2.0 * DEFAULT_AIR_GAP) * 1e3
    if not 1.0 <= core_mm <= max_core_mm:
        raise ValueError(
            f"Iron core diameter must be between 1.0 and {max_core_mm:.3f} mm "
            "to keep the radial air gap >= 0.5 mm."
        )
    if turns not in COIL_TURN_CHOICES:
        raise ValueError(f"Turns must be one of: {', '.join(map(str, COIL_TURN_CHOICES))}.")
    if not 0.05 <= wire_mm <= 5.0:
        raise ValueError("Copper wire diameter must be between 0.05 and 5.0 mm.")
    if not 0.05 <= lamination_mm <= 2.0:
        raise ValueError("Lamination thickness must be between 0.05 and 2.0 mm.")
    if not 0.1 <= voltage <= 200.0:
        raise ValueError("Voltage must be between 0.1 and 200 V.")

    hub_diameter_m = FIXED_MAGNET_BORE_DIAMETER
    air_gap_m = 0.5 * (hub_diameter_m - core_mm * 1e-3)
    magnet_thickness_m = max(0.0, 0.5 * (FIXED_MAGNET_OUTER_SIDE - hub_diameter_m))
    constants = MotorConstants(
        wire_diameter_m=wire_mm * 1e-3,
        lamination_thickness_m=lamination_mm * 1e-3,
        voltage_V=voltage,
        fan_load_coefficient_Nm_per_rad_s_sq=DEFAULT_FAN_COEFF,
        simulation_time_s=DEFAULT_SIMULATION_TIME,
    )
    geom = Geometry(
        magnet_thickness_m=magnet_thickness_m,
        magnet_depth_m=depth_mm * 1e-3,
        hub_diameter_m=hub_diameter_m,
        core_diameter_m=core_mm * 1e-3,
        air_gap_m=air_gap_m,
    )
    return material, geom, turns, constants


def get_user_inputs(demo_defaults: bool = False) -> tuple[dict, Geometry, int, MotorConstants]:
    if demo_defaults:
        material = MAGNET_MATERIALS["1"]
        geom = Geometry(
            magnet_thickness_m=8e-3,
            magnet_depth_m=20e-3,
            hub_diameter_m=24e-3,
            core_diameter_m=23e-3,
            air_gap_m=DEFAULT_AIR_GAP,
        )
        return material, geom, 10, MotorConstants()

    print("\nChoose all design parameters in one line.")
    print("Permanent magnet material choices:")
    for key, mat in MAGNET_MATERIALS.items():
        print(
            f"  {key}. {mat['name']}: Br={mat['Br_T']:.2f} T, "
            f"Hc={mat['Hc_A_per_m']/1000:.0f} kA/m, mu_r={mat['mu_r']:.2f}"
        )
    max_core_mm = (FIXED_MAGNET_BORE_DIAMETER - 2.0 * DEFAULT_AIR_GAP) * 1e3
    print(
        "\nEnter values as:\n"
        "  material_no [1, 2, 3, 4, 5], "
        "axial_depth_mm [range 2 to 150 mm], "
        f"core_diameter_mm [range 1 to {max_core_mm:.3f} mm], "
        f"coil_turns [{', '.join(map(str, COIL_TURN_CHOICES))}], "
        "copper_wire_diameter_mm [range 0.05 to 5.0 mm], "
        "lamination_thickness_mm [range 0.05 to 2.0 mm], "
        "voltage_V [range 0.1 to 200 V]\n"
        "Example: 1, 20, 23, 10, 0.5, 1.0, 12\n"
        "Example with more turns: 2, 25, 22, 40, 0.7, 0.5, 24\n"
    )
    while True:
        raw = input("Design values [press Enter for default example]: ").strip()
        if raw == "":
            raw = "1, 50, 39, 200, 5, 0.05, 200"
        try:
            return parse_single_line_inputs(raw)
        except ValueError as exc:
            print(f"Input error: {exc}")


def steel_h_for_b(B_tesla: float) -> float:
    """Interpolate the laminated steel B-H curve and extrapolate past 1.9 T."""
    B_abs = abs(B_tesla)
    b_vals = STEEL_BH_CURVE[:, 0]
    h_vals = STEEL_BH_CURVE[:, 1]
    if B_abs <= b_vals[-1]:
        return float(np.interp(B_abs, b_vals, h_vals))
    slope = (h_vals[-1] - h_vals[-2]) / (b_vals[-1] - b_vals[-2])
    return float(h_vals[-1] + slope * (B_abs - b_vals[-1]))


def reluctance_model(material: dict, geom: Geometry, constants: MotorConstants) -> dict:
    r_core = geom.core_radius_m
    r_hub = geom.hub_radius_m
    r_mag_mean = geom.mean_magnet_radius_m
    depth = geom.magnet_depth_m
    pole_arc_fraction = 0.82

    A_gap = max(1e-12, pole_arc_fraction * 2.0 * math.pi * r_core * depth)
    A_mag = max(1e-12, pole_arc_fraction * 2.0 * math.pi * r_mag_mean * depth)
    A_core = max(1e-12, geom.core_diameter_m * depth)
    A_return = max(1e-12, pole_arc_fraction * math.pi * geom.outer_radius_m * depth)

    l_m_avg = geom.average_magnet_length_m
    F_pm = material["Hc_A_per_m"] * l_m_avg
    R_pm = l_m_avg / (MU0 * material["mu_r"] * A_mag)
    R_gap = geom.air_gap_m / (MU0 * A_gap)
    l_core = math.pi * r_core
    l_return = 1.2 * geom.outer_diameter_m
    R_return_air = l_return / (MU0 * A_return)
    R_linear = R_pm + R_gap + R_return_air

    phi_br_limit = material["Br_T"] * A_gap
    phi_hi = min(phi_br_limit, STEEL_BH_CURVE[-1, 0] * A_core)

    def residual(phi_guess: float) -> float:
        B_core_guess = phi_guess / A_core
        H_core_guess = steel_h_for_b(B_core_guess)
        return phi_guess * R_linear + H_core_guess * l_core - F_pm

    if residual(phi_hi) <= 0.0:
        phi_uncapped = phi_hi
    else:
        lo, hi = 0.0, phi_hi
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if residual(mid) <= 0.0:
                lo = mid
            else:
                hi = mid
        phi_uncapped = 0.5 * (lo + hi)

    phi = min(phi_uncapped, phi_br_limit)
    B_gap = phi / A_gap
    B_core = phi / A_core
    H_core = steel_h_for_b(B_core)
    if phi > 0 and H_core > 0:
        R_core = H_core * l_core / phi
        mu_r_eff = B_core / (MU0 * H_core)
    else:
        R_core = l_core / (MU0 * constants.mu_r_steel_unsat * A_core)
        mu_r_eff = constants.mu_r_steel_unsat
    R_total = R_pm + R_gap + R_core + R_return_air

    return {
        "F_pm_A_turn": F_pm,
        "magnet_average_length_m": l_m_avg,
        "R_pm_A_per_Wb": R_pm,
        "R_gap_A_per_Wb": R_gap,
        "R_core_A_per_Wb": R_core,
        "R_return_air_A_per_Wb": R_return_air,
        "R_total_A_per_Wb": R_total,
        "phi_Wb": phi,
        "phi_uncapped_Wb": phi_uncapped,
        "phi_br_limit_Wb": phi_br_limit,
        "B_gap_T": B_gap,
        "B_core_T": B_core,
        "H_core_A_per_m": H_core,
        "A_gap_m2": A_gap,
        "A_mag_m2": A_mag,
        "A_core_m2": A_core,
        "A_return_m2": A_return,
        "mu_r_steel_effective": mu_r_eff,
        "pole_arc_fraction": pole_arc_fraction,
    }


def motor_parameters(geom: Geometry, turns: int, mag: dict,
                     magnetic: dict, constants: MotorConstants) -> dict:
    r = geom.core_radius_m
    depth = geom.magnet_depth_m
    core_volume = math.pi * r * r * depth
    core_mass = DENSITY_STEEL * core_volume
    rotor_inertia = 0.5 * core_mass * r * r

    wire_area = math.pi * (0.5 * constants.wire_diameter_m) ** 2
    mean_turn_length = 2.0 * (math.pi * r + depth)
    wire_length = turns * mean_turn_length
    coil_resistance = max(1e-6, RHO_COPPER * wire_length / wire_area)

    A_coil = max(1e-12, geom.core_diameter_m * depth)
    k_t = constants.commutation_factor * turns * magnetic["B_gap_T"] * A_coil
    k_e = k_t

    L_airgap = MU0 * turns * turns * magnetic["A_gap_m2"] / max(geom.air_gap_m, 1e-6)
    L_coil = float(np.clip(L_airgap, 1e-6, 0.05))

    copper_mass = DENSITY_COPPER * wire_area * wire_length
    current_limit = constants.current_density_limit_A_per_m2 * wire_area

    return {
        "rotor_inertia_kg_m2": rotor_inertia,
        "core_volume_m3": core_volume,
        "core_mass_kg": core_mass,
        "coil_resistance_ohm": coil_resistance,
        "coil_inductance_H": L_coil,
        "wire_length_m": wire_length,
        "wire_area_m2": wire_area,
        "copper_mass_kg": copper_mass,
        "current_limit_A": current_limit,
        "torque_constant_Nm_per_A": k_t,
        "back_emf_constant_V_s_per_rad": k_e,
        "coil_area_m2": A_coil,
    }


def simulate(geom: Geometry, turns: int, material: dict, constants: MotorConstants,
             magnetic: dict, motor: dict) -> pd.DataFrame:
    dt = constants.dt_s
    steps = int(constants.simulation_time_s / dt) + 1
    t = np.linspace(0.0, constants.simulation_time_s, steps)

    current = np.zeros(steps)
    omega = np.zeros(steps)
    back_emf = np.zeros(steps)
    torque = np.zeros(steps)
    fan_torque = np.zeros(steps)
    P_in = np.zeros(steps)
    P_out = np.zeros(steps)
    P_cu = np.zeros(steps)
    P_eddy = np.zeros(steps)
    P_hyst = np.zeros(steps)
    P_mech_loss = np.zeros(steps)

    R = motor["coil_resistance_ohm"]
    L = motor["coil_inductance_H"]
    J = max(motor["rotor_inertia_kg_m2"], 1e-12)
    k_t = motor["torque_constant_Nm_per_A"]
    k_e = motor["back_emf_constant_V_s_per_rad"]
    current_limit = motor["current_limit_A"]
    B = magnetic["B_core_T"]
    V_core = motor["core_volume_m3"]
    d_lam = constants.lamination_thickness_m
    rho_steel = RHO_STEEL_ELECTRICAL

    for i in range(1, steps):
        e = k_e * omega[i - 1]
        limit_ratio = max(current[i - 1] / max(current_limit, 1e-9), 0.0)
        current_limit_drop = constants.voltage_V * (limit_ratio ** 2) / constants.current_limit_softness
        di_dt = (constants.voltage_V - R * current[i - 1] - e - current_limit_drop) / L
        current[i] = max(0.0, current[i - 1] + di_dt * dt)

        torque_i = k_t * current[i]
        fan_torque_i = constants.fan_load_coefficient_Nm_per_rad_s_sq * omega[i - 1] ** 2
        f_mag_prev = constants.pole_pairs * omega[i - 1] / (2.0 * math.pi)
        p_eddy_prev = (math.pi ** 2 * d_lam ** 2 * f_mag_prev ** 2 * B ** 2 / (6.0 * rho_steel)) * V_core
        p_hyst_prev = constants.steinmetz_k_W_per_m3_Hz_Talpha * f_mag_prev * (abs(B) ** constants.steinmetz_alpha) * V_core
        iron_loss_torque = (p_eddy_prev + p_hyst_prev) / max(abs(omega[i - 1]), 1.0)
        friction = constants.viscous_damping_Nm_s * omega[i - 1]
        if abs(omega[i - 1]) > 1e-9:
            friction += constants.coulomb_friction_Nm * math.copysign(1.0, omega[i - 1])
        net_torque = torque_i - fan_torque_i - friction - iron_loss_torque
        omega[i] = max(0.0, omega[i - 1] + (net_torque / J) * dt)

        f_mag = constants.pole_pairs * omega[i] / (2.0 * math.pi)
        back_emf[i] = k_e * omega[i]
        torque[i] = torque_i
        fan_torque[i] = constants.fan_load_coefficient_Nm_per_rad_s_sq * omega[i] ** 2
        P_in[i] = constants.voltage_V * current[i]
        P_out[i] = max(0.0, fan_torque[i] * omega[i])
        P_cu[i] = current[i] * current[i] * R
        P_eddy[i] = (math.pi ** 2 * d_lam ** 2 * f_mag ** 2 * B ** 2 / (6.0 * rho_steel)) * V_core
        P_hyst[i] = constants.steinmetz_k_W_per_m3_Hz_Talpha * f_mag * (abs(B) ** constants.steinmetz_alpha) * V_core
        P_mech_loss[i] = constants.viscous_damping_Nm_s * omega[i] ** 2
        if omega[i] > 1e-9:
            P_mech_loss[i] += constants.coulomb_friction_Nm * omega[i]

    total_loss = P_cu + P_eddy + P_hyst + P_mech_loss
    efficiency = np.zeros(steps)
    useful_plus_losses = P_out + total_loss
    positive_power = useful_plus_losses > 1e-9
    efficiency[positive_power] = np.clip(P_out[positive_power] / useful_plus_losses[positive_power], 0.0, 1.0)

    energy_copper = np.cumsum(P_cu) * dt
    energy_eddy = np.cumsum(P_eddy) * dt
    energy_hysteresis = np.cumsum(P_hyst) * dt
    energy_mechanical_loss = np.cumsum(P_mech_loss) * dt
    work_output = np.cumsum(P_out) * dt
    lifted_height = work_output / (0.25 * 9.81)  # Demonstration: equivalent lift of 0.25 kg.

    return pd.DataFrame({
        "time_s": t,
        "speed_rad_s": omega,
        "speed_rpm": omega * 60.0 / (2.0 * math.pi),
        "current_A": current,
        "back_emf_V": back_emf,
        "electromagnetic_torque_Nm": torque,
        "fan_load_torque_Nm": fan_torque,
        "input_power_W": P_in,
        "fan_output_power_W": P_out,
        "shaft_output_power_W": P_out,
        "copper_loss_W": P_cu,
        "eddy_current_loss_W": P_eddy,
        "hysteresis_loss_W": P_hyst,
        "mechanical_loss_W": P_mech_loss,
        "total_loss_W": total_loss,
        "efficiency": efficiency,
        "energy_copper_J": energy_copper,
        "energy_eddy_current_J": energy_eddy,
        "energy_hysteresis_J": energy_hysteresis,
        "energy_mechanical_loss_J": energy_mechanical_loss,
        "work_output_J": work_output,
        "demo_lifted_height_m_for_0p25kg": lifted_height,
    })


def plot_3d_configuration(geom: Geometry, material: dict, magnetic: dict,
                          out_path: Path, show: bool) -> None:
    """PyVista version of the 3D motor geometry illustration.

    Why PyVista instead of Matplotlib 3D:
        PyVista/VTK uses a real depth buffer, so when two opaque surfaces
        overlap, the surface physically closer to the camera is the one shown.
        This is exactly what Matplotlib 3D often fails to do for complex
        overlapping surfaces.

    Install once if needed:
        pip install pyvista vtk
    """
    try:
        import pyvista as pv
    except ImportError as exc:
        raise ImportError(
            "PyVista is required for the 3D motor illustration. Install it with:\n"
            "  pip install pyvista vtk"
        ) from exc

    bore_r = geom.hub_radius_m
    core_r = geom.core_radius_m
    half = geom.outer_half_side_m
    depth = geom.magnet_depth_m

    color_n = '#c62828'
    color_s = '#1565c0'
    color_rotor = 'magenta'
    color_yellow = 'yellow'
    color_flux = '#facc15'
    color_core_flux = '#22c55e'

    def quad_mesh(points, quads):
        if len(points) == 0 or len(quads) == 0:
            return pv.PolyData()
        faces = []
        for q in quads:
            faces.extend([4, q[0], q[1], q[2], q[3]])
        return pv.PolyData(np.asarray(points, dtype=float), np.asarray(faces, dtype=np.int64))

    def rectangular_quad(p0, p1, p2, p3):
        return pv.PolyData(
            np.asarray([p0, p1, p2, p3], dtype=float),
            np.asarray([4, 0, 1, 2, 3], dtype=np.int64),
        )

    def square_face_with_circular_bore(x_face, z_positive=True, n=96):
        """Constant-x square face with a circular hole, split into N/S halves."""
        ys = np.linspace(-half, half, n)
        zs = np.linspace(-half, half, n)
        points = []
        index = {}
        quads = []

        for i in range(n - 1):
            for j in range(n - 1):
                y_c = 0.25 * (ys[j] + ys[j + 1] + ys[j] + ys[j + 1])
                z_c = 0.25 * (zs[i] + zs[i] + zs[i + 1] + zs[i + 1])
                if (z_c >= 0.0) != z_positive:
                    continue

                corners = [
                    (ys[j], zs[i]),
                    (ys[j + 1], zs[i]),
                    (ys[j + 1], zs[i + 1]),
                    (ys[j], zs[i + 1]),
                ]
                # Keep only cells outside the bore. This creates the visible
                # square magnet face with a circular hole.
                if any(math.hypot(y, z) < bore_r for y, z in corners):
                    continue

                q = []
                for y, z in corners:
                    key = (round(y, 12), round(z, 12))
                    if key not in index:
                        index[key] = len(points)
                        points.append((x_face, y, z))
                    q.append(index[key])
                quads.append(q)
        return quad_mesh(points, quads)

    def bore_wall(theta0, theta1, ntheta=96, nx=24):
        """Inner circular bore wall."""
        xs = np.linspace(-0.5 * depth, 0.5 * depth, nx)
        thetas = np.linspace(theta0, theta1, ntheta)
        points = []
        for xi in xs:
            for th in thetas:
                points.append((xi, bore_r * math.cos(th), bore_r * math.sin(th)))
        quads = []
        for ix in range(nx - 1):
            for it in range(ntheta - 1):
                a = ix * ntheta + it
                quads.append([a, a + 1, a + 1 + ntheta, a + ntheta])
        return quad_mesh(points, quads)

    def add_cylinder_between(plotter, p0, p1, radius, color):
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        center = 0.5 * (p0 + p1)
        direction = p1 - p0
        length = float(np.linalg.norm(direction))
        if length <= 1e-12:
            return
        cyl = pv.Cylinder(center=center, direction=direction, radius=radius,
                          height=length, resolution=24)
        plotter.add_mesh(cyl, color=color, smooth_shading=True)

    off_screen = not show
    plotter = pv.Plotter(window_size=(1200, 900), off_screen=off_screen)
    plotter.set_background('white')

    # Front and back square magnet faces with circular bore.
    for x_face in (-0.5 * depth, 0.5 * depth):
        plotter.add_mesh(square_face_with_circular_bore(x_face, z_positive=True),
                         color=color_n, opacity=1.0, smooth_shading=True)
        plotter.add_mesh(square_face_with_circular_bore(x_face, z_positive=False),
                         color=color_s, opacity=1.0, smooth_shading=True)

    # Outer square side walls, split into red N and blue S regions.
    side_faces = [
        ([(-0.5 * depth, -half, 0.0), (0.5 * depth, -half, 0.0),
          (0.5 * depth, -half, half), (-0.5 * depth, -half, half)], color_n),
        ([(-0.5 * depth, -half, -half), (0.5 * depth, -half, -half),
          (0.5 * depth, -half, 0.0), (-0.5 * depth, -half, 0.0)], color_s),
        ([(-0.5 * depth, half, 0.0), (0.5 * depth, half, 0.0),
          (0.5 * depth, half, half), (-0.5 * depth, half, half)], color_n),
        ([(-0.5 * depth, half, -half), (0.5 * depth, half, -half),
          (0.5 * depth, half, 0.0), (-0.5 * depth, half, 0.0)], color_s),
        ([(-0.5 * depth, -half, -half), (0.5 * depth, -half, -half),
          (0.5 * depth, half, -half), (-0.5 * depth, half, -half)], color_s),
        ([(-0.5 * depth, -half, half), (0.5 * depth, -half, half),
          (0.5 * depth, half, half), (-0.5 * depth, half, half)], color_n),
    ]
    for pts, color in side_faces:
        plotter.add_mesh(rectangular_quad(*pts), color=color, opacity=1.0,
                         show_edges=True, edge_color='black')

    # Circular bore wall colored by local pole half.
    plotter.add_mesh(bore_wall(0.0, math.pi), color=color_n, opacity=1.0,
                     smooth_shading=True)
    plotter.add_mesh(bore_wall(math.pi, 2.0 * math.pi), color=color_s, opacity=1.0,
                     smooth_shading=True)

    # Filled laminated iron rotor. PyVista/VTK depth buffer correctly resolves
    # whether the magenta rotor or the surrounding magnet surface is in front.
    rotor = pv.Cylinder(center=(0.0, 0.0, 0.0), direction=(1.0, 0.0, 0.0),
                        radius=core_r, height=depth, resolution=160, capping=True)
    plotter.add_mesh(rotor, color=color_rotor, opacity=1.0, smooth_shading=True)

    # Optional thin lamination lines on the rotor surface.
    n_lam = max(4, min(28, int(depth / max(constants_lam := 1.0e-3, 1.0e-6))))
    for x_lam in np.linspace(-0.48 * depth, 0.48 * depth, n_lam):
        circle = pv.Circle(radius=1.003 * core_r, resolution=120)
        circle.points[:, 0] = x_lam
        circle.points[:, 1] = circle.points[:, 0] * 0.0 + circle.points[:, 1]
        # pv.Circle lies in the x-y plane by default. Rebuild it in y-z plane.
        ths = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False)
        pts = np.column_stack([
            np.full_like(ths, x_lam),
            1.003 * core_r * np.cos(ths),
            1.003 * core_r * np.sin(ths),
        ])
        lines = []
        for i in range(len(ths)):
            lines.extend([2, i, (i + 1) % len(ths)])
        ring = pv.PolyData(pts, lines=np.asarray(lines, dtype=np.int64))
        plotter.add_mesh(ring, color='black', line_width=1, opacity=0.35)

    # Flux arrows on the front side. These are illustrative, not FEA results.
    x_arrow = 0.58 * depth
    for a in np.linspace(0.0, 2.0 * math.pi, 8, endpoint=False):
        r0 = 0.86 * half
        r1 = 1.10 * bore_r
        start = np.asarray([[x_arrow, r0 * math.cos(a), r0 * math.sin(a)]])
        vec = np.asarray([[0.0, (r1 - r0) * math.cos(a), (r1 - r0) * math.sin(a)]])
        plotter.add_arrows(start, vec, mag=1.0, color=color_flux)

    for yy_core in np.linspace(-0.45 * core_r, 0.45 * core_r, 5):
        start = np.asarray([[x_arrow, yy_core, 0.72 * core_r]])
        vec = np.asarray([[0.0, 0.0, -1.45 * core_r]])
        plotter.add_arrows(start, vec, mag=1.0, color=color_core_flux)

    # Rotation arrow around the horizontal shaft.
    rot_r = 1.20 * core_r
    a = np.linspace(math.radians(35), math.radians(310), 80)
    pts = np.column_stack([np.full_like(a, 0.7 * depth),
                           rot_r * np.cos(a),
                           rot_r * np.sin(a)])
    curve = pv.Spline(pts, len(pts))
    plotter.add_mesh(curve.tube(radius=0.00018), color=color_yellow)
    end = np.asarray([[pts[-1, 0], pts[-1, 1], pts[-1, 2]]])
    tangent = np.asarray([[0.0, -0.02 * math.sin(a[-1]), 0.02 * math.cos(a[-1])]])
    plotter.add_arrows(end, tangent, mag=1.0, color=color_yellow)

    # Shaft extension and simple fan placed ahead of the motor.
    fan_x = 1.55 * depth
    blade_len = 0.85 * half
    add_cylinder_between(plotter, (0.5 * depth, 0.0, 0.0), (fan_x, 0.0, 0.0),
                         radius=0.06 * core_r, color=color_yellow)
    for a0 in [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]:
        add_cylinder_between(plotter, (fan_x, 0.0, 0.0),
                             (fan_x, blade_len * math.cos(a0), blade_len * math.sin(a0)),
                             radius=0.04 * core_r, color=color_yellow)
    hub = pv.Sphere(center=(fan_x, 0.0, 0.0), radius=0.09 * half,
                    theta_resolution=24, phi_resolution=24)
    plotter.add_mesh(hub, color=color_yellow, smooth_shading=True)

    # Text labels.
    label_points = np.asarray([
        (-0.15 * depth, half * 0.68, half * 0.68),
        (-0.15 * depth, -half * 0.68, half * 0.68),
        (-0.15 * depth, half * 0.68, -half * 0.68),
        (-0.15 * depth, -half * 0.68, -half * 0.68),
        (0.62 * depth + 0.02, 0.0, half * 1.2),
        (fan_x + 0.005, half * 0.70, -half * 1.1),
        (-0.50 * depth, -half * 1.08, 0.0),
    ])
    labels = ['N', 'N', 'S', 'S', 'rotation', 'fan', f"B_gap {magnetic['B_gap_T']:.3f} T"]
    plotter.add_point_labels(label_points, labels, font_size=18, text_color='black',
                             point_color=color_yellow, point_size=6,
                             always_visible=True, shape_opacity=0.15)

    # Axes, camera, and saved screenshot.
    plotter.show_axes()
    plotter.camera_position = [
        (1.6 * depth, -2.6 * half, 1.7 * half),
        (0.35 * depth, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    ]
    plotter.camera.zoom(0.3)

    if show:
        plotter.show(screenshot=str(out_path))
    else:
        plotter.show(screenshot=str(out_path), auto_close=True)

def plot_speed(df: pd.DataFrame, out_path: Path, show: bool) -> None:
    fig, ax = plt.subplots(figsize=(4, 2.25))
    ax.plot(df["time_s"], df["speed_rpm"], color=PLOT_COLOR, linewidth=2.0)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rotor Speed [rpm]")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=220)
    if not show:
        plt.close(fig)



def plot_current_back_emf(df: pd.DataFrame, out_path: Path, show: bool) -> None:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(4, 2.8), sharex=True)
    ax1.plot(df["time_s"], df["current_A"], color=PLOT_COLOR, linewidth=1.8)
    ax1.set_ylabel("Current [A]")
    ax1.grid(True, alpha=0.3)
    ax2.plot(df["time_s"], df["back_emf_V"], color=PLOT_COLOR, linewidth=1.8, linestyle="-")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Back EMF [V]")
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=220)
    if not show:
        plt.close(fig)


def plot_power_efficiency(df: pd.DataFrame, out_path: Path, show: bool) -> None:
    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(4.5, 3.5), sharex=True,
                                         gridspec_kw={"height_ratios": [2, 1]})
    x = df["time_s"]
    styles = ["-", "--", ":", "-.", (0, (3, 1, 1, 1)), (0, (5, 2))]
    ax_top.plot(x, df["input_power_W"], label="Input", color=PLOT_COLOR, linestyle=styles[0], linewidth=1.8)
    ax_top.plot(x, df["shaft_output_power_W"], label="Fan out", color=PLOT_COLOR, linestyle=styles[1], linewidth=1.8)
    ax_top.plot(x, df["copper_loss_W"], label="Copper", color=PLOT_COLOR, linestyle=styles[2])
    ax_top.plot(x, df["eddy_current_loss_W"], label="Eddy", color=PLOT_COLOR, linestyle=styles[3])
    ax_top.plot(x, df["hysteresis_loss_W"], label="Hysteresis", color=PLOT_COLOR, linestyle=styles[4])
    ax_top.plot(x, df["mechanical_loss_W"], label="Mechanical", color=PLOT_COLOR, linestyle=styles[5])
    ax_top.set_ylabel("Power and Loss [W]")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend(ncol=2, fontsize=8)

    ax_bot.plot(x, 100.0 * df["efficiency"], color=PLOT_COLOR, linewidth=1.8)
    ax_bot.set_xlabel("Time (s)")
    ax_bot.set_ylabel("Efficiency [%]")
    ax_bot.set_ylim(0, max(5, min(100, 110 * df["efficiency"].max())))
    ax_bot.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=220)
    if not show:
        plt.close(fig)


def plot_work_demo(df: pd.DataFrame, out_path: Path, show: bool) -> None:
    fig, ax1 = plt.subplots(figsize=(4, 2.25))
    ax1.plot(df["time_s"], df["work_output_J"], color=PLOT_COLOR, linewidth=1.8, label="Work")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Fan Work [J]")
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(df["time_s"], df["demo_lifted_height_m_for_0p25kg"], color=PLOT_COLOR, linewidth=1.5, linestyle="--", label="Lift")
    ax2.set_ylabel("Lift Height for 0.25 kg [m]")
    fig.tight_layout()
    fig.savefig(out_path, dpi=220)
    if not show:
        plt.close(fig)


def plot_dashboard_grid(df: pd.DataFrame, geom: Geometry, magnetic: dict,
                        out_path: Path, show: bool) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(9, 5.6))
    ax_speed, ax_current, ax_emf, ax_power, ax_eff, ax_work = axes.flat

    x = df["time_s"]
    ax_speed.plot(x, df["speed_rpm"], color=PLOT_COLOR, linewidth=1.8)
    ax_speed.set_ylabel("Speed [rpm]")
    ax_speed.grid(True, alpha=0.3)

    ax_current.plot(x, df["current_A"], color=PLOT_COLOR, linewidth=1.8, linestyle="-")
    ax_current.set_ylabel("Current [A]")
    ax_current.grid(True, alpha=0.3)

    ax_emf.plot(x, df["back_emf_V"], color=PLOT_COLOR, linewidth=1.8, linestyle="-")
    ax_emf.set_ylabel("Back EMF [V]")
    ax_emf.set_xlabel("Time (s)")
    ax_emf.grid(True, alpha=0.3)

    styles = ["-", "--", ":", "-.", (0, (3, 1, 1, 1)), (0, (5, 2))]
    ax_power.plot(x, df["input_power_W"], label="Input", color=PLOT_COLOR, linestyle=styles[0], linewidth=1.4)
    ax_power.plot(x, df["fan_output_power_W"], label="Fan out", color=PLOT_COLOR, linestyle=styles[1], linewidth=1.4)
    ax_power.plot(x, df["copper_loss_W"], label="Copper", color=PLOT_COLOR, linestyle=styles[2], linewidth=1.0)
    ax_power.plot(x, df["eddy_current_loss_W"], label="Eddy", color=PLOT_COLOR, linestyle=styles[3], linewidth=1.0)
    ax_power.plot(x, df["hysteresis_loss_W"], label="Hyst.", color=PLOT_COLOR, linestyle=styles[4], linewidth=1.0)
    ax_power.plot(x, df["mechanical_loss_W"], label="Mech.", color=PLOT_COLOR, linestyle=styles[5], linewidth=1.0)
    ax_power.set_ylabel("Power/Loss [W]")
    ax_power.set_xlabel("Time (s)")
    ax_power.grid(True, alpha=0.3)
    ax_power.legend(fontsize=6, ncol=2)

    ax_eff.plot(x, 100.0 * df["efficiency"], color=PLOT_COLOR, linewidth=1.8)
    ax_eff.set_ylabel("Efficiency [%]")
    ax_eff.set_xlabel("Time (s)")
    ax_eff.set_ylim(0, max(5, min(100, 110 * df["efficiency"].max())))
    ax_eff.grid(True, alpha=0.3)

    ax_work.plot(x, df["work_output_J"], color=PLOT_COLOR, linewidth=1.8)
    ax_work.set_ylabel("Fan Work [J]")
    ax_work.set_xlabel("Time (s)")
    ax_work.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=220)
    if not show:
        plt.close(fig)


def write_summary(path: Path, material: dict, geom: Geometry, turns: int,
                  constants: MotorConstants, magnetic: dict, motor: dict,
                  df: pd.DataFrame) -> None:
    steady = df.iloc[-1]
    lines = [
        "Simplified permanent-magnet brushed DC motor simulation summary",
        "=" * 68,
        "",
        f"Magnet material: {material['name']}",
        f"Material note: {material['note']}",
        f"Br = {material['Br_T']:.4g} T",
        f"Hc = {material['Hc_A_per_m']:.4g} A/m",
        f"Magnet recoil relative permeability = {material['mu_r']:.4g}",
        "",
        "Geometry",
        f"Computed average magnet radial thickness = {geom.magnet_thickness_m:.6g} m",
        f"Magnet axial depth = {geom.magnet_depth_m:.6g} m",
        f"Fixed square outside magnet side length = {geom.outer_side_m:.6g} m",
        f"Computed magnet bore diameter = {geom.hub_diameter_m:.6g} m",
        f"Iron core diameter = {geom.core_diameter_m:.6g} m",
        f"Computed radial air gap = {geom.air_gap_m:.6g} m",
        f"Turns = {turns}",
        "",
        "Reluctance model",
        f"Average magnetization length l_m_avg = {magnetic['magnet_average_length_m']:.6g} m",
        f"Equivalent magnet MMF F_pm = Hc*l_m_avg = {magnetic['F_pm_A_turn']:.6g} A-turn",
        f"R_pm = {magnetic['R_pm_A_per_Wb']:.6g} A/Wb",
        f"R_gap = {magnetic['R_gap_A_per_Wb']:.6g} A/Wb",
        f"R_core = {magnetic['R_core_A_per_Wb']:.6g} A/Wb",
        f"R_return_air = {magnetic['R_return_air_A_per_Wb']:.6g} A/Wb",
        f"R_total = {magnetic['R_total_A_per_Wb']:.6g} A/Wb",
        f"Flux phi = {magnetic['phi_Wb']:.6g} Wb",
        f"B_gap = {magnetic['B_gap_T']:.6g} T",
        f"B_core = {magnetic['B_core_T']:.6g} T",
        f"H_core from B-H curve = {magnetic['H_core_A_per_m']:.6g} A/m",
        f"Effective steel mu_r = {magnetic['mu_r_steel_effective']:.6g}",
        "",
        "Electrical and mechanical constants",
        f"Driving voltage = {constants.voltage_V:.6g} V",
        f"Rotor inertia J = {motor['rotor_inertia_kg_m2']:.6g} kg m^2",
        f"Copper wire diameter = {constants.wire_diameter_m:.6g} m",
        f"Current density limit = {constants.current_density_limit_A_per_m2:.6g} A/m^2",
        f"Current limit from wire area = {motor['current_limit_A']:.6g} A",
        f"Coil resistance = {motor['coil_resistance_ohm']:.6g} ohm",
        f"Coil inductance = {motor['coil_inductance_H']:.6g} H",
        f"Torque constant = {motor['torque_constant_Nm_per_A']:.6g} N m/A",
        f"Back-EMF constant = {motor['back_emf_constant_V_s_per_rad']:.6g} V s/rad",
        f"Lamination thickness = {constants.lamination_thickness_m:.6g} m",
        f"Fan load coefficient = {constants.fan_load_coefficient_Nm_per_rad_s_sq:.6g} N m/(rad/s)^2",
        f"Viscous damping = {constants.viscous_damping_Nm_s:.6g} N m s",
        f"Coulomb friction = {constants.coulomb_friction_Nm:.6g} N m",
        "",
        "Final time-domain values",
        f"Final speed = {steady['speed_rpm']:.6g} rpm",
        f"Final current = {steady['current_A']:.6g} A",
        f"Final back EMF = {steady['back_emf_V']:.6g} V",
        f"Final input power = {steady['input_power_W']:.6g} W",
        f"Final shaft output power = {steady['shaft_output_power_W']:.6g} W",
        f"Final efficiency = {100.0 * steady['efficiency']:.6g} %",
        f"Total output work = {steady['work_output_J']:.6g} J",
        f"Total copper energy loss = {steady['energy_copper_J']:.6g} J",
        f"Total eddy-current energy loss = {steady['energy_eddy_current_J']:.6g} J",
        f"Total hysteresis energy loss = {steady['energy_hysteresis_J']:.6g} J",
        f"Total mechanical energy loss = {steady['energy_mechanical_loss_J']:.6g} J",
        "",
        "CSV columns needed for independent efficiency analysis:",
        ", ".join(df.columns),
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def save_design_inputs(path: Path, material: dict, geom: Geometry, turns: int,
                       constants: MotorConstants, magnetic: dict, motor: dict) -> None:
    data = {
        "material": material,
        "geometry": asdict(geom),
        "turns": turns,
        "constants": asdict(constants),
        "magnetic_estimates": magnetic,
        "motor_estimates": motor,
    }
    rows = []
    for group, values in data.items():
        if isinstance(values, dict):
            for key, value in values.items():
                rows.append({"group": group, "parameter": key, "value": value})
        else:
            rows.append({"group": group, "parameter": group, "value": values})
    pd.DataFrame(rows).to_csv(path, index=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Simplified PM brushed motor simulation.")
    parser.add_argument("--demo-defaults", action="store_true",
                        help="Run with default inputs instead of interactive prompts.")
    parser.add_argument("--no-show", action="store_true",
                        help="Save plots without opening interactive windows.")
    parser.add_argument("--output-dir", default="motor_sim_outputs",
                        help="Directory for CSV, plots, and summary files.")
    args = parser.parse_args()

    material, geom, turns, constants = get_user_inputs(args.demo_defaults)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    show = not args.no_show

    magnetic = reluctance_model(material, geom, constants)
    motor = motor_parameters(geom, turns, material, magnetic, constants)
    df = simulate(geom, turns, material, constants, magnetic, motor)

    csv_path = out_dir / "motor_time_domain_results.csv"
    inputs_path = out_dir / "motor_design_inputs_and_estimates.csv"
    summary_path = out_dir / "motor_simulation_summary.txt"
    geometry_path = out_dir / "motor_3d_configuration.png"
    speed_path = out_dir / "rotation_speed_time.png"
    back_emf_path = out_dir / "back_emf_current_time.png"
    analysis_path = out_dir / "power_losses_efficiency_time.png"
    work_path = out_dir / "fan_work_output_demo.png"
    dashboard_path = out_dir / "motor_dashboard_2x3.png"

    df.to_csv(csv_path, index=False)
    save_design_inputs(inputs_path, material, geom, turns, constants, magnetic, motor)
    write_summary(summary_path, material, geom, turns, constants, magnetic, motor, df)

    plot_3d_configuration(geom, material, magnetic, geometry_path, show)
    plot_speed(df, speed_path, False)
    plot_current_back_emf(df, back_emf_path, False)
    plot_power_efficiency(df, analysis_path, False)
    plot_work_demo(df, work_path, False)
    plot_dashboard_grid(df, geom, magnetic, dashboard_path, show)
    if show:
        plt.show()

    final = df.iloc[-1]
    print("\nSimulation complete.")
    print(f"Output directory: {out_dir.resolve()}")
    print(f"Time-domain CSV: {csv_path}")
    print(f"Design/constant CSV: {inputs_path}")
    print(f"Summary file: {summary_path}")
    print(f"Final speed: {final['speed_rpm']:.2f} rpm")
    print(f"Final current: {final['current_A']:.3f} A")
    print(f"Final back EMF: {final['back_emf_V']:.3f} V")
    print(f"Final efficiency: {100.0 * final['efficiency']:.2f} %")
    print(f"Final fan torque: {final['fan_load_torque_Nm']:.6g} N m")
    print("\nFor efficiency analysis, use:")
    print("  efficiency = fan_output_power_W / (fan_output_power_W + total_loss_W)")
    print("  total_loss_W = copper_loss_W + eddy_current_loss_W + hysteresis_loss_W + mechanical_loss_W")


if __name__ == "__main__":
    main()
