"""
Parameter sweep to find the maximum-efficiency configuration for the PM DC motor simulation.

Strategy (from analytical reasoning):
  efficiency ≈ back_EMF / V = k_e * omega / V  (classical DC motor result)
  → maximize B_gap (NdFeB N52, large core, small air gap)
  → minimize R (thick wire, thin laminations)
  → maximize k_t (more turns, large A_coil)
  → high voltage drives omega high → fan load output scales as omega^3

Sweep approach: fix the clearly-optimal discrete choices first, then sweep the
continuous ones (axial depth, turns) to find the best combination.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from motor_simulation_analysis import (
    MAGNET_MATERIALS, Geometry, MotorConstants, FIXED_MAGNET_BORE_DIAMETER,
    DEFAULT_AIR_GAP, COIL_TURN_CHOICES, DEFAULT_FAN_COEFF, DEFAULT_SIMULATION_TIME,
    reluctance_model, motor_parameters, simulate,
)

import itertools

def evaluate(material_key, depth_mm, core_mm, turns, wire_mm, lam_mm, voltage):
    material = MAGNET_MATERIALS[material_key]
    hub_diameter_m = FIXED_MAGNET_BORE_DIAMETER
    air_gap_m = 0.5 * (hub_diameter_m - core_mm * 1e-3)
    from motor_simulation_analysis import FIXED_MAGNET_OUTER_SIDE
    magnet_thickness_m = max(0.0, 0.5 * (FIXED_MAGNET_OUTER_SIDE - hub_diameter_m))
    constants = MotorConstants(
        wire_diameter_m=wire_mm * 1e-3,
        lamination_thickness_m=lam_mm * 1e-3,
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
    try:
        magnetic = reluctance_model(material, geom, constants)
        motor = motor_parameters(geom, turns, material, magnetic, constants)
        df = simulate(geom, turns, material, constants, magnetic, motor)
        final = df.iloc[-1]
        return {
            "material": material_key,
            "depth_mm": depth_mm,
            "core_mm": core_mm,
            "turns": turns,
            "wire_mm": wire_mm,
            "lam_mm": lam_mm,
            "voltage": voltage,
            "efficiency": final["efficiency"],
            "speed_rpm": final["speed_rpm"],
            "final_back_emf_V": final["back_emf_V"],
            "input_power_W": final["input_power_W"],
            "output_power_W": final["fan_output_power_W"],
            "copper_loss_W": final["copper_loss_W"],
        }
    except Exception as e:
        return None


MAX_CORE_MM = (FIXED_MAGNET_BORE_DIAMETER - 2.0 * DEFAULT_AIR_GAP) * 1e3  # ~39 mm

# ----- Parameter grid -----
# Fixed (theoretically optimal):
MATERIALS   = ["1"]          # NdFeB N52 — highest energy product
WIRE_MMS    = [5.0]          # thickest wire → lowest copper loss
LAM_MMS     = [0.05]         # thinnest laminations → lowest eddy loss

# Swept:
DEPTHS_MM   = [20, 40, 60, 80, 100, 130, 150]
CORE_MMS    = [30, 35, 37, 38, 39]     # bounded by bore – 2*air_gap
TURNS_LIST  = COIL_TURN_CHOICES        # [10, 20, 40, 60, 80, 100, 150, 200]
VOLTAGES    = [12, 24, 48, 100, 200]

grid = list(itertools.product(MATERIALS, DEPTHS_MM, CORE_MMS, TURNS_LIST,
                               WIRE_MMS, LAM_MMS, VOLTAGES))

print(f"Evaluating {len(grid)} configurations …")

results = []
for i, (mat, dep, core, turns, wire, lam, volt) in enumerate(grid):
    if core > MAX_CORE_MM:
        continue
    r = evaluate(mat, dep, core, turns, wire, lam, volt)
    if r is not None:
        results.append(r)
    if (i + 1) % 200 == 0:
        print(f"  {i+1}/{len(grid)} done …")

results.sort(key=lambda x: x["efficiency"], reverse=True)

print("\n===== Top-10 configurations by steady-state efficiency =====")
header = f"{'Mat':>3} {'Dep':>5} {'Core':>5} {'Turn':>5} {'Wire':>5} {'Lam':>5} {'Volt':>5} | {'η%':>7} {'RPM':>9} {'EMF':>7} {'P_in':>7} {'P_out':>7} {'P_cu':>7}"
print(header)
print("-" * len(header))
for r in results[:10]:
    print(
        f"{r['material']:>3} {r['depth_mm']:>5.0f} {r['core_mm']:>5.1f} {r['turns']:>5} "
        f"{r['wire_mm']:>5.2f} {r['lam_mm']:>5.3f} {r['voltage']:>5.0f} | "
        f"{100*r['efficiency']:>7.3f}% {r['speed_rpm']:>9.1f} {r['final_back_emf_V']:>7.2f} "
        f"{r['input_power_W']:>7.2f} {r['output_power_W']:>7.2f} {r['copper_loss_W']:>7.3f}"
    )

best = results[0]
print("\n===== Best configuration =====")
for k, v in best.items():
    print(f"  {k}: {v:.5g}" if isinstance(v, float) else f"  {k}: {v}")

print("\n===== Equivalent single-line input string for motor_simulation_analysis.py =====")
print(f"{best['material']}, {best['depth_mm']:.0f}, {best['core_mm']:.1f}, "
      f"{best['turns']}, {best['wire_mm']:.2f}, {best['lam_mm']:.3f}, {best['voltage']:.0f}")
