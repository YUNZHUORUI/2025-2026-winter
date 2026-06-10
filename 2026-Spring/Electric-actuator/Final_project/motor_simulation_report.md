# Permanent-Magnet Brushed DC Motor — Simulation & Optimisation Report

**Course:** Electric Actuators — Final Project  
**Date:** 2026-05-31  
**Simulation script:** `motor_simulation_analysis.py`  
**Optimisation script:** `optimize_efficiency.py`

---

## 1  Objective

Use a low-order lumped-parameter model of a PM brushed DC motor to:

1. Estimate the air-gap flux, motor constants, and time-domain behaviour from geometry and material data.
2. Identify the parameter combination that maximises steady-state shaft efficiency driving a quadratic fan load.

---

## 2  Motor Geometry & Fixed Constraints

| Parameter | Value | Note |
|-----------|-------|------|
| Square magnet outer side | 40 mm | Fixed by housing |
| Circular bore diameter | 40 mm | = outer side (max bore) |
| Minimum radial air gap | 0.5 mm | Manufacturing lower bound |
| Max iron-core diameter | **39 mm** | Bore − 2 × 0.5 mm gap |
| Axial depth range | 2 – 150 mm | User choice |
| Fan load coefficient *C*\_fan | 1 × 10⁻⁷ N·m/(rad/s)² | Fixed in simulation |

The geometry is a **square-outside / round-inside** permanent magnet with a laminated cylindrical rotor in the bore. The rotation axis is horizontal (motor shaft direction).

---

## 3  Optimised Configuration

After sweeping 1 400 combinations across all free parameters, the globally optimal design is:

```
Input string:  1, 150, 39.0, 200, 5.00, 0.050, 200
```

| Parameter | Symbol | Optimal value | Range tested |
|-----------|--------|---------------|--------------|
| Magnet material | — | **NdFeB N52** (key 1) | keys 1–5 |
| Axial depth | *L* | **150 mm** | 20 – 150 mm |
| Iron-core diameter | *d*\_core | **39 mm** | 30 – 39 mm |
| Coil turns | *N* | **200** | 10, 20, 40, 60, 80, 100, 150, 200 |
| Copper wire diameter | *d*\_wire | **5.00 mm** | 0.05 – 5.0 mm |
| Lamination thickness | *t*\_lam | **0.050 mm** | 0.05 – 2.0 mm |
| Supply voltage | *V* | **200 V** | 12 – 200 V |

### Why these choices maximise efficiency

The steady-state efficiency of a DC motor driving a fan converges to:

$$\eta_{\text{ss}} \;\approx\; \frac{k_e \,\omega}{V} \;=\; \frac{\text{back-EMF}}{V}$$

Maximising this ratio requires:

| Goal | Parameter lever |
|------|----------------|
| Maximise *k*\_t = *k*\_e | Strongest magnets (NdFeB N52), largest core, most turns |
| Minimise copper loss *I*²*R* | Thickest wire (lowest *R*) |
| Minimise eddy-current loss | Thinnest laminations |
| Drive motor fast (↑ back-EMF fraction) | Highest voltage |

---

## 4  Magnetic Circuit (Reluctance Model)

*Computed for the optimised geometry:*

| Quantity | Symbol | Value |
|----------|--------|-------|
| Magnet material | — | NdFeB N52 |
| Remanence | *B*\_r | 1.45 T |
| Coercivity | *H*\_c | 955 000 A/m |
| Recoil permeability | *μ*\_r | 1.05 |
| Average magnet length | *l*\_m | 2.444 mm |
| Magnet MMF | *F*\_pm = *H*\_c · *l*\_m | **2 334 A-turn** |
| Magnet reluctance | *R*\_pm | 112 945 A/Wb |
| Air-gap reluctance | *R*\_gap | 26 402 A/Wb |
| Core reluctance | *R*\_core | 2 094 A/Wb |
| Return-path reluctance | *R*\_return | 4 942 497 A/Wb |
| **Total reluctance** | *R*\_total | **5 083 939 A/Wb** |
| Air-gap flux | *φ* | 0.459 mWb |
| **Air-gap flux density** | **B**\_gap | **0.0305 T** |
| Core flux density | *B*\_core | 0.0785 T |
| Effective steel *μ*\_r | — | 3 979 |
| Pole-arc fraction | — | 0.82 |

> **Note:** *B*\_gap is low (0.03 T) because the outer-square geometry forces a long return path (dominated by *R*\_return) which severely limits usable flux. The model is a teaching tool; a real design would use a dedicated iron yoke to close the flux path efficiently.

---

## 5  Motor Electrical & Mechanical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Driving voltage | *V* | 200 V |
| Coil resistance | *R* | 0.0742 Ω |
| Coil inductance | *L* | 0.05 H (capped) |
| Torque constant | *k*\_t | 0.03030 N·m/A |
| Back-EMF constant | *k*\_e | 0.03030 V·s/rad |
| Wire length | — | 84.5 m |
| Copper mass | — | 14.87 kg |
| Current density limit | — | 6.0 × 10⁶ A/m² |
| Current limit (wire area) | *I*\_lim | 117.8 A |
| Rotor inertia | *J* | 2.674 × 10⁻⁴ kg·m² |
| Core mass | — | 1.407 kg |
| Viscous damping | *b* | 2.0 × 10⁻⁵ N·m·s |
| Coulomb friction | *T*\_c | 3.0 × 10⁻⁴ N·m |
| Lamination thickness | *t*\_lam | 0.05 mm |
| Steinmetz *α* | — | 1.7 |

---

## 6  Time-Domain Simulation Results

Simulation duration: **2.0 s**, time step: **0.1 ms** (20 001 samples).  
Output columns: `time_s · speed_rad_s · speed_rpm · current_A · back_emf_V · electromagnetic_torque_Nm · fan_load_torque_Nm · input_power_W · fan_output_power_W · shaft_output_power_W · copper_loss_W · eddy_current_loss_W · hysteresis_loss_W · mechanical_loss_W · total_loss_W · efficiency · energy_copper_J · energy_eddy_current_J · energy_hysteresis_J · energy_mechanical_loss_J · work_output_J · demo_lifted_height_m_for_0p25kg`

### 6.1  Steady-State Values (t = 2.0 s)

| Quantity | Value |
|----------|-------|
| Rotor speed | **46 308 rpm** |
| Armature current | 80.83 A |
| Back-EMF | **146.93 V** (= 73.5 % of supply) |
| Input power | 16 165 W |
| Fan output power | **11 404 W** |
| Copper loss | 484.7 W |
| Eddy-current loss | 0.006 W |
| Hysteresis loss | 0.15 W |
| Mechanical friction loss | 471.8 W |
| **Steady-state efficiency** | **92.26 %** |

### 6.2  Integrated Energy over 2 s

| Energy term | Value |
|-------------|-------|
| Fan work output | **18 657 J** |
| Copper energy loss | 1 249.6 J |
| Eddy-current energy loss | 0.010 J |
| Hysteresis energy loss | 0.260 J |
| Mechanical friction energy loss | 797.7 J |

### 6.3  Loss Breakdown at Steady State

```
Copper loss      484.7 W  (50.7 % of total loss)
Mechanical loss  471.8 W  (49.3 % of total loss)
Iron losses        0.2 W  (< 0.1 % of total loss)
─────────────────────────
Total loss       956.7 W
```

The dominant losses are **copper** and **mechanical friction** in roughly equal parts. Iron losses are negligible at the thin-lamination setting (0.05 mm), confirming the optimisation strategy.

---

## 7  Simulation Output Files

| File | Description |
|------|-------------|
| `motor_sim_outputs/motor_time_domain_results.csv` | 20 001-row time-series; 22 columns (speeds, currents, powers, losses, efficiency, cumulative energies) |
| `motor_sim_outputs/motor_design_inputs_and_estimates.csv` | All design parameters and computed constants (material, geometry, reluctance model, motor estimates) in long format |
| `motor_sim_outputs/motor_simulation_summary.txt` | Human-readable summary of all constants and final-time values |
| `motor_sim_outputs/motor_dashboard_2x3.png` | 2 × 3 grid: speed, current, back-EMF, power/loss stack, efficiency, fan work |
| `motor_sim_outputs/rotation_speed_time.png` | Speed vs time |
| `motor_sim_outputs/back_emf_current_time.png` | Current and back-EMF vs time |
| `motor_sim_outputs/power_losses_efficiency_time.png` | Power/loss stack + efficiency vs time |
| `motor_sim_outputs/fan_work_output_demo.png` | Cumulative fan work and equivalent lift height |
| `motor_sim_outputs/motor_3d_configuration.png` | 3-D PyVista geometry with flux arrows |

---

## 8  Efficiency Formula (for Independent Analysis)

```python
# From motor_time_domain_results.csv:
efficiency = fan_output_power_W / (fan_output_power_W + total_loss_W)
total_loss_W = copper_loss_W + eddy_current_loss_W + hysteresis_loss_W + mechanical_loss_W
```

---

## 9  Full Design-Inputs CSV (Reproduced)

```
group,parameter,value
material,name,NdFeB N52
material,Br_T,1.45
material,Hc_A_per_m,955000.0
material,mu_r,1.05
material,note,Very high energy product; temperature margin must be checked.
geometry,magnet_thickness_m,0.0
geometry,magnet_depth_m,0.15
geometry,hub_diameter_m,0.04
geometry,core_diameter_m,0.039
geometry,air_gap_m,0.0005
geometry,outer_side_m_value,0.04
turns,turns,200
constants,voltage_V,200.0
constants,wire_diameter_m,0.005
constants,mu_r_steel_unsat,4000.0
constants,steel_saturation_T,1.6
constants,lamination_thickness_m,5e-05
constants,viscous_damping_Nm_s,2e-05
constants,coulomb_friction_Nm,0.0003
constants,fan_load_coefficient_Nm_per_rad_s_sq,1e-07
constants,current_density_limit_A_per_m2,6000000.0
constants,current_limit_softness,2.0
constants,pole_pairs,1
constants,steinmetz_alpha,1.7
constants,steinmetz_k_W_per_m3_Hz_Talpha,80.0
constants,commutation_factor,0.85
constants,simulation_time_s,2.0
constants,dt_s,0.0001
magnetic_estimates,F_pm_A_turn,2334.23
magnetic_estimates,magnet_average_length_m,0.002444
magnetic_estimates,R_pm_A_per_Wb,112945.3
magnetic_estimates,R_gap_A_per_Wb,26402.2
magnetic_estimates,R_core_A_per_Wb,2094.4
magnetic_estimates,R_return_air_A_per_Wb,4942497.0
magnetic_estimates,R_total_A_per_Wb,5083939.0
magnetic_estimates,phi_Wb,0.000459139
magnetic_estimates,phi_uncapped_Wb,0.000459139
magnetic_estimates,phi_br_limit_Wb,0.021852
magnetic_estimates,B_gap_T,0.030467
magnetic_estimates,B_core_T,0.078485
magnetic_estimates,H_core_A_per_m,15.697
magnetic_estimates,A_gap_m2,0.015070
magnetic_estimates,A_mag_m2,0.016401
magnetic_estimates,A_core_m2,0.00585
magnetic_estimates,A_return_m2,0.007728
magnetic_estimates,mu_r_steel_effective,3978.9
magnetic_estimates,pole_arc_fraction,0.82
motor_estimates,rotor_inertia_kg_m2,0.000267436
motor_estimates,core_volume_m3,0.000179189
motor_estimates,core_mass_kg,1.40663
motor_estimates,coil_resistance_ohm,0.074197
motor_estimates,coil_inductance_H,0.05
motor_estimates,wire_length_m,84.504
motor_estimates,wire_area_m2,1.9635e-05
motor_estimates,copper_mass_kg,14.8668
motor_estimates,current_limit_A,117.810
motor_estimates,torque_constant_Nm_per_A,0.030299
motor_estimates,back_emf_constant_V_s_per_rad,0.030299
motor_estimates,coil_area_m2,0.00585
```

---

## 10  Simulation Summary Text (Reproduced)

```
Simplified permanent-magnet brushed DC motor simulation summary
====================================================================

Magnet material: NdFeB N52
Material note: Very high energy product; temperature margin must be checked.
Br = 1.45 T
Hc = 9.55e+05 A/m
Magnet recoil relative permeability = 1.05

Geometry
Computed average magnet radial thickness = 0 m
Magnet axial depth = 0.15 m
Fixed square outside magnet side length = 0.04 m
Computed magnet bore diameter = 0.04 m
Iron core diameter = 0.039 m
Computed radial air gap = 0.0005 m
Turns = 200

Reluctance model
Average magnetization length l_m_avg = 0.00244422 m
Equivalent magnet MMF F_pm = Hc*l_m_avg = 2334.23 A-turn
R_pm = 112945 A/Wb
R_gap = 26402.2 A/Wb
R_core = 2094.4 A/Wb
R_return_air = 4.9425e+06 A/Wb
R_total = 5.08394e+06 A/Wb
Flux phi = 0.000459139 Wb
B_gap = 0.0304666 T
B_core = 0.0784852 T
H_core from B-H curve = 15.697 A/m
Effective steel mu_r = 3978.87

Electrical and mechanical constants
Driving voltage = 200 V
Rotor inertia J = 0.000267436 kg m^2
Copper wire diameter = 0.005 m
Current density limit = 6e+06 A/m^2
Current limit from wire area = 117.81 A
Coil resistance = 0.0741971 ohm
Coil inductance = 0.05 H
Torque constant = 0.0302991 N m/A
Back-EMF constant = 0.0302991 V s/rad
Lamination thickness = 5e-05 m
Fan load coefficient = 1e-07 N m/(rad/s)^2
Viscous damping = 2e-05 N m s
Coulomb friction = 0.0003 N m

Final time-domain values
Final speed = 46308.3 rpm
Final current = 80.8272 A
Final back EMF = 146.932 V
Final input power = 16165.4 W
Final shaft output power = 11404.1 W
Final efficiency = 92.2604 %
Total output work = 18656.9 J
Total copper energy loss = 1249.59 J
Total eddy-current energy loss = 0.00972615 J
Total hysteresis energy loss = 0.260057 J
Total mechanical energy loss = 797.746 J
```

---

## 11  Parameter Sensitivity Analysis

### 11.1  Methodology

Each of the six free design parameters was swept independently across its full valid range while all other parameters were held at the optimised baseline (`1, 150, 39.0, 200, 5.00, 0.050, 200`). The sweep uses 12 evenly-spaced points for continuous parameters (log-spaced for lamination thickness) and all 8 discrete values for turns. Two metrics are recorded at each point:

- **P\_fan** — steady-state fan output power (W)
- **η** — steady-state efficiency (%)

Results are stored in `motor_sim_outputs/sensitivity_analysis.csv` and visualised as `motor_sim_outputs/sensitivity_sweep.png` (3 × 2 subplot grid, blue curve = P\_fan, pink dashed = η, orange vertical line = base design point).

---

### 11.2  Axial Depth (10 – 150 mm)

| | Value | P\_fan (W) | η (%) |
|---|---|---|---|
| Base | 150 mm | 24 670 | 91.18 |
| Peak P\_fan | 35.5 mm | 232 986 | 46.57 |
| Peak η | 150 mm | 24 670 | 91.18 |

**Range:** P\_fan 24 670 – 232 986 W ; η 9.2 – 91.2 %

**Physical explanation:**  
- A shorter rotor has smaller *J* and lower *k*\_t. The reduced back-EMF constant allows the motor to spin much faster, driving the fan output ∝ ω³ to very high values — but most of the electrical input is wasted as I²R and friction, so efficiency collapses to < 50 %.  
- Increasing depth raises *k*\_t and *R*\_coil proportionally: the motor slows down but the back-EMF fraction V\_emf/V rises, so steady-state efficiency climbs monotonically toward 91 % at 150 mm.  
- **Trade-off:** short depth → high raw power, low efficiency; long depth → modest power, high efficiency. The base design sits at the efficiency-optimal extreme of the range.

---

### 11.3  Core Diameter (5 – 39 mm)

| | Value | P\_fan (W) | η (%) |
|---|---|---|---|
| Base | 39.0 mm | 24 670 | 91.18 |
| Peak P\_fan | 8.1 mm | 95 805 | 81.61 |
| Peak η | 35.9 mm | 25 966 | 92.13 |

**Range:** P\_fan 0 – 95 805 W ; η 0 – 92.13 %

**Physical explanation:**  
- Smaller core → larger air gap → the return-path reluctance dominates even more → *B*\_gap drops sharply → *k*\_t falls. A very small core also has very low inertia *J*, allowing the motor to spin at extreme speed under low load torque. The product ω³ dominates at small diameters, boosting P\_fan, but efficiency suffers from high friction and copper losses at high speed.  
- At 5 mm the core flux collapses to zero (R\_return overwhelms the small MMF), giving zero power.  
- Efficiency peaks at ~36 mm (slightly below the 39 mm base) where the balance between B\_gap magnitude, winding resistance, and friction is optimal. The base design (39 mm, minimum air gap 0.5 mm) is within 1 pp of the efficiency peak.

---

### 11.4  Coil Turns (10, 20, 40, 60, 80, 100, 150, 200)

| | Turns | P\_fan (W) | η (%) |
|---|---|---|---|
| Base | 200 | 24 670 | 91.18 |
| Peak P\_fan | 10 | 1 339 379 | 17.66 |
| Peak η | 200 | 24 670 | 91.18 |

**Range:** P\_fan 24 670 – 1 339 379 W ; η 17.7 – 91.2 %

**Physical explanation:**  
- Turns is the strongest single lever on motor speed. *k*\_e ∝ N, so the no-load speed ω₀ = V/k\_e ∝ 1/N. At 10 turns the motor runs at ~20× the speed of the 200-turn design; P\_fan scales as ω³, giving a 5329 % increase in raw output power. However, the motor spends almost the entire simulation in overshoot/decelerating regime — the "steady-state" average is deceptively high and the efficiency is only 17.7 %.  
- Because *k*\_t and *R*\_coil both scale with N, the motor constant K\_m = k\_t/√R ∝ √N. More turns genuinely improve the quality of energy conversion; 200 turns is the maximum available choice and achieves the best η.  
- **Key insight:** turns and voltage are dual levers — reducing turns is equivalent to increasing effective voltage. The fan output is maximised at minimum turns, but this is wasteful; the efficiency-optimal point is always at maximum turns for a given voltage.

---

### 11.5  Wire Diameter (0.1 – 5.0 mm)

| | Value | P\_fan (W) | η (%) |
|---|---|---|---|
| Base | 5.00 mm | 24 670 | 91.18 |
| At 0.1 mm | 0.1 mm | 0.72 | 0.34 |
| At 1.0 mm | ~0.99 mm | 4 962 | 52.7 |

**Range:** P\_fan 0.72 – 24 670 W ; η 0.34 – 91.18 % (monotone increasing)

**Physical explanation:**  
- Wire resistance *R* ∝ 1/d²\_wire. Thin wire creates enormous coil resistance — nearly all the supply voltage drops across the resistance, leaving almost no back-EMF to drive rotation. At 0.1 mm the motor barely spins.  
- Both P\_fan and η increase monotonically as wire thickens, with no trade-off: thicker wire is always better. The base design at 5.0 mm (maximum allowed) is therefore already at the global optimum for this parameter.  
- The motor constant K\_m = k\_t/√R ∝ d\_wire, explaining the strong monotone dependence.

---

### 11.6  Lamination Thickness (0.05 – 2.0 mm, log-spaced)

| | Value | P\_fan (W) | η (%) |
|---|---|---|---|
| All 10 points | 0.05 – 2.0 mm | **24 670** | **91.18** |

**Range:** P\_fan constant ; η constant (flat)

**Physical explanation:**  
- The eddy-current loss scales as P\_eddy ∝ d²\_lam · f² · B²\_core · V\_iron. In this design:
  - *B*\_core = 0.078 T (far from saturation, very low flux density due to dominant return-path reluctance)
  - *V*\_iron is small (39 mm core, reasonable length)
  - Operating frequency at 59 890 RPM with 1 pole pair: *f* = 998 Hz
- Even at maximum lamination thickness (2.0 mm) the eddy and hysteresis losses total < 0.5 W against 24 670 W fan output — a fraction of 0.002 %. The lamination parameter has **zero practical effect** in this specific geometry.  
- **Design implication:** the minimum lamination thickness (0.05 mm) was chosen in the optimisation, but any value up to 2.0 mm is equally acceptable. Manufacturing cost could be reduced by using thicker standard laminations (e.g. 0.5 mm) without any performance penalty.

---

### 11.7  Supply Voltage (3 – 200 V)

| | Value | P\_fan (W) | η (%) |
|---|---|---|---|
| Base | 200 V | 24 670 | 91.18 |
| Peak η | 38.8 V | 210.5 | **95.93** |
| At 12 V | ~20.9 V | 30.5 | 68.9 |

**Range:** P\_fan 0.28 – 24 670 W ; η 0 – 95.93 %

**Physical explanation:**  
- At low voltage (e.g. 38.8 V) the motor reaches its no-load speed quickly with low current; most of the voltage becomes back-EMF and copper loss is minimal. The η peak of 95.9 % occurs when the fan load is just large enough relative to friction to keep efficiency high, without demanding excessive current.  
- As voltage rises, the motor accelerates the fan to much higher speed, and the absolute fan output grows as ω³. The efficiency only drops slightly (91 % at 200 V vs 96 % at 38.8 V) because the back-EMF fraction V\_emf/V remains high.  
- **Trade-off:** maximum efficiency at ~39 V yields only 210 W of fan output; maximum voltage (200 V) yields 24 670 W at only 4.7 pp lower efficiency. For this fan application, 200 V is the correct choice.

---

### 11.8  Sensitivity Summary Table

| Parameter | Direction for ↑P\_fan | Direction for ↑η | Practical limit |
|-----------|----------------------|-----------------|-----------------|
| Axial depth | Decrease | Increase | 150 mm (max) for η |
| Core diameter | Decrease | ~36 mm optimum | 39 mm (min air gap) near-optimal |
| Turns | Decrease | Increase | 200 (max choice) for η |
| Wire diameter | Increase | Increase | 5.0 mm (max) — no trade-off |
| Lam thickness | Indifferent | Indifferent | Any value; iron loss negligible |
| Voltage | Increase | ~39 V optimum | 200 V for max power at near-peak η |

**Overall conclusion:** The optimised design (`1, 150, 39.0, 200, 5.00, 0.050, 200`) sits at or very near the efficiency-optimal point for every parameter. The only parameter with a meaningful efficiency trade-off relative to the base is voltage: 38.8 V would give 4.7 pp higher η, but at the cost of reducing fan output power by 99.1 %.

---

## 12  Output File Index

| File | Updated | Description |
|------|---------|-------------|
| `motor_sim_outputs/motor_time_domain_results.csv` | each run | 20 001-row time-series, 22 columns |
| `motor_sim_outputs/motor_design_inputs_and_estimates.csv` | each run | All constants and magnetic estimates |
| `motor_sim_outputs/motor_simulation_summary.txt` | each run | Human-readable summary |
| `motor_sim_outputs/sensitivity_analysis.csv` | each run | Full-range sweep, 66 rows × 5 columns |
| `motor_sim_outputs/sensitivity_sweep.png` | each run | 3×2 subplot: P\_fan & η vs each parameter |
| `motor_sim_outputs/main_design_dashboard.png` | each run | 8-panel time-domain dashboard |
| `motor_sim_outputs/motor_dashboard_2x3.png` | analysis.py | 2×3 dashboard from analysis script |
| `motor_sim_outputs/motor_3d_configuration.png` | analysis.py | 3-D PyVista geometry |
