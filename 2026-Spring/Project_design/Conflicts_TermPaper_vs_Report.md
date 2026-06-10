# Content Conflicts: Term Paper vs. DFM Report

Comparison of **Term Paper** (`main-26b175fa.tex`, IEEE "Solar-Assisted Tilt-Rotor UAV / SolarWing-V") and **Report** (`main-49d9f9b8.tex` + `sections/*`, "Solift H-VTOL").

Two kinds of problems exist: (1) **cross-document conflicts** where the two papers state different things about the same design, and (2) **internal conflicts** where a single document contradicts itself. Both should be reconciled, because a grader reading them together will notice immediately.

---

## A. Cross-document conflicts (term paper ≠ report)

| # | Item | Term paper says | Report says | Recommended fix |
|---|------|-----------------|-------------|-----------------|
| 1 | **Project / aircraft name** | "SolarWing-V", title "Solar-Assisted Tilt-Rotor UAV" | "Solift", "TECHPLANE001", "H-VTOL" | Pick ONE name everywhere. |
| 2 | **Propulsion configuration** | **Four** tilt-rotor nacelles, quad layout (2 wingtip + 2 tail) | **Three-axis**: 2 front tilting rotors + **1** rear fixed lift motor (Option C, selected) | Decide 3 vs 4 motors; make both docs match. This is the single biggest conflict. |
| 3 | **Airfoil** | NACA 4415 | MH114 | Pick one airfoil; all $C_L$/$L/D$/Reynolds text depends on it. |
| 4 | **Wing arrangement** | Single high-wing **monoplane**, tapered (λ=0.6), washout 2° | **Stacked triplane**, rectangular planform (ch.4) | Pick one. (Report is also internally split — see B1.) |
| 5 | **Wing structure / materials** | Pultruded CF-tube spar, Depron foam ribs + CF face sheets, composite fuselage | Balsa framework + XPS foam shell + heat-shrink film (ch.4); prepreg CFRP/Nomex (ch.5) | Pick one build standard. |
| 6 | **Cruise L/D** | CFD = **20.8** (α=4°); design assumption 12 | **4.8** (ch.6); **45.6 / 46.3** (ch.7) | Settle on one credible value. None of these agree. |
| 7 | **FEA peak stress / deflection** | Spar **148 MPa**, tip deflection **82 mm**; bracket 201 MPa, SF 2.5 | Wing **0.325 MPa**, deflection **0.207 mm** | Off by ~1000×. Re-run/re-state one consistent FEA. |
| 8 | **FEA load cases** | 3-g pull-out, 7 m/s gust, 12 N·m + 60 N nacelle torque | 70 N distributed + 40 N localized (steady cruise) | Align the load definitions. |
| 9 | **Battery mass** | **2.45 kg** | **1.68 kg** (1680 g) | Pick one; flows into the mass budget. |
| 10 | **Battery chemistry** | Si-anode **solid-state** Li-ion | Si-anode **LiPo** | Use one term. |
| 11 | **Motors** | All 4 = T-Motor U8 Lite KV100; ESC FLAME 80A | Front ×2 Sunnysky X4112S 340 KV; rear ×1 T-Motor U8 ~85 KV | Align motor list. |
| 12 | **Autopilot firmware** | Cube Orange+ on **ArduPlane/ArduCopter** (also mentions Pixhawk 6X) | Cube Orange+ on **PX4 v1.15** | Pick one FC + one firmware. |
| 13 | **Rotor / prop sizes** | Wingtip rotors ~0.50 m; tail 0.25 m | Front 22″ (~0.56 m); rear 28″ (~0.71 m) folding | Align diameters. |
| 14 | **Mission range / radius** | Delivery **radius 10 km** (Methods); "ideal 15 km" (lit review) | **Range ≥ 120 km** (FR-003, specs) | Huge gap. Decide radius vs total range and make consistent. |
| 15 | **Cargo containers** | 2× SF Express **F3** (300×200×160 mm, 2.5 kg each) | Cargo bay **500×300×200 mm** | Make bay dimensions fit the stated containers. |
| 16 | **Institution** | GTIIT, Shantou, China | Technion, Haifa, Faculty of Mech. Eng. (title page) | Same team — unify affiliation. (Course 034371 and advisor A. Dwivedi already match.) |

---

## B. Internal conflicts *within the report*

1. **Wing geometry contradicts itself.** Ch.4 (Concept) = stacked triplane, each wing 1200 mm × 480 mm, **AR 2.5**. Ch.5 (Tech Specs) = single wing, span 3050 mm, **AR 8.03**, area 1.159 m². These describe two different aircraft. → Fix ch.4 or ch.5 to match.
2. **L/D stated three ways.** Ch.6 = **4.8**; ch.7 intro = **45.6**; ch.7 airfoil trade-off = **46.3**. → One value.
3. **Empty weight.** Ch.5 mass budget OEW = **7.0 kg**; SRR PR-002 = empty **≤ 4.5 kg**. → Reconcile.
4. **Wing materials.** Ch.4 = balsa/XPS/film (student build); ch.5 = prepreg CFRP/Nomex honeycomb (aerospace). → Pick one.

## C. Internal conflicts *within the term paper*

1. **Payload.** Methods (§3.1) says "payload of up to **2 kg**", but the spec table, mass budget, abstract and conclusion all say **5.0 kg**. → Change the 2 kg line to 5 kg.
2. **CFD domain wingspan.** §3.3 sets "L = wingspan = **2.4 m**", but wingspan is **3.05 m** everywhere else. → Fix the 2.4 m.
3. **Rotor description.** §3.1 first says "four tilt-rotor nacelles … two on rear stabilizer," then describes the rear pair as "smaller fixed-pitch elevon motors" (i.e. not tilting). → Clarify whether the tail rotors tilt.

---

## Suggested priority

Fix in this order: **#2 (motor count) → #3/#4 (airfoil + wing) → #6/#7 (L/D + FEA numbers) → #14 (range) → #9/#11/#12 (battery, motors, firmware) → naming/affiliation → internal B/C items.**

Items #2, #3, #4, #6, #7 and #14 are the ones a reviewer will catch first, because the two documents are describing what looks like two different aircraft. The cleanest path is to treat the **report's ch.5 Tech Specs** as the single source of truth (it is the most complete, self-consistent spec sheet) and bring the term paper — and report ch.4/ch.6/ch.7 — into line with it.
