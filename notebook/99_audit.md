# 99 — Audit of Entries 00–04 and the Project Documents

*Audit entry, written 2026-11-12, before any fix. This is a report only: nothing in entries 00–04, the README, `project_details.md` or the proposal was changed. Every finding has a proposed fix, and the fixes are applied in STEP 2 only after the author approves this audit.*

---

## How this audit was done

- **Derivations** were re-checked symbolically (sympy) and numerically (numpy/scipy) by the scripts in [`checks/`](checks/). Run them with `python notebook/checks/run_all.py` from a Python environment with sympy, numpy and scipy installed. Each script prints PASS or FAIL per claim. **A FAIL marks a finding about the notebook, not a bug in the script.**
- **References** were checked against publisher metadata (Crossref, by DOI), arXiv abstract pages, PMLR / NeurIPS proceedings, the ETH Research Collection and the publishers' sites. dblp and OpenReview blocked automated access and were not used.
- **Vehicle and simulator facts** were read from source files: MuJoCo Menagerie `cf2.xml`, the MuJoCo documentation, RotorPy `crazyflie_params.py` and `multirotor.py`, the Bitcraze firmware and documentation, and the Förster (2015) thesis PDF. Numbers derived from these sources rather than read from them are marked *derived*.

Severity scale: **Critical** (a later phase would have to be redone), **Major** (a stated result or prediction is wrong or unsupported), **Minor** (imprecision, inconsistency or missing caveat).

---

## Summary of findings

| ID | Severity | Where | Finding |
|---|---|---|---|
| A-03-1 | **Major** | 03 | The "sharpest" standing test is false: torque-free energy is conserved under **either** sign of the gyroscopic term. Only the world-frame angular-momentum **vector** detects a flipped sign. |
| B-1 | **Major** | 02 | The relative-degree table puts a position reference at "0" integrations. A reference is not a plant input. |
| B-2 | **Major** | 04 vs 01 | c_tf/a ≈ 1/15 contradicts entry 01 and every verified source (Förster 0.18, firmware 0.21–0.23, RotorPy 1.12). |
| B-3 | **Major** | 04 (+02) | The mass and drag predictions assume an integral action that has not been decided. |
| E-1 | **Critical** | 01 / 04 / proposal | Förster's identified motor time constant is about 65 ms, which is **1.5 times the attitude time scale**. Motor lag is part of the real nominal plant, not only an injected mismatch. |
| E-2 | **Critical** | 00 / 01 / Phase 2 | The Menagerie Crazyflie is actuated by a **body wrench**, with moment gears of negative sign and an "arbitrary" thrust range (TWR 1.32). It has no rotors. |
| F-1 | **Critical** | proposal, Assumption 1 / Result 2(i) | Recursive feasibility of an MPC-for-tracking is a **nominal-model** result. Under plant–model mismatch it does not follow from Assumption 1. |
| F-2 | **Critical** | proposal eq. (3) | A steady-state map Γ has zero useful output for lag and delay, which have zero DC effect. |
| A-02-1 | Major | 02 | With rotor thrusts as inputs, the nonlinear model has no vector relative degree (the decoupling matrix is singular). "Relative degree 4 from torque" holds only in the hover linearization. |
| A-04-1 | Major | 04 | a·m·g/I_xx is not dimensionless (units s⁻²). |
| A-04-2 | Major | 04 | The mass "constant residual acceleration" is an open-loop statement. In closed loop it becomes a position offset or zero, depending on the disturbance model. |
| C-1…C-8 | Major | README / project_details / PDF | Hard versus soft placement, phase count, source of the non-degradation guarantee, number of vehicles, entry numbering, file paths and the bibliography all disagree. |
| D-1 | **Major** | README §14 | 9 README references are wrong or fabricated: [9], [10], [11], [12], [14], [16], [17], [20], [21]. The PDF list is correct for all 40 items checked, apart from two missing issue fields. |
| E-3…E-6 | Major | 01 / 03 | The inertia is not diagonal (up to 11 % off-diagonal). The c_tf range in entry 01 is unsupported at its top end. RotorPy is an outlier and uses a different rotor order. The firmware pitch sign is confirmed inverted. |
| A-x | Minor | various | Listed in section A. |

All other derivations in 00–04 pass. Across the six scripts, 50 checks pass and 6 fail; each failure is one of the findings above.

---

## A. Derivations in 00–04, re-checked

### A.1 Pass/fail table

| Entry | Claim | Check | Result |
|---|---|---|---|
| 00 | R_x(φ)e₃, R_y(θ)e₃, R_z(ψ)e₁ as written | [check_00_frames.py](checks/check_00_frames.py) | PASS |
| 00 | +roll lifts left side; +pitch is nose down; +yaw is nose left | same | PASS |
| 00 | Each elementary matrix is a right-hand rotation (dR/dangle at 0 = skew(axis)) | same | PASS |
| 01 | Rotor torque r_i × f_i e₃ = (y_i f_i, −x_i f_i, 0) | [check_01_mixer.py](checks/check_01_mixer.py) | PASS |
| 01 | Mixer M rows (thrust, roll, pitch, yaw) | same | PASS |
| 01 | Rows of M mutually orthogonal; det M = −16 a² c ≠ 0 | same | PASS |
| 01 | Closed-form inverse f₁…f₄ | same (M · M⁻¹_text = I) | PASS |
| 01 | Yaw/roll torque per unit rotor spread = c_tf/a | same | PASS |
| 02 | b₃ for pitch θ = (sin θ, 0, cos θ) | [check_02_translational.py](checks/check_02_translational.py) | PASS |
| 02 | Hover T = mg; coordinated tilt T = mg/cos θ; a_lat = g tan θ; TWR 2 gives 60° and 1.73 g | same | PASS |
| 02 | Inversion T_req = m‖a + g e₃‖, b₃_req = (a + g e₃)/‖·‖ (1000 random tests) | same | PASS (singular at a = −g e₃, see A-02-3) |
| 02 | "Relative degree 4 from τ" for lateral position | same (decoupling matrix) | **Qualified**, see A-02-1 |
| 03 | R(q) equals q ⊗ v ⊗ q* (Hamilton, body to world) | [check_03_rotational.py](checks/check_03_rotational.py) | PASS |
| 03 | RᵀR = I, det R = 1; b₃ is the third column; b₃ invariant under rotation about body z | same | PASS |
| 03 | q̇ = ½ q ⊗ [0, ω] ⇔ Ṙ = R[ω]× with body ω; q·q̇ = 0 | same | PASS |
| 03 | Numerical convention test (body rate about +y for 0.3 s) | same (RK4, renormalised) | PASS: nose (0.9553, 0, −0.2955), b₃ (0.2955, 0, 0.9553) |
| 03 | Torque-free energy conserved with the correct sign | same | PASS |
| 03 | "A wrong gyroscopic sign makes the energy drift" | same | **FAIL**, see A-03-1 |
| 04 | Hover (f_i = mg/4, level, at rest) is an equilibrium; the yaw family is an equilibrium | [check_04_hover.py](checks/check_04_hover.py) | PASS |
| 04 | Linearization: ẍ = g δθ, ÿ = −g δφ, z̈ = δT/m, δθ̇ = ω_y, ω̇_y = τ_y/I_yy | same (13-state Jacobian) | PASS |
| 04 | τ_y → x transfer function = g/(I_yy s⁴); x decoupled from collective thrust at hover | same | PASS |
| 04 | Hover authority τ_max = a·m·g·min(1, TWR − 1) | [check_01_mixer.py](checks/check_01_mixer.py) (exact LP, 60 TWR values) | PASS (relative error 8 × 10⁻¹⁶) |
| 04 | Same for yaw with c_tf in place of a | same | PASS |
| 04 | Mass bias Δz̈ = −g Δm/(m + Δm) | [check_04_hover.py](checks/check_04_hover.py) | PASS as algebra; open-loop only, see A-04-2 |
| 04 | Drag trim δθ ≈ d_x v/(mg) vs the exact trim with the full R D Rᵀ model | same | PASS (≤ 0.2 % error up to 2 m/s with placeholder magnitudes) |
| 04 | c_tf/a ≈ 1/15 | [check_01_mixer.py](checks/check_01_mixer.py) | **FAIL**, see B-2 |
| 04 | a·m·g/I_xx is a dimensionless group | [check_04_hover.py](checks/check_04_hover.py) | **FAIL**, see A-04-1 |

### A.2 Findings from the re-check

**A-03-1 (Major). The gyroscopic-sign test in entry 03 does not test the sign.** Entry 03 calls torque-free energy conservation "the sharpest" test and says that "if the sign of the cross-product term is wrong, torque-free rotational energy will drift". This is false. With the flipped sign, I ω̇ = +ω × Iω, we still have d/dt(½ωᵀIω) = ωᵀ(ω × Iω) = 0, and also d/dt‖Iω‖² = 0. Both quantities are conserved under **either** sign, because a cross product is orthogonal to both of its factors. Numerically (4 s, RK4) the relative energy drift is 3 × 10⁻¹⁵ with the correct sign and 8 × 10⁻¹⁵ with the flipped sign. What does detect the flip is the **world-frame angular momentum vector** R I ω: it is constant with the correct sign and drifts by 12 % with the flipped sign. Standing test 4 checks only the magnitude ‖R I ω‖ = ‖Iω‖, so it would also miss the error.
*Fix:* rewrite the energy check as a check of the energy and of the integrator, not of the sign. Make standing test 4 check the vector **L_W = R I ω** for constancy. Keep the energy check as test 4b.

**A-02-1 (Major). Relative degree with rotor-thrust inputs.** The inputs are u = f, equivalently (T, τ). Then ẍ = (T/m) b₃,ₓ(q), so the input T already appears at the **second** derivative of x, with coefficient b₃,ₓ/m. That coefficient vanishes at hover. The 2nd-derivative decoupling matrix of (x, y, z) with respect to (T, τ) has rank 1, so the nonlinear system has no well-defined vector relative degree. The standard remedy is dynamic extension: make T a state and T̈ an input, which gives vector relative degree (4, 4, 4, 2) for (x, y, z, ψ). This is shown for the quadrotor by Mistler, Benallegue and M'Sirdi (RO-MAN 2001, DOI 10.1109/ROMAN.2001.981968). The statement "lateral position is relative degree 4 from torque" is exact in the **hover linearization**, where x is decoupled from δT. Entry 04 confirms that decoupling, and the check script passes it.
*Fix:* in entry 02, say "in the linearization about hover" wherever relative degree is counted, and add a short paragraph on why the nonlinear count needs dynamic extension. The cascade argument, which is what entry 03 uses, is unaffected.

**A-02-2 (Minor).** "Relative degree" and "number of integrations" are used interchangeably. For the reference rows of the table they are different things; see B-1.

**A-02-3 (Minor).** The inversion is singular at a_des = −g e₃ (free fall: T = 0 and b₃ is undefined). It needs one sentence, because a reference shift that demands near-free-fall makes b₃_req ill-conditioned.

**A-04-1 (Major). a·m·g/I_xx is not dimensionless.** Its units are (m · kg · m s⁻²)/(kg m²) = s⁻². It is the maximum angular acceleration at hover. The dimensionless quantity is a time ratio formed with its inverse square root, t_att = (I_xx/(a m g))^{1/2}. This is the attitude time scale, about 43 ms for Förster's parameters (*derived*, [check_params_sources.py](checks/check_params_sources.py)). The normalized severities should then be τ_m/t_att, delay/t_att, horizon/t_att and Δt/t_att.
*Fix:* entry 04 replaces the "three dimensionless groups" with TWR, c_tf/a, I_zz/I_xx, and time ratios against t_att. The user's planned entry 07 ("a·m·g/Ixx" as a severity) inherits this correction.

**A-04-2 (Major). The mass bias is open-loop.** Δz̈ = −g Δm/(m + Δm) is the acceleration if the controller applies exactly mg. A feedback controller does not do that. The MPC raises thrust in proportion to the altitude error, so without a disturbance model the result is a **constant altitude offset**, and with one the offset is zero. This is the same conditionality as B-3, and it is fixed together with it.

**A-04-3 (Minor).** The column header "Relative degree from torque" in the classification table gives mass as 2. Mass acts through thrust, not torque.
*Fix:* rename the column "Channel order from its input" and say which input.

**A-04-4 (Minor).** The hover-authority formula holds for f_min = 0. The general form, checked against the LP, is τ_max = a·min(4 f_max − mg, mg − 4 f_min). Entry 01 says the real vehicle idles at a small positive thrust, so the general form should be stated.

**A-04-5 (Minor).** "The cliff is at τ_m/T_horizon" mixes two different timescales. The phase loss of a first-order lag at crossover ω_c is arctan(τ_m ω_c). That depends on the closed-loop bandwidth, not on the horizon. The horizon matters through whether the MPC's prediction is still valid over it. Both ratios belong in entry 07, with τ_m ω_c as the primary one.

**A-03-2 (Minor).** "I_zz roughly the sum of the other two": for the sources it is 0.88 (Förster) and 1.01 (RotorPy) times I_xx + I_yy. That is fine, but the claim should cite the identified values once they are committed.

**A-00-1 (Minor).** The MuJoCo expectations in entry 00 are now confirmed. Free-joint linear velocity is in the world frame and angular velocity in the body frame, and qpos uses a w-first quaternion. One caveat must be added: the free-joint linear velocity is that of the **body-frame origin**, which equals the centre-of-mass velocity only because the Menagerie model has `inertial pos="0 0 0"`. Source: MuJoCo docs, overview and XML reference (see E-2).

**A-00-2 (Minor).** The Crazyflie pitch-sign warning in entry 00 is **confirmed**. The Bitcraze coordinate-system documentation says reported pitch is "positive counter-clockwise – a positive pitch raises the drone's nose". The firmware's `kalman_core.c` stores `.pitch = -pitch*RAD_TO_DEG` ("adjusted for the legacy CF2 body coordinate system"). Entry 00 can now cite this instead of "historically".

**A-01-1 (Minor).** The motor numbering correspondence in entry 01 is **confirmed** by the firmware mixer (`power_distribution_quadrotor.c`): M1 front-right, M2 rear-right, M3 rear-left, M4 front-left. Bitcraze's propeller page says the CCW props go on M1 and M3 and the CW props on M2 and M4. So our rotors 1 (front-left = M4) and 3 (back-right = M2) are clockwise, as entry 01 assumes. Entry 01 can drop "as we recall it".

---

## B. The three known issues: confirmed, with proposed fixes

### B-1. "Position reference: 0 integrations" (entry 02): confirmed, conceptually wrong

The table in entry 02 lists *injection points* in its rows, but its numbers count *integrators between the plant input and the output*. A reference is not a plant input. Whatever reference the MPC receives, the vehicle still realizes it through the same four rotor thrusts and the same chain: rotor thrusts → τ → ω → q → b₃ → v → p. That is four integrators to lateral position, and two from collective thrust to height. The "0" row also contradicts the sentence two lines below it, that a reference-level correction is realized "over the MPC horizon".

What actually differs between injection points is not the plant's integrator count. It is:

1. **Who computes the plant input.** A post-solve feedforward (on torque or body rate) is added *after* the optimizer, so it bypasses the input constraints and the MPC's knowledge of the chain. A reference shift is turned into inputs *by the constraint-aware MPC*, which plans the whole four-integrator chain under the rotor box and rate limits.
2. **Preview.** A reference shift is seen across the whole horizon, so the MPC can start the rotate-then-translate manoeuvre early. A feedforward acts only at the current step.
3. **The closed-loop transfer from reference to output.** The correction reaches the output through T_ry(z), the reference-to-output map of the closed loop. That map has finite bandwidth and DC gain about 1 (exactly 1 if the MPC is offset-free). Under saturation it becomes a nonlinear map. This transfer function, not "0 integrations", is the object the realizability ratio and the small-gain bound are computed on.

*Proposed fix:* split the table in two.
- **Table (a): plant structure.** Integrators from each *plant input* to each output: rotor thrust/torque gives 4 lateral and 2 vertical. Inner-loop-assumed interfaces (body rate, attitude, acceleration) give 3, 2 and 2, each labelled with its idealization.
- **Table (b): injection points.** For each point (post-solve torque feedforward, post-solve body-rate feedforward, MPC model residual, MPC reference), give: who computes the input, whether input constraints are respected, whether there is preview, and the transfer from the correction to the output (G_plant, G_plant, model-dependent, T_ry).

The thesis sentence becomes: *the same plant chain is used by every injection point; what differs is whether the correction passes through the constraint-aware optimizer, with preview, and through which closed-loop transfer.* The note in project_details.md about the "relative-degree-3 effect at a body-rate feedforward interface" stays valid as a row of table (a).

### B-2. c_tf/a ≈ 1/15 in entry 04: confirmed contradiction

Entry 01 gives c_tf = 0.006–0.025 m. With a = 0.0325 m that is c_tf/a = 0.18–0.77, so 1/15 = 0.067 lies outside entry 01's own range ([check_01_mixer.py](checks/check_01_mixer.py)). The verified sources (section E) give:

| Source | c_tf (m) | arm (m) | c_tf/a |
|---|---|---|---|
| Förster 2015, eq. 3.5 | 0.005965 (plus offset 1.56 × 10⁻⁵ N·m per rotor) | 0.046 (*derived*: half of the 0.092 m motor-to-motor distance) | **0.183** |
| Bitcraze firmware, 2.1+ props | 0.006993 | 0.046 | 0.215 |
| Bitcraze firmware, legacy props | 0.007351 | 0.046 | 0.226 |
| RotorPy | 0.0339 (= 7.8 × 10⁻¹⁰ / 2.3 × 10⁻⁸) | 0.043 | **1.115** (outlier, 5.7× Förster) |

So 1/15 matches no source. The upper end of entry 01's range (0.025 m) is not supported by any verified source either, and RotorPy lies outside the range altogether.

*Proposed fix:*
- Entry 04 deletes "≈ 1/15" and writes c_tf/a symbolically, with the committed value (≈ 0.18 from Förster) once entry 01 commits parameters.
- Entry 01's "typical order" table is replaced by a cited parameter table (see E).
- RotorPy's k_m is recorded as inconsistent with the source it cites. When RotorPy is used as the second simulator, its c_tf must be overridden to the Förster value, or the discrepancy must be treated as a deliberate cross-simulator mismatch. That is the author's decision (open question Q4).

### B-3. The "drag does nothing" prediction (entry 04): confirmed, and it extends to mass

The drag paragraph says "A well-tuned MPC with integral-like action already rejects smooth state-dependent disturbances". No such action has been decided. Entry 06 must decide it. The prediction table should therefore carry a condition column. The two cases differ qualitatively:

| Mismatch | Nominal MPC **without** disturbance model | Nominal MPC **with** disturbance model (offset-free) |
|---|---|---|
| Mass | Constant altitude offset, proportional to Δm g divided by the vertical feedback gain. A residual removes it, so it **helps**. | Offset removed by the MPC itself. A residual is redundant: **little change**, or slightly faster transients. |
| Drag | Velocity-dependent position lag on constant-velocity segments. A residual reduces it: **helps**, by an amount set by drag size. | Constant-velocity offset removed; transients remain. **Little change.** |
| Motor lag | Phase loss; equilibrium unchanged. A state-dependent residual adds loop gain: **hurts past a threshold**. | Same, and the observer adds its own dynamics. The threshold moves: earlier if the observer is fast, later if it is slow. |
| Delay | As lag, with an earlier threshold. | As lag. |

Two observations sharpen this.
- **The "does nothing" regime for drag may be a property of the integral action, not of drag.** That is a stronger and more testable statement than the current one.
- **Drag is small on this vehicle.** Förster's model makes the drag coefficient proportional to the sum of rotor speeds, K_aero = −9.18 × 10⁻⁷ kg rad⁻¹. At hover that gives d ≈ 7 × 10⁻³ N·s/m and a trim tilt of about 0.025 rad per m/s (*derived*, order of magnitude only). So drag may be "invisible" mainly because it is small at Crazyflie speeds. Entry 07 should normalize drag severity as d·v_max/(mg).

*Proposed fix:* every prediction in entry 04 (and in README §6) becomes conditional on the disturbance-model decision. The table gets the two columns above. Entry 04 records that entry 06 must make the decision and that the predictions are re-registered at that point. Entry 02's "the drone will slowly sink or climb" gets the same open-loop qualifier.

---

## C. Inconsistencies between README, project_details.md and the proposal

| # | Topic | README | project_details.md | Proposal PDF |
|---|---|---|---|---|
| C-1 | **Hard layer contents** | min(r_H, α_sg, α_cbf), analytic only | α_safe (stability margin) + complete-non-realizability **veto** + extreme-OOD **veto**; graded realizability is in the **soft** layer | as README (eq. 9) |
| C-2 | **Uncertainty** | soft layer, information only | soft layer as calibrated uncertainty, **and** in the hard layer as an extreme-OOD veto | soft only ("not as a safety bound") |
| C-3 | **Soft layer** | ADP actor–critic (ADHDP), increments in trust, rate-limited | hand-designed: uncertainty, graded realizability, effectiveness, temporal smoothing; "soft only reduces α"; RL **deferred** | as README |
| C-4 | **Source of non-degradation** | "admissible initial policy [14]" then statistical acceptance test; RQ4 "statistical non-degradation" | Simplex / run-time assurance: "provably reverts", "cannot break the controller"; formal theorem **deferred** | statistical test (Result 3); says exact policy-iteration monotonicity "cannot be assumed" |
| C-5 | **Phases** | 8 phases, mapped to 4 semesters | 12 phases (0–11), with realizability (5) before the oracle (6) and the residual (7) | 4 semesters; Result 2 in semester 3 |
| C-6 | **Vehicles / simulators** | Crazyflie-scale + "a second, heavier vehicle"; unnamed second simulator | ≥ 3 vehicles (light / nominal / heavy) | "a second vehicle"; unnamed second simulator |
| C-7 | **Injection points** | a single one: the MPC reference | "injection-point-aware", several candidate points (attitude / acceleration / velocity references) | a single one: the MPC reference |
| C-8 | **Notebook numbering** | reference-shift derivation in "notebook 07" | 05 discretization, 06 MPC, 07 reference shaping, 08 realizability, 99 open questions | — |

Also:
- Entry 04 announces "05, the nominal MPC", while entry 02 and project_details place discretization at 05. The plan for this session is 05 discretization, 06 MPC, 07 perturbations, 08 reference shaping, 09 realizability, 10 loop gain, 11 barrier, 12–15.
- README §9 says the proposal is at `docs/Research_Proposal.pdf`, but the file is at the repository root, and the README tree omits `project_details.md`.
- project_details §5 lists `01_forces_torques.md` (the actual file is `01_forces_and_torques.md`), `PROJECT_NORTH_STAR.md` (actual: `project_details.md`), and `ROADMAP.md` / `PROGRESS.md`, which do not exist.
- Venue: the proposal plans the conference paper in semester 2 on "the hard layer and the regime map", with Result 2 in semester 3. The project decision now puts Results 1–2 in the IROS 2027 paper, so Result 2 moves earlier.
- README [19] is the GitHub repository; the PDF [19] is the iCONEECT paper. The README body cites [19] for the *paper*. (Self-citation; not checked externally.)
- Wang et al. [12]: the README calls the technique a high-order construction for barrier bounds, and the session brief describes it as a "disturbance-observer / HO-RCBF method". The verified arXiv abstract expands HO-RCBF as **high-order reciprocal** CBF. It achieves robustness "without knowing the disturbance bound", via adaptive mechanisms. **Neither abstract mentions a disturbance observer.** Whether the paper uses one is **[UNVERIFIED]**: the TAC full text is paywalled and was not read.

### Proposed single source of truth

**The notebook is the single source of truth for every technical statement.** The other documents relate to it as follows:
- **README** is a public *summary* of the notebook. It is regenerated whenever an entry changes, in the same commit, and never states anything the notebook does not.
- **The proposal PDF** is a dated, submitted artifact (October 2026) and is not edited. An errata list in the notebook (a short section of entry 14, or a file `98_errata_proposal.md`) records where the theory now differs from it.
- **project_details.md** is the v1 planning document. It gets a banner, "Superseded by the notebook and README as of [date]; kept for history", and is not edited further.

Content resolution (recommended, pending the author's approval):

| Conflict | Resolution | Reason |
|---|---|---|
| C-1, C-2 | Hard layer = min(r_H, α_sg, α_cbf), analytic quantities only. Uncertainty and OOD signals go to the soft layer only. A binary non-realizability veto is the special case r_H = 0 and needs no separate rule. | An uncertainty-based veto in the hard layer would make Result 2 depend on calibration of the ensemble, which cannot be certified. |
| C-3 | Soft layer = learned (ADP) policy choosing α ∈ [0, ᾱ]. The hand-designed rule survives as baseline 4. | Matches RQ3 and the proposal. |
| C-4 | Two separate guarantees, named separately. **(i) Deterministic:** α = 0 reproduces the nominal MPC exactly, and Result 2 (safety for any α ≤ ᾱ under stated assumptions). This is the run-time-assurance part. **(ii) Statistical:** non-degradation of accepted policies (Result 3), from the safe-policy-improvement test (Thomas et al. 2015; Laroche et al. 2019). **ADP theory supplies neither.** The "admissible initial policy" of Liu & Wei 2014 is a *convergence* premise for exact policy iteration, which the proposal itself says does not apply under partial observability. README §3 must stop implying otherwise. | Removes the ambiguity flagged in the brief. |
| C-5 | README's 8 phases, re-mapped to the IROS 2027 deadline. project_details' 12 phases are archived. | One roadmap. |
| C-6 | Crazyflie 2.x (Förster parameters) + one heavier vehicle; second simulator RotorPy. "≥ 3 vehicles" dropped, or moved to future work. | Project decision. |
| C-7 | One injection point (the reference), with the other points kept only as *comparison rows* in entry 02's table (b) and as baseline 7 (residual inside the MPC model). | The multi-point claim is unsupported by the current method. |
| C-8 | Adopt the session numbering (05–15, 99 audit). Entry 04's forward pointer and README's "notebook 07" are updated. | — |

---

## D. Reference check (README §14 vs proposal PDF)

The PDF list is correct for every item checked. The README list diverges from it in 11 places, and in 9 of them the README entry is wrong or does not exist. **The README bibliography must be replaced by the corrected PDF list.** The README has 43 entries; the PDF has 40, plus CasADi, IPOPT and MuJoCo in the README.

| # | README entry | Verdict | Correct citation (verified) |
|---|---|---|---|
| 1–8 | as PDF | OK | Add DOIs, e.g. [2] 10.1016/j.automatica.2013.02.003; [5] RA-L 6(2):3769–3776, 10.1109/LRA.2021.3061307 |
| 9 | Wabersich & Zeilinger, CDC 2021 | **WRONG** (no such CDC 2021 paper) | K. P. Wabersich, M. N. Zeilinger, "A predictive safety filter for learning-based control of constrained nonlinear dynamical systems," *Automatica* 129:109597, 2021. DOI 10.1016/j.automatica.2021.109597 |
| 10 | Berkenkamp, Moriconi, Schoellig, Krause, "Safe learning of dynamic systems," CDC 2017 | **WRONG** (title does not exist) | F. Berkenkamp, M. Turchetta, A. P. Schoellig, A. Krause, "Safe model-based reinforcement learning with stability guarantees," NeurIPS 30, 2017. arXiv:1705.08551 |
| 11 | X. Zhang, St. John, Yang, Grizzle, CDC 2020 | **WRONG** (not found; treat as fabricated) | T. Zhang, J. Xu, H. Zhang, "Enhancing safety in model-based reinforcement learning with high-order control barrier functions," *Int. J. Robust Nonlinear Control* 35(9):3844–3855, 2025. DOI 10.1002/rnc.7888 (the PDF omits the issue "(9)") |
| 12 | Q. Wang, X. Zhang, H. Wang, TAC 67(8) 2022 | **WRONG** (not found; treat as fabricated) | X. Wang, H. Zhang, S. Wang, W. Xiao, M. Guay, "Safe learning control with optimality and stability guarantees," *IEEE TAC*, early access, 2026. DOI 10.1109/TAC.2026.3707508; arXiv:2501.15373 (v1 had a different title). The brief's suggested correction is **confirmed**. |
| 13 | Lewis & Vrabie | OK | IEEE CAS Magazine 9(3):32–50, 2009. DOI 10.1109/MCAS.2009.933854 |
| 14 | Y. Liu, C. Wei, "Admissible initial policies in ADP," Automatica 115, 2020 | **WRONG** (not found) | D. Liu, Q. Wei, "Policy iteration adaptive dynamic programming algorithm for discrete-time nonlinear systems," *IEEE TNNLS* 25(3):621–634, 2014. DOI 10.1109/TNNLS.2013.2281663 |
| 15 | Lakshminarayanan et al. | OK | NeurIPS 30, 2017. arXiv:1612.01474 |
| 16 | Rawlings & de la Peña, CDC 2007 | **WRONG** (not found) | D. Limon, I. Alvarado, T. Alamo, E. F. Camacho, "MPC for tracking piecewise constant references for constrained linear systems," *Automatica* 44(9):2382–2387, 2008. DOI 10.1016/j.automatica.2008.01.023 |
| 17 | Garone, Di Cairano, Bemporad, IEEE CSM 38(1), 2018 | **WRONG** (no such article) | E. Garone, S. Di Cairano, I. Kolmanovsky, "Reference and command governors for systems with constraints: A survey on theory and applications," *Automatica* 75:306–328, 2017. DOI 10.1016/j.automatica.2016.08.013 |
| 18 | Mesbahi & Egerstedt, 2010 | OK, but a different work from the PDF | Both are real. Book: Princeton UP 2010, DOI 10.1515/9781400835355. PDF: H. Zhang, F. L. Lewis, A. Das, *IEEE TAC* 56(8):1948–1952, 2011, DOI 10.1109/TAC.2011.2139510. Keep Zhang–Lewis–Das, because the sentence it supports is "cooperative optimal control on graphs". |
| 19 | GitHub repo | self-citation, not checked | Use the PDF's iCONEECT paper entry, and link the repository separately |
| 20 | Ames, Xu, Grizzle, Tabuada, CDC 2014, "…autonomous driving" | **WRONG** (mash-up of two papers) | Ames, Coogan, Egerstedt, Notomista, Sreenath, Tabuada, "Control barrier functions: Theory and applications," ECC 2019, pp. 3420–3431. DOI 10.23919/ECC.2019.8796030. **Add** for the *discrete-time* condition actually used: A. Agrawal, K. Sreenath, "Discrete control barrier functions for safety-critical control of discrete systems with application to bipedal robot navigation," RSS XIII, 2017. DOI 10.15607/RSS.2017.XIII.073 |
| 21 | S. Sha, PhD thesis 2020 | **WRONG** (not found; treat as fabricated) | L. Sha, "Using simplicity to control complexity," *IEEE Software* 18(4):20–28, 2001. DOI 10.1109/MS.2001.936213. The brief's suggested correction is **confirmed**. |
| 22–23 | | OK | [22] DOI 10.1126/scirobotics.abm6597; [23] AIChE J 49(2):426–437, DOI 10.1002/aic.690490213 |
| 24 | Hanover et al., RA-L **7(1)** | **issue wrong** | RA-L **7(2)**:690–697, 2022. DOI 10.1109/LRA.2021.3131690 |
| 25 | Liu et al., arXiv:2101.09298 | OK | A journal version exists: *ASME J. Autonomous Vehicles and Systems* 1(4):041003, 2021, DOI 10.1115/1.4053244. Cite the journal version. |
| 26–35 | | OK | [26] CDC 2023 pp. 6533–6540, DOI 10.1109/CDC49753.2023.10383965; [27] "J. Aparicio Ojea" is the full surname; [29]–[33] arXiv IDs, titles and authors confirmed; [35] note the corrigendum *Automatica* 39(10):1845–1846, 2003 |
| 36 | Jiang, Teel, Praly | OK, add the issue | MCSS 7(2):95–120, 1994. DOI 10.1007/BF01211469 |
| 37–39 | | OK | [38] TAC 63(11):3735–3750, DOI 10.1109/TAC.2018.2798803; [39] PMLR 37:2380–2388 |
| 40 | acados | OK, add the issue | MPC 14(1):147–183, 2022. DOI 10.1007/s12532-021-00208-8 |
| 41–43 | CasADi, IPOPT, MuJoCo | OK | DOIs 10.1007/s12532-018-0139-4; 10.1007/s10107-004-0559-y; 10.1109/IROS.2012.6386109 |

Verified additions that later entries will need:
- W. Xiao, C. Belta, "High-order control barrier functions," *IEEE TAC* 67(7):3655–3662, 2022. DOI 10.1109/TAC.2021.3105491.
- Agrawal & Sreenath, RSS 2017 (above).
- Ferramosca et al., "MPC for tracking with optimal closed-loop performance," *Automatica* 45(8):1975–1978, 2009. DOI 10.1016/j.automatica.2009.04.007.
- Thomas, Theocharous, Ghavamzadeh, "High-confidence off-policy evaluation," AAAI 29(1), 2015. DOI 10.1609/aaai.v29i1.9541.
- Miyato et al., "Spectral normalization for GANs," ICLR 2018. arXiv:1802.05957.
- Fazlyab et al., "Efficient and accurate estimation of Lipschitz constants for deep neural networks," NeurIPS 32, 2019. arXiv:1906.04893.
- Mistler, Benallegue, M'Sirdi, RO-MAN 2001, DOI 10.1109/ROMAN.2001.981968.
- J. Förster, "System identification of the Crazyflie 2.0 nano quadrocopter," **Bachelor thesis**, ETH Zurich, 2015. DOI 10.3929/ethz-b-000214143.
- S. Folk, J. Paulos, V. Kumar, "RotorPy," arXiv:2306.04485, 2023 (ICRA 2023 workshop paper).
- MuJoCo Menagerie (Zakka, Tassa et al.), GitHub, 2022, with the official BibTeX in the repository README.

Unverified items: none in the bibliography itself. Two items stay **[UNVERIFIED]**: whether Wang et al. [12] use a disturbance observer (full text not read), and the page numbers of the AAAI 2015 paper.

---

## E. Vehicle and simulator facts (project decisions)

**E-1 (Critical). Motor lag is part of the real nominal plant.** Förster identifies the command-to-thrust dynamics as H(z) = 7.23 × 10⁻⁸ / (1 − 0.96954 z⁻¹), from data at 500 Hz. That gives τ_m ≈ 65 ms (77 ms from the second data series) (*derived*). RotorPy uses τ_m = 72 ms on rotor *speed*. The attitude time scale is t_att = (I_xx/(a m g))^{1/2} ≈ 43 ms, so τ_m/t_att ≈ 1.5 ([check_params_sources.py](checks/check_params_sources.py)). On a Crazyflie the motor is therefore *slower* than the attitude dynamics it drives. An MPC that assumes ideal actuators, flying a plant with the real motor, already sits on the "lag" axis of the mismatch study at zero injected mismatch.

This has to be decided before entry 06. Two options:
- **(a)** The nominal MPC model includes the identified motor lag: an augmented state with delivered thrusts, and the input is commanded thrusts. The "lag mismatch" is then an *error* in τ_m.
- **(b)** The nominal model keeps ideal actuators and the plant's nominal τ_m is itself a mismatch. Then "zero mismatch" is not physically meaningful for this vehicle.

Option (a) is recommended. It also changes entry 00's statement that "the nominal MPC model uses only the rigid-body state with ideal actuators" (open question Q1).

**E-2 (Critical). The Menagerie Crazyflie is wrench-actuated.** In `cf2.xml`, `body_thrust` is a motor with gear (0 0 1 0 0 0) and ctrlrange 0–0.35 N, and `x_moment`, `y_moment`, `z_moment` have gear (0 0 0 −1e-5 …), so **positive control gives negative moment**. The range is ±1 × 10⁻⁵ N·m, all applied at the site "actuation" at the body origin. There are no rotor sites and no arm geometry. The README calls the ctrlranges "currently arbitrary". The thrust range alone implies TWR = 1.32. Mass is 27 g; inertia is diag(2.40, 2.40, 3.23) × 10⁻⁵ kg·m², taken from Landry (MIT), not from Förster. The integrator is RK4; the timestep is MuJoCo's default 0.002 s. The axis convention of the mesh frame is not stated in the file (assumed FLU, **[UNVERIFIED]**).

Consequences:
- Our wrapper must own the actuation: rotor thrust commands → (motor lag, delay) → per-rotor box clip → M → wrench.
- The wrench is applied either through `xfrc_applied` (world-frame force and torque at the CoM) or through the existing actuators with the sign of the moment gears inverted and their ctrlranges widened.
- Mass and inertia must be overridden to the committed (Förster) values.
- Phase 2 must test the moment sign explicitly.

**E-3 (Major). Förster's inertia is not diagonal.** I_CF has off-diagonal terms up to 11 % of the diagonal (I_yz = 1.80 × 10⁻⁶ vs I_yy = 1.67 × 10⁻⁵). Entry 03's "diag for a symmetric quadrotor" is an idealization that must be stated: either the nominal model uses the full matrix, or the diagonal approximation is a (small) deliberate mismatch. Recommendation: the plant uses the full matrix, the MPC uses the diagonal, and the noise floor at zero injected mismatch (entry 05) absorbs it.

**E-4 (Major). Parameter sources disagree, and entry 01's table is uncited.**

| Quantity | Förster 2015 | Bitcraze firmware | RotorPy | Menagerie |
|---|---|---|---|---|
| mass (g) | 28.0 | 29 | 30 | 27 |
| I_xx, I_zz (10⁻⁵ kg·m²) | 1.66, 2.93 (full matrix) | — | 1.43, 2.89 | 2.40, 3.23 |
| arm (m) | 0.046 (*derived*) | 0.046 | 0.043 | — |
| c_tf (m) | 0.005965 + offset | 0.00699 (2.1+ props) | 0.0339 | — |
| f_max per rotor (N) | ≈ 0.15 | 0.12 (2.1+), 0.18 (thrust kit) | 0.144 (*derived*) | — |
| TWR | 2.18 | 1.69 (2.1+ props) | 1.95 | 1.32 (ctrlrange) |
| motor lag | 65–77 ms on thrust (*derived*) | — | 72 ms on rotor speed | none |

Förster's thrust and torque maps are affine with offsets: f = 2.13 × 10⁻¹¹ cmd² + 1.03 × 10⁻⁶ cmd + 5.48 × 10⁻⁴, and τ = 0.005965 f + 1.56 × 10⁻⁵. The torque offset cancels in the yaw sum because Σ s_i = 0. The thrust offset sets a non-zero f_min, so A-04-4 applies.

*Fix:* entry 01 gets a cited parameter table, Förster as primary. Every number gets a source and a page or equation reference, and the cross-source spread is recorded as uncertainty.

**E-5 (Major). RotorPy conventions differ from ours.** Its rotor order is r1 (+,+), r2 (+,−), r3 (−,−), r4 (−,+), i.e. front-left, front-right, back-right, back-left. That is a *clockwise* numbering, whereas entry 01 numbers counter-clockwise. Rotor directions are [1, −1, 1, −1] in RotorPy's own sign convention, which must be mapped. RotorPy's lag acts on rotor speed (`rotor_accel = (1/tau_m)*(cmd - speed)`). Its k_m/k_eta = 0.0339 m contradicts the Förster value its own comment cites. A permutation matrix and a convention check are needed before RotorPy results are compared.

**E-6 (Minor).** The facts recorded in entry 00 and entry 01 as "to verify in Phase 2" are now verified (A-00-1, A-00-2, A-01-1). One remains: the MuJoCo integrator default is semi-implicit Euler, but the Menagerie model sets RK4 explicitly. Entry 05 must state which integrator the plant uses and run the order check on it.

---

## F. Forward issues the later entries must resolve

These are not errors in 00–04, but they sit in the README and proposal claims that entries 05–14 must make precise. They are recorded now because two of them change what entries 06 and 08 must contain.

- **F-1 (Critical). Feasibility under mismatch.** Limon et al. (2008, 2018) prove recursive feasibility of MPC-for-tracking for the **nominal** model. With plant–model mismatch, the shifted candidate solution is not feasible in general. Proposal Result 2(i) says it "follows from Assumption 1 alone", and that is not true once x_{k+1} ≠ f(x_k, u_k). Entry 06 must choose between:
  - **(a)** hard input constraints only (box + rate, which stay feasible because holding u_{k−1} is always admissible), soft state constraints, and a terminal ingredient that does not cause infeasibility (a terminal cost on the artificial steady state, without a terminal set); or
  - **(b)** a robust (tube or constraint-tightening) MPC for tracking.

  Option (a) keeps feasibility trivial and moves all safety into the barrier bound and Assumption 2. Recommended.
- **F-2 (Critical). Γ is a steady-state map.** Proposal eq. (3) maps the predicted mismatch to the reference offset "that cancels it at steady state". Lag and delay have **zero DC effect** (entry 04: they do not move the equilibrium), so a steady-state Γ either does nothing for them or, through a noisy d̂, injects state-dependent gain. Either way the oracle test of RQ1 is ill-posed for lag and delay unless entry 08 defines what the "correct" reference correction is for a dynamic mismatch. One option is a horizon-wise shift computed from the MPC's prediction model rather than its steady state. This may also be the cleanest explanation of the "hurts" regime, and it should be derived before Phase 4.
- **F-3. The small-gain loop is closed through the window w_k.** The window w_k contains past inputs as well as the state. γ_cl must therefore be the gain from a reference perturbation to the **window**, measured in the norm in which L_Δ is certified. Δr(w) also has a non-zero bias along the nominal trajectory. That calls for an ISS small-gain argument with the bias as an exogenous input. An incremental (Lipschitz-only) argument is not enough.
- **F-4. State-dependent α.** α_k is chosen from a state that depends on w_k (through r_H, among others). So the map w ↦ α(w) Δr(w) has Lipschitz constant ≤ ᾱ L_Δ + Δ̄ L_α, not ᾱ L_Δ. The gain-form bound ‖α_k Δr_k‖ ≤ ᾱ ‖Δr_k‖ does hold for any α_k. Entry 14 should state Result 2(ii) in that gain (ISS) form, which needs no Lipschitz property of α.
- **F-5. Target Result 1 must include state and terminal constraints.** r_H checks only the input box and rate constraints. If the MPC also has state constraints (tilt, altitude) or a terminal constraint, "no input constraint active" does not imply the unconstrained affine law. Either the MPC has no hard state constraints (consistent with F-1(a)), or r_H must also cover them. Once that holds, the proof is short: the unconstrained minimizer is feasible, hence optimal for the convex QP.
- **F-6. Safe policy improvement must test on fresh data.** The union bound over iterations is valid only if each test uses rollouts not used to *propose* the candidate (Thomas et al. split the data into a candidate-selection set and a safety-test set). Empirical-Bernstein radii also need an a-priori bounded cost range, so the per-rollout cost must be clipped or bounded explicitly. "Bounded because ISS" does not give a known constant.
- **F-7. Reference dimension.** Neither document says what r̃ contains (position only, or full state or output). Entry 06 must fix it, because S_j = ∂u*/∂r̃, Γ and T_ry all depend on it.

---

## G. Proposed fix list for STEP 2 (awaiting approval)

1. **Entry 00:**
   - add the confirmed MuJoCo facts and the body-origin caveat (A-00-1);
   - replace "historically" with the cited Bitcraze pitch-sign evidence (A-00-2);
   - amend "nominal MPC uses ideal actuators" per the E-1 decision.
2. **Entry 01:**
   - replace the "typical order" table with a cited parameter table (E-4), including the cross-source spread;
   - correct the c_tf range;
   - confirm the motor mapping (A-01-1);
   - note the Förster affine offsets and the resulting f_min;
   - record the Menagerie wrench actuation (E-2) and the RotorPy rotor order (E-5).
3. **Entry 02:**
   - split the relative-degree table into plant structure and injection points, and reframe the thesis sentence (B-1);
   - qualify "relative degree" as linearized, with the dynamic-extension remark (A-02-1);
   - note the free-fall singularity (A-02-3) and the open-loop qualifier on mass.
4. **Entry 03:**
   - fix the gyroscopic-sign test and standing test 4 (A-03-1);
   - state the diagonal-inertia idealization against Förster's full matrix (E-3).
5. **Entry 04:**
   - remove "≈ 1/15" (B-2);
   - replace the "dimensionless groups" with TWR, c_tf/a, I_zz/I_xx and time ratios against t_att (A-04-1);
   - make all predictions conditional on the disturbance model (B-3, A-04-2);
   - fix the table header (A-04-3);
   - give the general f_min authority formula (A-04-4);
   - separate τ_m ω_c from τ_m/T_horizon (A-04-5);
   - update the forward pointer to entry 05 = discretization (C-8).
6. **Each changed entry:** its check script is rewritten so that every claim in the *revised* text PASSes, and the FAIL lines of this audit become regression checks.
7. **README:**
   - bibliography replaced by the corrected list (D);
   - fix the proposal path;
   - reword §3 "Safe policy improvement" so that it does not imply an ADP-theory guarantee (C-4);
   - make §6 predictions conditional (B-3);
   - re-map phases and entries (C-5, C-8);
   - name RotorPy and the Crazyflie 2.x.
8. **project_details.md:** a superseded banner only.

---

## H. Open questions for the author

- **Q1 (E-1).** Should the nominal MPC model include the identified motor lag, so that the "lag mismatch" is an error in τ_m (recommended)? Or should it keep ideal actuators, so that the vehicle's own lag counts as mismatch?
- **Q2 (F-1).** Hard input constraints only, with soft state constraints and no terminal set (feasibility trivial; recommended)? Or a robust MPC for tracking?
- **Q3 (B-3).** Should the nominal MPC carry a disturbance model (offset-free)? This decides which baseline is "nominal". With it, baseline 8 (offset-free MPC) merges into baseline 1.
- **Q4 (E-5, B-2).** For RotorPy: override its c_tf to Förster's value, or keep its own value as a deliberate cross-simulator mismatch?
- **Q5 (E-4).** Which propeller set is modelled? f_max is 0.15 N (Förster, 2.0 props) or 0.12 N (firmware, 2.1+ props), and TWR changes from 2.18 to 1.69. That decides which side of the TWR = 2 authority knee the vehicle sits on (entry 04).
- **Q6 (C).** Approve the notebook as the single source of truth, with the proposal frozen and project_details archived?

*This audit is a report. No entry is changed until the author approves the fix list above, item by item or as a whole.*
