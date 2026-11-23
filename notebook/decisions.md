# Decisions Log

*Every design decision that later entries depend on, with its date, the alternatives considered, and the reason. A decision is changed only by a new dated entry here, never by silently editing an old one.*

---

## 2026-11-12 — Decisions Q1–Q6 (from the audit, [99_audit.md](99_audit.md), section H)

### Q1. Motor lag belongs to the nominal model; delay does not.

**Decision.** The nominal MPC model includes the identified motor lag. The delivered rotor thrusts are states, $\tau_m\dot{\mathbf f} = \mathbf f^{\text{cmd}} - \mathbf f$, and the commanded rotor thrusts are the input. The "motor-lag mismatch" is an **error in $\tau_m$** (for example ±30–50 percent). Actuation delay remains a **pure mismatch**: it exists in the plant and not in the model.

**Alternative rejected.** Keeping ideal actuators in the nominal model, so that the vehicle's own lag counts as mismatch.

**Why.** On the Crazyflie the identified motor time constant (≈ 65 ms, Förster 2015) is 1.5 times the attitude time scale $t_{\text{att}} \approx 42$ ms (entry 04). With ideal actuators, "zero mismatch" would not be physically meaningful for this vehicle. Every experiment would start on the lag axis of the mismatch study.

**Affects.** Entry 00 (actuator state), entry 01 (input = commanded thrusts, lag on thrust), entry 03 (17-state nominal model), entry 04 (equilibrium, transfer functions, lag-error prediction), entries 06 and 07.

### Q2. Hard input constraints only, soft state constraints, no terminal set.

**Decision.** The nominal MPC has hard constraints only on the input: the per-rotor box and the per-step rate limit on the commanded thrusts. State constraints (tilt, altitude) are soft. There is no terminal set; a terminal cost on the artificial steady state of the MPC-for-tracking formulation is allowed. Target Result 2 is stated in three parts:
- **(i) Feasibility, which is trivial:** holding the previous command is always admissible.
- **(ii) Input-to-state stability:** from Assumption 2 plus the small-gain bound.
- **(iii) Safety of the state limits:** from the barrier bound.

**Assumption 2 is verified numerically** for every mismatch in the set (closed-loop stability, constraint satisfaction and a finite reference-to-state gain), not assumed.

**Alternative rejected.** A robust (tube or constraint-tightening) MPC for tracking.

**Why.** Recursive feasibility of MPC for tracking (Limon et al. 2008, 2018) is a nominal-model result and does not survive plant–model mismatch automatically (audit F-1). With input-only hard constraints, feasibility holds by construction, and safety moves where it can be certified: into the barrier bound and the numerically verified Assumption 2. This also keeps Target Result 1 clean: with no hard state or terminal constraints, "no input constraint active" means "the unconstrained affine law applies" (audit F-5).

**Affects.** Entries 06, 09, 11 and 14.

### Q3. The primary nominal MPC has no disturbance model.

**Decision.** The primary nominal MPC has **no** disturbance model (no integral action). Offset-free MPC is a separate, strong baseline. A supervisor on top of an offset-free MPC is an **optional extension**. All predictions are conditional on this choice, as in the two-column table of entry 04.

**Why.** The project asks what a learned reference correction adds to a nominal controller. Without integral action, the correction has a constant-offset job to do (mass, drag), and offset-free MPC is then the natural competitor that does that job without learning. With integral action in the nominal MPC, mass and drag corrections would be mostly redundant, and the study would reduce to lag and delay.

**Affects.** Entry 04 (predictions), entries 06, 08 and 15 (baselines).

### Q4. RotorPy: override its yaw coefficient for the main check, keep one native run.

**Decision.** For the main cross-simulator check, RotorPy's torque-to-thrust ratio is overridden with our committed value, and its rotor order is mapped to ours. One additional run keeps RotorPy's native value as a robustness check. The discrepancy is recorded in entry 01.

**Why.** RotorPy's $k_m/k_\eta = 0.0339$ m is 5.7 times the value of Förster, the source it cites, and 4.8 times the firmware's. A cross-simulator check should test independent rotor and aerodynamic *models*, not an inconsistent constant.

### Q5. Vehicle: Crazyflie 2.1+ with standard propellers; mixed parameter sources.

**Decision.** The target hardware is the Crazyflie 2.1+ with standard propellers, pending purchase confirmation. The parameter sources are:
- **Bitcraze firmware, 2.1+ propellers:** maximum thrust 0.12 N per rotor, torque-to-thrust ratio 0.00699 m, mass 0.029 kg and arm 0.046 m. These give TWR ≈ 1.69.
- **Förster (2015):** the inertia (full matrix in the plant, its diagonal in the MPC) and the motor time constant.

The source of every parameter is recorded in entry 01. $f_{\max}$ and the thrust map will be **re-identified by a bench test** once the hardware arrives.

**Consequence (entry 04).** TWR < 2 means torque authority at hover is ceiling-limited: 69 percent of the floor-limited value, with a maximum coordinated tilt of 54°. Saturation and realizability limits will therefore be active more often.

**Caveat.** The inertia and motor lag were identified on the 2.0 airframe, and the thrust and torque constants are for the 2.1+. The cross-source TWR spread (1.69 to about 2.5) straddles the authority knee at TWR = 2.

> **Amendment to Q5 (2026-11-12, after the STEP 2 review).** The text above is kept as recorded; this amendment supersedes it where they differ.
> - **Thrust floor.** $f_{\min}$ is the firmware's `THRUST_MIN` = 0.012817578393224994 N per rotor (2.1+ props), not 0. It is active by default (`CONFIG_ENABLE_THRUST_BAT_COMPENSATED`, `default y`). Commands below it switch the motor off (`motors.c`), so the true per-rotor set is $\{0\}\cup[f_{\min}, f_{\max}]$, and the MPC uses the convex box $[f_{\min}, f_{\max}]$.
> - **Corrected consequence.** The authority knee is at TWR = $2 - f_{\min}/f_{\text{hover}}$ = 1.82, not 2. At TWR = 1.69 the vehicle is still ceiling-limited, but closer to the knee: torque authority is **84 percent** of the floor-limited value (not 69 percent), and the maximum downward acceleration inside the box is $g - 4f_{\min}/m$ = 8.0 m/s² (not $g$). The bench re-identification can therefore move the vehicle across the knee.
> - **`THRUST_MAX` is a firmware cap.** The value 0.12 N is chosen for thrust consistency over battery discharge, not the full-battery physical maximum. The bench test must run under the same firmware thrust configuration as flight.
> - **$\tau_m$ is added to the bench re-identification list.** At $\tau_m/t_{\text{att}} = 1.53$ it is the highest-leverage parameter, and its value comes from a single-pole fit on the 2.0.

### Q6. The notebook is the single source of truth.

**Decision.** The notebook is the single source of truth for every technical statement. The README and any proposal or paper drafts are derived documents and must not contradict it. When they disagree, the notebook wins and the derived document is corrected. The October 2026 proposal is a frozen, dated artifact. [project_details.md](project_details.md) is superseded and carries a banner; it is kept for history only.

---

## 2026-11-12 — Control period and plant step (entry 05)

**Decision.**
- **Control period** $\Delta t = 10$ ms (100 Hz). This gives $\Delta t/t_{\text{att}} = 0.236$, $\Delta t/\tau_m = 0.155$, and a sample-and-hold phase loss of 6.8° at $1/t_{\text{att}}$.
- **Plant step** $h = 1$ ms, RK4 with quaternion renormalisation.
- **Prediction model:** exact ZOH (linear) or at least one RK4 step per period (nonlinear), never forward Euler.
- **Simulation timing:** zero computation delay, as an explicit modelling assumption.

**Alternatives.**
- 50 Hz: 13.5° hold phase and only two samples per $t_{\text{att}}$. Rejected, because the MPC is itself the attitude loop.
- 200–500 Hz: little gain, against a larger optimization per second.

**Open.** Solve time at 100 Hz is unverified until Phase 3. A change of $\Delta t$ will be a dated amendment here.

## 2026-11-12 — Deployment of the nominal MPC (entry 06)

**Decision.** The nominal controller is a linear MPC in error form about hover, solved **on board** the Crazyflie's STM32F405 (Cortex-M4F, 168 MHz), at a target rate of 100 Hz. A nonlinear MPC is at most a simulation-only comparison.

**Alternative rejected.** Offboard MPC on a PC, with commands sent by radio. The radio latency would then be a real, **unmeasured delay**, and delay is one of the injected mismatches. The nominal loop tolerates only about 2.5 control periods of it (entry 06).

**Precedent (verified).**
- TinyMPC (ICRA 2024): on-board linear MPC on the Crazyflie 2.1, 12 states.
- Linder (Linköping MSc thesis, 2024): on-board DAQP outer-loop MPC on a Crazyflie 2.0 at 100 Hz.
- Wikner, Arnström and Axehill (arXiv:2603.09342, 2026): on-board DAQP inner-loop MPC on the Crazyflie 2.1 STM32F405, 12 states, N = 15, 500 Hz, worst case 1375 µs.

**Correction to the brief.** The brief attributed "12 states at 500 Hz" to Linder (2024). That result is Wikner et al. (2026); Linder's is an outer loop at 100 Hz.

**Open.** On-board solve time and memory for our larger problem (86 variables) are an estimate (entry 06, section 10) until measured on hardware.

## 2026-11-12 — Nominal MPC design (entry 06)

- **Model:** the 16-state error model (17-state quaternion model minus the redundant $q_0$), in the heading frame.
- **Thrust estimate:** the delivered thrusts are *estimated* by the controller's own lag model, because the vehicle has no thrust sensor.
- **Reference (resolves audit F-7 for the primary design):** $\mathbf r = (x, y, z, \psi)$, piecewise constant over the horizon, as in MPC for tracking. An artificial steady state $\boldsymbol\theta \in \mathbb R^4$ carries offset cost $\mathbf T = 100\,\mathbf G^\top\mathbf P\mathbf G$. A trajectory (preview) formulation is deferred to entry 08, with F-2.
- **Horizon and weights:** N = 20 (4.7 $t_{\text{att}}$, 3.1 $\tau_m$). Bryson weights; $\mathbf R = \mathbf I/(0.0489\ \text{N})^2$, from the rotor's ceiling-side headroom. Riccati terminal cost.
- **Constraints:**
  - hard: input box $[f_{\min}, f_{\max}]$ and rate limit 0.0268 N per period;
  - soft: tilt 20° and altitude ≥ 0, with one slack per family ($\rho_1 = 10^4$, $\rho_2 = 10^2$).
- **Solver:** DAQP (dual active set), with OSQP as a cross-check.

## 2026-11-12 — Re-registered predictions (after entry 06)

These supersede, for quantitative use, the provisional predictions of entry 04, which stay in entry 04 unchanged. They hold for the nominal MPC of entry 06: setpoint formulation, no disturbance model, the linear closed loop with the controller's internal lag model.

| Mismatch | Re-registered prediction |
|---|---|
| Delay | Nominal loop unstable beyond $D$ = **25.4 ms = 2.54 $\Delta t$ = 0.60 $t_{\text{att}}$** of injected delay. The effective loop delay at the cliff is $D + \Delta t/2$ = 30.4 ms. The sweep grid $\{0, \dots, 8\}\,\Delta t$ brackets it. |
| Motor-lag error | Nominal loop unstable only when the true $\tau_m$ exceeds **3.28 times** the modelled one (0.212 s = 5.0 $t_{\text{att}}$). **Within the planned ±30–50 % sweep, a lag error cannot destabilize the nominal loop.** Any "hurts" regime for lag errors must come from the correction path (entry 10), not from the nominal loop. |
| Mass | Steady altitude offset of **−49 mm for +10 %** mass (−99 mm for +20 %), linear in the error. A residual correction should remove it (helps). |
| Drag | No steady effect in hover or on set-points. On a 1 m/s ramp it adds **0.023 m** to an intrinsic setpoint-tracking lag of 0.47 m, so a residual correction for drag can only help a little, and the reference formulation (entry 08) matters twenty times more. |

## 2026-11-12 — Amendment: computation delay (entry 06, section 11)

**Decision.** The controller is implemented with a **fixed one-step computation delay, modelled**: implementation (b). The command computed from the state at $t_k$ is applied at $t_{k+1}$. The MPC predicts $\hat{\mathbf x}_{k+1|k}$ with the command in flight and optimizes from it. The hardware will run (b). On a missed deadline it applies the previous plan's second move.

**Alternative.** (a) Apply-on-solve, with the computation delay (0 to $t_{\text{solve}}$) as unmodelled jitter.

**Why (b).**
- **Budget:** injected delay plus estimation latency gets 24.6 ms under (b), against about 16.4 ms under (a). The (a) figure holds only if the estimated 9 ms worst-case on-board solve holds, and it ignores that jitter is harder on a loop than a fixed delay.
- **Cost:** under (b) the roll/pitch gain margin falls from 5.8 to 2.66 and the phase margin from 41.5° to 35.6°. A lower lag-error cliff also appears at 0.186 $\tau_m$ (12 ms), far outside any credible motor-model error.

**Amends.** Entry 05's assumption of zero computation delay in simulation. From entry 07 on, simulations and analyses include the modelled one-step delay.

**New line item.** **State-estimation latency** $D_{\text{est}}$ (on-board EKF plus positioning system) is an **unmeasured delay** and must be bench-measured. The remaining budget for injected delay is

$$D_{\text{inj}} \le D_{\text{tot}} - \Delta t/2 - D_{\text{comp}} - D_{\text{est}} = 39.6 - 5 - 10 - D_{\text{est}} = 24.6\ \text{ms} - D_{\text{est}} .$$

## 2026-11-12 — Re-registered predictions, amended for implementation (b)

The table "Re-registered predictions (after entry 06)" above is kept as recorded; it holds for implementation (a). For the implementation that will actually run, (b):

| Mismatch | Implementation (a), as registered above | Implementation (b), amended |
|---|---|---|
| Delay | cliff at 25.4 ms = 2.54 $\Delta t$ | cliff at **24.6 ms = 2.46 $\Delta t$ = 0.58 $t_{\text{att}}$** of injected delay beyond the modelled computation period; effective loop delay at the cliff 39.6 ms |
| Motor-lag error | unstable above 3.28 $\tau_m$ | unstable **above 2.86 $\tau_m$ and below 0.186 $\tau_m$** |
| Mass +10 % | −49 mm | **−53 mm** |
| Drag on a 1 m/s ramp | +23 mm on a 0.466 m lag | **+24 mm on a 0.476 m lag** |

## 2026-11-12 — Lag error is the benign control case; delay is the primary destabilizing case

**Finding.** Both lag-error cliffs (above 2.86 $\tau_m$, and below 0.186 $\tau_m$ under implementation (b)) lie far outside the planned ±30–50 percent sweep. **Motor-lag error is therefore the benign control case of the study, and actuation delay is the primary destabilizing case**: its cliff of 2.46 periods sits inside the delay grid.

**Decision.** Lag error stays in the study, because it tests the *correction path*: whether a state-dependent residual can harm a loop that is itself comfortably stable (entry 10). Its sweep is extended to **4 $\tau_m$** so that the upper cliff itself is observed once. A point at 0.15 $\tau_m$ is also added, to observe the lower cliff of implementation (b) once (entry 07).

## 2026-11-12 — Perturbation models and the admissible set (entry 07)

- **Actuator chain** of the plant, in order:
  1. clipped command
  2. modelled computation delay (one period)
  3. injected delay buffer (1 ms resolution)
  4. motor lag
  5. mixer
  6. rigid body (full inertia), with rotor drag and mass
- **Motor lag:** the thrust form is the primary plant, the same as the model and the identification, so that a lag error is purely an error in $\tau_m$. The rotor-speed form ($f \propto \Omega^2$, as in RotorPy) is a structural variant: identical for small signals, asymmetric for large steps.
- **Mass:** a point mass at the centre of mass. Inertia and centre of mass are unchanged; off-centre payloads are excluded.
- **Drag:** the Förster model scaled with rotor speed, absent from the nominal plant. Its grid goes to ×16, deliberately exaggerated.
- **Grids:**
  - lag error $\{0.15, \dots, 4\}\,\tau_m$
  - delay $\{0.5, \dots, 8\}\,\Delta t$
  - mass $\{0.8, \dots, 1.3\}$
  - drag $\{1, \dots, 16\}$
  - 36 combinations
- **Admissible set Θ (Assumption 2, verified numerically, linear loop, implementation b):** the stable grid points. Stable: all mass and drag points, lag 0.3–2.0, delay up to 2 $\Delta t$, and 32 of 36 combinations. These are verified points, not a certified region. The unstable points are the stress tests.
- **Finding (re-registered prediction).** A lag error narrows the delay margin. A motor *faster* than modelled by 50 percent cuts the delay cliff from 24.6 ms to **15.8 ms**; 1.5× slower cuts it to 22.7 ms; 2× slower, to 16.2 ms. The four unstable combinations are exactly lag ×0.5 with 2 $\Delta t$ of delay.
- **Constraint on entry 10.** Near the cliffs, the closed-loop gain grows in the attitude states, not in position: the peak gain from reference to position stays at 1.00. The loop-gain bound must be computed on a gain that includes attitude.

## 2026-11-12 — Mechanism of the lag-error / delay interaction (entry 07 addition)

The hypothesis was that the controller's open-loop thrust estimate lags the true thrust and raises the loop gain. It was **refuted as stated**.
- **A faster motor keeps its phase margin but raises the crossover** (25 → 43 rad/s), so the same phase is used up by less delay.
- **The higher gain comes from the plant's own reduced lag attenuation.** It is ×1.93 at 100 rad/s, equal to the ratio $|1+j\omega\tau_m|/|1+j\omega\tau_m/2|$.
- **A measured-thrust counterfactual shows the same asymmetry.** The estimate actually *widens* the nominal delay cliff (24.6 ms, against 15.3 ms with measured thrust).

## 2026-11-12 — Reference content (resolves F-7) and the reference correction (resolves F-2) (entry 08)

- **Reference content.**
  - The MPC tracks a planned trajectory over its horizon.
  - Full linear-flatness feedforward (velocity, tilt, body rates, delivered thrust, and commanded thrust with the motor-lag inversion) is computed from the **plan's analytic derivatives** up to fifth order.
  - The artificial steady state is dropped, since feasibility is already trivial (Q2).
  - The constant set-point MPC for tracking (entry 06) is kept as an ablation, and all baselines use the same reference treatment as the method they are compared with.
- **Correction rule.** A reference correction shifts only position and yaw over the horizon and is **never differentiated**. A finite-difference derivative would amplify it by up to $2.5/\Delta t^5$.
- **The oracle.**
  - $\Delta\mathbf r^\star = \arg\min \|\mathbf y(\mathbf r + \Delta\mathbf r) - \mathbf r\|^2 + \lambda\|\Delta\mathbf r\|^2$ over the trajectory window, with $\lambda = 0.01$: the loop is inverted only where its gain exceeds −20 dB.
  - For mass and drag it equals the proposal's steady-state offset, up to $1/(1+\lambda)$.
  - For lag error and delay it is zero on set-points and nonzero on trajectories. **This resolves F-2.**
- **Taxonomy.**
  - Feedforward corrections leave the closed-loop eigenvalues unchanged and cannot move a cliff.
  - Feedback (state-dependent) corrections can move it either way. Tilt feedback of ±0.2 m/rad moves the delay cliff by −15 / +18 percent.
  - The learned approximator of the oracle is a feedback correction. **Entry 10 must bound its state sensitivity, including attitude.**
- **Refuted hypothesis.** The delay oracle is *not* a time advance by D. It is a small multi-derivative correction, linear in D: 2.96 mm at 20 ms, 5.6× smaller than $D\,v_{\max}$.
- **Preview.**
  - About 0.5 s of plan preview recovers 95 percent or more of the oracle's benefit; the 0.21 s horizon window alone recovers 72–93 percent.
  - **A correction from past errors alone makes tracking worse** (−63 to −90 percent).
- **Re-registered.** The trajectory law's cliffs equal implementation (b)'s to within 0.03 ms (delay 24.6 ms; lag 0.186 and 2.86), so the amended predictions stand unchanged.
- **New prediction for research question 1.** On smooth trajectories inside Θ, lag error and delay cost about a millimetre of tracking: 0.68 mm RMS at 20 ms on the figure-8. Mass and drag cost tens to hundreds of millimetres. A learned correction can therefore gain little for lag and delay, while its feedback character can cost margin.

## 2026-11-12 — Critical open problem before entry 05 and beyond: F-2

**The reference correction for a dynamic (zero-DC) mismatch is undefined.** The proposal maps the predicted mismatch to "the reference offset that cancels it at steady state". Motor-lag error and delay do not move the equilibrium (entry 04), so they have no steady-state effect, and such a map has nothing to cancel for them. It either does nothing, or, through a noisy prediction, injects a purely state-dependent signal that adds loop gain.

**Consequence.** Entry 08 must define the correct reference correction for zero-DC mismatches, for example a horizon-wise correction computed from the MPC's prediction model rather than from its steady state, **before** any realizability work (entry 09) or oracle test (Phase 4) begins. Without that definition the oracle test of research question 1 is ill-posed for lag and delay. Mass and drag are unaffected, since both have a steady-state effect.

**Two related notes.**
- **The "hurts" regime may weaken or vanish.** With the identified lag now in the nominal model (Q1), the unmodelled lag is only the error in $\tau_m$. With injection at the reference, the correction passes through the constraint-aware MPC. Both weaken the mechanism that harmed the earlier study. If the regime disappears for lag errors, that is a finding to report, and delay becomes the main test case.
- **The predictions of entry 04 are provisional** and must be **re-registered after entry 06**, once the controller's horizon, weights and crossover frequency are fixed and the margins can be computed.

> **Resolved (2026-11-12, entry 08).** The correct correction is the oracle of entry 08, a regularised least-squares reference correction along the trajectory. For zero-DC mismatches it is zero on set-points and nonzero only on moving references. See the entry-08 decision above. The predictions were re-registered after entry 06 as required.
