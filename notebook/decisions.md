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

## 2026-11-12 — Critical open problem before entry 05 and beyond: F-2

**The reference correction for a dynamic (zero-DC) mismatch is undefined.** The proposal maps the predicted mismatch to "the reference offset that cancels it at steady state". Motor-lag error and delay do not move the equilibrium (entry 04), so they have no steady-state effect, and such a map has nothing to cancel for them. It either does nothing, or, through a noisy prediction, injects a purely state-dependent signal that adds loop gain.

**Consequence.** Entry 08 must define the correct reference correction for zero-DC mismatches, for example a horizon-wise correction computed from the MPC's prediction model rather than from its steady state, **before** any realizability work (entry 09) or oracle test (Phase 4) begins. Without that definition the oracle test of research question 1 is ill-posed for lag and delay. Mass and drag are unaffected, since both have a steady-state effect.

**Two related notes.**
- **The "hurts" regime may weaken or vanish.** With the identified lag now in the nominal model (Q1), the unmodelled lag is only the error in $\tau_m$. With injection at the reference, the correction passes through the constraint-aware MPC. Both weaken the mechanism that harmed the earlier study. If the regime disappears for lag errors, that is a finding to report, and delay becomes the main test case.
- **The predictions of entry 04 are provisional** and must be **re-registered after entry 06**, once the controller's horizon, weights and crossover frequency are fixed and the margins can be computed.
