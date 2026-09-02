# Clean-Room Journal Study — Project North Star (v1)

*Planning document for the clean-room rebuild. Everything here WILL change as derivations and experiments land — this is a north star, not a contract. Written to be self-contained: if the working chat is lost, this file plus the frozen Paper 1 repo is enough to resume mid-project. Deliberately evaluative and specific.*

*Governing discipline (from supervisor): rebuild implementation and derivation from zero; Paper 1 is a FROZEN empirical artifact (repo `uav-mpc-learning`), reproduced later as a check, never copied. Rule: if you cannot explain a line of math, a signal, or an ordering dependency in plain language, you do not move past it. Oracle-before-network at every stage.*

---

## 1. Foundational abstract (draft — will change)

Learned residual corrections are widely added to quadrotor model predictive control (MPC) to compensate model mismatch, and are usually reported to help. Our prior empirical study found the picture is not uniform: the *same* accurate residual can help (mass), do nothing (drag), or actively destabilize the loop (motor lag near a cliff), and the deciding factor was not the accuracy of the learned model but the structural relationship between what the residual predicts and what the controller can actually act on. This paper rebuilds that investigation from first principles and turns the observation into a principled, checkable framework.

We identify that whether a learned correction is *usable* depends on a quantity distinct from both model accuracy and formal controllability: **finite-horizon correction realizability** — whether the requested correction can actually be produced, within the relevant decision horizon, at the chosen *injection point*, under the true actuator magnitude and rate limits. A correction may be accurately predicted, and the vehicle may be nominally controllable, yet the correction remain unrealizable at a given interface (e.g. a post-solve body-rate feedforward for a relative-degree-3 lateral effect) while being trivially realizable at another (e.g. a reference-level offset over the MPC horizon). We make this quantity explicit, injection-point-aware, and normalized to vehicle actuator authority.

Building on this, we propose a two-layer supervisor that scales a learned correction by a continuous factor α ∈ [0,1]: a *hard safety envelope* (stability margin, complete-non-realizability veto, extreme-out-of-distribution veto) sets the maximum admissible correction, and a *soft performance layer* (calibrated ensemble uncertainty, graded realizability, expected effectiveness, temporal consistency) decides how much of that admissible correction to actually use. The learned model can only add a bounded, supervised correction on top of an always-on, unmodified nominal MPC; whenever safety or realizability is threatened, α → 0 and the loop reverts to the nominal controller — a run-time-assurance (Simplex-style) guarantee that the learned component can help but cannot break the controller. The residual may only modify an external reference supplied to the nominal MPC; the MPC formulation, dynamics, constraints, solver, and stability baseline remain unchanged.

We evaluate in a high-fidelity simulator across quadrotor platforms of materially different inertia and thrust-to-weight, with perturbations (mass, drag, motor lag, delay, and combinations) applied at severities relative to each vehicle's own operating limits, and show that the supervisor preserves beneficial correction while eliminating harmful correction, and that the realizability criterion correctly predicts — before deployment — whether a given (mismatch, injection point, horizon) triple is correctable. The result is a design-time and run-time tool for safe, realizability-aware residual augmentation of MPC.

*Deferred: formal stability theorem for the do-no-harm margin; reinforcement-learning adaptation of the soft layer; hardware transfer.*

---

## 2. Research problem & prior-work verification (honest)

### 2.1 The problem
Learned residuals are added to quadrotor MPC to fix model mismatch, but practice offers no principled, checkable way to predict — before deployment — whether a given residual will help, do nothing, or destabilize the loop; and no run-time mechanism that lets the learned correction improve the controller while guaranteeing it cannot break it. Our prior work showed the deciding factor is not model accuracy but whether the correction is *realizable* at the interface where it is injected, over the horizon that matters. That distinction is currently neither formalized as a usable criterion nor built into a supervisory architecture.

### 2.2 What is genuinely prior work (verified, not assumed)
Residual/learning-augmented MPC is a mature, crowded field (data-driven MPC, real-time neural MPC, GP-MPC, energy-regularized residuals). More pointedly, several recent works overlap our *specific* ideas and MUST be positioned against — we do not claim these ideas are untouched:

- **Prediction accuracy ≠ usable correction; reachability of the requested increment matters.** This core insight already appears, in a different setting, in ISS-certification work for Koopman learning control over finite horizons (projection-residual / selected-channel-margin idea): a predictor with small held-out residual can still fail because the requested output increment lies partly outside the constrained reachable set. Our contribution is NOT discovering this; it is making it *injection-point-aware*, applying it to characterize the helps/does-nothing/hurts partition on quadrotors, and operationalizing it in a run-time supervisor.
- **Fallback / run-time assurance for MPC.** Fallback-Safe MPC pairs a runtime monitor with a fallback strategy (quadrotor example). Simplex/RTA is an established architecture. Our fallback is not novel as a concept; its use to gate a *graded, realizability-and-uncertainty-scaled* residual is the specific angle.
- **Gated / supervised learned corrections.** Contact-gated residual RL on MPC (residual active only in a regime); autopilot-preserving residual command supervision for fixed-wing UAVs with a no-op fallback and CLF/CBF-style gates. These are close in spirit (a learned correction with a safety gate and fallback). Differences to defend: they gate ON/OFF or by regime/risk; ours scales continuously by a *two-layer* construction separating a hard safety envelope from a soft performance layer, driven by an explicit realizability quantity.

### 2.3 Where the novelty actually sits (the defensible claim)
The integrated package, not any single piece:
1. An **empirical characterization** of when residual augmentation helps / does nothing / hurts on quadrotors, rebuilt from first principles, with each regime *explained* by realizability rather than accuracy.
2. **Injection-point-aware finite-horizon correction realizability** as an explicit, computable, vehicle-normalized criterion — the same channel unrealizable at one injection point and realizable at another, and a criterion that predicts which.
3. A **two-layer supervisor** (hard safety envelope × soft performance layer, continuous α, RTA fallback) that operationalizes the criterion at run time, with the invariant that graded/estimated quantities stay in the soft layer and only certified bounds enter the hard layer.
4. A **cross-vehicle generalization** result: one supervisory law, normalized to actuator authority, holding across lightweight/nominal/heavy platforms without per-vehicle retuning.

*This is the claim to test and defend. If the clean rebuild changes it, update this section.*

---

## 3. Aims, goals, objectives

### 3.1 Aim (one sentence)
Produce a principled, first-principles-validated framework and run-time supervisor that predicts and enforces when a learned residual correction may be trusted to augment quadrotor MPC, such that it can help but cannot break the controller.

### 3.2 Goals
- G1: A correct, self-derived nominal quadrotor + MPC baseline the author can fully explain.
- G2: A formal, computable definition of finite-horizon correction realizability, injection-point-aware and vehicle-normalized.
- G3: A supervisor that scales a learned correction by α via a hard-envelope × soft-layer construction with an RTA fallback.
- G4: Evidence, across ≥3 vehicles and the mass/drag/lag/delay perturbation suite (incl. composites), that the supervisor preserves benefit, removes harm, and that realizability predicts the regime.

### 3.3 Objectives (concrete, checkable)
1. Derive quadrotor state, frames, forces, torques, rotational/translational dynamics, motor mapping — explainable without code.
2. Build and validate a simulator interface: commanded inputs produce responses matching the derived equations (hover, tilt, climb, yaw).
3. Build nominal MPC from the mathematics (state, control, discrete model, horizon, cost, constraints, reference, warm start) — every term justified.
4. Validate nominal MPC on hover, line, circle, figure-eight; establish the tracking floor.
5. Introduce mismatches one at a time (mass, drag, lag, delay); predict each qualitatively *before* running; reproduce Paper 1's degradation partition or investigate the discrepancy.
6. Define, precisely, the learned quantity: what the residual predicts and why predicting it should help control.
7. Compute finite-horizon realizability at candidate injection points; run the ORACLE (perfect correction) through each injection point BEFORE any network.
8. Only after the control pathway is validated by oracle: train the residual (trajectory-level holdouts, verified frames/signs/timing, genuinely unseen perturbations), then add calibrated uncertainty (deep ensemble).
9. Build the two-layer supervisor last; expose an explicit sweepable strictness knob for later calibration.
10. Cross-vehicle generalization with normalized thresholds; regime map + adversarial stress rollout.

---

## 4. Possible contributions (will firm up with evidence)
- **C1 (headline):** Injection-point-aware finite-horizon correction realizability — an explicit, computable, vehicle-normalized criterion distinguishing model accuracy, controllability, realizability, and effective injection; shown to predict the helps/does-nothing/hurts partition.
- **C2:** A two-layer run-time supervisor (hard safety envelope × soft performance layer; continuous α; Simplex/RTA fallback) that provably reverts to the unmodified nominal MPC and preserves benefit while removing harm.
- **C3:** A first-principles empirical study across ≥3 normalized vehicles + the perturbation suite (incl. composites) demonstrating the criterion and the supervisor, including a regime map and an adversarial stress rollout.
- **C-deferred:** formal do-no-harm stability theorem (pursue only if the empirical margin is clean and non-vacuous); RL-adapted soft layer; hardware transfer.

*Razor: every component must either implement the realizability/safety criterion or enable the do-no-harm guarantee. If it does neither, it is out of scope or future work.*

---

## 5. File tree (north star — will change)

```
uav-mpc-safe-residual/                 (clean-room; .git + .venv preserved)
├── README.md                          # what this is, how to reproduce
├── CLAUDE.md                          # standing rules for Claude Code (explain-before-proceed, oracle-before-network, Sacred Rules)
├── ROADMAP.md                         # phase tracker (checkboxes)
├── PROGRESS.md                        # running session log (survives shutdowns)
├── PROJECT_NORTH_STAR.md              # this file
├── requirements.txt                   # pinned, grown per-phase (CPU-only)
│
├── notebook/                          # DERIVATION NOTEBOOK — the heart of the rebuild
│   ├── 00_state_and_frames.md         # state vector, world/body frames, rotation rep
│   ├── 01_forces_torques.md           # gravity, thrust, drag; torques; motor→wrench map
│   ├── 02_translational_dynamics.md   # p_dot, v_dot; why mg; how tilt makes lateral accel
│   ├── 03_rotational_dynamics.md       # attitude kinematics + rigid-body rotational dynamics
│   ├── 04_hover_equilibrium.md         # equilibrium, thrust-to-weight, what each motor does
│   ├── 05_discretization.md            # continuous→discrete, integrator choice, why
│   ├── 06_mpc_formulation.md           # state/control/cost/constraints/horizon, each term justified
│   ├── 07_reference_shaping.md         # external-reference injection; causality (u_{t-1}); frames
│   ├── 08_realizability.md             # finite-horizon correction realizability: definition + derivation
│   └── 99_open_questions.md            # anything not yet explainable (must be emptied before moving on)
│
├── src/
│   ├── dynamics/                       # self-derived quadrotor model (NOT copied from Paper 1)
│   ├── sim/                            # simulator interface + command→response validation
│   ├── mpc/                            # nominal MPC built from notebook/06
│   ├── reference/                      # reference-shaping injection (attitude/accel/vel)
│   ├── perturbations/                  # mass, drag, lag, delay, composites
│   ├── realizability/                  # C1 primitive: horizon Gramian, injection-point-aware ratio
│   ├── residual/                       # learned model + deep ensemble + uncertainty (built LATE)
│   ├── supervisor/                     # two-layer α, hard envelope × soft layer, RTA fallback (built LAST)
│   └── harness/                        # config-driven single-run + resumable sweeps + provenance
│
├── configs/                           # one config = one reproducible run
├── experiments/                       # sweep specs (regime map, oracle, injection comparison, stress)
├── results/                           # run outputs (gitignored) + tracked index/manifests
├── figures/                           # generated figures
└── tests/                             # unit tests, esp. realizability + supervisor invariants
```

---

## 6. Phase-wise plan

*Ordered by dependency; each phase has an UNDERSTANDING gate (author can explain it) and a VALIDATION gate (a check passes). No phase proceeds until both gates pass. Claude Code is minimally used in early phases — the author writes the physics/MPC by hand, Claude (chat) tutors and checks. Claude Code returns for harness/sweeps/supervisor.*

**Phase 0 — Setup & scaffolding.** README, CLAUDE.md (explain-before-proceed + oracle-before-network as rules), ROADMAP, PROGRESS, this file. *Gate: scaffolding committed; rules written.*

**Phase 1 — Quadrotor physics from first principles (notebook, NO code).** State, frames, forces, torques, rotational + translational dynamics, motor→wrench map, hover equilibrium. *Understanding gate: author explains every term in plain language. Validation gate: notebook 00–04 complete, open-questions empty.*

**Phase 2 — Simulator interface + command→response validation.** Send known commands (hover, tilt, climb, yaw); verify responses match the derived equations. *Gate: measured responses match hand-derived predictions within tolerance.*

**Phase 3 — Nominal MPC from the mathematics.** Build MPC per notebook/06; validate on hover, line, circle, figure-eight; establish tracking floor. *Gate: author justifies every cost/constraint/horizon choice; tracking floor established and explained.*

**Phase 4 — Mismatch characterization.** Introduce mass, drag, lag, delay one at a time; PREDICT each qualitatively first; then composites. Reproduce (or investigate divergence from) Paper 1's partition. *Gate: predictions logged before runs; degradation curves explained; Paper 1 reproduction checked.*

**Phase 5 — Define the learned quantity + realizability.** State precisely what the residual predicts and why it should help. Build the finite-horizon realizability primitive (injection-point-aware, normalized). *Gate: the "what is injected, where, in which frame, at what time index" question is answered on paper (notebook 07–08).*

**Phase 6 — Oracle-before-network.** Feed the TRUE known correction through each candidate injection point; measure horizon-level realization. Select the injection point(s) that actually transmit. *Gate: an oracle correction demonstrably improves tracking through the chosen injection point (or the honest boundary is established).*

**Phase 7 — Residual model + uncertainty.** Only now: train the residual with trajectory-level holdouts, verified frames/signs/timing, genuinely unseen perturbations; add deep-ensemble uncertainty; calibrate. *Gate: no temporal leakage (whole trajectories/configs held out); uncertainty rises OOD.*

**Phase 8 — Supervisor (two-layer α + RTA fallback).** Hard envelope (α_safe, realizability veto, extreme-OOD veto) × soft layer (uncertainty, graded realizability, effectiveness, temporal smoothing); α→0 fallback; expose sweepable strictness knob. Enforce invariants (α ≤ α_safe; soft only reduces; asymmetric smoothing; normalized thresholds). *Gate: Simplex property (α=0 → byte-identical nominal MPC) verified; invariants unit-tested.*

**Phase 9 — Single-vehicle validation.** Supervisor calibration curve (strictness → useful-retained / harmful-accepted / false-rejection / tracking / fallback-frequency — MUST be able to embarrass the supervisor); regime map (supervised vs unsupervised); adversarial stress rollout. *Gate: calibration curve honestly locates a middle regime or honestly shows none.*

**Phase 10 — Cross-vehicle generalization.** Lightweight + heavy vehicles; same supervisory law via normalized thresholds; failure-boundary-relative severities; confirmatory regime maps + stress rollouts. *Gate: law reproduces across platforms (Level-1 solid; Level-2 attempted honestly).*

**Phase 11 — Analysis, figures, writing.** Consolidate; decide from evidence whether to pursue the deferred stability theorem and ablations; target venue.

**Critical path:** 0→1→2→3→4→5→6→7→8→9→10→11. Riskiest gates: Phase 6 (does any injection point transmit?) and Phase 10 (does the law transfer?). Fallbacks: conference-level result; single-vehicle-only claim; boundary result if realizability is genuinely limiting.

---

## 7. Standing rules (carry into CLAUDE.md)
1. Explain-before-proceed: no line of math/signal/ordering passes without a plain-language explanation.
2. Oracle-before-network: never train to fix what a perfect correction can't fix through the same pathway.
3. MPC formulation/dynamics/constraints/solver/stability baseline UNCHANGED; residual may only modify an external reference. Simplex: α=0 → byte-identical nominal MPC.
4. Residual→reference is one-tick causal (shape tick t from u_mpc at t−1).
5. Hard layer = certified bounds only; graded/estimated quantities live in the soft layer; soft only reduces α.
6. Normalize every physical threshold to a vehicle-specific limit.
7. CPU-only; deterministic (seed everything); every result traceable to config + git hash.
8. Paper 1 (`uav-mpc-learning`) is frozen; reproduce it as a check, never copy it.

---

*End north star v1. Next action: Phase 1 — quadrotor physics on paper, starting with notebook/00_state_and_frames.md, author-written, Claude-checked.*