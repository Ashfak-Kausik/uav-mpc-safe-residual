# 07 — Perturbation Models and the Admissible Mismatch Set

*Derivation notebook, entry 07. Defines exactly how each of the four mismatches enters the simulated plant, normalises each severity against the vehicle's own limits and against the nominal controller's cliffs, and verifies Assumption 2 numerically over the resulting grids. The plant is implemented in [checks/plant07.py](checks/plant07.py), and every number is computed in [checks/check_07_perturbations.py](checks/check_07_perturbations.py). The controller is the nominal MPC of entry 06 in implementation (b), with the one-period computation delay modelled.*

---

## Why this entry exists

The research question is how a correction behaves *under mismatch*, so the mismatches have to be defined as precisely as the controller. Three things must be fixed before any experiment.
1. **Where each mismatch enters the plant**, and in what order relative to the others.
2. **How its severity is measured.** A severity in newtons or milliseconds means nothing across vehicles. It has to be normalised against the vehicle's limits (entry 04) and against the nominal loop's cliffs (entry 06).
3. **The admissible set Θ** of Assumption 2: the mismatches for which the nominal loop is itself stable. Decision Q2 requires this to be *verified numerically*, not assumed. Outside Θ the supervisor makes no claim, and those points become stress tests.

---

## 1. The perturbed plant

The plant is the nonlinear rigid body of entries 02–03, with the **full** inertia matrix, integrated with RK4 at $h = 1$ ms (entry 05). In front of it is an actuator chain, applied in this order at every plant step:

1. **Command.** The MPC's commanded rotor thrusts, already clipped to the box $[f_{\min}, f_{\max}]$ (entry 06).
2. **Computation delay.** One control period. It is part of implementation (b), and *modelled* by the MPC (entry 06, section 11).
3. **Injected delay $D$.** A buffer that exists only in the plant, with a resolution of one plant step (1 ms).
4. **Motor lag.** Time constant $\tau_m^{\text{true}} = s_\tau\,\tau_m$, in thrust form or rotor-speed form (section 2).
5. **Mixer.** $(T, \boldsymbol\tau) = \mathbf M\mathbf f$ (entry 01).
6. **Rigid body.** The wrench acts on the body. Rotor drag (section 4) acts on the body. The mass is $m' = s_m\,m$ (section 5).

The order matters. The delay sits *before* the motor lag, because a late command is still filtered by the motor once it arrives. The clip sits *before* the delay, because the firmware limits the command it sends. At zero mismatch ($s_\tau = 1$, $D = 0$, no drag, $s_m = 1$) the plant differs from the MPC's model only by the deliberate simplifications of the noise floor (entry 05): full versus diagonal inertia, nonlinear versus linear dynamics, and continuous versus sampled time. In a closed-loop step test it converges to within 0.2 mm, which is the noise floor of these tests.

---

## 2. Motor lag: form and severity

**Decision: the primary plant uses the lag on thrust, the same form as the MPC's model. The rotor-speed form is kept as a secondary, structural variant.** Two reasons.
- **Thrust form, so "lag error" means one thing.** With the same form in plant and model, a lag-error severity measures exactly one thing: the error in $\tau_m$. Förster identified the lag in this form, from motor command to measured thrust (entry 01).
- **Speed form, because it is physically closer.** A motor is a speed-controlled machine: $\tau_m\dot\Omega = \Omega^{\text{cmd}} - \Omega$, with $f \propto \Omega^2$. RotorPy, our second simulator, uses this form. In normalised speed $\tilde\Omega = \Omega/\Omega_{\text{hover}}$ no thrust coefficient is needed: $f = f_{\text{hover}}\tilde\Omega^2$.

**How different are they?**
- **Small signals: the same.** For a 1 percent thrust step the two responses differ by 0.06 percent of the step. This confirms entry 01's statement that they share the small-signal time constant.
- **Large steps: different, and asymmetric.**
  - From hover to the ceiling, the speed form reaches 63 percent in 70 ms, against 65 ms for the thrust form. Speed must rise by more than thrust in relative terms.
  - From hover to the floor, the speed form takes 50 ms.
  - So the speed form is slower upward and faster downward, which no time-constant error can mimic. It is a structural mismatch, used in the nonlinear tests, and in the cross-simulator check through RotorPy.

All four rotors share the same true time constant. Per-rotor spread is not part of the study.

**Severity grid:** $s_\tau \in \{0.15, 0.3, 0.5, 0.7, 1.3, 1.5, 2, 3, 4\}$. The planned ±30–50 percent errors are extended down to 0.15 and up to 4, so that both cliffs of implementation (b), at 0.186 and 2.86 (decisions.md), are observed once.

| $s_\tau$ | 0.15 | 0.3 | 0.5 | 1 | 1.5 | 2 | 3 | 4 |
|---|---|---|---|---|---|---|---|---|
| $\tau_m^{\text{true}}$ (ms) | 10 | 19 | 32 | 65 | 97 | 129 | 194 | 259 |
| $\tau_m^{\text{true}}/t_{\text{att}}$ | 0.23 | 0.46 | 0.76 | 1.53 | 2.29 | 3.06 | 4.58 | 6.11 |
| $\tau_m^{\text{true}}/T_{\text{horizon}}$ | 0.05 | 0.10 | 0.16 | 0.32 | 0.48 | 0.65 | 0.97 | 1.29 |

---

## 3. Delay: buffer and severity

The injected delay is a plant-only buffer at 1 ms resolution, so any delay that is a whole number of milliseconds is represented exactly. The analysis model uses the exact integer and fractional constructions of entry 05.

**Severity grid** (entry 05), with the cliff of implementation (b), 24.6 ms of injected delay:

| $D/\Delta t$ | 0.5 | 1 | 1.5 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|---|---|
| $D$ (ms) | 5 | 10 | 15 | 20 | 30 | 40 | 60 | 80 |
| fraction of the cliff | 0.20 | 0.41 | 0.61 | 0.81 | 1.22 | 1.62 | 2.44 | 3.25 |

The total delay the loop sees is the injected $D$ plus the modelled computation period, plus the hold (entries 05–06). In hardware experiments, the unmeasured estimation latency is added on top.

---

## 4. Rotor drag: model and severity

**Model.** $\mathbf F_{\text{drag}} = -\mathbf R\,\mathbf D\,\mathbf R^\top\mathbf v$ (entry 01), with $\mathbf D = \operatorname{diag}(d, d, d_z)$. Following Förster, the coefficients scale with the sum of the rotor speeds: $d = s_d\,d_{\text{hover}}\cdot\frac14\sum_i\sqrt{f_i/f_{\text{hover}}}$, and $d_z/d = 10.3/9.18$. Here $d_{\text{hover}} \approx 7\times10^{-3}$ N·s/m is the order-of-magnitude value at hover (entry 01). The nominal plant has **no** drag ($s_d = 0$), as entry 01 committed. The real vehicle's drag corresponds to $s_d \approx 1$.

**Severity.** Drag is weak on this vehicle, and two normalised quantities make that plain:
- the velocity time constant $m/d = 4.1$ s, which is **96 attitude time scales**;
- the trim tilt it requires, $d/(mg) = 0.025$ rad per m/s, per unit $s_d$.

The grid $s_d \in \{1, 2, 4, 8, 16\}$ therefore goes far beyond the physical value, so that the drag regime is not decided by its smallness alone.

---

## 5. Mass: model and severity

**Decision: the mass error is a point mass at the centre of mass.** The inertia about the centre of mass is unchanged and the centre of mass does not move. A real payload would also change the inertia and, if mounted off-centre, move the centre of mass, adding a torque bias. Those are different mismatches and are excluded, so that "mass error" tests exactly what entry 04 analysed: a gain and bias error on the vertical channel.

**Feasibility.** Hover inside the input box needs $f_{\min} \le m'g/4 \le f_{\max}$, i.e. $0.18 \le s_m \le 1.69$.

**Severity grid** $s_m \in \{0.8, 0.9, 1.1, 1.2, 1.3\}$, normalised by what the mass change does to authority:

| $s_m$ | 0.8 | 0.9 | 1.1 | 1.2 | 1.3 |
|---|---|---|---|---|---|
| effective TWR | 2.11 | 1.87 | 1.53 | 1.41 | 1.30 |
| ceiling headroom at hover / nominal | 1.29 | 1.15 | 0.85 | 0.71 | 0.56 |

A lighter vehicle crosses the authority knee (TWR 1.82, entry 04) and becomes floor-limited. At +30 percent only 56 percent of the nominal headroom above hover is left, so mass errors also probe saturation, not only the vertical offset.

---

## 6. Combinations

The grid of combinations is the product mass $\{1, 1.2\}$ × drag $\{1, 4\}$ × lag $\{0.5, 1, 1.5\}$ × delay $\{0, 1, 2\}\,\Delta t$: **36 points**. The lag levels include a motor *faster* than modelled, because that turned out to be the dangerous direction in combination (section 7).

---

## 7. Assumption 2: the admissible set Θ

**Method.** For each grid point, the linear closed loop of entry 06 (implementation b, with the controller's internal lag model) is built with the perturbed plant. Mass and drag are linearised about the perturbed plant's *own* trim: thrust $m'g$, so the input gain becomes $1/m'$ while the tilt coupling stays $g$, and drag adds $-d/m'$ on the velocities. The loop is stable if its spectral radius is below 1. Nonlinear closed-loop tests with the constrained QP then confirm representative points (section 8).

**Single mismatches.**

| Mismatch | Stable (in Θ) | Unstable (outside Θ; stress tests) |
|---|---|---|
| Mass | all: −20 ... +30 % | — |
| Drag | all: ×1 ... ×16 | — |
| Lag error | 0.3 ... 2.0 $\tau_m$ | 0.15, 3, 4 $\tau_m$ (cliffs 0.186 and 2.86) |
| Delay | up to 2 $\Delta t$ (20 ms) | 3, 4, 6, 8 $\Delta t$ (cliff 2.46 $\Delta t$) |

**Combinations.** 32 of the 36 points are stable. The four unstable ones are exactly those where **a motor faster than modelled ($s_\tau = 0.5$) meets 2 $\Delta t$ of delay**, whatever the mass and drag. The combined cliffs show why:
- with $s_\tau = 0.5$ the delay cliff falls from 24.6 ms to **15.8 ms**;
- with $s_\tau = 2$ it falls to 16.2 ms;
- with $s_\tau = 1.5$ it falls to 22.7 ms.

A lag error that is harmless on its own therefore narrows the delay margin, and most in the direction one might think benign: a quicker motor. This is the predictor of implementation (b) at work. It expects a slow motor and commands accordingly, so a fast motor overshoots, and delay finishes the job.

**The admissible set**, for this study, is the set of grid points above that are stable: all single-mismatch points except the six listed as unstable, and 32 of the 36 combinations. It is a set of verified points, not a region. Between grid points stability is plausible from the smooth trend of the spectral radius, but not certified.

**Gains toward the cliffs (input to entry 10).** The loop-gain bound of entry 10 needs the nominal closed loop's gain from a reference perturbation, and how it grows near the cliffs.
- **The gain to position is not the right indicator.** Its peak stays at 1.00 even at 24 ms of delay, because position does not peak.
- **The gain to tilt is.** Its peak, in rad per m of reference, is 1.75 at zero delay, 1.74 at 10 ms, **4.0 at 20 ms** and **31.7 at 24 ms**. Toward the upper lag cliff it is 1.86 (×1.5), 3.16 (×2) and 45.6 (×2.8).

The small-gain bound of the proposal must therefore be computed on a gain that includes the attitude states, not only the tracked outputs. That is a constraint on entry 10. All of these gains are frequency-sweep estimates.

---

## 8. Nonlinear confirmation

The constrained QP is solved at every period (implementation b), on the nonlinear plant of section 1. The test is a step of 0.5 m in x and 0.2 m in z, flown for 3 s.

| Case | Inside Θ? | Result |
|---|---|---|
| no mismatch | yes | converges; last-second error 0.2 mm (the noise floor of this test) |
| mass +20 % | yes | settles **106 mm low**, exactly the linear prediction (2 × 53 mm); lateral error below 1 mm |
| drag ×4 | yes | set-point reached, no steady effect (as predicted for set-points) |
| lag ×0.5 and ×1.5, thrust and speed form | yes | all converge |
| delay 20 ms | yes, near the cliff | converges, with tilt overshoot beyond the soft 20° limit |
| delay 30 ms | no | **diverges** |
| lag ×3 | no | **does not settle** within 3 s |

The nonlinear plant crosses its cliffs where the linear analysis says it should, to the resolution of the grid.

---

## 9. What this means for the study

- **Delay is the primary destabilizing mismatch.** Its cliff (2.46 $\Delta t$) lies inside the grid, and the gains grow steeply toward it.
- **Lag error is the benign control case on its own**, as decisions.md records. It stays in the study for two reasons: it tests the correction path (entry 10), and it **narrows the delay margin in combination**, by up to 36 percent at a ±50 percent error.
- **Mass** gives a clean, predicted offset (−53 mm per 10 percent), and probes saturation at larger errors.
- **Drag** is weak at this scale. Its grid deliberately exaggerates it.
- **Stress tests**, outside Θ: lag 0.15, 3 and 4 $\tau_m$; delay 3 to 8 $\Delta t$; and the four lag-0.5 / delay-2 $\Delta t$ combinations.

---

## What this entry commits us to, and what is open

**Committed.**
- **The plant's actuator chain**, in order: clipped command → modelled computation delay → injected delay buffer (1 ms resolution) → motor lag → mixer → rigid body with full inertia, rotor drag and mass.
- **Lag:** thrust form as the primary plant, matching the model and the identification; rotor-speed form as a structural variant, identical for small signals and asymmetric for large steps (70 ms up, 50 ms down, against 65 ms). Grid 0.15–4 $\tau_m$.
- **Delay:** plant-only buffer. Grid $\{0.5, \dots, 8\}\,\Delta t$, reported as a fraction of the 24.6 ms cliff.
- **Drag:** the Förster model, scaled with rotor speed, absent from the nominal plant. Grid ×1–×16, normalised by $m/d$ = 96 $t_{\text{att}}$ and 0.025 rad of trim tilt per m/s.
- **Mass:** a point mass at the centre of mass (inertia and centre of mass unchanged). Grid 0.8–1.3, normalised by effective TWR and headroom. Feasible range 0.18–1.69.
- **Combinations:** 36 points.
- **The admissible set Θ:** the verified-stable grid points of section 7. Stress tests are the unstable ones.

**Deferred, noted, not forgotten.**
- Per-rotor lag spread, off-centre payloads (inertia and centre-of-mass changes), and wind.
- A certified (not swept) closed-loop gain over Θ: entry 10.
- Trajectory-tracking (ramp) versions of the drag and mass predictions, once entry 08 fixes the reference formulation.

**Open for the next entry (08, reference shaping).** Entry 08 must first resolve the critical open problem F-2: what the correct reference correction is for a mismatch with no steady-state effect. Lag error and delay are now known to be the cases that matter. It must then derive the map from predicted mismatch to reference shift through the MPC's steady-state target and, where needed, over the horizon. It must state its one-tick causality and saturation, and its relation to offset-free MPC. It must also decide whether the reference becomes a full trajectory over the horizon, which entry 06 deferred to it.

*This entry is locked. We do not proceed to 08 until the following can be explained in plain language: the order of the actuator chain and why the delay sits before the motor lag; why the primary plant's lag is on thrust while the rotor-speed form is kept as a variant; why a mass error is modelled as a point mass at the centre of mass; why drag's grid exaggerates it; why a quicker-than-modelled motor shortens the delay margin; why the gain to tilt, not to position, is what grows near a cliff; and what the admissible set Θ is and is not (a set of verified points, not a certified region).*
