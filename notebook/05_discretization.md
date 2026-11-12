# 05 — Discretization

*Derivation notebook, entry 05. Continues the clean-room rebuild from entries 00 to 04. This entry decides how continuous time becomes discrete time, twice: once for the simulated plant and once for the controller's prediction model. It also measures what each choice costs, so that later results can be told apart from discretization artefacts. All numbers are computed in [checks/check_05_discretization.py](checks/check_05_discretization.py) from the committed parameters in [checks/params.py](checks/params.py).*

---

## Why this entry exists

Entries 00 to 04 are written in continuous time. Nothing we build will run in continuous time. The simulator advances the plant in small fixed steps. The MPC predicts with a discrete model and acts at discrete instants, holding each command until the next. Every one of these choices changes the closed loop a little. If we leave them implicit, their effects will later be mistaken for the effects we inject on purpose: a small delay, a small lag error, a small loss of margin. This entry makes each choice explicit, justifies it against the vehicle's own timescales, and quantifies what it costs.

There are **two clocks**:
- **The plant clock** (step $h$): how finely the simulated vehicle is integrated. It should be fine enough that the plant is, for practical purposes, the continuous system of entries 01–03.
- **The controller clock** (period $\Delta t$): how often the MPC measures, solves and issues a new rotor-thrust command, which is then held constant until the next instant.

The two timescales from entry 04 that everything is measured against are the attitude time scale $t_{\text{att}} = \sqrt{I_{xx}/(a\,m\,g)} = 42$ ms and the motor time constant $\tau_m = 65$ ms.

---

## The control period

**What the real vehicle does.** The Crazyflie firmware runs its main loop at 1 kHz, its attitude controller at 500 Hz and its position controller at 100 Hz (`RATE_MAIN_LOOP`, `ATTITUDE_RATE`, `POSITION_RATE` in `src/modules/interface/stabilizer_types.h`, verified 2026-11-12). The fast inner loop is what lets its position controller run slowly.

**What our MPC must do.** Our MPC has no inner loop under it. Its input is the four commanded rotor thrusts (entry 01), so it *is* the attitude controller as well as the position controller. Its period must therefore be judged against the attitude time scale and the motor lag, not against the trajectory's timescales.

| Rate | $\Delta t$ | $\Delta t/t_{\text{att}}$ | $\Delta t/\tau_m$ | Hold phase loss at $\omega = 1/t_{\text{att}}$ |
|---|---|---|---|---|
| 500 Hz | 2 ms | 0.047 | 0.031 | 1.4° |
| 200 Hz | 5 ms | 0.118 | 0.077 | 3.4° |
| **100 Hz** | **10 ms** | **0.236** | **0.155** | **6.8°** |
| 50 Hz | 20 ms | 0.473 | 0.309 | 13.5° |
| 25 Hz | 40 ms | 0.945 | 0.619 | 27.1° |

The last column is the phase that holding a command constant over one period costs at the attitude frequency (derived in the last section of this entry).

**Decision: $\Delta t = 10$ ms (100 Hz).** The reasoning:
- **Resolution.** At 100 Hz there are about four control instants per attitude time scale and six per motor time constant. The motor's response to a command is resolved over several samples, so the MPC can shape it rather than react to it after the fact.
- **Phase.** The hold costs 6.8° at the attitude frequency. That is affordable for a loop that has to manufacture all of its margin around four integrators and a lag (entry 04). At 50 Hz the cost doubles to 13.5°, and there are only two samples per attitude time scale. That is too coarse for a controller that is itself the attitude loop.
- **Cost of going faster.** Faster rates help less and less while costing more. The number of horizon steps for a fixed horizon length grows as $1/\Delta t$, and so does the optimization's size. The firmware's own position loop, which also commands the full vehicle through an inner loop, runs at 100 Hz.

**What this decision does not yet establish.** Whether the MPC can be *solved* within 10 ms on a CPU is unverified here. It depends on the horizon and solver chosen in entry 06, and on measured solve times (Phase 3). If it cannot, the period changes by a dated amendment in [decisions.md](decisions.md), and every table in this entry is recomputed by the check script.

**Zero computation delay is a modelling assumption.** In simulation the MPC is solved instantly: the command computed from the state at $t_k$ is applied from $t_k$. A real implementation applies it one solve-time later, which adds up to one $\Delta t$ of delay. That extra delay would be an *addition* to the injected delays below, and it is not part of the nominal model.

---

## The plant clock

**Decision: plant step $h = 1$ ms, RK4, with quaternion renormalisation after each step.** That gives 10 plant steps per control period, with $h/t_{\text{att}} = 0.024$ and $h/\tau_m = 0.015$. The MuJoCo Menagerie model sets the RK4 integrator but no timestep, so MuJoCo's default of 2 ms would apply (entry 01 and audit E-2). We set 1 ms explicitly, for two reasons. First, it makes the plant's delay buffer resolve delays in 1 ms increments (see the delay section). Second, it keeps the plant's own integration error negligible against everything else.

**Order check.** The reference implementation of the full 17-state nonlinear plant (rigid body with the full inertia matrix, plus motor lag) is RK4 with renormalisation. Its global error at $T = 0.2$ s, against a $10^{-5}$ s reference, is:

| $h$ | 4 ms | 2 ms | 1 ms |
|---|---|---|---|
| global error | $1.3\times10^{-8}$ | $7.9\times10^{-10}$ | $4.9\times10^{-11}$ |

The ratios of 16.4 and 16.2 show genuine fourth order. Renormalising the quaternion after each step does not degrade it, because the projection error per step is of higher order than the RK4 error. This is the check entry 02 promised. The same check must be repeated on MuJoCo's own integrator in Phase 2, because MuJoCo, not this reference code, will be the plant.

---

## The controller's prediction model

### Exact zero-order-hold discretization of the linear model

Over one control period the command is held constant. For a linear model $\dot x = Ax + Bu$ with $u$ constant on $[t_k, t_k + \Delta t)$, the solution at the next instant is exact:

$$x_{k+1} = \Phi\,x_k + \Gamma\,u_k, \qquad \Phi = e^{A\Delta t}, \qquad \Gamma = \int_0^{\Delta t} e^{As}\,ds\;B .$$

Both matrices come from a single matrix exponential:

$$\exp\!\left(\begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix}\Delta t\right) = \begin{bmatrix} \Phi & \Gamma \\ 0 & I \end{bmatrix}.$$

This is not an approximation. For the hover linearisation of entry 04 with the motor lag, it matches a fine continuous integration under a held input to $10^{-15}$.

**Which states.** The linearisation can be written with 17 states (the quaternion's four components, entry 03) or with 16 (a three-component small-angle rotation vector $\delta\boldsymbol\theta$ in place of the quaternion). At hover the 17-state Jacobian has an identically zero row and column for $q_0$: the unit-norm constraint removes that direction to first order. The other three components satisfy $\delta q_{1,2,3} = \delta\boldsymbol\theta/2$. The two models give identical predictions under this map (checked). The 16-state form is the minimal one: 3 position, 3 velocity, 3 attitude, 3 rate and 4 delivered thrusts.

**The motor lag mode.** For the lag states, $\Phi$ contains $e^{-\Delta t/\tau_m} = 0.8567$ on the diagonal. Each period, the delivered thrust closes 14.3 percent of its gap to the command.

### Forward Euler, and why it is not used

Forward Euler replaces $\Phi$ by $I + A\Delta t$ and $\Gamma$ by $B\Delta t$. At our period, it fails in three distinct ways:

1. **Wrong lag.** Euler's per-step lag decrement is $\Delta t/\tau_m = 0.155$ instead of $1 - e^{-\Delta t/\tau_m} = 0.143$, which is **7.9 percent too large**. The Euler model believes the motor is faster than it is. That is exactly the kind of lag error this project injects on purpose (decision Q1), so an Euler prediction model would add an *uncontrolled* lag mismatch on top of the controlled one.
2. **Truncated chain.** One Euler step lets the input move only the four lag states. In the exact model, a held command already moves the rates, the attitude, the velocity and the position within the same period, because the chain is integrated continuously in between. Over a horizon, this truncation accumulates. Here is the predicted lateral position under a held pitch-torque command over 0.5 s, compared with the exact model:

   | $\Delta t$ | 1 ms | 2 ms | 5 ms | **10 ms** | 20 ms | 40 ms |
   |---|---|---|---|---|---|---|
   | relative error | 1.5 % | 2.9 % | 7.1 % | **14.0 %** | 26.7 % | 50.6 % |

   The error grows roughly linearly with $\Delta t$, because the lag and the four integrators each lose about half a step. At 10 ms, a 14 percent error in the predicted lateral response is far larger than any mismatch effect we hope to measure.
3. **Instability of fast modes.** For a mode $\lambda$, the Euler update is stable only if $|1 + \lambda\Delta t| < 1$, i.e. $\Delta t < -2\,\mathrm{Re}\,\lambda/|\lambda|^2$. For a damped oscillation with natural frequency $\omega_n$ and damping ratio ζ, that is $\Delta t < 2\zeta/\omega_n$. This is where Euler breaks:

   | $\omega_n$ | ζ = 0.3 | ζ = 0.7 |
   |---|---|---|
   | $0.5/t_{\text{att}}$ | 50.8 ms | 118.5 ms |
   | $1/t_{\text{att}}$ | 25.4 ms | 59.2 ms |
   | $2/t_{\text{att}}$ | 12.7 ms | 29.6 ms |
   | $4/t_{\text{att}}$ | 6.3 ms | 14.8 ms |

   The check confirms numerically that a mode at $\omega_n = 2/t_{\text{att}}$, ζ = 0.3, is Euler-stable just below 12.7 ms and unstable just above. The lag mode alone is Euler-stable up to $2\tau_m = 129$ ms. At $\Delta t = 10$ ms, Euler loses any well-damped (ζ = 0.7) mode faster than 140 rad/s ($5.9/t_{\text{att}}$), and any lightly damped (ζ = 0.3) mode faster than $2.5/t_{\text{att}}$. The closed-loop speeds are fixed in entry 06, so this table is a constraint on entry 06, not yet a verdict.

**Decision.** The prediction model is never forward Euler. A linear (LTI) MPC uses the exact ZOH matrices above. A nonlinear MPC uses at least one RK4 step per period. On the linear part, a single RK4 step reproduces the exact transition matrix to a relative error of $5\times10^{-6}$, which is negligible.

### Quaternions in discrete time

**Plant.** The plant integrates the quaternion with RK4 and renormalises after each step (entry 03), which the order check above validates.

**Exact update for a held rate.** When the body rate is constant over a step, the exact quaternion update is the exponential map:

$$\mathbf q_{k+1} = \mathbf q_k \otimes \exp\!\big([0,\ \boldsymbol\omega\,\Delta t/2]\big), \qquad \exp\!\big([0,\ \mathbf v]\big) = \big(\cos\|\mathbf v\|,\ \tfrac{\mathbf v}{\|\mathbf v\|}\sin\|\mathbf v\|\big).$$

It stays on the unit sphere by construction. Over 1000 steps it reproduces the analytic rotation to $10^{-10}$, with $\|\mathbf q\| = 1$ to machine precision. The right multiplication matches entry 03's body-frame rate.

**The MPC's prediction model** can handle orientation in one of two ways. Entry 06 chooses between them; both are recorded here with their costs.
- **(a) Error state about a reference attitude.** The prediction is written in a three-component rotation vector $\delta\boldsymbol\theta$, defined by $\mathbf q = \bar{\mathbf q}\otimes\exp([0, \delta\boldsymbol\theta/2])$, with $\bar{\mathbf q}$ the hover attitude (for an LTI MPC on the hover linearisation) or the current measured attitude (if re-linearised at each solve). There is no norm constraint to maintain, and the model has the minimal 16 states. The price is that it is accurate only for small $\delta\boldsymbol\theta$, which is the linearisation error that entry 06 must bound or test.
- **(b) Full quaternion in a nonlinear model.** The prediction carries $\mathbf q$ and integrates it with RK4 at one step per period, without renormalisation inside the horizon, since renormalisation is not a smooth operation an optimizer can differentiate through. Over a 0.5 s horizon (50 steps), the norm drifts by $8.5\times10^{-11}$, $5.4\times10^{-9}$ and $3.5\times10^{-7}$ at body rates of 5, 10 and 20 rad/s. That is negligible for prediction, so no renormalisation is needed inside the horizon. The attitude error in the cost must still be the sign-corrected error quaternion of entry 03, never a component difference.

---

## Simulator versus controller timing

The full timing of one control period is fixed as follows. It is implemented in our own simulator interface, because the Menagerie model is wrench-actuated (entry 01).

1. At $t_k$, read the plant state (MuJoCo state, entry 00).
2. Solve the MPC and obtain the commanded rotor thrusts $\mathbf f^{\text{cmd}}_k$. Clip them to the box $[f_{\min}, f_{\max}]$ (entry 01).
3. Hold $\mathbf f^{\text{cmd}}_k$ constant over the next 10 plant steps (zero-order hold).
4. At each plant step:
   - (a) The delay buffer, which exists in the plant only, releases the command issued $D$ earlier, at a resolution of $h$.
   - (b) The motor lag advances exactly, $f_i \leftarrow f_i + (1 - e^{-h/\tau_m})(f_i^{\text{cmd,delayed}} - f_i)$. This update is exact for a command held over the step (checked).
   - (c) The mixer turns the delivered thrusts into $(T, \boldsymbol\tau) = \mathbf M\mathbf f$.
   - (d) The wrench is applied to the body for one MuJoCo step.

**Why the lag is implemented in our interface, not in MuJoCo's actuator dynamics.** MuJoCo can give an actuator a first-order activation filter. With identical time constants on all rotors, and with clipping applied to the commands *before* the lag, a lag on the four rotor thrusts is mathematically the same as a lag on the wrench, because the mixer is linear (checked). The two conditions matter, though. Entry 07 may give the rotors different time constants as part of a lag mismatch, and the firmware's clipping acts on rotor commands. We also have not verified how MuJoCo integrates activation states inside its RK4 step. Implementing the lag in the interface, with the exact exponential update, removes all three questions.

**Applying the wrench.** In MuJoCo the wrench can be applied through `xfrc_applied` or through the model's (re-geared) actuators. The frame in which `xfrc_applied` expresses its torque, and the point at which it applies its force, must be verified against the MuJoCo documentation and tested in Phase 2 **[UNVERIFIED here]**. If the torque is world-frame, the body torque $\boldsymbol\tau$ must be rotated by $\mathbf R(\mathbf q)$ first.

---

## Delay in discrete time

Actuation delay is a pure plant mismatch (decision Q1). The MPC's model does not contain it. It has to be represented in two places: in the plant (to inject it), and in any *analysis* model that reasons about it (entries 07 and 10).

**Integer delays, $D = d\,\Delta t$.** The analysis model is augmented with a shift register holding the last $d$ commands, which adds $4d$ states:

$$x_{k+1} = \Phi x_k + \Gamma\,s^{(d)}_k, \qquad s^{(1)}_{k+1} = u_k, \qquad s^{(i)}_{k+1} = s^{(i-1)}_k\ \ (i = 2,\dots,d).$$

The oldest buffered command drives the plant. For $d = 3$ this reproduces the delayed system exactly (checked).

**Fractional delays, $D = (d + \varepsilon)\Delta t$ with $0 < \varepsilon < 1$.** The delayed command switches *within* a period. For the first $\varepsilon\Delta t$ the plant still sees $u_{k-d-1}$, and for the rest it sees $u_{k-d}$. Integrating the two pieces separately gives

$$x_{k+1} = \Phi x_k + \Gamma_0\,u_{k-d} + \Gamma_1\,u_{k-d-1},$$

$$\Gamma_0 = \int_0^{(1-\varepsilon)\Delta t} e^{As}\,ds\,B, \qquad \Gamma_1 = e^{A(1-\varepsilon)\Delta t}\int_0^{\varepsilon\Delta t} e^{As}\,ds\,B,$$

with $\Gamma_0 + \Gamma_1 = \Gamma$. This is the modified z-transform construction, derived here rather than cited. For a delay of $1.3\,\Delta t$ it matches a fine continuous simulation (checked). The register then needs $d + 1$ slots.

**Rational (Padé) approximations** of $e^{-sD}$ are needed only if a later entry needs a *continuous-time* rational model, for example for a frequency-domain gain computation in entry 10. They are not needed for discrete-time work, which uses the exact constructions above. Their accuracy at $\omega = 1/t_{\text{att}}$:
- **First order**, $(1 - sD/2)/(1 + sD/2)$: phase error below 0.5° up to $2\Delta t$, but 3.6° at $4\Delta t$ and 21.5° at $8\Delta t$. It is not usable over the whole grid.
- **Second order**: phase error of at most 1.6° (1.53° at $8\Delta t$) over the whole grid.

If a rational model is used, it is at least second order.

**The delay sweep grid** used by the perturbation study (entry 07) is defined in units of the control period and reported in all three units:

| $D/\Delta t$ | 0 | 0.5 | 1 | 1.5 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|---|---|---|
| $D$ (ms) | 0 | 5 | 10 | 15 | 20 | 30 | 40 | 60 | 80 |
| $D/t_{\text{att}}$ | 0 | 0.12 | 0.24 | 0.35 | 0.47 | 0.71 | 0.95 | 1.42 | 1.89 |
| $D/\tau_m$ | 0 | 0.08 | 0.15 | 0.23 | 0.31 | 0.46 | 0.62 | 0.93 | 1.24 |

The half-period points test the fractional construction. All values are multiples of the 1 ms plant step, so the plant's buffer represents them exactly. The grid deliberately extends beyond where we expect the nominal loop to lose stability. Assumption 2 is verified numerically on it (decision Q2), and the stress test needs points past that boundary. The grid is rescaled if entry 06 locates the boundary elsewhere.

---

## How discretization itself creates mismatch

A sampled controller is not the continuous controller of the earlier entries. The differences must be measured now, so that they are never mistaken for injected mismatch.

**Sample-and-hold acts like a delay of $\Delta t/2$.** Holding a command constant over a period has the frequency response

$$H_{\text{ZOH}}(j\omega) = \frac{1 - e^{-j\omega\Delta t}}{j\omega\Delta t} = e^{-j\omega\Delta t/2}\,\frac{\sin(\omega\Delta t/2)}{\omega\Delta t/2}.$$

Its phase is exactly $-\omega\Delta t/2$ below the Nyquist frequency, which is the phase of a pure delay of $\Delta t/2$. Its magnitude is a slight low-pass: 0.23 percent loss at $\omega = 1/t_{\text{att}}$ (both checked). At our period this is **5 ms**, which is $0.12\,t_{\text{att}}$ and $0.077\,\tau_m$.

**Where this matters and where it does not.** The distinction is easy to blur, so we state it precisely.
- **It does not matter at the sampling instants.** An MPC whose model is discretized with an exact ZOH already contains the hold. At the instants where it predicts, its model is not wrong about it, so the hold is not a model *mismatch* there.
- **It does matter for the loop.** Compared with the continuous loop, the sampled loop has lost the phase of a $\Delta t/2$ delay. Any continuous-time analysis of the closed loop must include it, and that includes the loop-gain computation of entry 10.
- **It sets the meaning of "zero injected delay".** At zero injected delay, the sampled loop already behaves as if it had about 5 ms of delay, relative to the continuous loop. Injected delays are therefore always reported as $D$ (what we inject). Any continuous-time analysis states the total effective delay $D + \Delta t/2$, plus one $\Delta t$ more if computation delay is ever modelled.

**What, then, is the discretization's contribution to model mismatch at zero injected mismatch?** Four things, and they define the noise floor that Phase 3 measures:
1. The prediction model's integration error. This is zero for an exact-ZOH LTI model, and $5\times10^{-6}$ relative on the linear part for one RK4 step per period.
2. The linearisation error, if the MPC is linear.
3. Behaviour between samples, which the prediction does not see.
4. The deliberate model simplifications that are not discretization at all, but appear at the same time: the diagonal inertia in the model against the full matrix in the plant (entry 03).

The plant's own integration error ($5\times10^{-11}$ over 0.2 s) is negligible against all four. No mismatch effect smaller than the measured noise floor will be claimed.

---

## What this entry commits us to, and what is open

**Committed.**
- **Control period** $\Delta t = 10$ ms (100 Hz): $\Delta t/t_{\text{att}} = 0.236$, $\Delta t/\tau_m = 0.155$, and a hold phase loss of 6.8° at $1/t_{\text{att}}$. The MPC is both the attitude and the position controller, so its period is set against $t_{\text{att}}$ and $\tau_m$.
- **Plant** integrated with RK4 at $h = 1$ ms (10 steps per period), renormalising the quaternion after each step. This is verified fourth order in the reference implementation; MuJoCo's timestep is set to 1 ms explicitly.
- **Prediction model**: exact ZOH for a linear model, at least one RK4 step per period for a nonlinear one, and never forward Euler. Euler would mis-state the motor lag by 7.9 percent, mis-predict the lateral response by 14 percent over 0.5 s, and lose any closed-loop mode faster than $2\zeta/\Delta t$.
- **Quaternions**: the exponential map is the exact update for a held rate. A nonlinear MPC may carry the quaternion without renormalisation inside a 0.5 s horizon (drift below $4\times10^{-7}$ at 20 rad/s). An LTI MPC uses the 16-state error form.
- **Timing chain**: commands held over the period; delay buffer, exact motor-lag update, mixer and wrench application all in our interface at the plant step; zero computation delay in simulation, as an explicit modelling assumption.
- **Delay**: integer delays as a $4d$-state shift register, fractional delays by the modified-z construction ($\Gamma_0 + \Gamma_1 = \Gamma$), Padé only where a continuous rational model is needed and then at least second order. The delay grid is $\{0, 0.5, 1, 1.5, 2, 3, 4, 6, 8\}\,\Delta t$.
- **Sample-and-hold** is equivalent in phase to a $\Delta t/2 = 5$ ms delay. It is part of every continuous-time loop analysis and is never confused with injected delay.

**Deferred, noted, not forgotten.**
- The horizon length and the MPC's form (LTI on the hover linearisation, or nonlinear), hence the choice between the two quaternion treatments: entry 06.
- The closed-loop speeds, which decide whether the Euler-stability table ever binds (it does not if Euler is never used, but it constrains anyone who would use it): entry 06.
- Solve-time measurement at 100 Hz, which could force a dated change of $\Delta t$: Phase 3.
- The frame and application point of MuJoCo's `xfrc_applied`, and the order check on MuJoCo's own RK4: Phase 2.

**Open for the next entry (06, the nominal MPC for tracking).** We now have discrete-time models of the plant and of the prediction, a fixed control period, and a quantified cost of sampling. Entry 06 builds the nominal MPC on them. It covers the state and input, the sign-corrected attitude error, the input-only hard constraints of decision Q2, a horizon chosen against $t_{\text{att}}$ and $\tau_m$, the absence of a disturbance model (decision Q3), and the terminal ingredients of MPC for tracking. It must also say which results are proven for the LTI design on the hover linearisation and which are only tested on the nonlinear plant.

*This entry is locked. We do not proceed to 06 until the following can be explained in plain language: why the MPC's period is judged against the attitude time scale and the motor lag rather than against the trajectory; why exact ZOH discretization is exact and forward Euler is not, in each of the three ways shown; why the exponential map keeps a quaternion on the unit sphere; why the motor lag lives in our simulator interface rather than in MuJoCo's actuator dynamics; how integer and fractional delays are represented; and why sample-and-hold costs the phase of a $\Delta t/2$ delay without being a model mismatch at the sampling instants.*
