# 06 — The Nominal MPC

*Derivation notebook, entry 06. Builds the controller that the whole project supervises and never modifies: a linear MPC for tracking on the lagged hover model of entries 04–05. Every number is computed in [checks/check_06_nominal_mpc.py](checks/check_06_nominal_mpc.py), using the design module [checks/mpc06.py](checks/mpc06.py) and the parameters in [checks/params.py](checks/params.py). Decisions are logged in [decisions.md](decisions.md).*

---

## What this entry is for

Everything in this project is measured against the nominal MPC. Zero trust means "this controller, unchanged". The realizability ratio (entry 09) is computed from its predicted inputs. The loop-gain bound (entry 10) is computed from its closed loop. Assumption 2 is a statement about its stability under each mismatch. So this entry has to fix the controller completely: where it runs, what it knows, what it optimizes, under which constraints, with which horizon and weights. It also has to state what we can *prove* about it and what we can only *test*.

---

## 1. Where the controller runs (deployment decision)

**Decision (dated in [decisions.md](decisions.md)).** The nominal controller is a **linear MPC, in error form about hover, solved on board the Crazyflie's STM32F405** (Cortex-M4F, 168 MHz, 192 kB SRAM), with a target rate of **100 Hz** (the period of entry 05). A nonlinear MPC is at most a simulation-only comparison.

**Why on board, and why linear.** Running on board keeps the loop free of a radio link. Offboard control would solve the MPC on a PC and send commands by radio. That is the alternative we rejected, because **radio latency becomes a real, unmeasured delay**. Delay is one of the four mismatches this project injects *on purpose*, and entry 05 showed that the loop's tolerance for delay is a few control periods. An unmeasured delay of that size would contaminate every delay result. A linear MPC with input-only hard constraints (decision Q2) is a convex QP of fixed structure, which is what embedded solvers handle within a fixed time budget. A nonlinear MPC would need sequential QPs or a nonlinear solver, which no verified precedent runs on this microcontroller.

**Verified precedent** (references [54]–[58] in the README list):
- **TinyMPC** (Nguyen, Schoedel, Alavilli, Plancher, Manchester, ICRA 2024, DOI 10.1109/ICRA57147.2024.10610987) ran a linear MPC on board the Crazyflie 2.1. It used 12 states and 4 inputs, at 500 Hz with N = 15, and at 100 Hz with N = 20 for its largest problem (316 variables). The solver is ADMM, with input bounds and some state constraints. The paper reports convergence within 7 iterations but gives no numeric solve time on the STM32.
- **Linder** (*Real-time certified MPC for a nano quadcopter*, Master's thesis, Linköping University, 2024) ran DAQP on board a Crazyflie **2.0** as an *outer-loop* MPC at 100 Hz (36 variables, 60 constraints). It computed set-points for the vehicle's own PID controllers, with solve times certified offline.
- **Wikner, Arnström, Axehill** (arXiv:2603.09342, 2026) ran an *inner-loop* MPC on the Crazyflie 2.1 STM32F405 with DAQP and TinyMPC. It used 12 states, 4 inputs, N = 15 and 500 Hz, with input-only constraints, a Riccati terminal cost and no terminal set, which is our structure. Its worst measured DAQP solve time was 1375 µs.
- **DAQP** itself: Arnström, Bemporad, Axehill, *IEEE TAC* 67(8), 2022, DOI 10.1109/TAC.2022.3176430.

**Closest prior work, and how ours differs.** Akbari, Frank and Greeff (*Tiny learning-based MPC for multirotors*, Mechatronics 115, 2026, DOI 10.1016/j.mechatronics.2025.103452) put a learned model of the residual acceleration (a Gaussian process) **inside** an embedded MPC. It runs on a Teensy 4.0 mounted on a Crazyflie 2.1 at about 100 Hz. The GP mean corrects the model, and its covariance tightens the thrust constraints. Our supervisor does the opposite. The learned correction **never enters the MPC's model or constraints**. It shifts only the reference, scaled by a trust level whose upper bound is computed analytically, and zero trust reproduces the unmodified controller exactly. Their design is baseline-7-like (residual inside the model, README). Ours keeps the nominal controller's guarantees intact by construction, at the price of whatever performance is lost by correcting through the reference. Measuring that price is a research question of this project, not an assumption.

---

## 2. What the MPC's model is

**State.** The MPC uses the 16-state error form of entry 05: position, velocity, attitude error (roll, pitch, yaw as a small rotation vector), body rates, and delivered rotor thrusts, all as deviations from hover. Its input is the four *commanded* rotor-thrust deviations. The 17-state quaternion model differs only by the redundant $q_0$ direction, which is constant to first order (entry 05). That direction is also uncontrollable, so it must be removed for the Riccati equation used below to have a solution. The 16-state model is controllable from the four commands (checked).

**Discretization.** Exact zero-order hold at $\Delta t = 10$ ms (entry 05).

**Heading frame.** The hover linearisation is written at zero yaw. At any other heading $\psi_0$, the same model holds exactly if position and velocity are expressed in the *heading frame*, i.e. rotated by $\mathbf R_z(-\psi_0)$. The check script shows that the Jacobian of the nonlinear model about hover at yaw 0.7, 2.0 and −2.8 rad, written in the heading frame, equals the zero-yaw model to $5\times10^{-7}$. So one LTI model serves every heading, and the MPC rotates its position and velocity inputs into the heading frame of the current yaw.

**The delivered thrusts are estimated, not measured.** The Crazyflie has no thrust sensor. The MPC therefore carries its own copy of the lag model, driven by the commands it has sent:

$$\hat{\mathbf f}_{k+1} = e^{-\Delta t/\tau_m}\,\hat{\mathbf f}_k + \big(1 - e^{-\Delta t/\tau_m}\big)\,\mathbf u_k .$$

With no mismatch this estimate is exact. Under a motor-lag error or an actuation delay it is not, because the true thrusts respond more slowly, or later, than the MPC believes. This is part of how those mismatches act, and every closed-loop analysis below includes it. The rest of the state (position, velocity, attitude, rates) comes from the simulator in this study. On hardware it would come from the state estimator, which is outside the scope of the simulation study (entry 00).

---

## 3. The reference and the artificial steady state

**What the reference contains.** The tracked outputs are position and yaw, $\mathbf y = (x, y, z, \psi) = \mathbf C\mathbf x$, so $\mathbf r \in \mathbb R^4$. This settles the reference-content question of the audit (F-7) for the primary design. Hover at any position and yaw is a steady state with zero input deviation, so the set of steady states is

$$\mathbf x_s = \mathbf G\,\boldsymbol\theta, \quad \mathbf u_s = \mathbf 0, \qquad \boldsymbol\theta \in \mathbb R^4,$$

where $\mathbf G$ places $\boldsymbol\theta$ into the position and yaw states.

**MPC for tracking.** Following Limon et al. (Automatica 2008; IEEE TAC 2018), the MPC does not steer to $\mathbf r$ directly. It steers to an **artificial steady state** $\boldsymbol\theta$, which is itself a decision variable, and pays an offset cost for $\boldsymbol\theta \ne \mathbf r$. The point is that a reachable target always exists, whatever the reference does.

**What this formulation assumes about the reference.** MPC for tracking is designed for *piecewise-constant* references: at each solve, $\mathbf r$ is held constant over the horizon. The correction therefore enters the whole horizon, but the MPC does not see *future changes* of the reference.

This needs saying, because entry 02 credited reference injection with "preview". For a constant shift, the shift is visible over the horizon as a constant; for a changing one, it is not previewed. There is a measurable cost on trajectories. On a 1 m/s ramp, the setpoint-only formulation lags by **0.47 m** in steady state, while rotor drag adds only **0.023 m** to that.

A trajectory version would track a full reference trajectory over the horizon, with velocity and acceleration feedforward obtained from differential flatness (entry 02). That version is **deferred to entry 08**, because it is tied to the critical open problem F-2: a correction for a zero-DC mismatch is likely to be a horizon-wise sequence, which the piecewise-constant formulation cannot carry. The predictions below are registered for the setpoint formulation.

---

## 4. The quadratic program

**Decision variables (condensed form).** $\mathbf z = (\mathbf U, \boldsymbol\theta, \mathbf s)$, with:
- $\mathbf U = (\mathbf u_0, \dots, \mathbf u_{N-1}) \in \mathbb R^{4N}$, the commanded-thrust deviations;
- $\boldsymbol\theta \in \mathbb R^4$, the artificial steady state;
- $\mathbf s = (s_{\text{tilt}}, s_{\text{alt}}) \in \mathbb R^2$, one slack per soft-constraint family.

The predicted states are eliminated through the exact discrete model, $\mathbf x_{k} = \Phi^k\mathbf x_0 + \sum_{j<k}\Phi^{k-1-j}\Gamma\,\mathbf u_j$.

**Cost.**

$$J = \sum_{k=0}^{N-1}\Big(\|\mathbf x_k - \mathbf G\boldsymbol\theta\|_{\mathbf Q}^2 + \|\mathbf u_k\|_{\mathbf R}^2\Big) + \|\mathbf x_N - \mathbf G\boldsymbol\theta\|_{\mathbf P}^2 + \|\boldsymbol\theta - \mathbf r\|_{\mathbf T}^2 + \rho_1(s_{\text{tilt}} + s_{\text{alt}}) + \rho_2(s_{\text{tilt}}^2 + s_{\text{alt}}^2).$$

$\mathbf P$ is the solution of the discrete algebraic Riccati equation for $(\Phi, \Gamma, \mathbf Q, \mathbf R)$. It is the terminal cost on the artificial steady state allowed by decision Q2.

**Hard constraints (inputs only, decision Q2):**
- Box: $f_{\min} - f_{\text{hover}} \le \mathbf u_k \le f_{\max} - f_{\text{hover}}$, i.e. $[-0.0583,\ +0.0489]$ N per rotor. This is the convex restriction of the firmware's set (entry 01).
- Rate: $\|\mathbf u_k - \mathbf u_{k-1}\|_\infty \le \delta_{\max} = 0.0268$ N, with $\mathbf u_{-1}$ the previously applied command.

**Soft constraints**, for $k = 1..N$:
- Tilt: $|\delta\phi_k| \le \phi_{\max} + s_{\text{tilt}}$ and $|\delta\theta_k| \le \phi_{\max} + s_{\text{tilt}}$, with $\phi_{\max} = 0.35$ rad (20°).
- Altitude: $z_k \ge z_{\min} - s_{\text{alt}}$, with $z_{\min} = 0$.
- Slacks: $s \ge 0$.

**No terminal set** (decision Q2) and **no disturbance model** (decision Q3).

**Sizes and sparsity.** The condensed QP has **86 variables** (80 inputs, 4 for the steady state, 2 slacks), 86 simple bounds, and **180 general inequality rows** (80 rate, 80 tilt, 20 altitude). Its Hessian is dense, but constant: it does not depend on the state, so it can be factorised offline and stored in flash. The equivalent sparse form has 406 variables and 320 equality rows (the dynamics). Its matrices are block-banded, which is the structure ADMM solvers such as TinyMPC exploit.

**Convexity.** The Hessian is positive definite: its smallest eigenvalue is 200 and its condition number $4.8\times10^3$. So the QP is strictly convex and its minimizer is unique. Positive definiteness follows from $\mathbf R \succ 0$, $\mathbf T \succ 0$ and $\rho_2 > 0$, and is checked numerically.

**Why one slack per family rather than one per step.** A first version had a slack for every constraint at every step (60 slacks). Almost all of them sat at zero, which made an active-set solver carry about 60 extra active bounds and raised the iteration count. One slack per family softens the constraints in the ∞-norm. This is still an exact penalty: for $\rho_1$ above the largest constraint multiplier, the slack is zero whenever the constraint can be met. It reduces the problem from 144 to 86 variables.

**Feasibility is trivial.** Holding the previous command over the whole horizon satisfies the box and rate constraints, because the previous command was in the box, and large enough slacks satisfy the soft constraints. So the QP has a feasible point for every state, every previous command and every reference, **whatever the mismatch**. This is the feasibility part of Target Result 2 (decision Q2). It is checked on 50 random states far from hover, and DAQP solved all of them.

---

## 5. Horizon and weights

**Horizon $N = 20$ (200 ms = 4.7 $t_{\text{att}}$ = 3.1 $\tau_m$).** With the Riccati terminal cost, the *unconstrained* MPC law does not depend on N at all: the finite-horizon optimum already equals the infinite-horizon LQR (section 6). So the horizon matters only for how far ahead constraints are seen. Three considerations set it:
- **The motor lag.** It settles to 95 percent in $3\tau_m$, so a horizon of $3.1\tau_m$ sees a commanded thrust change fully delivered.
- **The attitude manoeuvre.** Rotating to a new tilt, holding it, and rotating back takes a few attitude time scales; 4.7 $t_{\text{att}}$ covers one.
- **The embedded budget.** The verified on-board precedents use N = 15 to 20.

**Weights (Bryson's rule).** Each state weight is $1/(\text{acceptable deviation})^2$.

| State | Acceptable deviation | Weight |
|---|---|---|
| position | 0.10 m | 100 |
| velocity | 0.50 m/s | 4 |
| roll, pitch | 0.35 rad (the soft tilt limit) | 8.2 |
| yaw | 0.50 rad | 4 |
| body rates | 2.0 rad/s | 0.25 |
| delivered thrusts | — (not penalised) | 0 |

The input weight is $\mathbf R = \mathbf I/h^2$, where $h = \min(f_{\max} - f_{\text{hover}},\ f_{\text{hover}} - f_{\min}) = 0.0489$ N is the rotor's thrust headroom about hover. That is the ceiling side, because the vehicle is ceiling-limited (entry 04). So a command that uses the whole headroom costs as much as a 0.1 m position error.

The offset weight is $\mathbf T = 100\,\mathbf G^\top\mathbf P\mathbf G$, a hundred times the cost of moving the steady state. The artificial steady state therefore follows the reference closely when the constraints allow, which recovers ordinary LQR tracking in the unconstrained case. It can still lag the reference when constraints bind, which is its purpose.

The slack penalties are $\rho_1 = 10^4$ and $\rho_2 = 10^2$. The rate limit $\delta_{\max}$ is a quarter of the thrust range per period: the full range in 40 ms, faster than the motor itself can follow (0.6 $\tau_m$). It bounds command chatter without restricting normal manoeuvres.

The 20° tilt limit keeps the vehicle where the linear model is good. At 20°, $\sin$ differs from its argument by 2 percent and $\cos$ from 1 by 6 percent. It is far inside the 54° coordinated-tilt limit of entry 04.

---

## 6. The unconstrained law (for Target Result 1)

When no constraint is active and the slacks are zero, the QP's first move is an affine function of the state and the reference. With the Riccati terminal cost, the optimal inputs for a *fixed* $\boldsymbol\theta$ are the LQR inputs $\mathbf u_k = \mathbf K(\mathbf x_k - \mathbf G\boldsymbol\theta)$, and the remaining cost is $(\mathbf x_0 - \mathbf G\boldsymbol\theta)^\top\mathbf P(\mathbf x_0 - \mathbf G\boldsymbol\theta)$. Minimizing that plus the offset cost over $\boldsymbol\theta$ gives

$$\boldsymbol\theta^* = \mathbf S\big(\mathbf G^\top\mathbf P\,\mathbf x_0 + \mathbf T\,\mathbf r\big), \qquad \mathbf S = \big(\mathbf G^\top\mathbf P\mathbf G + \mathbf T\big)^{-1},$$

$$\mathbf u_0 = \mathbf K_x\mathbf x_0 + \mathbf K_r\mathbf r, \qquad \mathbf K_x = \mathbf K\big(\mathbf I - \mathbf G\mathbf S\mathbf G^\top\mathbf P\big), \qquad \mathbf K_r = -\mathbf K\mathbf G\mathbf S\mathbf T .$$

On 20 random small states the QP's first move equals this law to $1.6\times10^{-15}$. This is the affine law that Target Result 1 refers to, and the sensitivity $\partial\mathbf u/\partial\mathbf r$ that the realizability ratio needs (entry 09).

**No steady-state offset for constant references, nominally.** At a closed-loop steady state, $\mathbf x = \mathbf G\boldsymbol\theta$ and $\boldsymbol\theta = \mathbf S(\mathbf G^\top\mathbf P\mathbf G\boldsymbol\theta + \mathbf T\mathbf r)$, which gives $\mathbf T\boldsymbol\theta = \mathbf T\mathbf r$, i.e. $\boldsymbol\theta = \mathbf r$. The check confirms a DC gain from $\mathbf r$ to $(x, y, z, \psi)$ equal to the identity. *Under mismatch* there is an offset, because the MPC has no disturbance model. That is the mass prediction below.

---

## 7. Loop transfer functions, crossover and margins

**Where the loop is broken.** The loop is broken at the **plant input**, i.e. the four commanded thrusts. The controller there is the affine law together with its internal lag model: $\mathbf u = (\mathbf I - \mathbf K_{x,f}\mathbf F(z))^{-1}\mathbf K_{x,\text{rigid}}\,\mathbf x_{\text{rigid}}$, where $\mathbf F(z) = (1-a)/(z-a)$ is the lag model's transfer function and $a = e^{-\Delta t/\tau_m}$. Breaking the loop here is what a delay or lag error at the actuators actually perturbs. The loop is expressed in wrench channels (collective, roll, pitch, yaw) through the mixer, where it is almost diagonal at hover. The margins below are single-loop margins, each channel broken with the others closed.

| Channel | Crossover $\omega_c$ | $\omega_c\,t_{\text{att}}$ | Phase margin | Gain margin | Delay margin $\text{PM}/\omega_c$ |
|---|---|---|---|---|---|
| Collective | 7.6 rad/s | 0.32 | 52.2° | 24.4 (27.8 dB) | 120 ms |
| Roll | 28.5 rad/s | 1.21 | 41.5° | 5.8 (15.3 dB) | 25.4 ms |
| Pitch | 28.5 rad/s | 1.20 | 41.4° | 5.8 (15.3 dB) | 25.4 ms |
| Yaw | 9.2 rad/s | 0.39 | 54.0° | 19.9 (26.0 dB) | 102 ms |

Each loop has one gain crossover. The roll and pitch loops are the binding ones: they carry the four-integrator lateral chain behind the motor lag (entry 04). Their crossover is about $1.2/t_{\text{att}}$, and $\omega_c\Delta t = 0.29$ rad is far below the Nyquist frequency (314 rad/s).

**The $D + \Delta t/2$ convention.** These loops are computed in discrete time with the exact ZOH model, so they already contain the sample-and-hold. Its $\Delta t/2$ phase is inside the 41.5° margin, not on top of it. So the delay margin of 25.4 ms is the **injected** delay $D$ the loop can additionally tolerate. Any continuous-time statement uses the total effective delay $D + \Delta t/2$, which is 30.4 ms at the cliff (entry 05). The margin estimate $\text{PM}/\omega_c$ agrees with the exact delay cliff, computed from the eigenvalues of the delayed closed loop, to better than 0.2 ms.

---

## 8. What is proven and what is only tested

**Proven** (for the LTI design on the hover linearisation; each is also checked numerically):
- The QP is strictly convex and feasible for every state, previous command and reference, under any plant. This is because only the input constraints are hard.
- Where no constraint is active, the MPC equals the affine law of section 6 (piecewise-affine structure of linear MPC).
- The unconstrained closed loop is stable: its spectral radius is 0.9587, including the internal lag model. It tracks constant references with zero offset, and its reference-to-position gain is finite (frequency-sweep estimate of the H∞ norm: 1.00, no peaking).

**Not proven, only tested:**
- **Stability of the *constrained* closed loop.** MPC for tracking guarantees it with a terminal *set* (Limon et al.). Decision Q2 removes the terminal set, so that guarantee does not apply. With the Riccati terminal cost, the optimal cost is a Lyapunov function only where the constraints are inactive along the predicted tail. Outside that region, stability is a property we test, and Assumption 2 is verified numerically, as decision Q2 requires.
- **Behaviour on the nonlinear plant.** A closed-loop test on the nonlinear plant used the full inertia matrix, RK4 at 1 ms and the constrained QP solved at every period. It flew a 1 m step in x, 0.5 m in z and 0.3 rad in yaw, simultaneously. It converged to within $10^{-4}$ m and rad. The commands stayed inside the thrust box (reaching the ceiling during the step) and inside the rate limit. The tilt stayed within 0.3° of the soft limit. Every DAQP solution was checked against the KKT conditions. Primal feasibility, relative stationarity and complementarity all held to $10^{-8}$; the observed worst values were about $10^{-15}$, $10^{-15}$ and $10^{-10}$. OSQP, a first-order solver, is used only as a loose sanity check: its first moves must agree to $10^{-3} f_{\max}$. The observed differences are reported rather than bounded tightly. A tight bound on a first-order solver failed once in 400 solves on a clean Linux machine. This is one test, not a proof. The systematic sweep over mismatches is in later entries.

**Assumption 2 for the nominal (no-mismatch) case** is therefore verified: Schur stability of the linear loop, a finite reference-to-state gain, and the nonlinear closed-loop test above. The mismatch sweep (mass, drag, lag error, delay, combinations) stays with entries 07 and 10.

---

## 9. Re-registered predictions

With the controller fixed, the provisional predictions of entry 04 can be made quantitative. They are recorded as **re-registered predictions** in [decisions.md](decisions.md), dated, without overwriting entry 04. All of them are for the setpoint formulation with no disturbance model, in the linear loop.
- **Delay cliff:** $D = 25.4$ ms = **2.54 $\Delta t$** = **0.60 $t_{\text{att}}$**. The effective loop delay there is $D + \Delta t/2 = 30.4$ ms.
- **Motor-lag cliff:** the true time constant must be **3.28 times** the nominal one (0.212 s = 5.0 $t_{\text{att}}$) before the loop loses stability. Errors of −90, −50, −30, +30, +50 and +100 percent all leave it stable. **So within the planned ±30–50 percent sweep, a lag error cannot destabilize the nominal loop.** If the "hurts" regime appears for lag errors at all, it must come from the correction path (entry 10), not from the nominal loop. The delay sweep, which crosses its cliff at about 2.5 $\Delta t$, is the stronger test.
- **Mass:** +10 percent gives a steady altitude offset of **−49 mm** and +20 percent gives −99 mm, linear in the error.
- **Drag:** in hover and on set-points, no steady effect. On a 1 m/s ramp it adds **0.023 m** to an intrinsic setpoint-tracking lag of 0.47 m.

---

## 10. Solve time

**Measured on a PC** (Python, one core, 400 closed-loop solves of the step test above):

| Solver | Median | 99th percentile | Worst | Iterations |
|---|---|---|---|---|
| DAQP (dual active set) | 0.046 ms | 1.1 ms | about 1.5 ms | median 3, max 94 |
| OSQP (ADMM, tolerance $10^{-7}$, polishing) | 1.4 ms | heavy tail, up to hundreds of ms | — | — |

All times are solver-internal. The DAQP worst case varies by a few tenths of a millisecond between runs. The worst cases occur during saturated transients, when many rate and box constraints are active; most solves take a handful of iterations. With the tight tolerance chosen here, OSQP is slow to converge in some transients. It serves only as a cross-check.

**On-board feasibility: an estimate, not a fact.** The closest verified anchor is Wikner, Arnström and Axehill: DAQP on the same microcontroller, worst case 1375 µs. Their problem had 12 states, N = 15 and input constraints only, about 60 condensed variables. Ours has 86 variables and, in addition, 180 general rows.
- If the per-iteration cost of a dense active-set method grows roughly with the square of the number of variables, ours costs about 2× theirs per iteration.
- At a comparable worst-case iteration count that suggests about 3 ms, and at three times their iterations about 9 ms. That is within, but close to, the 10 ms period.
- Typical solves (a few iterations) are far below the budget.

The worst case is the open risk. Three mitigations are available:
- warm-starting the active set from the previous solve;
- a shorter horizon (N = 15 cuts the variables to 66);
- certifying the worst-case iteration count offline, as Linder did, instead of relying on a measured sample.

**Memory.** The constant Hessian and constraint matrix need about 30 kB and 62 kB in single precision. That does not fit beside the firmware in 192 kB of SRAM, so they have to live in flash (1 MB). The on-board solve time must be measured on the hardware before any timing claim is made.

---

## 11. Amendment (2026-11-12): computation delay

*Added after review. Sections 7–10 above describe implementation (a) and are kept unchanged, so that the earlier numbers stay visible. This section supersedes them where they differ.*

**The problem.** Entry 05 assumed zero computation delay: the command computed from the state at $t_k$ is applied at $t_k$. On board, the solve takes time. The estimate of section 10 allows up to about 9 ms in the worst case, against a delay margin of 25.4 ms. That is not negligible. There are two ways to implement the controller.

- **(a) Apply-on-solve.** The command is applied as soon as it is computed, somewhere between $t_k$ and $t_k + t_{\text{solve}}$. The delay varies from solve to solve (jitter), and the MPC does not model it. Sections 7–10 analyse this case with zero delay.
- **(b) Fixed one-step computation delay, modelled.** The command computed from the state at $t_k$ is applied at exactly $t_{k+1}$. The MPC knows this. It first predicts the next state with the command already in flight, $\hat{\mathbf x}_{k+1|k} = \Phi\,[\mathbf x_{\text{rigid},k};\ \hat{\mathbf f}_k] + \Gamma\,\mathbf u_{k-1}$, then solves the same QP from $\hat{\mathbf x}_{k+1|k}$. Its thrust estimate is updated with the command actually applied, $\hat{\mathbf f}_{k+1} = a\hat{\mathbf f}_k + (1-a)\mathbf u_{k-1}$. The model is thereby augmented by the four-component command in flight. The QP itself is unchanged.

**Both analysed the same way** (loop broken at the controller output, internal lag model included, discrete time, so the hold is inside the margins):

| | (a) apply-on-solve | (b) one-step delay, modelled |
|---|---|---|
| Nominal spectral radius | 0.9587 | 0.9587 (the predictor is exact) |
| Roll/pitch crossover | 28.5 rad/s ($1.21/t_{\text{att}}$) | 25.2 rad/s ($1.07/t_{\text{att}}$) |
| Roll/pitch phase margin | 41.5° | 35.6° |
| Roll/pitch gain margin | 5.8 (15.3 dB) | 2.66 (8.5 dB) |
| Collective / yaw: crossover, PM, GM | 7.6 rad/s, 52°, 24.4 / 9.2 rad/s, 54°, 19.9 | 7.3 rad/s, 50°, 8.9 / 8.8 rad/s, 51°, 7.4 |
| Delay cliff (injected, beyond what is modelled) | 25.4 ms = 2.54 $\Delta t$ | 24.6 ms = 2.46 $\Delta t$ = 0.58 $t_{\text{att}}$ |
| Lag-error cliffs | above 3.28 $\tau_m$ only | below **0.186 $\tau_m$** (12 ms) and above **2.86 $\tau_m$** |
| Mass +10 %: altitude offset | −49 mm | −53 mm |
| 1 m/s ramp: set-point lag (drag adds) | 0.466 m (+0.023 m) | 0.476 m (+0.024 m) |

**The delay budget.** The loop tolerates a total effective delay, measured against the ideal continuous loop. Everything that delays the loop is paid out of it:

$$\underbrace{D_{\text{inj}} + D_{\text{est}}}_{\text{budget}} \;=\; \underbrace{D_{\text{tot}}}_{\text{tolerable}} - \underbrace{\Delta t/2}_{\text{hold}} - \underbrace{D_{\text{comp}}}_{\text{computation}}.$$

- **(a)** $D_{\text{tot}}$ = 30.4 ms. The computation delay is unmodelled and can reach the worst-case solve time. With the estimated 9 ms, the budget is **16.4 ms − $D_{\text{est}}$**. Two caveats make this optimistic. The 9 ms figure is itself an estimate. And a time-varying jitter is in general harder on a loop than a fixed delay of the same size, which the fixed-delay analysis does not capture.
- **(b)** $D_{\text{tot}}$ = 39.6 ms (5 ms hold + 10 ms modelled computation + 24.6 ms). The computation delay is modelled, so it costs nothing extra as long as every solve finishes within one period. The budget is **24.6 ms − $D_{\text{est}}$**.

**State-estimation latency, $D_{\text{est}}$, is an unmeasured delay.** On hardware, the state fed to the MPC comes from the on-board estimator, which fuses a positioning system with the IMU. Both the positioning system's latency and the estimator's own delay reach the loop as delay. Neither is measured yet. They are a **line item to be bench-measured**, and every injected-delay result on hardware must be read against $24.6\ \text{ms} - D_{\text{est}}$. In simulation $D_{\text{est}} = 0$, because the simulator provides the true state.

**Decision: (b).** It costs margin. The gain margin falls from 5.8 to 2.66, the phase margin from 41.5° to 35.6°, and a lower lag-error cliff appears: a true motor more than five times faster than modelled would destabilize the predictor. It saves more than it costs. The budget for injected delay plus estimation latency rises from about 16.4 ms (and only if the 9 ms worst-case solve estimate holds) to 24.6 ms. The computation delay also becomes fixed and known instead of a jitter of unknown size. The lower lag cliff, at 12 ms against an identified 65 ms, lies far outside any credible motor-model error, and the planned ±50 percent sweep stays stable. **The hardware will run (b).** If a solve misses the 10 ms deadline, the controller applies the second move of the previous solution, $\mathbf u_{1|k-1}$, which is already computed. That keeps the timing fixed. How often deadlines are missed is part of the on-board measurement.

**PC solve times under (b)** are essentially those of section 10: median 0.05 ms, worst case 1.5–3.4 ms across runs, up to 97 iterations during saturated transients. The predicted initial state shifts the QP slightly. Every solution passes the KKT check above, and the nonlinear closed-loop test passes under (b) as well.

**Consequence for the predictions.** The cliffs are re-registered in [decisions.md](decisions.md) as a dated amendment, with the section 9 values kept. All later entries (07 onward) use implementation (b).

---

## What this entry commits us to, and what is open

**Committed.**
- **Deployment:** linear MPC solved on board the STM32F405 at 100 Hz; nonlinear MPC only as a simulation comparison; offboard control rejected because of unmeasured radio delay.
- **Model:** the 16-state error model of entries 04–05 (exact ZOH, 10 ms), in the heading frame, with the delivered thrusts estimated by the controller's own lag model.
- **Reference:** $\mathbf r = (x, y, z, \psi)$, piecewise constant over the horizon, with an artificial steady state $\boldsymbol\theta \in \mathbb R^4$ and offset cost $\mathbf T = 100\,\mathbf G^\top\mathbf P\mathbf G$.
- **QP:** 86 variables, strictly convex. Hard input box $[f_{\min}, f_{\max}]$ and rate limit 0.0268 N per period. Soft tilt (20°) and altitude constraints with one slack per family. Riccati terminal cost, no terminal set, no disturbance model. Feasible for every state under any plant.
- **Horizon and weights:** N = 20 (4.7 $t_{\text{att}}$, 3.1 $\tau_m$), Bryson weights as tabulated, $\mathbf R = \mathbf I/(0.0489\ \text{N})^2$.
- **Unconstrained law:** $\mathbf u = \mathbf K_x\mathbf x + \mathbf K_r\mathbf r$, equal to the QP where no constraint is active.
- **Nominal loop:** roll and pitch crossover 28.5 rad/s ($1.2/t_{\text{att}}$) with 41.5° phase margin, gain margin 5.8 and delay margin 25.4 ms. The collective and yaw loops are slower and have larger margins.
- **Assumption 2, nominal case:** verified, for both implementations.
- **Implementation (amendment, section 11):** (b), a fixed one-step computation delay modelled by prediction. Roll/pitch crossover 25.2 rad/s, PM 35.6°, GM 2.66. Injected-delay cliff 24.6 ms. Stable for a true motor time constant between 0.186 and 2.86 times the modelled one. Budget for injected delay plus estimation latency: 24.6 ms − $D_{\text{est}}$. On a missed deadline the controller applies the previous plan's second move.
- **Re-registered predictions:** as in section 9 (implementation a), amended for implementation (b) in decisions.md.

**Deferred, noted, not forgotten.**
- The trajectory (preview) formulation, with flat-state feedforward over the horizon: entry 08, with F-2.
- The mismatch sweep that verifies Assumption 2 for every mismatch in the set: entries 07 and 10.
- Measured on-board solve time and memory, and the choice between warm-started DAQP, a shorter horizon, or certified complexity: hardware phase.
- The state estimator on hardware (out of scope in simulation, entry 00).

**Open for the next entry (07, perturbation models).** The nominal controller is fixed and its cliffs are known. Entry 07 writes the augmented plant for each mismatch:
- the motor lag on thrust or on rotor speed, with per-rotor time constants;
- the delay buffer;
- rotor drag;
- mass, which also changes the inertia, if the change is a payload;
- combinations.

It normalises each severity against the vehicle's limits and against the cliffs computed here, so that the sweep grids bracket them.

*This entry is locked. We do not proceed to 07 until the following can be explained in plain language: why the MPC runs on board and why that excludes the radio from the delay budget; why the thrust states must be estimated and what that does under mismatch; what the artificial steady state is for and why the reference is piecewise constant here; why feasibility is trivial with input-only hard constraints; why the horizon does not change the unconstrained law; why constrained stability is tested rather than proven without a terminal set; where the loop is broken for the margins and why the hold is already inside them; and why, with this controller, a lag error of ±50 percent cannot destabilize the nominal loop while a delay of 2.5 periods can.*
