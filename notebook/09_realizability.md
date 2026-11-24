# 09 — The Realizability Ratio and Target Result 1

*Derivation notebook, entry 09. Defines the realizability ratio $r_H$ of the hard layer, proves Target Result 1 for the MPC actually used, states the region where the ratio is valid, and measures it against two exact references: the MPC's own piecewise-affine law, and a linear program for physical deliverability. Implemented in [checks/realiz09.py](checks/realiz09.py); every number is computed in [checks/check_09_realizability.py](checks/check_09_realizability.py). The MPC is the trajectory-tracking MPC of entry 08 with the constraints of entry 06, in implementation (b).*

---

## Why this entry exists

Entry 08 defined what correction *should* be applied: the oracle $\Delta\mathbf r^\star$. This entry asks what fraction of a requested correction the rotors can *deliver* through the unchanged MPC without changing how the MPC behaves. That fraction is the first bound of the hard layer, the one that guards against saturation. The proposal claimed three things about it:
1. it is a cheap ratio test on the MPC's predicted inputs;
2. below it, the correction is transmitted in full (Target Result 1);
3. it is exact for the linearised MPC and approximate otherwise.

Each claim is made precise here, and the second is proved.

---

## 1. Definition

At each solve, write every inequality row of the QP (entry 06), with the slacks at zero, in the form $\mathbf G\mathbf U \le \mathbf h(\mathbf x_0, \mathbf u_{-1})$. Here $\mathbf U$ stacks the $N = 20$ predicted commands. The rows are:
- **input box:** upper and lower, $2 \times 80$ rows;
- **input rate:** upper and lower, $2 \times 80$ rows, the first one relative to the command in flight;
- **soft tilt:** $|\phi_k|, |\theta_k| \le 20°$, $2 \times 40$ rows;
- **soft altitude:** $z_k \ge 0$, 20 rows.

That is **420 rows**.

A reference correction $\Delta\mathbf R$ (the position and yaw shift over the horizon, entry 08) requests the change of the predicted inputs

$$\delta\mathbf U = \frac{\partial\mathbf U^{\text{aff}}}{\partial\Delta\mathbf R}\,\Delta\mathbf R ,$$

where $\mathbf U^{\text{aff}} = -\mathbf H_U^{-1}\mathbf f(\mathbf x_0, \text{plan} + \Delta\mathbf R)$ is the unconstrained minimiser. Because the MPC is linear and time-invariant, this sensitivity is **one constant matrix** (80 × 84), computed offline. The proposal's "sensitivity about hover" is therefore not an approximation here: it is the sensitivity at every state.

**The realizability ratio** is the largest fraction of the request that keeps every row satisfied:

$$r_H = \max\{\beta \in [0,1] : \mathbf G(\mathbf U^\star + \beta\,\delta\mathbf U) \le \mathbf h\} = \min\Big(1,\ \min_{i:\ \mathbf g_i^\top\delta\mathbf U > 0}\frac{h_i - \mathbf g_i^\top\mathbf U^\star}{\mathbf g_i^\top\delta\mathbf U}\Big),$$

with $\mathbf U^\star$ the QP's current solution.
- A row already at its bound that the correction pushes further gives $r_H = 0$. A rotor at its ceiling asked for more thrust is exactly that case (checked).
- **One difference from the proposal:** it tested only the input box and rate. The soft tilt and altitude rows are rows of the same QP. If one of them becomes active, the MPC leaves its affine law just as surely, and Target Result 1 would be false (audit F-5). They must be in the test.

---

## 2. Target Result 1, and its proof

**Statement.** Consider the MPC at one solve, with state $\mathbf x_0$, command in flight $\mathbf u_{-1}$, and reference window $\mathbf F$ (plan plus any correction). Suppose that at $\mathbf F$ no row of the QP is active, i.e. the unconstrained minimiser $\mathbf U^{\text{aff}}(\mathbf F)$ satisfies all 420 rows strictly. Then, for every $\beta \in [0, r_H]$, the MPC's solution at $\mathbf F + \beta\,\Delta\mathbf F$ is the affine law $\mathbf U^{\text{aff}}(\mathbf F + \beta\,\Delta\mathbf F)$, with both slacks zero. **The correction is transmitted in full, and the applied command is affine in β.**

**Proof.** The QP is strictly convex (entry 06), so a point satisfying its KKT conditions is its unique minimiser. Take the candidate $\mathbf U = \mathbf U^{\text{aff}}(\mathbf F + \beta\Delta\mathbf F)$, $\mathbf s = \mathbf 0$, with every row multiplier zero. Then:
- **Stationarity in $\mathbf U$** holds by definition of the affine minimiser.
- **Stationarity in $\mathbf s$.** The cost gradient there is $\rho_1 > 0$, balanced by the multiplier of the bound $\mathbf s \ge 0$, which is therefore $\rho_1 > 0$. That bound is active, with $\mathbf s = 0$, so complementarity holds.
- **Primal feasibility.** The candidate is $\mathbf U^\star + \beta\delta\mathbf U$, because the law is affine in the window. That is exactly the point the ratio test certifies for $\beta \le r_H$.

All KKT conditions hold, so the candidate is the solution. ∎

**How tight.** Over 40 random cases satisfying the premise:
- the constrained QP's solution equals the affine law to $10^{-9}$ at 0.25, 0.75 and 0.999 of $r_H$;
- at 1.05 $r_H$ it departs from it.

So $r_H$ is the exact boundary, not a conservative margin.

**What it covers.** The result is exact for the MPC we run, because that MPC *is* linear. The proposal said it would hold "to first order" for a nonlinear MPC; that caveat is gone. On the nonlinear plant the applied command is still exactly affine in β; what is no longer linear is the plant's response to it. In the linear closed-loop analysis, the whole loop is linear in α when the correction stays inside $r_H$ at every step. A run in which $r_H$ never fell below 1 gives $\mathbf y(0.5) = (\mathbf y(0) + \mathbf y(1))/2$ to $10^{-12}$.

---

## 3. The validity region, and the piecewise-affine structure

A linear MPC with linear constraints is a **piecewise-affine** function of its parameters (Bemporad, Morari, Dua and Pistikopoulos, 2002; README [35]). Here the parameters are the state, the command in flight, the plan window and the correction. Parameter space splits into **critical regions** (polytopes). In each, one set of constraints is active and the solution is one affine function.

The region with **no active constraint** is the set of parameters where the unconstrained law satisfies all rows. **$r_H$ is the distance, along the direction of the correction, from the current parameter to the boundary of that region**, measured in units of the requested correction. The premise "no constraint active" is the statement that the current parameter lies in that region. That is the validity region of the ratio test.

**When constraints are already active.** The MPC is then in a different critical region, with a different affine law. The exact transmission ratio is the distance to the boundary of *that* region, computed from the KKT sensitivity of the current active set: how the solution and the active multipliers move with β. We call it $r_{\text{PWA}}$. It ends where an inactive row becomes active or an active multiplier reaches zero. It is exact for the QP, but it requires the active set and a solve with it.

---

## 4. Measuring the error against the exact answers

Two exact references:
- **$r_{\text{PWA}}$**: the largest fraction transmitted without changing the MPC's active set, the exact answer to "how much passes through unchanged".
- **$r_{\text{LP}}$**: the largest fraction of the requested *position* change over the horizon that *any* admissible input sequence could deliver. This is a linear program over all input sequences and the physical answer to "how much can the rotors do". It is written around the ratio-test candidate, so $r_{\text{LP}} \ge r_H$ whenever the current solution is feasible.

A first, naive formulation of the LP returned wrong optima: β ≈ 0 where the ratio test's candidate was strictly feasible, because of poor conditioning. Rewriting it in the null space of the output equality, with normalised rows, removed the problem; every LP in the study now solves.

**Scenarios.**
- **Three closed loops** with the constrained QP, each 6 s at 1 m altitude:
  - the 3 s figure-8 of entry 08;
  - a 2.2 s figure-8 at +30 % mass, peak acceleration 5.1 m/s²;
  - a 2.0 s figure-8 at +30 % mass, peak acceleration 6.2 m/s².
- **Two representative corrections**, evaluated every third step:
  - a mass-type vertical shift of 0.16 m (the size of the oracle for +30 % mass, entry 08);
  - a drag-type lateral lead of 0.05 m along the velocity.

| Scenario | Evaluations | Constraints active | $r_H < 1$ | $r_H > r_{\text{PWA}}$ | $r_{\text{LP}} - r_H$: max / mean |
|---|---|---|---|---|---|
| 3.0 s figure-8, nominal | 298 | 0 % | 50 % | 0 | 0.021 / 0.0001 |
| 2.2 s figure-8, mass ×1.3 | 298 | 69 % | 58 % | 64 | 0.030 / 0.0012 |
| 2.0 s figure-8, mass ×1.3 | 298 | 79 % | 60 % | 71 | 0.032 / 0.0020 |

What this shows:
1. **Where Result 1's premise holds, $r_H = r_{\text{PWA}}$ to $10^{-12}$** in all 452 such evaluations. There the ratio test *is* the exact answer.
2. **$r_H$ never exceeds the physical ratio and is within 0.032 of it everywhere** (mean gap at most 0.002). So the ratio test is not conservative in practice: almost all of what the rotors could deliver, the affine law asks for.
3. **When constraints are already active, $r_H$ over-states the exact transmission ratio** in 135 of 596 evaluations, sometimes by the whole range. In those cases the cheap test says "transmitted in full" while the MPC is in fact about to change its active set. **The ratio test is a certificate only when no constraint is active.**
4. **The correction is often not fully realizable** even on the moderate trajectory: $r_H < 1$ in half the evaluations. A 0.16 m altitude shift requested all at once, over the whole horizon, asks for a large immediate thrust change. This is why trust should rise gradually. The soft layer's rate limit on increases of α (proposal) is not a cosmetic choice: it keeps the requested input change small enough to be realizable.

**The rule for the hard layer (decision, [decisions.md](decisions.md)).** The realizability bound is $r_H$ when the current QP solution has **no active constraint**. The solver reports its active set, so this costs nothing to know. Otherwise the bound is $r_{\text{PWA}}$, the exact region-exit ratio of the current active set.

---

## 5. Cost

**On the PC (Python):** $r_H$ takes 0.22 ms per evaluation, $r_{\text{PWA}}$ 2.9 ms, and the LP 23 ms. $r_H$ is the cheapest by more than 5× and 50×.

**Operation count of $r_H$:**
- the box and rate rows cost $O(Nm)$: 640 multiply-accumulates;
- the tilt and altitude rows need the predicted states, $O(N^2 m)$: 9600 multiply-accumulates;
- at most 420 divisions.

**On board this is an estimate, not a measurement.** At about one multiply-accumulate per cycle at 168 MHz, it is of order 0.1 ms, against a 10 ms period. $r_{\text{PWA}}$ needs a solve with the current active set's KKT matrix. An active-set solver such as DAQP keeps a factorisation of exactly that matrix, so the cost should be of the order of one solver iteration; that is unmeasured.

**Normalisation.** $r_H$, $r_{\text{PWA}}$ and $r_{\text{LP}}$ are dimensionless fractions of the requested correction, so they transfer across vehicles without rescaling.

---

## What this entry commits us to, and what is open

**Committed.**
- **Definition.** $r_H$ is the ratio test over all 420 QP rows (box, rate, soft tilt, soft altitude) with the slacks at zero, along $\delta\mathbf U = (\partial\mathbf U^{\text{aff}}/\partial\Delta\mathbf R)\,\Delta\mathbf R$. That sensitivity is a constant matrix, because the MPC is linear and time-invariant.
- **Target Result 1, proved for the MPC we run.** If no row is active at the current solve and β ≤ $r_H$, the constrained QP's solution is the affine law with zero slacks: the correction is transmitted in full and the command is affine in β. The bound is sharp (checked at 1.05 $r_H$). The proposal's "to first order" caveat does not apply: the MPC is linear, and only the plant's response is not.
- **Validity region.** $r_H$ is the exit distance from the critical region with no active constraint. With constraints active, the exact transmission ratio is $r_{\text{PWA}}$ of the current active set, and $r_H$ can over-state it (135 of 596 evaluations in aggressive runs).
- **Error against the physical LP.** $r_H$ never exceeds $r_{\text{LP}}$ and is within 0.032 of it (mean at most 0.002).
- **Hard-layer rule.** Use $r_H$ when no constraint is active, otherwise $r_{\text{PWA}}$.

**Deferred, noted, not forgotten.**
- Measured on-board cost of $r_H$ and $r_{\text{PWA}}$ (hardware phase).
- The same measurement on the nonlinear plant, and with the oracle corrections of entry 08 instead of representative ones. Both are part of the oracle experiments of Phase 4.
- How the soft layer's rate limit on trust should be set from $r_H$: entry 12.

**Open for the next entry (10, the loop-gain bound).** Realizability guards against saturation; it says nothing about the second mechanism, the feedback loop that a state-dependent correction closes (entry 08). Entry 10 computes the closed-loop gain from a reference perturbation to the window the learned correction reads, *including attitude*, as entry 07 requires. It certifies a Lipschitz bound for the learned map, and checks that the resulting small-gain bound on α is not vacuous.

*This entry is locked. We do not proceed to 10 until the following can be explained in plain language: what the ratio test computes and why it must include the soft tilt and altitude rows; why the correction's input sensitivity is a constant matrix here; the proof of Target Result 1 and why it is exact for our MPC rather than first-order; what a critical region is and why $r_H$ is a distance to its boundary; why $r_H$ stops being a certificate once a constraint is active, and what replaces it; and why a correction requested all at once is less realizable than one introduced gradually.*
