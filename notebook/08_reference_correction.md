# 08 — The Reference Correction

*Derivation notebook, entry 08. Resolves the two open problems that every later entry depends on: what the "correct" reference correction is for each mismatch, including those with no steady-state effect (audit F-2), and what the reference supplied to the MPC contains (F-7). Implemented in [checks/ref08.py](checks/ref08.py); every number is computed in [checks/check_08_reference_correction.py](checks/check_08_reference_correction.py). The analysis uses the linear closed loop of entries 06–07 (implementation b, thrust estimate), and checks that no constraint becomes active along the test trajectories, so that the unconstrained law is the MPC.*

---

## Why this entry exists

The proposal maps a predicted mismatch to "the reference offset that cancels it at steady state". Entries 04 and 07 showed why that is not enough. Motor-lag error and delay have no steady-state effect, so such a map has nothing to cancel, and they are exactly the mismatches that bring the loop near its stability limits. Without a definition of the right correction, the oracle test of research question 1 is ill-posed: one cannot ask whether a perfect correction helps when "perfect" is undefined. This entry gives that definition and works out what it implies.

---

## 1. Taxonomy: feedforward and feedback corrections

The single most important distinction for this project is not *what* a correction is for, but *what it depends on*.

**A feedforward correction** $\Delta r_k$ depends only on time, the planned reference, and slowly varying parameters (a mass, a drag coefficient, a delay), but **not on the measured state**. It enters the closed loop as an exogenous input. The closed loop's dynamics, and therefore its eigenvalues, are unchanged. Formally, with closed-loop state $\mathbf z_k$,

$$\mathbf z_{k+1} = \mathbf A_{\text{cl}}\,\mathbf z_k + \mathbf B_r\,(\text{plan} + \Delta r)_k ,$$

and $\mathbf A_{\text{cl}}$ does not contain $\Delta r$. So a feedforward correction **cannot create or remove a stability cliff**. It changes tracking, nothing else. The check confirms this directly: the response to an initial perturbation is identical, to machine precision, with and without an arbitrary feedforward correction.

**A feedback correction** depends on the measured or estimated state. Take a linear one, $\Delta r_k = \mathbf K_c\,\mathbf x_{\text{rigid},k}$, applied as a constant shift over the horizon. The MPC's input then gains a term $\mathbf F_{\text{pos}}^{\Sigma}\mathbf K_c\,\mathbf x_k$, where $\mathbf F_{\text{pos}}^{\Sigma}$ is the gain of a horizon-constant position shift. The closed-loop matrix becomes $\mathbf A_{\text{cl}}(\mathbf K_c)$. **A feedback correction adds a loop, so it moves the cliffs, in either direction.** How much depends sharply on which state it feeds back. The delay cliff of the trajectory law is 24.6 ms; under three forms of feedback correction:

| Feedback correction $\Delta r = \mathbf K_c\mathbf x$ | Gain | Delay cliff |
|---|---|---|
| on measured velocity (a "lead" of $c$ periods) | $c = -2 \dots +4$ | 24.5–24.6 ms (moves < 0.1 ms) |
| on position | $-0.5 \dots +1$ | 24.4–24.9 ms (moves < 0.35 ms) |
| on **tilt** | −0.2 m/rad | **20.8 ms** (narrows 15 %) |
| on **tilt** | −0.1 m/rad | 22.6 ms |
| on **tilt** | +0.1 m/rad | 26.7 ms |
| on **tilt** | +0.2 m/rad | **29.0 ms** (widens 18 %) |

Velocity and position feedback act on the slow outer loop and hardly matter. **Tilt feedback acts on the attitude loop, where the gain is (entry 07), and moves the cliff by 15–18 percent.**

**The connection to the earlier study.** Its learned residual was fed the measured state, so it was a *feedback* correction. A feedback correction can narrow a cliff, which is the "hurts" mechanism. An accurate learned model does not protect against this, because accuracy is about the size of the correction and the cliff depends on its *state sensitivity*.

**What entry 10 must bound** is therefore the state sensitivity (Lipschitz constant) of the learned map. That must include its sensitivity to **tilt and attitude**, not only to position or velocity, measured against the tilt-inclusive closed-loop gain found in entry 07.

---

## 2. The reference content (resolves F-7)

**Decision: the MPC tracks a reference *trajectory* over its horizon.** The plan supplies, at every sample, the position and yaw and their analytic derivatives up to fifth order. The MPC builds its state and input references over the horizon by the linear, small-angle, differential flatness of the lagged hover model (entries 02–04):

| Reference | From |
|---|---|
| position, yaw | $\mathbf r$ |
| velocity, yaw rate | $\dot{\mathbf r}$ |
| roll $= -\ddot y/g$, pitch $= \ddot x/g$ | $\ddot{\mathbf r}$ |
| body rates $\omega_x = -\dddot y/g$, $\omega_y = \dddot x/g$ | $\dddot{\mathbf r}$ |
| delivered thrusts $\mathbf f = \mathbf M^{-1}(m\ddot z,\ -I_{xx}y^{(4)}/g,\ I_{yy}x^{(4)}/g,\ I_{zz}\ddot\psi)$ | $\mathbf r^{(4)}$ |
| commanded thrusts $\mathbf u_{\text{ref}} = \mathbf f + \tau_m\dot{\mathbf f}$ (inverts the motor lag) | $\mathbf r^{(5)}$ |

The cost becomes $\sum\|\mathbf x_k - \boldsymbol\xi_k\|_{\mathbf Q}^2 + \|\mathbf u_k - \mathbf u_{\text{ref},k}\|_{\mathbf R}^2 + \|\mathbf x_N - \boldsymbol\xi_N\|_{\mathbf P}^2$. The constraints, horizon, weights and implementation (b) are as in entry 06. The artificial steady state of MPC for tracking is dropped. Its purpose was feasibility under reference changes with a terminal set, and decision Q2 already makes feasibility trivial without one.

**Properties** (checked):
- The feedback part of the unconstrained law is exactly the LQR gain $\mathbf K$ (difference $10^{-13}$).
- The cliffs are those of entry 06 to within 0.03 ms and 0.005: delay 24.6 ms; lag 0.186 and 2.86.
- **The 1 m/s ramp is tracked with no steady lag** ($10^{-12}$ m), where the constant set-point of entry 06 lags 0.476 m.
- A figure-8 at 0.83 m/s and 19° of tilt is tracked to 0.0005 mm RMS on the nominal plant, against 232 mm with a constant set-point.

**The correction is never differentiated.** A first version derived the feedforward by finite differences of the reference samples, so a correction added to the reference was differentiated too. The fifth-order central difference weights its samples by up to $2.5/\Delta t^5 = 2.5\times10^{10}$, so any sample-to-sample roughness is amplified enormously. In that discarded version, commands reached about ±100 N against a box of ±0.05 N, and a step became a 2000° "tilt". These figures were observed in the discarded code and are not part of the standing checks; the amplification factor is. The rule is therefore:
- **feedforward is computed only from the plan's analytic derivatives;**
- **a correction $\Delta r$ shifts only the position and yaw entries of the reference over the horizon.**

This also matters for entry 09: a correction must not be able to reach the actuators through a differentiator.

**The constant set-point version** (entry 06's MPC for tracking) is kept as an **ablation**, and every baseline uses the same reference treatment as the method it is compared with.

---

## 3. The oracle (makes research question 1 well posed)

**Definition.** Over a window of the trajectory, the true closed loop, with the mismatch, maps a reference correction linearly to the output: $\mathbf y = \mathbf y_0 + \mathbf S\,\Delta\mathbf r$. Here $\mathbf y_0$ is the output without correction and $\mathbf S$ is built from the loop's impulse responses. The **oracle correction** is

$$\Delta\mathbf r^\star = \arg\min_{\Delta\mathbf r}\ \|\mathbf y_0 + \mathbf S\Delta\mathbf r - \mathbf r_{\text{des}}\|^2 + \lambda\|\Delta\mathbf r\|^2 = (\mathbf S^\top\mathbf S + \lambda\mathbf I)^{-1}\mathbf S^\top(\mathbf r_{\text{des}} - \mathbf y_0).$$

Corrections are allowed from the first full window to 0.5 s before the end, since a correction's effect after the window would go unscored.

**The choice $\lambda = 0.01$.** Both terms are in metres, so λ is dimensionless. In the frequency domain the solution inverts the closed loop's reference-to-output gain $T$ only where $|T|^2 \gtrsim \lambda$, i.e. where $|T| \ge 0.1$ (−20 dB). Above the loop's bandwidth, where $|T| \to 0$, the oracle does not try to invert a gain it cannot use. That would demand huge corrections for negligible benefit, and would leave the realizable region that entry 09 measures. A side effect, visible below: steady corrections come out scaled by $1/(1+\lambda) = 0.99$.

**(a) Mass and drag: the oracle is the steady-state offset of the proposal.**
- **Mass:** for a 20 percent mass error on a hover set-point, the oracle is a constant altitude shift of **104.03 mm**. That is exactly the steady offset (105.07 mm) divided by $1+\lambda$, to $10^{-5}$ mm.
- **Drag:** for 4× drag on the 1 m/s ramp, the oracle is a constant lead of 93.8 mm against a steady lag of 94.8 mm. That is again the offset divided by $1+\lambda$, to within 1.5 percent.

For mismatches with a steady-state effect, the proposal's map Γ is therefore the correct correction, and the oracle reduces to it.

**(b) Lag error and delay: zero on set-points, nonzero on trajectories (the resolution of F-2).** On a held set-point, the oracle for 20 ms of delay and for ×1.5 or ×0.5 lag error is **exactly zero**: these mismatches change nothing at rest, so there is nothing to correct. On the figure-8 it is not zero (up to 2.96 mm at 20 ms). **This is the resolution of F-2.** The correct correction for a zero-DC mismatch is not a steady offset. It is a time-varying correction along the trajectory, which exists only when the reference moves, and which the steady-state map of the proposal cannot represent.

**(c) Is the delay oracle a time advance of the reference by about $D$? Tested: no.**
- **Not an advance.** The best pure time advance $\mathbf r(t+\tau) - \mathbf r(t)$ fits it with $\tau = 1$ ms, not 20 ms, and leaves 85 percent of it unexplained.
- **Not a single derivative.** No single derivative of the plan explains it ($R^2 \le 0.56$). Jointly, the plan's first to fifth derivatives explain 95 percent ($R^2 = 0.948$).
- **Small, and linear in $D$.** Its peak is 1.45 mm at 10 ms and 2.96 mm at 20 ms, and it is **5.6 times smaller** than a naive advance $D\,v_{\max} = 16.6$ mm.

The reason: the flatness feedforward and the feedback loop already absorb most of a delay's effect on a smooth trajectory. A delay of 20 ms costs only 0.68 mm RMS on the figure-8. What remains is a small, filtered, multi-derivative correction, not a time shift. **Delay hurts this loop through its stability margin, not through its tracking error.** A learned delay correction therefore has little tracking to gain and loop-gain risk to carry, which is a direct input to the supervisor's trust policy.

**(d) A feedforward oracle cannot move a cliff.** By section 1, the oracle (a precomputed sequence) leaves the closed-loop eigenvalues unchanged:
- **Inside the admissible set** (20 ms of delay), a 1 mm perturbation added on top of the oracle-corrected run decays.
- **Beyond the cliff** (30 ms), it grows more than a hundredfold, whatever the correction.

So within Θ the oracle improves tracking, and beyond a cliff no feedforward correction can restore stability. There the nominal loop is itself unstable: Assumption 2 fails, and the supervisor has no claim to make. Its only meaningful action, zero trust, does not help either. This is precisely why Θ bounds the scope (entry 07).

**How much preview does the oracle need?** The full oracle is non-causal in both directions. About half its weight falls on *past* errors, which are measured anyway. On the future side, 95 percent of its weight lies within **1.1 s for delay** and 0.38–0.46 s for the other mismatches. That is far more than the MPC's own 0.2 s horizon window. Limiting the preview, with the same weights masked and not re-optimised, so this is a lower bound:

| Preview available | Delay 20 ms | Mass +20 % | Drag ×4 | Lag ×1.5 |
|---|---|---|---|---|
| none (past errors only) | **−88 %** (worse) | −63 % | −90 % | −83 % |
| 0.21 s (the horizon window) | 72 % | 80 % | 93 % | 87 % |
| 0.5 s | 95 % | 97 % | 98 % | 96 % |
| unlimited | 97 % | 99 % | 98 % | 97 % |

*(Fraction of the tracking error removed on the figure-8.)*

Two consequences follow.
- **A planned trajectory supplies the preview.** The whole plan is known in advance, and 0.5 s captures 95 percent or more. A learned approximator of the oracle must therefore receive at least about 0.5 s of plan preview as input.
- **A correction computed from past errors alone makes tracking worse in every case.** That is the feedback-correction danger of section 1, appearing in the oracle itself.

---

## 4. The learned correction's job

The learned residual of the proposal approximates the oracle. It predicts $\Delta\mathbf r^\star$ from three inputs: the plan preview (about 0.5 s or more), the measured state, and the recent inputs (which carry the actuator state the mismatch hides in). Two things follow, and both carry into entries 09 and 10.

1. **The oracle is feedforward; its approximator is not.** The oracle depends on the plan and on the mismatch parameters, not on the state. The approximator sees the state, so it becomes a *feedback* correction whether we like it or not. Its state sensitivity, above all to attitude (section 1), is what can narrow a cliff. **Entry 10 must bound that sensitivity**, and the hard layer must scale the correction down where the bound is violated.
2. **The oracle's size is the most any correction can usefully achieve.** For mass and drag it is large: tens to hundreds of millimetres removed. For lag error and delay it is small: about a millimetre. **Entry 09** asks whether a requested correction can be delivered by the rotors within the horizon (the realizability ratio). Its inputs are the oracle's $\Delta\mathbf r$ and the MPC's sensitivity $\mathbf F_{\text{pos}}$ of the input to a position shift over the horizon.

---

## 5. Tests over the admissible set

**Test trajectories:**
- a 0.1 m step in x (a set-point, with no plan derivatives);
- a 1 m/s ramp, entered smoothly over 1 s;
- a figure-8, $x = 0.28\sin(\omega t)$, $y = 0.14\sin(2\omega t)$, with a period of 3 s, faded in over 1.5 s. Its peak speed is 0.83 m/s and its peak acceleration 3.4 m/s², which is 19° of tilt, inside the 20° soft limit and far inside the authority limits of entry 04.

Along all three, nominally and at mass +30 %, delay 20 ms and lag ×2, the commanded thrusts stay inside the box and rate limits and the tilt inside its soft limit. So the unconstrained (linear) analysis *is* the MPC on these tests.

**The oracle's tracking gain over the nominal MPC, figure-8, every single-mismatch point inside Θ** (RMS error over the interior, excluding the first 1.5 s and the last 1 s):

| Mismatch | Nominal MPC | With oracle |
|---|---|---|
| mass 0.8 / 0.9 / 1.1 / 1.2 / 1.3 | 105 / 53 / 53 / 105 / 158 mm | 1.0 / 0.5 / 0.5 / 1.0 / 1.5 mm |
| drag ×1 / 2 / 4 / 8 / 16 | 11 / 21 / 39 / 66 / 104 mm | 0.2 / 0.4 / 0.9 / 2.1 / 5.3 mm |
| lag ×0.3 / 0.5 / 0.7 / 1.3 / 1.5 / 2.0 | 1.5 / 1.1 / 0.6 / 0.6 / 1.1 / 2.1 mm | 0.05 / 0.03 / 0.02 / 0.02 / 0.03 / 0.07 mm |
| delay 5 / 10 / 15 / 20 ms | 0.17 / 0.34 / 0.51 / 0.68 mm | 0.006 / 0.012 / 0.017 / 0.023 mm |

The oracle improves tracking at every one of the 20 points, removing more than 90 percent of the error at each (at least 95 percent at all but drag ×16). Note the scale, though. Mass and drag errors are tens to hundreds of millimetres. **Lag-error and delay errors are around a millimetre, so in absolute terms there is little to correct.** This sharpens research question 1: for lag error and delay, a learned correction can at best gain a millimetre, while its feedback character can cost a cliff.

---

## What this entry commits us to, and what is open

**Committed.**
- **Taxonomy.**
  - Feedforward corrections (plan and parameter dependent) leave the closed-loop eigenvalues unchanged: they cannot move a cliff.
  - Feedback corrections (state dependent) add a loop and move cliffs both ways. Tilt feedback moves the delay cliff by 15–18 percent at ±0.2 m/rad; velocity and position feedback hardly at all.
- **Reference content (F-7).**
  - The MPC tracks a planned trajectory over its horizon, with full linear-flatness feedforward computed from the plan's analytic derivatives up to fifth order, including the motor-lag inversion.
  - There is no artificial steady state.
  - A correction shifts only position and yaw over the horizon and is never differentiated.
  - The constant set-point MPC for tracking is kept as an ablation, and baselines share the reference treatment.
- **The oracle (F-2).**
  - Definition: regularised least squares over the trajectory window, $\lambda = 0.01$ (inverting the loop only where its gain exceeds −20 dB).
  - Mass and drag: it reduces to the steady-state offset (exactly, up to $1/(1+\lambda)$).
  - Lag error and delay: it is zero on set-points and nonzero on trajectories.
  - For delay it is not a time advance but a small, multi-derivative correction, linear in $D$.
  - It is feedforward, so it cannot move a cliff.
  - Its useful preview is about 0.5 s of plan. Without preview, a correction makes tracking worse.
- **The trajectory law's cliffs** equal entry 06's (implementation b) to within 0.03 ms. They are registered in [decisions.md](decisions.md).

**Deferred, noted, not forgotten.**
- The oracle on the nonlinear plant with the constrained QP, and on aggressive trajectories that activate constraints. That is where realizability bites: entry 09.
- The oracle for combined mismatches and outside Θ, which serve as stress tests.
- A re-optimised (rather than masked) oracle for each preview length, if the learned approximator's preview must be chosen precisely.

**Open for the next entry (09, the realizability ratio).** The oracle tells us what correction *should* be applied. Entry 09 asks what fraction of it the rotors can *deliver*, and under which conditions that fraction can be computed in closed form from the MPC's predicted inputs. It covers the validity region (unchanged active set), the piecewise-affine structure of explicit MPC, a proof of Target Result 1, and the error of the closed-form ratio against the exact linear program. The sensitivity it needs is $\mathbf F_{\text{pos}}$ of this entry.

*This entry is locked. We do not proceed to 09 until the following can be explained in plain language: why a feedforward correction cannot move a stability cliff and a feedback correction can; why tilt is the state whose feedback matters; why the MPC now tracks a trajectory with flatness feedforward, and why a correction must never be differentiated; what the oracle is, why λ = 0.01, and why it reduces to the steady offset for mass and drag but is zero on set-points for lag and delay; why the delay oracle is not a time advance; why a correction computed from past errors alone makes tracking worse; and why the learned approximator is a feedback correction even though the oracle it imitates is not.*
