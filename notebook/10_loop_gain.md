# 10 — The Loop-Gain Bound

*Derivation notebook, entry 10. Computes the closed-loop gain from a reference correction to the signal the learned map reads, over the admissible mismatch set, with a certified H∞ norm. Certifies a Lipschitz bound for the learned map. Then checks, on a realistic estimator, whether the resulting small-gain bound on trust is vacuous. It is, in its generic form; a structured form that is not vacuous is derived and committed. Implemented in [checks/loop10.py](checks/loop10.py); every number is computed in [checks/check_10_loop_gain.py](checks/check_10_loop_gain.py). The loop is the trajectory MPC of entry 08 in implementation (b), linearised at hover.*

---

## Why this entry exists

Entry 08 showed that a correction computed from the current state closes a feedback loop around the MPC. Entry 09 bounded saturation, which is the first way a correction can do harm; this entry bounds the second, the loop. The proposal's claim was a small-gain condition:

$$\alpha\,L_\Delta\,\gamma < 1 ,$$

where $\alpha$ is trust, $L_\Delta$ is the Lipschitz constant of the learned map, and $\gamma$ is the gain of the closed loop from a correction back to what the map reads. Three questions have to be settled before this can be used:
1. What exactly is $\gamma$, in which norm, and over which mismatch set? (Entry 07 requires it to include tilt; audit F-3 requires the gain to the whole window the map reads.)
2. How is $L_\Delta$ certified rather than estimated?
3. Is the bound useful, or does it forbid trust that is in fact safe? The original specification demands this check.

---

## 1. The loop, and its scaling

**Signals.** The correction $\boldsymbol\delta_k \in \mathbb R^4$ shifts position and yaw over the whole horizon (entry 08). The learned map reads $\mathbf y_k = (\mathbf x_{\text{rigid},k}, \mathbf u_{k-1})$: the 12 rigid-body states (position, velocity, tilt, yaw, rates) and the command in flight. Every signal is divided by its acceptable deviation from entry 06 (Bryson scaling), and the correction by $S_\delta = (0.1, 0.1, 0.1\ \text{m}, 0.5\ \text{rad})$. Gains and Lipschitz constants are then dimensionless and measured in the same Euclidean norm, which is what makes their product meaningful.

**The loop.** With mismatch $\theta$ (mass, drag, lag, delay), the closed loop of entry 08 is an LTI system $G_\theta : \boldsymbol\delta \mapsto \mathbf y$. The correction enters through the MPC's affine dependence on the reference (the constant matrix FPOS_SUM, entry 08) into the command, and is read back through the state. No quantity is linearised beyond what entry 08 already did.

**The window (F-3).** A map that reads $L+1$ samples $(\mathbf y_k, \dots, \mathbf y_{k-L})$ sees $[G; z^{-1}G; \dots; z^{-L}G]$. A delay does not change a magnitude at any frequency, so this stacked operator has norm exactly $\sqrt{L+1}\,\|G\|_\infty$ (checked to $10^{-4}$ for $L = 2$). The window costs $\sqrt{L+1}$ in the generic bound. This matters in Section 4.

---

## 2. A certified H∞ norm

A frequency sweep gives only a lower bound on $\|G\|_\infty$, so it cannot be used to certify. We use the bisection method of Boyd, Balakrishnan and Kabamba (1989) [59]. The discrete system is mapped to continuous time by the bilinear transform $z = (1+s)/(1-s)$, which maps the unit circle onto the imaginary axis and so preserves the H∞ norm. Then $\gamma > \|G\|_\infty$ holds exactly when a Hamiltonian matrix $H(\gamma)$ has no imaginary eigenvalue. Bisection on $\gamma$ returns an upper bound to relative tolerance $10^{-6}$. The eigenvalue test uses a numerical threshold. That threshold is the one place where "certified" means "certified up to floating point", and the sweep cross-checks it: the sweep lies below the certified value, within $10^{-5}$ relative.

**Sanity checks against earlier entries.**
- **Nominal gain to the full read signal: 1.000.** At low frequency a position shift of 0.1 m produces a position change of 0.1 m, which is one deviation unit. The loop adds no amplification.
- **To tilt alone: 0.504 scaled, that is 1.76 rad per metre.** Entry 07 found 1.75. This confirms that tilt is in the read signal and correctly scaled.

## 3. The gain over the admissible set

| mismatch | certified $\|G_\theta\|_\infty$ |
|---|---|
| mass ×0.8 / 0.9 / 1.1 / 1.2 / 1.3 | 1.097 / 1.043 / 1.032 / 1.062 / 1.092 |
| drag ×1 / 2 / 4 / 8 / 16 | 1.001 / 1.002 / 1.005 / 1.009 / 1.015 |
| lag ×0.3 / 0.5 / 0.7 / 1.3 / 1.5 / 2.0 | 4.905 / 2.016 / 1.377 / 1.272 / 1.517 / 2.622 |
| delay 5 / 10 / 15 / 20 ms | 1.298 / 1.810 / 2.874 / 6.284 |
| **worst of 32 stable combinations** (drag ×4, lag ×1.5, delay 20 ms) | **13.41** |

The pattern is the one entries 06 and 07 predict. Mass and drag barely change the loop, because the MPC's feedback dominates them. Lag error and delay push the gain up as they approach their cliffs: the loop is close to instability there, so a reference perturbation rings. **We take $\bar\gamma = 13.41$ as the supremum over the admissible set.** This is a grid supremum, not a proven bound over the continuum. The gain is a continuous function of $\theta$ inside the stable set, but it rises steeply near the cliffs, so a grid point is only as good as the margin of Θ to the cliffs (entry 07).

**Is the small-gain bound valid, and how loose is it?** For linear corrections $\boldsymbol\delta = \alpha K \mathbf y$ with $\|K\|_2 = 1$, the exact stability boundary $\alpha^\star$ is computed from the eigenvalues and compared with $1/\gamma$:

| set point | $1/\gamma$ | worst rank-one $K$ | tilt feedback | 30 random $K$: min / median |
|---|---|---|---|---|
| nominal | 1.000 | 1.131 | 1.986 | 1.587 / 2.206 |
| delay 20 ms | 0.159 | 0.465 | 0.859 | 0.691 / 1.390 |
| worst combination | 0.075 | 0.143 | 0.318 | 0.202 / 0.682 |

Every exact boundary lies above $1/\gamma$, so the bound is valid. Against the worst direction, a rank-one $K$ aligned with the peak singular vectors, it is loose by a factor of only 1.13 to 2.9. **The H∞ part of the bound is therefore not the problem.**

---

## 4. A certified Lipschitz bound for the learned map

**Product of spectral norms.** For a network $W_n\sigma(\dots\sigma(W_1 x))$ with 1-Lipschitz activations (tanh, ReLU), the constant is at most $\prod_i \|W_i\|_2$. This is certified, cheap, and known to be loose. On three random 176-64-64-4 tanh networks the bound exceeds the largest sampled Jacobian norm by 2.5–3.1×.

**Spectral normalisation** (Miyato et al., 2018 [48]) divides each layer by its spectral norm during training and scales it to a chosen value. The certified constant is then *set* rather than measured; layers scaled to $0.5^{1/3}$ give exactly 0.5. This is the committed route, because the hard layer needs a number that is known before deployment.

**LipSDP** (Fazlyab et al., 2019 [49]) is tighter, but it needs an SDP solver. That is deferred: the product bound with spectral normalisation is sufficient for the argument below, and Section 5 shows the looseness that matters is elsewhere.

**Ensemble and saturation.** The deployed correction is the ensemble mean, saturated onto a ball (entry 08). The mean of maps with constants $L_i$ has constant at most $\bar L = \frac1M\sum L_i$, and projection onto a convex set is 1-Lipschitz. So the deployed map's constant is at most the members' mean (checked on 2000 random pairs).

---

## 5. Is the bound vacuous? A realistic estimator

To answer this honestly, the learned map is replaced by a map whose job it would actually do: a **mass (vertical-bias) estimator**. It averages over a window of $L$ samples:

$$\hat d_k = \frac{v_{z,k} - v_{z,k-L}}{L\,\Delta t} - \frac{1}{L}\sum_{i=1}^{L}\frac{\sum_j \hat f_{j,k-i}}{m}\,(\text{about hover}),\qquad \delta z_k = \kappa\,\hat d_k .$$

The first term is the measured vertical acceleration; the second is the acceleration the thrust estimates predict. $\kappa = 0.0643$ m per m/s² is the steady altitude correction per unit of acceleration bias, computed from the oracle of entry 08. This map is linear, so its Lipschitz constant is exactly known, and so is its exact stability boundary: the loop augmented with the delay lines is an LTI system whose eigenvalues can be computed.

**Three bounds on the trust $\alpha$, with $\varepsilon = 0.1$ margin:**
- **generic:** $\alpha_{\text{sg}} = (1-\varepsilon)/(L_\Delta\,\gamma_{\text{read}})$, where $\gamma_{\text{read}}$ is the certified gain to exactly the samples the estimator reads (the $\sqrt{\cdot}$ window factor included);
- **structured:** $\alpha' = (1-\varepsilon)/\|E\,G\|_\infty$, where $E$ is the estimator as an LTI filter and $\|E G\|_\infty$ is the certified gain of the composed loop;
- **exact:** the first $\alpha$ at which the augmented loop has an eigenvalue on the unit circle.

| window | $L_\Delta$ | generic (nominal / worst) | structured (nominal / worst) | exact (nominal / worst) |
|---|---|---|---|---|
| 0.1 s | 4.595 | 0.083 / 0.065 | 8.01 / 1.66 | 9.00 / 4.83 |
| 0.3 s | 1.565 | 0.145 / 0.114 | 17.0 / 2.62 | 19.0 / 13.6 |
| 1.0 s | 0.503 | 0.249 / 0.196 | 57.6 / 8.73 | none below 50 / 43.8 |

**The generic bound is nearly vacuous.** It allows at most a quarter of full trust, while full trust is safe with a wide margin. The bound is 70–200× conservative, and it gets worse as the window grows, because the window factor $\sqrt{L}$ grows while the true loop gain falls. The cause is not the H∞ norm (Section 3 showed that part is tight) and not the network bound (Section 4). It is the **product itself**. $L_\Delta$ is the worst-case gain over all input directions and all frequencies; $\gamma$ is the worst case over all output directions and frequencies. An averaging estimator has its gain at low frequency in a direction the loop barely excites there, and the product multiplies two worst cases that never occur together.

**The structured bound is not vacuous.** It allows full trust ($\alpha' \ge 1.66$) at every set point and window, it is always below the exact boundary (so valid), and nominally it is only 11 % (0.1 s) and 10 % (0.3 s) below the exact value.

**Consequence for the architecture (decision).** The learned map is split into two parts:
1. **a fixed LTI feature filter $E$** (averaging, differencing over the window), certified jointly with the loop as $\|E\,G_\theta\|_\infty$ over Θ;
2. **a low-dimensional network $N$** on the filter's outputs, with a certified, spectrally normalised constant $L_N$.

The bound becomes $\alpha\,L_N\,\sup_\theta\|E G_\theta\|_\infty < 1$. Only the network's nonlinearity is bounded crudely, and it acts on a few features instead of a 176-dimensional window.

**Consequence for the window.** The estimator needs $v_{z,k-L}$, a *past state*. The proposal's window $(\mathbf x_k, \mathbf u_{k-L..k-1})$ contains only the current state and past inputs, so it cannot form a residual (measured minus predicted acceleration) at all. **The window must include past states.**

---

## 6. Gain form, and a state-dependent trust (F-3, F-4)

Write the deployed correction as

$$\Delta\mathbf r_k = \alpha_k N(\mathbf w_{\text{nom}}) + \alpha_k\big(N(\mathbf w_k) - N(\mathbf w_{\text{nom}})\big),$$

where $\mathbf w_{\text{nom}}$ is the window along the nominal trajectory.
- The first term does not depend on the loop's response. It is a **bounded exogenous input**, the bias the correction is meant to inject, bounded by the saturation radius.
- The second term is bounded pointwise by $\bar\alpha L_N\|E\mathbf e\|$, with $\mathbf e$ the deviation from nominal and $\bar\alpha$ the largest trust the soft layer can command.

A pointwise sector bound is all the small-gain theorem needs. So **$\alpha_k$ may vary with time and state without any Lipschitz condition on $\alpha$ itself** (audit F-4). The closed-loop gain from the exogenous bias to the deviation is then at most

$$\frac{\gamma}{1 - \gamma\,\bar\alpha\,L_N}$$

in ISS gain form (audit F-3). This is the quantity the hard layer certifies. **It is a linearised-loop statement:** the nonlinear plant's departure from $G_\theta$ is measured with the oracle in Phase 4 and enters the theorems of entry 14 as a stated assumption.

---

## What this entry commits us to, and what is open

**Committed.**
- **Loop and norm.** $G_\theta$ maps the scaled correction to the scaled read signal (12 rigid states including tilt, plus the command in flight). Its norm is the certified H∞ norm by Hamiltonian bisection [59], cross-checked by a sweep. A window of $L+1$ samples costs exactly $\sqrt{L+1}$.
- **Gain over Θ.** Nominal 1.000; mass and drag at most 1.10; lag and delay up to 4.90 and 6.28 singly. The supremum over the 32-point combination grid is $\bar\gamma = 13.41$ (drag ×4, lag ×1.5, delay 20 ms).
- **Small-gain validity.** For linear corrections it is always valid, and only 1.13–2.9× conservative in the worst direction.
- **Network constant.** Product of spectral norms with spectral normalisation; the ensemble mean and the saturation do not increase it.
- **The generic bound is nearly vacuous** for a realistic estimator ($\alpha \le 0.25$ against a true boundary of 4.8 or more). The **structured bound** $\alpha L_N \sup_\theta\|E G_\theta\|_\infty < 1$ is valid and allows full trust everywhere tested. The learned map is therefore a fixed LTI feature filter followed by a low-dimensional, spectrally normalised network.
- **The window includes past states**, not only past inputs.
- **Gain form:** the bias is an exogenous input, trust is handled as a sector bound with $\bar\alpha$, and there is no Lipschitz condition on $\alpha$.

**Deferred, noted, not forgotten.**
- LipSDP for $L_N$ (needs an SDP solver; only worth it if $L_N$ turns out to be the binding factor).
- A proof that $\bar\gamma$ over the continuum of Θ is not much larger than the grid supremum. The plan: a finer grid near the worst corner, plus continuity, recorded with entry 07's margins.
- The choice of filter $E$ (which averages, which differences, which window lengths) belongs to the residual-model design of Phase 3. This entry fixes only its form and how it is certified.

**Open.**
- How much of the nonlinear plant's behaviour the linear bound covers: measured with the oracle in Phase 4, stated as an assumption in entry 14.
- How $\bar\alpha$ and the rate limit on trust (entry 09) are combined into one soft-layer rule: entry 12.

*This entry is locked. We do not proceed to 11 (the barrier bound) until the following can be explained in plain language: what $G_\theta$ maps from and to, and why every signal is scaled; why a sweep cannot certify an H∞ norm and what the Hamiltonian test does instead; why a window of $L+1$ samples costs exactly $\sqrt{L+1}$; why lag and delay raise the gain and mass and drag do not; why the generic product bound is nearly vacuous even though each factor is tight; what the structured bound computes and why it is valid; why the window must include past states; and why a pointwise bound on trust is enough and no Lipschitz condition on α is needed.*
