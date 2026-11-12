# 01: Forces and Torques

*Derivation notebook, entry 01. Continues the clean-room rebuild from entry 00. Written as we reason through the physics and decide what enters our model, not as a textbook.*

---

## What this entry is for

Entry 00 told us what numbers describe the drone at an instant. This entry tells us what changes those numbers. Everything that moves the drone is either a force, which changes how it translates (position and velocity), or a torque, which changes how it rotates (orientation and angular velocity). So our job here is to list every force and every torque acting on the drone, decide which of them belong in our nominal model and which we deliberately leave out, and work out how the one thing we can actually command, the four rotors, produces those forces and torques. Once we have that, entry 02 can write the equations of motion.

This entry also defines the control input and its limits. Those limits are what the realizability ratio in the research proposal is computed against, so they are derived here with the same care as the forces.

We inherit entry 00's convention without restating it in full: right-handed z-up world with gravity along negative world z, right-handed FLU body (x forward, y left, z up) with thrust along positive body z, and the right-hand rule for rotations, so that positive roll lifts the left side and **positive pitch is nose down**.

---

## The forces (what makes the drone translate)

We began by listing every force we could think of acting on a quadrotor in flight, then sorted them into what belongs in the nominal model and what does not.

**Gravity.** Always points along negative world z, and it is naturally a world-frame quantity because "down" is defined relative to the ground, not the drone. It is constant. It belongs in the nominal model:

$$\mathbf{F}_g = -mg\,\mathbf{e}_3 .$$

**Thrust.** The four propellers push the drone along its own body z direction, so total thrust always points along positive body z. This is a body-frame quantity, and it is the single most important consequence of orientation: when the drone tilts, its thrust tilts with it, so a tilted thrust gains a horizontal component. This is the entire reason a quadrotor can move sideways at all, since it has no way to push sideways directly. Thrust belongs in the nominal model. Written in the world frame it is

$$\mathbf{F}_T = T\,\mathbf{b}_3 = T\,R(\mathbf{q})\,\mathbf{e}_3 ,$$

where $T \ge 0$ is the total thrust magnitude and $\mathbf{b}_3$ is the body z axis expressed in the world frame.

**Rotor drag.** We call this one out by name, for two reasons. First, it is the dominant modellable aerodynamic effect on a small quadrotor in forward flight, and it is the standard target of the learned-residual quadrotor literature, so any reader of this work will expect it. Second, its structure matters to our thesis. When a spinning rotor moves sideways through the air, the advancing and retreating blades see different airspeeds, and the result is a force that opposes the motion and is approximately **linear** in the velocity. The standard model is

$$\mathbf{F}_{\text{drag}} \approx -R\,\mathbf{D}\,R^{\top}\mathbf{v}, \qquad \mathbf{D} = \operatorname{diag}(d_x, d_y, d_z),$$

with $R = R(\mathbf{q})$. Reading it from right to left: $R^{\top}\mathbf{v}$ is the velocity expressed in the body frame, $\mathbf{D}$ scales each body-axis component, and $R$ rotates the resulting force back into the world frame. This is the body-frame computation that entry 00 warned would be the cost of keeping velocity in the world frame.

| Symbol | Meaning | Units |
|---|---|---|
| $\mathbf{F}_{\text{drag}}$ | Rotor-drag force, expressed in the world frame | N |
| $R$ | Rotation matrix $R(\mathbf{q})$, body to world | none |
| $R^{\top}$ | Its transpose, world to body | none |
| $\mathbf{v}$ | Linear velocity, world frame | m/s |
| $\mathbf{D}$ | Diagonal matrix of drag coefficients along the body axes | N·s/m |
| $d_x, d_y, d_z$ | Drag coefficients along body x, y and z | N·s/m |

Four qualifications keep this model honest.

- **It is a near-hover approximation.** The physical force scales with rotor speed as well as with velocity. Förster's identification of the Crazyflie 2.0 models it as $\mathbf{K}_{\text{aero}}\,(\sum_i \Omega_i)\,\mathbf{v}$ in the body frame, with $\mathbf{K}_{\text{aero}} = -\operatorname{diag}(9.18, 9.18, 10.3)\times 10^{-7}$ kg/rad. Near hover the rotor speeds are roughly constant, so the coefficients can be treated as constants: $d_{x,y} \approx 7\times 10^{-3}$ N·s/m at the hover rotor speed (*derived* from Förster's maps, order of magnitude only, [checks/params.py](checks/params.py)). In aggressive flight they vary with thrust.
- **The velocity is the velocity relative to the air.** We assume still air, so this equals $\mathbf{v}$. Wind is outside the study.
- **The in-plane coefficients are typically the larger ones.** $d_x$ and $d_y$ describe motion in the plane of the rotors, where the effect arises, and $d_z$ is typically smaller. The values are vehicle-specific and must be identified, not assumed.
- **It also produces a torque.** The force acts at the rotor hubs, which do not lie exactly at the height of the centre of mass. We neglect this torque initially and record it as an excluded effect.

Rotor drag does **not** enter the nominal model. It is one of the four mismatches we study. Two structural facts about it are worth recording now. It acts in the translational channel, which entry 02 will show is several integrations away from the rotor torques. And it depends on the state, through both $\mathbf{v}$ and $\mathbf{q}$, so a learned correction for it also depends on the state. That second fact is what makes the loop gain of the correction path, and therefore the small-gain bound of the proposal, a real concern and not a formality.

**Body drag.** Air resistance on the airframe itself opposes motion and grows with the square of the speed. At the speeds a vehicle of this size reaches, it is smaller than rotor drag. **We make the decision here:** the "drag" mismatch in this project is the linear rotor-drag model above. Quadratic body drag is excluded from both the nominal model and the mismatch set, and is left to the residual if a higher-fidelity simulator introduces it.

**Higher-order aerodynamic effects (blade flapping, ground effect, variation of thrust with inflow, interaction between rotors).** These are real and we acknowledge them, but we deliberately leave them out of the nominal model. They are small in our operating range, difficult to model precisely, and exactly the kind of unmodelled mismatch a learned residual exists to capture. We note them here so they are on record.

So the nominal force model reduces to two forces: **gravity along negative world z, and total thrust along positive body z.** Rotor drag is added as a mismatch. Everything else is left in the gap that the residual learns.

---

## The torques (what makes the drone rotate)

A quadrotor has no flaps, rudder, or moving control surfaces. It has only four spinning propellers. So the interesting question is how four propellers, purely by spinning at different speeds, make the drone roll, pitch, and yaw. Roll and pitch use one mechanism and yaw uses a completely different one, and the distinction explains a lot about how a quadrotor behaves.

**Roll and pitch come from thrust differences across the frame.** If the rotors on one side of the drone push harder than the rotors on the other side, that side lifts more and the drone tips. So roll and pitch are produced by an imbalance in thrust between opposite sides, acting through the physical arm of the frame. Formally each rotor contributes a torque $\mathbf{r}_i \times f_i\,\mathbf{e}_3$, where $\mathbf{r}_i$ is its position in the body frame; the cross product is what turns "a push at a distance" into "a rotation."

**Yaw comes from a reaction torque, not from any sideways push.** Every spinning propeller drags the surrounding air, and by Newton's third law the air drags back, producing a reaction torque on the airframe in the sense **opposite** to the propeller's spin. If all four propellers spun the same direction, the drone would spin uncontrollably, so the standard layout spins two propellers clockwise and two counter-clockwise. At equal speeds their reaction torques cancel and there is no net yaw. To yaw on purpose, we break that balance: speeding up one pair while slowing the other leaves a net reaction torque, and total thrust is unchanged.

These are two different mechanisms. Roll and pitch are thrust differences acting through a lever. Yaw is a difference in aerodynamic reaction torques. How their strengths compare is a property of the vehicle, and we quantify it below.

All torques in this entry are expressed in the **body frame**, which is the frame in which entry 03 writes the rotational dynamics.

---

## The rotor model (what one rotor actually produces)

Before we can write the map from four rotors to a wrench, we need what a single rotor produces. Let $\Omega_i \ge 0$ be the angular speed of rotor $i$. Two standard relations, both quadratic in speed because both are aerodynamic:

$$f_i = k_f\,\Omega_i^2, \qquad \tau_{\text{react},i} = k_m\,\Omega_i^2 = \frac{k_m}{k_f}\,f_i = c_{\tau f}\,f_i .$$

The first is the thrust the rotor produces along body z. The second is the magnitude of the reaction torque it exerts on the airframe about body z, opposite in sense to its own spin. The important consequence is the last equality: **the reaction torque is proportional to the thrust**, with a single constant $c_{\tau f} = k_m / k_f$ that has units of length. A length is what makes it directly comparable to the moment arm that drives roll and pitch.

| Symbol | Meaning | Units |
|---|---|---|
| $\Omega_i$ | Angular speed of rotor $i$ | rad/s |
| $f_i$ | Thrust of rotor $i$, along body z | N |
| $\tau_{\text{react},i}$ | Magnitude of the reaction torque on the body from rotor $i$ | N·m |
| $k_f$ | Thrust coefficient | N·s² |
| $k_m$ | Reaction-torque coefficient | N·m·s² |
| $c_{\tau f}$ | Torque-to-thrust ratio $k_m / k_f$ | m |

Both relations describe a rotor in still air at steady speed. The variation of thrust with the vehicle's own motion is one of the higher-order effects excluded above.

**The control input.** We take the four rotor thrusts as the control input throughout:

$$\mathbf{u} = (f_1, f_2, f_3, f_4) .$$

The map between $f_i$ and $\Omega_i$ is a fixed monotone square root, so carrying $\Omega_i$ adds nothing at this stage.

**Motor lag in the nominal model (decision Q1).** The rotors do not deliver a commanded thrust instantly. Entry 00 records why the nominal model includes this lag on the Crazyflie: the motor is slower than the attitude dynamics. The input of the MPC is therefore the vector of **commanded** thrusts, $\mathbf{u} = \mathbf{f}^{\text{cmd}}$. The **delivered** thrusts $\mathbf{f}$ are states with a first-order lag on thrust:

$$\tau_m\,\dot f_i = f_i^{\text{cmd}} - f_i, \qquad i = 1,\dots,4 .$$

The lag acts on **thrust**, not on rotor speed, for two reasons. First, Förster identified the vehicle's lag exactly in this form, from motor command to measured thrust. His transfer function $H(z) = 7.23\times10^{-8}/(1 - 0.96954\,z^{-1})$ at 500 Hz gives $\tau_m \approx 65$ ms (*derived*; his second data series gives 77 ms). Second, a lag on thrust keeps the nominal model linear in its input, which entry 06 needs. A lag on rotor speed, $\tau_m\dot\Omega_i = \Omega_i^{\text{cmd}} - \Omega_i$ with $f_i = k_f\Omega_i^2$, has the *same* time constant in the linearization about any operating point (because $\delta f_i = 2k_f\Omega_i\,\delta\Omega_i$). It differs only for large steps. RotorPy uses the rotor-speed form. Entry 07 decides which form the plant uses and treats the difference as part of the perturbation model.

---

## The airframe geometry (committing to X, not plus)

The rotor-to-wrench map is not universal; it depends on where the rotors sit. We commit here to the **X configuration**, in which the arms lie at 45 degrees to the body x and y axes, and not the plus configuration, in which each arm lies along an axis. Two reasons: X is what the Crazyflie and essentially all current quadrotors fly, and X gives all four rotors a lever on both roll and pitch.

With arm length $l$ measured from the centre of mass to each rotor, define

$$a = \frac{l}{\sqrt{2}},$$

which is the effective moment arm of each rotor about the body x and y axes. Note that $a < l$. Using $l$ where $a$ belongs overstates roll and pitch authority by 41 percent, which would silently corrupt every realizability computation downstream.

### Rotor positions and spin directions

With $+x$ forward and $+y$ to the left, the four rotors are numbered counter-clockwise seen from above, starting at the front left. Adjacent rotors counter-rotate.

| Rotor | Position | $\mathbf{r}_i$ (body frame) | Spin seen from above | Reaction torque on body | $s_i$ |
|-------|----------|----------------------|----------------------|-------------------------|-------|
| 1 | front-left | $(+a, +a, 0)$ | clockwise | $+z$ | $+1$ |
| 2 | back-left | $(-a, +a, 0)$ | counter-clockwise | $-z$ | $-1$ |
| 3 | back-right | $(-a, -a, 0)$ | clockwise | $+z$ | $+1$ |
| 4 | front-right | $(+a, -a, 0)$ | counter-clockwise | $-z$ | $-1$ |

Here $s_i$ is the sign of the reaction torque about body z. A rotor that spins clockwise seen from above spins in the negative sense about body z, so its reaction on the airframe is positive. Diagonal pairs (1, 3) and (2, 4) share a spin direction, which is what makes the reaction torques cancel at equal thrusts.

**Correspondence with the Crazyflie (verified).** The Crazyflie's own motor numbering is different from ours. Its firmware mixer (`power_distribution_quadrotor.c`) places M1 at front-right, M2 at rear-right, M3 at rear-left and M4 at front-left, numbering clockwise seen from above. So M1, M2, M3, M4 are our rotors 4, 3, 2, 1. The Bitcraze propeller documentation mounts the counter-clockwise propellers on M1 and M3 and the clockwise ones on M2 and M4. Our rotor 1 (= M4) and rotor 3 (= M2) are therefore clockwise, as in the table. Phase 2 still tests the mapping in simulation, because a permuted motor order produces a simulator that hovers correctly and turns the wrong way.

### From rotor position to roll and pitch torque

Each rotor's thrust acts along body $+z$:

$$\mathbf{f}_i = f_i\,\mathbf{e}_3 = \begin{bmatrix} 0 \\ 0 \\ f_i \end{bmatrix}.$$

The torque it produces about the centre of mass is $\boldsymbol{\tau}_i = \mathbf{r}_i \times \mathbf{f}_i$. For a rotor at $\mathbf{r}_i = (x_i, y_i, 0)$:

$$
\boldsymbol{\tau}_i =
\begin{bmatrix} x_i \\ y_i \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ f_i \end{bmatrix}
= \begin{bmatrix} y_i f_i \\ -x_i f_i \\ 0 \end{bmatrix},
\qquad\text{so}\qquad
\boxed{\tau_{x,i} = y_i f_i}, \quad \boxed{\tau_{y,i} = -x_i f_i}.
$$

The rotor's $y$ position sets its contribution to **roll** torque, and its $x$ position sets its contribution to **pitch** torque, with a minus sign. Because every rotor in the X configuration has $x_i, y_i = \pm a$, every rotor contributes to both axes.

As a check on the signs, take rotor 1 at $(+a, +a, 0)$:

$$\boldsymbol{\tau}_1 = \begin{bmatrix} +a f_1 \\ -a f_1 \\ 0 \end{bmatrix}.$$

Rotor 1 is on the left, so pushing harder lifts the left side: positive roll, as the first component says. Rotor 1 is at the front, so pushing harder lifts the nose. Positive pitch is nose down, so lifting the nose is negative pitch, as the second component says. Both signs agree with entry 00.

---

## The rotor-to-wrench map (the heart of this entry)

Summing the four contributions gives the map from the four rotor thrusts to the wrench the dynamics care about. Total thrust is the plain sum. Roll torque is $\tau_x = \sum_i y_i f_i$ and pitch torque is $\tau_y = -\sum_i x_i f_i$. Yaw torque is the signed sum of reaction torques, $\tau_z = c_{\tau f}\sum_i s_i f_i$. Written as a matrix:

$$
\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}
=
\underbrace{\begin{bmatrix}
1 & 1 & 1 & 1 \\
a & a & -a & -a \\
-a & a & a & -a \\
c_{\tau f} & -c_{\tau f} & c_{\tau f} & -c_{\tau f}
\end{bmatrix}}_{\textstyle \mathbf{M}}
\begin{bmatrix} f_1 \\ f_2 \\ f_3 \\ f_4 \end{bmatrix}
$$

Reading the rows in order:

- all four rotors add to total thrust;
- the two left rotors (1, 2) minus the two right rotors (3, 4) give positive roll, which lifts the left side;
- the two rear rotors (2, 3) minus the two front rotors (1, 4) give positive pitch, which lifts the tail and is therefore **nose down**;
- the two clockwise rotors (1, 3) minus the two counter-clockwise rotors (2, 4) give positive yaw, which is nose left.

All four agree with entry 00's sign table.

**The map is invertible, and cleanly so.** The four rows of $\mathbf{M}$ have sign patterns $(+\,+\,+\,+)$, $(+\,+\,-\,-)$, $(-\,+\,+\,-)$, $(+\,-\,+\,-)$, which are mutually orthogonal. So $\mathbf{M}$ is invertible, with

$$
f_1 = \tfrac{T}{4} + \tfrac{\tau_x}{4a} - \tfrac{\tau_y}{4a} + \tfrac{\tau_z}{4c_{\tau f}}, \qquad
f_2 = \tfrac{T}{4} + \tfrac{\tau_x}{4a} + \tfrac{\tau_y}{4a} - \tfrac{\tau_z}{4c_{\tau f}},
$$
$$
f_3 = \tfrac{T}{4} - \tfrac{\tau_x}{4a} + \tfrac{\tau_y}{4a} + \tfrac{\tau_z}{4c_{\tau f}}, \qquad
f_4 = \tfrac{T}{4} - \tfrac{\tau_x}{4a} - \tfrac{\tau_y}{4a} - \tfrac{\tau_z}{4c_{\tau f}} .
$$

Invertibility matters for the project: any wrench we might want has exactly one rotor-thrust combination that produces it, so there is no allocation ambiguity to resolve. The only question that can ever arise is whether that unique combination is *admissible*. That question is the next section.

**How strong is yaw relative to roll and pitch?** The inverse makes the comparison explicit. A roll torque $\tau_x$ costs each rotor a thrust change of $\tau_x / (4a)$, and a yaw torque $\tau_z$ costs each rotor $\tau_z / (4c_{\tau f})$. For the same rotor-thrust spread, the ratio of available yaw torque to available roll torque is therefore

$$\frac{c_{\tau f}}{a},$$

a dimensionless number that belongs to the vehicle. It is not universal. Its committed value for our vehicle is

$$\frac{c_{\tau f}}{a} = \frac{0.00699\ \text{m}}{0.0325\ \text{m}} = 0.215,$$

from the Bitcraze firmware's torque-to-thrust constant for the 2.1+ propellers and the 46 mm arm (parameter table below). The verified sources agree to within about 20 percent: Förster's 2.0 identification gives 0.183, and the firmware's legacy-propeller value gives 0.226. RotorPy is the exception: its coefficients give 1.12, about five times the others, although its comment cites Förster. We treat that as an error in RotorPy and record how we handle it below. The angular acceleration each torque produces also depends on the inertia about that axis, which entry 04 brings in.

**Summary table.**

| Output | Comes from | Mechanism | Scales with |
|--------|-----------|-----------|-------------|
| Total thrust $T$ | sum of the four rotor thrusts | all push along body z | direct sum |
| Roll torque $\tau_x$ | left pair minus right pair | lever across the frame | effective arm $a = l/\sqrt{2}$ |
| Pitch torque $\tau_y$ | rear pair minus front pair | lever across the frame | effective arm $a = l/\sqrt{2}$ |
| Yaw torque $\tau_z$ | clockwise pair minus counter-clockwise pair | aerodynamic reaction from spin direction | torque-to-thrust ratio $c_{\tau f}$ |

---

## Actuator limits, and why the admissible wrench set is not a box

This section exists because our contribution is about what a correction can actually be made to do, and the limits that decide that live at the rotor, not at the wrench.

**The physical limits are per rotor, and the lower bound is not negative:**

$$f_{\min} \le f_i \le f_{\max}, \qquad f_{\min} \ge 0, \qquad i = 1,\dots,4 .$$

The upper bound is the obvious one. **The lower bound is the one that is easy to forget.** Fixed-pitch propellers spinning in one direction can only push; a rotor cannot pull the airframe down. So the input set is a box in the positive orthant, not a symmetric box about zero. The derivations keep $f_{\min}$ general. For our vehicle it is the firmware's thrust floor, $f_{\min} = 0.0128$ N per rotor (18 percent of hover thrust), and the parameter table below explains what the firmware actually does there. Entry 04 gives the authority formulas for general $f_{\min}$, so the bench test can change it without rederivation.

With motor lag in the nominal model, these limits apply to the **commanded** thrusts. The delivered thrusts, being first-order filtered versions of commands inside the box, stay inside the same box automatically.

**The consequence: the admissible wrench set is a skewed box that cannot be separated axis by axis.** The image of the four-dimensional input box under the invertible map $\mathbf{M}$ is a polytope in $(T, \tau_x, \tau_y, \tau_z)$ space, specifically a parallelotope with eight faces, one for each rotor bound. It is *not* a set of independent intervals on $T$ and on each torque. The available torque depends on how much total thrust is being commanded, and on the other torques:

- at hover each rotor sits at $mg/4$, a fraction $1/\text{TWR}$ of its range, and the torque available in each direction is set by the smaller of the distance up to the ceiling and the distance down to the floor;
- near maximum thrust, every rotor is close to its ceiling, so there is almost no room left to push any rotor *harder*, and torque authority collapses;
- near zero thrust, rotors sit against their floor and cannot be reduced further, so authority collapses the other way;
- the roll, pitch, and yaw demands compete for the same four scalars, so a large demand on one axis consumes the authority of the others.

This is a first-class result for our project and we flag it now so it is not discovered late. **If realizability is computed against independent bounds on $T$ and $\boldsymbol{\tau}$, it will systematically overestimate available authority**, and it will do so worst in exactly the aggressive, high-thrust, coupled-axis regime we have chosen as our testbed. The correct test is whether the requested wrench lies in the polytope $\mathbf{M}\,[f_{\min}, f_{\max}]^4$, which is the elementwise test

$$f_{\min} \;\le\; \mathbf{M}^{-1}\begin{bmatrix} T \\ \boldsymbol{\tau} \end{bmatrix} \;\le\; f_{\max},$$

and it is cheap because we have $\mathbf{M}^{-1}$ in closed form above.

**Rate limits.** We also bound how fast the **commanded** thrust may change between control steps:

$$|f^{\text{cmd}}_{i,k} - f^{\text{cmd}}_{i,k-1}| \le \delta_{\max} .$$

This is a design limit on the command, not a physical law of the motor; the motor's own slowness is already the lag above. Its value is chosen in entry 06. Together with the motor lag, this is what turns the geometric question ("is this wrench reachable at all") into the finite-horizon question ("is it reachable *within the horizon that matters*").

**How this connects to the realizability ratio.** Because the MPC works directly with the rotor thrusts as its input, the limits above *are* the input constraints of the research proposal, with no further mapping:

| Proposal symbol | Meaning | Defined here as |
|---|---|---|
| $u_k$ | input at step $k$ | commanded thrusts $(f^{\text{cmd}}_1, \dots, f^{\text{cmd}}_4)$ |
| $u_{\min},\ u_{\max}$ | input box limits | $f_{\min},\ f_{\max}$ on every rotor |
| $\delta_{\max}$ | per-step rate limit | bound on $\lvert f^{\text{cmd}}_{i,k} - f^{\text{cmd}}_{i,k-1}\rvert$, chosen in entry 06 |

The realizability ratio is a ratio test against these box and rate limits in rotor-thrust space. That is the polytope test above, carried out in the coordinates where the polytope is a box. This is the reason for choosing rotor thrusts as the input: the constraint set is as simple as it can be exactly where the test is applied.

**Two reference quantities we will normalise against.** At hover every rotor carries a quarter of the weight, $f_i = mg/4$, and the thrust-to-weight ratio is

$$\text{TWR} = \frac{4 f_{\max}}{mg} .$$

TWR is the single most useful scalar for comparing vehicles, since it says how much of the rotor range is left above hover for manoeuvres and corrections. It will be one of the normalising groups in the cross-vehicle study.

---

## The committed vehicle parameters

The vehicle is the **Crazyflie 2.1+ with standard propellers** (decision Q5 in [decisions.md](decisions.md); hardware purchase pending confirmation). No single source covers this vehicle completely, so the parameter set is a deliberate mix. Each number below carries its source, and the same values live in code in [checks/params.py](checks/params.py), which every check script imports.

Sources (all verified, see [99_audit.md](99_audit.md), sections D and E):
- **FW**: Bitcraze `crazyflie-firmware`, `src/platform/interface/platform_defaults_cf2.h`, constants for the 2.1+ propellers.
- **FO**: J. Förster, *System identification of the Crazyflie 2.0 nano quadrocopter*, Bachelor thesis, ETH Zurich, 2015, DOI 10.3929/ethz-b-000214143, Appendix A. **Note:** FO identifies the Crazyflie **2.0**, not the 2.1+.
- **RP**: RotorPy, `rotorpy/vehicles/crazyflie_params.py`; used only for comparison.

| Parameter | Symbol | Committed value | Source | Status |
|---|---|---|---|---|
| Mass | $m$ | 0.029 kg | FW `CF_MASS` | weigh on arrival |
| Arm, centre to rotor | $l$ | 0.046 m | FW `ARM_LENGTH` (FO: 0.092 m between opposite motors) | fixed |
| Effective moment arm | $a = l/\sqrt2$ | 0.0325 m | derived | — |
| Max thrust per rotor | $f_{\max}$ | 0.12 N | FW `THRUST_MAX`, 2.1+ props: a firmware cap, see below | **re-identify by bench test** |
| Min thrust per rotor | $f_{\min}$ | 0.0128 N (= 0.012817578393224994) | FW `THRUST_MIN`, 2.1+ props, see below | **re-identify by bench test** |
| Thrust map (command → thrust) | — | not needed while $f$ is the input | FO gives it for the 2.0 | **re-identify by bench test** |
| Torque-to-thrust ratio | $c_{\tau f}$ | 0.00699 m | FW `THRUST2TORQUE`, 2.1+ props | — |
| Inertia (plant) | $\mathbf{I}$ | FO full matrix: diagonal (16.57, 16.66, 29.26) × 10⁻⁶ kg·m², off-diagonal up to 1.80 × 10⁻⁶ | FO | 2.0 airframe |
| Inertia (MPC model) | $\mathbf{I}_{\text{nom}}$ | diagonal of the above | FO | deliberate idealization, entry 03 |
| Motor time constant | $\tau_m$ | 0.065 s (alternative series: 0.077 s) | derived from FO eq. 3.16 (3.15), single-pole fit | 2.0 motors; **re-identify by bench test** |
| Rotor drag | $d_{x}, d_y$ | ≈ 7 × 10⁻³ N·s/m at hover | derived from FO $\mathbf{K}_{\text{aero}}$ | order of magnitude |
| Command rate limit | $\delta_{\max}$ | — | design choice | entry 06 |

Derived groups (computed in [checks/params.py](checks/params.py)): **TWR = 4 f_max/(m g) = 1.69**, $f_{\min}/f_{\text{hover}} = 0.18$, $c_{\tau f}/a = 0.215$, $I_{zz}/I_{xx} = 1.77$.

**What the firmware's two thrust limits mean.** Both are verified in the Bitcraze firmware (`platform_defaults_cf2.h`, `motors.c`, `stabilizer.c`, top-level `Kconfig`; checked 2026-11-12).
- **`THRUST_MAX` = 0.12 N is a firmware cap, not the motor's physical maximum.** The header's own comment says the value is "a trade-off between consistency of thrust over all battery levels and maximum performance with a full battery". The firmware scales commands so that a given command produces about the same thrust at any battery voltage, and caps them at the thrust it can still deliver on a drained battery. A full battery can push harder. **The bench test must therefore run under the same firmware thrust configuration as flight** (battery compensation on, 2.1+ propeller setup). Otherwise it measures a different $f_{\max}$ from the one the controller will meet.
- **`THRUST_MIN` = 0.0128 N is a cutoff.** Both limits are defined only under `CONFIG_ENABLE_THRUST_BAT_COMPENSATED`, which is `default y` for the Crazyflie 2.x. Its propeller setup defaults to `CRAZYFLIE_21_PLUS`, and the normal flight loop applies the compensation on every iteration (`stabilizer.c`, `batteryCompensation`). In `motorsCompensateBatteryVoltage` (`motors.c`), a command below `THRUST_MIN` returns **zero**: the motor is switched off, not held at the minimum. So the true per-rotor set is **non-convex**: $\{0\} \cup [f_{\min}, f_{\max}]$. The MPC uses the convex box $[f_{\min}, f_{\max}]$, which is a conservative restriction. It gives up only the option of switching a motor off, which no tracking manoeuvre should rely on. At 18 percent of hover thrust, $f_{\min}$ is not negligible. Entry 04 shows that it moves the authority knee from TWR = 2 to 1.82.

**Cross-source spread, recorded as uncertainty.**
- Mass: 27 g (Menagerie), 28 g (FO), 29 g (FW), 30 g (RP).
- Maximum thrust: 0.12 N (FW, 2.1+ props), ≈ 0.15 N (FO, 2.0 props), 0.18 N (FW, thrust-upgrade kit). That is a TWR between 1.69 and about 2.5. Entry 04 shows that $\text{TWR} = 2 - f_{\min}/f_{\text{hover}} = 1.82$ is the boundary between two saturation regimes, so this spread matters. It is why $f_{\max}$ is the first number to re-identify.
- $c_{\tau f}$: 0.00597 m (FO), 0.00699 (FW 2.1+), 0.00735 (FW legacy props), 0.0339 m (RP, rejected, see below).

Two caveats are deliberate and stated here so that they are never forgotten. **The inertia and the motor lag come from the 2.0, the thrust and torque constants from the 2.1+.** The airframes are close but not identical. A bench test on the purchased vehicle, under the flight firmware's thrust configuration, will re-identify $f_{\max}$, $f_{\min}$, the thrust map and **$\tau_m$**, and this table will then change. Of these, $\tau_m$ has the highest leverage. It is 1.53 times the attitude time scale (entry 04), it sits in the nominal model (decision Q1), and its current value is a single-pole fit on a different vehicle version.

---

## How the simulators differ from this model

**MuJoCo Menagerie Crazyflie (`bitcraze_crazyflie_2/cf2.xml`).** The model is actuated by a **body wrench**, not by four rotors. It has a collective-thrust motor with range 0–0.35 N and three body-moment motors with gear $-10^{-5}$, all applied at one site at the body origin. Two traps follow:
- The moment gears are **negative**, so a positive control produces a negative moment.
- The ranges are, in the model's own README, "currently arbitrary". The thrust range alone would mean TWR = 1.32.

The model also has no rotor geometry, and its mass (27 g) and inertia (from an MIT identification) differ from ours. Our own simulator interface therefore owns the actuation. Commanded thrusts pass through the delay buffer (plant only), then the motor lag, then the per-rotor box. The mixer $\mathbf{M}$ turns the result into a wrench, and the interface applies that wrench to the body directly, overriding mass and inertia with the values above. Phase 2 tests the moment sign explicitly.

**RotorPy (second simulator, decision Q4).** RotorPy differs from us in three ways.
- **Rotor order.** It numbers rotors front-left, front-right, back-right, back-left (clockwise seen from above), while ours run counter-clockwise. Its directions vector $[1,-1,1,-1]$ is in its own sign convention. A fixed permutation maps its rotors to ours, and Phase 2 checks it.
- **Motor lag on rotor speed**, with $\tau_m = 0.072$ s (see above).
- **Yaw coefficient** $k_m/k_\eta = 0.0339$ m, which is 5.7 times Förster's value although the file cites Förster.

For the main cross-simulator check we **override RotorPy's $c_{\tau f}$ with our committed value** and map its rotor order to ours. One additional run keeps RotorPy's native value, as a robustness check.

---

## What this entry commits us to, and what is open

**Committed.**
- The nominal force model is gravity along negative world z plus total thrust along positive body z.
- The drag mismatch is the linear rotor-drag model $-R\,\mathbf{D}\,R^{\top}\mathbf{v}$, valid near hover in still air.
- The airframe is X configuration with effective moment arm $a = l/\sqrt{2}$, rotors numbered front-left, back-left, back-right, front-right, with adjacent rotors counter-rotating and rotors 1 and 3 clockwise seen from above.
- Per-rotor thrust and reaction torque are quadratic in rotor speed, with reaction torque proportional to thrust through $c_{\tau f} = k_m/k_f$.
- The control input is the vector of four **commanded** rotor thrusts. The delivered thrusts follow them through a first-order lag on thrust with time constant $\tau_m$, which is part of the nominal model (decision Q1).
- The rotor-to-wrench map is the matrix $\mathbf{M}$ above, which is invertible with the closed-form inverse given. Positive $\tau_y$ is nose down.
- The input constraint set is the per-rotor box $f_{\min} \le f^{\text{cmd}}_i \le f_{\max}$ with $f_{\min} \ge 0$, and a per-step rate bound on the command. Its image in wrench space is a thrust-dependent polytope, and realizability is tested in rotor-thrust space, where that polytope is a box.
- The vehicle is the Crazyflie 2.1+ with the cited parameter mix above: FW thrust limits and torque constant, FO inertia and motor lag. TWR = 1.69, $f_{\min}/f_{\text{hover}} = 0.18$, $c_{\tau f}/a = 0.215$. The MPC's input box is $[f_{\min}, f_{\max}]$, a convex restriction of the firmware's true set $\{0\}\cup[f_{\min}, f_{\max}]$.
- The motor mapping M1–M4 → rotors 4, 3, 2, 1 is verified from the firmware.
- The simulators are adapted to this model: the Menagerie model's actuation is replaced by our own interface. In RotorPy, $c_{\tau f}$ is overridden and the rotor order permuted, with one native-value run kept as a robustness check.

**Deliberately left out of the nominal model.**
- Rotor drag, added later as a mismatch. Quadratic body drag and the torque produced by rotor drag, excluded altogether.
- Blade flapping, ground effect, variation of thrust with inflow, and similar higher-order effects, left in the gap the learned residual is meant to capture.
- **Rotor gyroscopic torque.** The spinning rotors carry angular momentum $\mathbf{h}_r = -J_r \sum_i s_i\,\Omega_i\,\mathbf{e}_3$ in the body frame, where $J_r$ is the inertia of one rotor about its axis. When the body rotates, this produces a torque $-\boldsymbol{\omega} \times \mathbf{h}_r$ on the airframe. We exclude it, and we say why, because it belongs to the same physical family as the body gyroscopic term we *do* keep in entry 03. The rotors counter-rotate in pairs, so at equal speeds their angular momenta cancel exactly and $\mathbf{h}_r = \mathbf{0}$. What remains is proportional to the speed *imbalance* between the two pairs, which is nonzero only while a yaw torque is being commanded.
- **Rotor spin-up torque.** Accelerating a rotor requires a torque on it, and the airframe receives the opposite reaction: $\tau_{z,\text{spin}} = J_r \sum_i s_i\,\dot\Omega_i$ about body z. This one deserves a sharper caveat than the previous. Commanding yaw *is* commanding $\dot\Omega_i$, and this term has the same sign as the aerodynamic reaction torque it accompanies, so during fast yaw transients it adds to the yaw torque and is not obviously negligible. We exclude it from the nominal model, and we flag it as an effect to check numerically once the simulator runs, not as a settled dismissal.

**Open, to be settled in later entries.**
- Whether the *plant's* motor lag acts on thrust or on rotor speed (entry 07). The nominal model's lag acts on thrust.
- The bench re-identification of $f_{\max}$, $f_{\min}$, the thrust map and $\tau_m$ on the purchased 2.1+ (hardware phase), under the flight firmware's thrust configuration. Until then, results are reported with the committed values and the TWR spread in mind.
- The command rate limit $\delta_{\max}$ (entry 06).

**Open for the next entry (02, translational dynamics).** We now have the forces and torques and we know they live in mixed frames, thrust in the body frame and gravity in the world frame, joined by the orientation. Entry 02 uses this to write the translational equations of motion: how the total force produces linear acceleration in the world frame once the body thrust is rotated in. Entry 03 then does the rotational half, taking the torques $\boldsymbol{\tau}$ produced by the map above and turning them into angular acceleration about the body axes.

*This entry is locked. We do not proceed to 02 until every choice above can be explained in plain language, in particular: the two different torque mechanisms, why the effective arm is $l/\sqrt{2}$ and not $l$, why positive pitch torque comes from the rear rotors, why the strength of yaw relative to roll is the ratio $c_{\tau f}/a$ and where its committed value of 0.215 comes from, why a rotor's lower thrust bound is not negative, why the admissible wrench set is a thrust-dependent polytope that becomes a simple box only in rotor-thrust coordinates, why the nominal model's motor lag acts on thrust, and which committed parameters come from which source and vehicle version.*