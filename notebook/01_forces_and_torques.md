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

- **It is a near-hover approximation.** The physical force scales with rotor speed as well as with velocity. Near hover the rotor speeds are roughly constant, so the coefficients can be treated as constants. In aggressive flight they vary with thrust.
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

The map between $f_i$ and $\Omega_i$ is a fixed monotone square root, so carrying $\Omega_i$ adds nothing at this stage. When we add motor lag we will have to decide whether the first-order lag acts on $f_i$ or on $\Omega_i$, because those are not the same model, and we will state that choice explicitly then.

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

**Correspondence with the Crazyflie.** The spin pattern above is expected to match the Crazyflie 2.x, where the front-left and back-right propellers turn clockwise. The Crazyflie's own motor numbering is different from ours: its M1 is the front-right motor and the numbering proceeds clockwise seen from above, so M1, M2, M3, M4 correspond to our rotors 4, 3, 2, 1. This mapping is recorded from the vehicle documentation as we recall it and must be confirmed in Phase 2, because a permuted motor order produces a simulator that hovers correctly and turns the wrong way.

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

a dimensionless number that belongs to the vehicle. We do not quote a universal value for it, because it varies widely. On larger quadrotors, with arms of 0.15 m or more, it is commonly of order one tenth, and yaw is by far the weakest axis. On a vehicle the size of the Crazyflie the arm is only a few centimetres, and the published values of $c_{\tau f}$ differ by a factor of several between sources, so the ratio may be anywhere from roughly 0.2 to above 0.5. **This ratio must be identified for our vehicle before any claim about yaw authority is made.** The angular acceleration each torque produces also depends on the inertia about that axis, which entry 04 brings in.

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

The upper bound is the obvious one. **The lower bound is the one that is easy to forget.** Fixed-pitch propellers spinning in one direction can only push; a rotor cannot pull the airframe down. So the input set is a box in the positive orthant, not a symmetric box about zero. In the derivations that follow we take $f_{\min} = 0$. On a real vehicle the motors are usually kept at a small positive idle thrust in flight, which raises $f_{\min}$ slightly and shrinks the set further.

**The consequence: the admissible wrench set is a skewed box that cannot be separated axis by axis.** The image of the four-dimensional input box under the invertible map $\mathbf{M}$ is a polytope in $(T, \tau_x, \tau_y, \tau_z)$ space, specifically a parallelotope with eight faces, one for each rotor bound. It is *not* a set of independent intervals on $T$ and on each torque. The available torque depends on how much total thrust is being commanded, and on the other torques:

- at hover each rotor sits at $mg/4$, a fraction $1/\text{TWR}$ of its range, and the torque available in each direction is set by the smaller of the distance up to the ceiling and the distance down to the floor;
- near maximum thrust, every rotor is close to its ceiling, so there is almost no room left to push any rotor *harder*, and torque authority collapses;
- near zero thrust, rotors sit against their floor and cannot be reduced further, so authority collapses the other way;
- the roll, pitch, and yaw demands compete for the same four scalars, so a large demand on one axis consumes the authority of the others.

This is a first-class result for our project and we flag it now so it is not discovered late. **If realizability is computed against independent bounds on $T$ and $\boldsymbol{\tau}$, it will systematically overestimate available authority**, and it will do so worst in exactly the aggressive, high-thrust, coupled-axis regime we have chosen as our testbed. The correct test is whether the requested wrench lies in the polytope $\mathbf{M}\,[f_{\min}, f_{\max}]^4$, which is the elementwise test

$$f_{\min} \;\le\; \mathbf{M}^{-1}\begin{bmatrix} T \\ \boldsymbol{\tau} \end{bmatrix} \;\le\; f_{\max},$$

and it is cheap because we have $\mathbf{M}^{-1}$ in closed form above.

**Rate limits.** Rotors cannot change thrust instantaneously either. We bound the rate of change:

$$|\dot f_i| \le \dot f_{\max} .$$

Together with the motor lag we introduce as a mismatch, this is what turns the geometric question ("is this wrench reachable at all") into the finite-horizon question ("is it reachable *within the horizon that matters*").

**How this connects to the realizability ratio.** Because the MPC works directly with the rotor thrusts as its input, the limits above *are* the input constraints of the research proposal, with no further mapping:

| Proposal symbol | Meaning | Defined here as |
|---|---|---|
| $u_k$ | input at step $k$ | $(f_1, f_2, f_3, f_4)$ |
| $u_{\min},\ u_{\max}$ | input box limits | $f_{\min},\ f_{\max}$ on every rotor |
| $\delta_{\max}$ | per-step rate limit | $\dot f_{\max}\,\Delta t$, for sample time $\Delta t$ |

The realizability ratio is a ratio test against these box and rate limits in rotor-thrust space. That is the polytope test above, carried out in the coordinates where the polytope is a box. This is the reason for choosing rotor thrusts as the input: the constraint set is as simple as it can be exactly where the test is applied.

**Two reference quantities we will normalise against.** At hover every rotor carries a quarter of the weight, $f_i = mg/4$, and the thrust-to-weight ratio is

$$\text{TWR} = \frac{4 f_{\max}}{mg} .$$

TWR is the single most useful scalar for comparing vehicles, since it says how much of the rotor range is left above hover for manoeuvres and corrections. It will be one of the normalising groups in the cross-vehicle study.

---

## Parameters to identify for the vehicle

Every symbol in this entry becomes a number in Phase 2. We list them here so that none is filled in by default. The typical values are for orientation only: they are recalled from published Crazyflie 2.x characterizations and are **not** to be used until they are checked against a cited source and against the simulator model.

| Parameter | Symbol | Typical order for a Crazyflie 2.x | Status |
|---|---|---|---|
| Mass | $m$ | about 0.03 kg | to identify |
| Arm length, centre to rotor | $l$ | about 0.046 m | to identify |
| Effective moment arm | $a = l/\sqrt{2}$ | about 0.033 m | follows from $l$ |
| Maximum thrust per rotor | $f_{\max}$ | about 0.15 N | to identify |
| Thrust-to-weight ratio | TWR | about 2 | follows from $m$, $f_{\max}$ |
| Torque-to-thrust ratio | $c_{\tau f}$ | 0.006 to 0.025 m, sources disagree | to identify, high priority |
| Thrust rate limit | $\dot f_{\max}$ | not established | to identify |
| Drag coefficients | $d_x, d_y, d_z$ | not established | to identify when drag is introduced |

Two of these deserve attention. A thrust-to-weight ratio near 2 means hover sits near the middle of each rotor's range, which entry 04 shows is exactly the boundary between two different saturation regimes. And the uncertainty in $c_{\tau f}$ spans a factor of four, which changes the yaw authority by the same factor.

---

## What this entry commits us to, and what is open

**Committed.**
- The nominal force model is gravity along negative world z plus total thrust along positive body z.
- The drag mismatch is the linear rotor-drag model $-R\,\mathbf{D}\,R^{\top}\mathbf{v}$, valid near hover in still air.
- The airframe is X configuration with effective moment arm $a = l/\sqrt{2}$, rotors numbered front-left, back-left, back-right, front-right, with adjacent rotors counter-rotating and rotors 1 and 3 clockwise seen from above.
- Per-rotor thrust and reaction torque are quadratic in rotor speed, with reaction torque proportional to thrust through $c_{\tau f} = k_m/k_f$.
- The control input is the vector of four rotor thrusts.
- The rotor-to-wrench map is the matrix $\mathbf{M}$ above, which is invertible with the closed-form inverse given. Positive $\tau_y$ is nose down.
- The input constraint set is the per-rotor box $f_{\min} \le f_i \le f_{\max}$ with $f_{\min} \ge 0$ and a rate bound. Its image in wrench space is a thrust-dependent polytope, and realizability is tested in rotor-thrust space, where that polytope is a box.

**Deliberately left out of the nominal model.**
- Rotor drag, added later as a mismatch. Quadratic body drag and the torque produced by rotor drag, excluded altogether.
- Blade flapping, ground effect, variation of thrust with inflow, and similar higher-order effects, left in the gap the learned residual is meant to capture.
- **Rotor gyroscopic torque.** The spinning rotors carry angular momentum $\mathbf{h}_r = -J_r \sum_i s_i\,\Omega_i\,\mathbf{e}_3$ in the body frame, where $J_r$ is the inertia of one rotor about its axis. When the body rotates, this produces a torque $-\boldsymbol{\omega} \times \mathbf{h}_r$ on the airframe. We exclude it, and we say why, because it belongs to the same physical family as the body gyroscopic term we *do* keep in entry 03. The rotors counter-rotate in pairs, so at equal speeds their angular momenta cancel exactly and $\mathbf{h}_r = \mathbf{0}$. What remains is proportional to the speed *imbalance* between the two pairs, which is nonzero only while a yaw torque is being commanded.
- **Rotor spin-up torque.** Accelerating a rotor requires a torque on it, and the airframe receives the opposite reaction: $\tau_{z,\text{spin}} = J_r \sum_i s_i\,\dot\Omega_i$ about body z. This one deserves a sharper caveat than the previous. Commanding yaw *is* commanding $\dot\Omega_i$, and this term has the same sign as the aerodynamic reaction torque it accompanies, so during fast yaw transients it adds to the yaw torque and is not obviously negligible. We exclude it from the nominal model, and we flag it as an effect to check numerically once the simulator runs, not as a settled dismissal.

**Open, to be settled in later entries.**
- Whether motor lag acts on thrust or on rotor speed (the perturbation entry).
- The numerical values of every parameter in the table above (Phase 2).
- The Crazyflie motor-numbering correspondence (Phase 2).

**Open for the next entry (02, translational dynamics).** We now have the forces and torques and we know they live in mixed frames, thrust in the body frame and gravity in the world frame, joined by the orientation. Entry 02 uses this to write the translational equations of motion: how the total force produces linear acceleration in the world frame once the body thrust is rotated in. Entry 03 then does the rotational half, taking the torques $\boldsymbol{\tau}$ produced by the map above and turning them into angular acceleration about the body axes.

*This entry is locked. We do not proceed to 02 until every choice above can be explained in plain language, in particular: the two different torque mechanisms, why the effective arm is $l/\sqrt{2}$ and not $l$, why positive pitch torque comes from the rear rotors, why the strength of yaw relative to roll is the ratio $c_{\tau f}/a$ and must be identified for our vehicle, why a rotor's lower thrust bound is not negative, and why the admissible wrench set is a thrust-dependent polytope that becomes a simple box only in rotor-thrust coordinates.*