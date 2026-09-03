# 01 — Forces and Torques
 
*Derivation notebook, entry 01. Continues the clean-room rebuild from entry 00. Written as we reason through the physics and decide what enters our model, not as a textbook.*
 
---
 
## What this entry is for
 
Entry 00 told us what numbers describe the drone at an instant. This entry tells us what changes those numbers. Everything that moves the drone is either a force, which changes how it translates (position and velocity), or a torque, which changes how it rotates (orientation and angular velocity). So our job here is to list every force and every torque acting on the drone, decide which of them belong in our nominal model and which we deliberately leave out, and work out how the one thing we can actually command, the four motors, produces those forces and torques. Once we have that, entry 02 can write the equations of motion.
 
A note on why frames matter here: entry 00 ended on the observation that the two dominant forces live in different frames, thrust in the body frame and gravity in the world frame, connected by the orientation. This entry makes that concrete, because getting a force into the wrong frame is one of the classic sources of silent error in quadrotor work. We inherit entry 00's convention without restating it: right-handed z-up world with gravity along negative world z, right-handed FLU body (x forward, y left, z up) with thrust along positive body z, right-hand rule for rotations so that positive roll lifts the left side and positive pitch is nose up.
 
---
 
## The forces (what makes the drone translate)
 
We began by listing every force we could think of acting on a quadrotor in flight, then sorted them into what belongs in the nominal model and what does not.
 
**Gravity:** Always points along negative world z, and it is naturally a world frame quantity because "down" is defined relative to the ground, not the drone. It is constant. It belongs in the nominal model.
 
**Thrust:** The four propellers push the drone along its own body z direction, so total thrust always points along positive body z. This is a body frame quantity, and it is the single most important consequence of orientation: when the drone tilts, its thrust tilts with it, so a tilted thrust gains a horizontal component. This is the entire reason a quadrotor can move sideways at all, since it has no way to push sideways directly. Thrust belongs in the nominal model, and the orientation is what we will use to rotate it from the body frame into the world frame so it can be added to gravity.
 
**Body drag (bluff-body air resistance):** Air resistance on the airframe opposes motion through the air, so it points opposite to the velocity. Drag is real and we care about it, but we make a deliberate choice: it does not enter the nominal model. Instead we introduce it later as one of the perturbations we study, so that the nominal model stays clean and the drag becomes part of the mismatch our controller and learned helper must deal with. When we do introduce it we will have to decide whether we model it as quadratic in speed (the honest bluff-body form) or linear (the tractable form), and we will state that choice explicitly at the time rather than letting it default.
 
**Rotor drag (induced drag from the spinning discs):** We call this one out separately rather than burying it under "higher order effects," for two reasons. First, it is the dominant modelable aerodynamic effect on a quadrotor in forward flight, and it is the canonical target of the learned-residual quadrotor literature, so any reader of this work will expect it by name. Second, its structure matters to our thesis. It is approximately linear in the body-frame velocity, of the form:
 
 $$\mathbf{F}_{\text{rotor drag}} \approx -\mathbf{R}\,\mathbf{D}\,\mathbf{R}^{\top}\mathbf{v}, \qquad \mathbf{D} = \operatorname{diag}(d_x, d_y, d_z)$$
 
 with the lateral coefficients much larger than the vertical one. It therefore acts in the *translational* channel, and the number of differentiations separating it from a body-rate command is exactly the kind of quantity our realizability criterion is built on. Like body drag, it stays out of the nominal model and enters as a perturbation.
-
-**Notation of Rotor Drag Model (for better understanding):**
-
-| Symbol | Meaning | Simple meaning |
-|---|---|---|
-| $F_{\text{rotor drag}}$ | Rotor-drag force | Aerodynamic force caused by the spinning rotors |
-| $R$ | Rotation matrix | Describes the drone's orientation relative to the world |
-| $D$ | Rotor-drag coefficient matrix | Describes how strongly drag acts along each body direction |
-| $R^\top$ | Transpose of $R$ | Converts a world-frame vector into the body frame |
-| $v$ | Linear velocity | How fast the drone is moving |
-| $-$ | Negative sign | Indicates that drag opposes the motion |
-| $d_x, d_y, d_z$ | Drag coefficients | Strength of drag along body X, Y, and Z directions |
 
 **Higher order aerodynamic effects (blade flapping, ground effect, propeller inflow variation, and similar):** These are real and we acknowledge them, but we deliberately leave them out of the nominal model. They are small, difficult to model precisely, and, importantly, they are exactly the kind of unmodeled mismatch our learned residual exists to capture. In other words, leaving them out is not an oversight; the gap they create is part of what the research is about. We note them here so they are on record, and we let them live in the residual rather than in the analytical model.
 
So the nominal force model reduces to two forces: **gravity along negative world z, and total thrust along positive body z.** Body drag and rotor drag are added as perturbations. Flapping, ground effect, and their relatives are left in the gap that the residual learns.
 
---
 
## The torques (what makes the drone rotate)
 
A quadrotor has no flaps, rudder, or moving control surfaces. It has only four spinning propellers. So the interesting question is how four propellers, purely by spinning at different speeds, make the drone roll, pitch, and yaw. Working through it, we found that roll and pitch use one mechanism and yaw uses a completely different one, which is a distinction worth stating clearly because it explains a lot about how a quadrotor behaves.
 
**Roll and pitch come from thrust differences across the frame.** If the rotors on one side of the drone push harder than the rotors on the other side, that side lifts more and the drone tips. So roll and pitch are produced by an imbalance in thrust between opposite sides, acting through the physical arm of the frame. Formally each rotor contributes a torque $\mathbf{r}_i \times f_i \mathbf{e}_3$, where $\mathbf{r}_i$ is its position in the body frame; the cross product is what turns "a push at a distance" into "a rotation."
 
**Yaw comes from a reaction torque, not from any sideways push.** Every spinning propeller drags the surrounding air, and by Newton's third law the air drags back on the drone, producing a reaction torque that tries to spin the body the opposite way to the propeller. If all four propellers spun the same direction, the drone would spin uncontrollably, so the standard layout spins two propellers clockwise and two counterclockwise. At equal speeds their reaction torques cancel and there is no net yaw. To yaw on purpose, we deliberately break that balance: speeding up the clockwise pair while slowing the counterclockwise pair lets their reaction torques dominate, turning the drone one way, and reversing it turns the drone the other way. Yaw is therefore driven by imbalancing an aerodynamic reaction, which is why it is the weakest and slowest of the three rotational axes.
 
The key insight is that these are two genuinely different mechanisms. Roll and pitch are thrust differences acting through a lever. Yaw is a difference in aerodynamic reaction torques from the direction of spin.
 
---
 
## The rotor model (what one motor actually produces)
 
Before we can write the map from four motors to a wrench, we need what a single rotor produces. Let $\Omega_i$ be the angular speed of rotor $i$. Two standard relations, both quadratic in speed because both are aerodynamic:
 
 $$f_i = k_f\,\Omega_i^2, \qquad \tau_{\text{react},i} = k_m\,\Omega_i^2 = \frac{k_m}{k_f}\,f_i = c_{\tau f}\,f_i$$
 
 The first is the thrust the rotor produces along body z. The second is the reaction torque it exerts on the airframe about body z, opposite in sense to its own spin. The important consequence is the last equality: **the reaction torque is proportional to the thrust**, with a single constant $c_{\tau f} = k_m / k_f$ that has units of length. That is the fair comparison against arm length, and it is a brutal one. For a typical airframe $c_{\tau f}$ is of order 0.01 m while the arm length $l$ is of order 0.15 m, roughly a factor of fifteen. This is the quantitative version of the qualitative claim above: yaw authority is not merely "weaker," it is smaller by more than an order of magnitude per unit of thrust asymmetry, and we should expect any correction demanding fast yaw to be far closer to unrealizable than the same correction demanding roll or pitch.
 
We treat $f_i$, not $\Omega_i$, as the control input throughout, since the map between them is a fixed monotone square root and carrying $\Omega_i$ adds nothing at this stage. When we add motor lag in the perturbation phase we will have to decide whether the first-order lag acts on $f_i$ or on $\Omega_i$, because those are not the same model, and we will state that choice explicitly then.
 
---
 
## The airframe geometry (committing to X, not plus)
 
The motor-to-wrench map is not universal; it depends on where the rotors sit. We commit here to the **X configuration**, in which the arms lie at 45 degrees to the body x and y axes, rather than the plus configuration in which each arm lies along an axis. Two reasons: X is what essentially all real quadrotors fly, and X gives all four rotors a lever on both roll and pitch rather than two each, which is the more favourable case for control authority and the more honest one to analyse.
 
With arm length $l$ measured from the centre of mass to each rotor, define
 
$$a \;=\; \frac{l}{\sqrt{2}}$$
 
which is the effective moment arm of each rotor about the body x and y axes. Note immediately that $a < l$: the X configuration trades some per-axis leverage for the fact that all four rotors contribute. Using $l$ where $a$ belongs overstates roll and pitch authority by 41 percent, which would silently corrupt every realizability computation downstream.
 
The four rotors, numbered and with spin directions assigned so that adjacent rotors counter-rotate:
 
| Rotor | Position (body frame) | Spin seen from above | Reaction torque on body |
|-------|----------------------|----------------------|-------------------------|
| 1 | front-left, $(+a, +a, 0)$ | clockwise | $+z$ |
| 2 | back-left, $(-a, +a, 0)$ | counter-clockwise | $-z$ |
| 3 | back-right, $(-a, -a, 0)$ | clockwise | $+z$ |
| 4 | front-right, $(+a, -a, 0)$ | counter-clockwise | $-z$ |
 
Diagonal pairs (1, 3) and (2, 4) share a spin direction; adjacent rotors oppose. This is the standard layout, and it is what makes the yaw reaction torques cancel at equal thrusts.
 
---
 
## The motor to wrench map (the heart of this entry)
 
Summing the four rotor contributions gives the map from the four commanded thrusts to the wrench the dynamics actually care about. Total thrust is the plain sum. Roll torque is $\tau_x = \sum_i r_{y,i} f_i$ and pitch torque is $\tau_y = -\sum_i r_{x,i} f_i$, both falling straight out of the cross product. Yaw torque is the signed sum of reaction torques. Written as a matrix:
 
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
 
Reading the rows in order: all four rotors add to total thrust; the two left rotors (1, 2) minus the two right rotors (3, 4) give positive roll, which lifts the left side, matching entry 00's sign convention; the two rear rotors (2, 3) minus the two front rotors (1, 4) give positive pitch, which is nose up, again matching entry 00; and the two clockwise rotors (1, 3) minus the two counter-clockwise rotors (2, 4) give positive yaw, which is nose left.
 
**The map is invertible, and cleanly so.** The four rows of $\mathbf{M}$ have sign patterns $(+\,+\,+\,+)$, $(+\,+\,-\,-)$, $(-\,+\,+\,-)$, $(+\,-\,+\,-)$, which are mutually orthogonal. So $\mathbf{M}$ is invertible with
 
$$
f_1 = \tfrac{T}{4} + \tfrac{\tau_x}{4a} - \tfrac{\tau_y}{4a} + \tfrac{\tau_z}{4c_{\tau f}}, \qquad
f_2 = \tfrac{T}{4} + \tfrac{\tau_x}{4a} + \tfrac{\tau_y}{4a} - \tfrac{\tau_z}{4c_{\tau f}},
$$
$$
f_3 = \tfrac{T}{4} - \tfrac{\tau_x}{4a} + \tfrac{\tau_y}{4a} + \tfrac{\tau_z}{4c_{\tau f}}, \qquad
f_4 = \tfrac{T}{4} - \tfrac{\tau_x}{4a} - \tfrac{\tau_y}{4a} - \tfrac{\tau_z}{4c_{\tau f}}
$$
 
Invertibility matters for the project: it means any wrench we might want has exactly one rotor-thrust combination that produces it, so there is no allocation ambiguity to resolve, and the only question that can ever arise is whether that unique combination is *admissible*. That question is the next section, and it is the one our whole thesis rests on.
 
Notice also the $1/(4c_{\tau f})$ on the yaw terms against $1/(4a)$ on the roll and pitch terms. Because $c_{\tau f} \ll a$, a modest yaw torque demands a large rotor-thrust spread, which is precisely the mechanism by which yaw commands run out of authority first.
 
**Summary table, for the plain-language version of the same content.**
 
| Output | Comes from | Mechanism | Scales with |
|--------|-----------|-----------|-------------|
| Total thrust | sum of the four rotor thrusts | all push along body z | direct sum |
| Roll torque | left pair minus right pair | lever across the frame | effective arm $a = l/\sqrt{2}$ |
| Pitch torque | rear pair minus front pair | lever across the frame | effective arm $a = l/\sqrt{2}$ |
| Yaw torque | clockwise pair minus counter-clockwise pair | aerodynamic reaction from spin direction | thrust-to-torque constant $c_{\tau f}$ |
 
---
 
## Actuator limits, and why the admissible wrench set is not a box
 
This section exists because our contribution is about what a correction can actually be made to do, and the limits that decide that live at the rotor, not at the wrench.
 
**The physical limits are per rotor, and the lower bound is zero:**
 
$$0 \le f_i \le f_{\max}, \qquad i = 1,\dots,4$$
 
The upper bound is the obvious one. The **lower bound of zero, not $-f_{\max}$, is the one that matters and the one that is easy to forget.** Fixed-pitch propellers spinning in one direction can only push; a rotor cannot pull the airframe down. So the input set in rotor-thrust space is a box in the positive orthant, not a symmetric box about zero.
 
**The consequence: the admissible wrench set is a polytope, not a box, and it is thrust-dependent.** The image of that four-dimensional box under $\mathbf{M}$ is a polytope in $(T, \tau_x, \tau_y, \tau_z)$ space. It is emphatically *not* a set of independent intervals on $T$ and on each torque. The available torque depends on how much total thrust is currently being commanded:
 
- near hover, roughly half of $f_{\max}$ per rotor, torque authority is roughly symmetric and near its maximum in every direction
- near maximum thrust, every rotor is close to its ceiling, so there is almost no room left to push any rotor *harder*, and torque authority collapses on the side that would require it
- near zero thrust, rotors sit against their floor of zero and cannot be reduced further, so authority collapses the other way
- the roll, pitch, and yaw demands compete for the same four scalars, so a large yaw demand eats the roll and pitch authority and vice versa
This is a first-class result for our project and we flag it now so it is not discovered late. **If realizability is computed against independent box bounds on $T$ and $\tau$, it will systematically overestimate available authority**, and it will do so worst in exactly the aggressive, high-thrust, coupled-axis regime we have chosen as our testbed. The correct test is membership of the requested wrench in the polytope $\mathbf{M}\,[0, f_{\max}]^4$, equivalently the elementwise test $0 \le \mathbf{M}^{-1}[T, \boldsymbol{\tau}]^{\top} \le f_{\max}$, which is cheap because we have $\mathbf{M}^{-1}$ in closed form above.
 
**Rate limits.** Rotors cannot change thrust instantaneously either. The rate bound $|\dot f_i| \le \dot f_{\max}$, together with the first-order motor lag we introduce in the perturbation phase, is what turns the geometric question ("is this wrench reachable at all") into the finite-horizon question ("is it reachable *within the horizon that matters*"). We record the constraint here and give it dynamics later.
 
**Two reference quantities we will normalise against.** At hover every rotor carries a quarter of the weight, $f_i = mg/4$, and the thrust-to-weight ratio is $\text{TWR} = 4 f_{\max} / (mg)$. TWR is the single most useful scalar for comparing our vehicles, since it says directly how much of the rotor box is left over above hover for corrections to use. It will be one of the normalising groups in the cross-vehicle study.
 
---
 
## What this entry commits us to, and what is open
 
**Committed.**
- The nominal force model is gravity along negative world z plus total thrust along positive body z.
- The airframe is X configuration with effective moment arm $a = l/\sqrt{2}$, rotors numbered front-left, back-left, back-right, front-right, with adjacent rotors counter-rotating.
- Per-rotor thrust and reaction torque are quadratic in rotor speed, with reaction torque proportional to thrust through $c_{\tau f} = k_m/k_f$.
- The motor to wrench map is the matrix $\mathbf{M}$ above, which is invertible with the closed-form inverse given.
- The input constraint set is the per-rotor box $0 \le f_i \le f_{\max}$ with a rate bound, whose image is a thrust-dependent polytope in wrench space, and that polytope, not a box, is what realizability must be tested against.
**Deliberately left out of the nominal model.**
- Body drag and rotor drag, both added later as perturbations.
- Blade flapping, ground effect, propeller inflow variation, and similar higher order effects, left in the gap the learned residual is meant to capture.
- **Rotor gyroscopic torque**, $\boldsymbol{\tau}_{\text{gyro}} = -\sum_i J_r \Omega_i (\boldsymbol{\omega} \times \mathbf{e}_3)$, which arises because each spinning disc resists having its axis tilted. We exclude it from the nominal model but record it explicitly rather than silently, because it belongs to the same physical family as the body gyroscopic term we *do* keep in entry 03, and consistency demands we say why we treat them differently: the rotor discs' inertia $J_r$ is two to three orders of magnitude below the airframe inertia, so this term is genuinely small even in aggressive flight, whereas the body term is not.
- **Rotor spin-up torque**, $\sum_i \pm J_r \dot\Omega_i$ about body z, the reaction to accelerating the discs themselves. This one deserves a sharper caveat than the previous. Commanding yaw *is* commanding $\dot\Omega$, and yaw authority is already weak, so during fast yaw transients this term is not obviously negligible relative to the steady reaction torque it accompanies. We exclude it from the nominal model, and we flag it as a candidate to check numerically once the simulator runs, rather than as a settled dismissal.
**Open for the next entry (02, translational dynamics).** We now have the forces and torques and we know they live in mixed frames, thrust in the body frame and gravity in the world frame, joined by the orientation. Entry 02 uses this to write the translational equations of motion: how the total force produces linear acceleration in the world frame once the body thrust is rotated in. Entry 03 then does the rotational half, taking the torques $\boldsymbol{\tau}$ produced by the map above and turning them into angular acceleration about the body axes.
 
*This entry is locked. We do not proceed to 02 until every choice above can be explained in plain language, in particular the two different torque mechanisms, why the effective arm is $l/\sqrt{2}$ and not $l$, why $c_{\tau f}$ being roughly fifteen times smaller than the arm is the real reason yaw is weak, and why the admissible wrench set is a thrust-dependent polytope rather than a box.*