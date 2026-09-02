# 03 — Rotational Dynamics
 
*Derivation notebook, entry 03. Continues the clean-room rebuild from entries 00 to 02. This entry completes the equations of motion by deriving how the drone's orientation and rotation change over time, and writes out the rotation matrix that entry 02 left symbolic.*
 
---
 
## What this entry is for
 
Entry 02 gave us the translational half of the equations of motion: how position and velocity change. This entry gives the rotational half: how the orientation quaternion and the body angular velocity change. Together the two halves are the full rate of change of all thirteen state numbers, which is everything we need to predict the drone's motion forward in time.
 
The structure of rotation mirrors translation exactly, which is worth keeping in mind because it makes the whole entry feel familiar. In translation we had a configuration (position) whose rate is the velocity, and a velocity whose rate comes from Newton's law with force and mass. In rotation we have a configuration (orientation) whose rate comes from the angular velocity, and an angular velocity whose rate comes from a rotational version of Newton's law with torque and inertia. The correspondence is: force becomes torque, mass becomes inertia, and linear velocity becomes angular velocity.
 
This entry also closes the loop that entry 02 opened. Entry 02 showed that a desired acceleration determines a required thrust *direction* $\mathbf{b}_3^{\text{req}}$, and that the vehicle cannot simply adopt that direction but must rotate to it. This entry supplies the dynamics of that rotation, and therefore supplies the missing links in the relative-degree ladder: it is the two extra differentiations that separate lateral position from body torque.
 
Two honest notes on how we handle this entry. First, the rotational equations are standard, well established results. Deriving quaternion kinematics from scratch is a long algebraic exercise that teaches little beyond the algebra itself, so we state the standard results, explain what every term means, verify each with a sanity check, and, for the two places where a convention error would be silent, verify numerically as well. That is exactly how these results are used and defended in practice. Second, we deliberately keep the model a rigid body: no structural flex, no battery or electrical dynamics, no propeller aerodynamics beyond the thrust and reaction torque of entry 01. Those effects are either left in the gap the residual learns or deferred as future sim to real extensions; they do not enter the nominal model.
 
---
 
## Notation (added to entries 00 to 02)
 
- **q** = (q₀, q₁, q₂, q₃), the unit quaternion representing orientation, with q₀² + q₁² + q₂² + q₃² = 1, scalar part first.
- **ω** = (ω_x, ω_y, ω_z)ᵀ, the body frame angular velocity, meaning the rates of rotation about the body x, y, and z axes respectively. Where prose says roll rate, pitch rate, and yaw rate, it means ω_x, ω_y, and ω_z. As recorded in entry 00, we deliberately avoid the aerospace symbols (p, q, r) because p and q are already taken by the position vector and the quaternion.
- **I**, the inertia tensor, a 3-by-3 matrix **expressed in the body frame**. For a symmetric quadrotor it is diagonal, I = diag(I_xx, I_yy, I_zz).
- **τ** = (τ_x, τ_y, τ_z)ᵀ, the body torques about the body x, y, z axes, produced by the four rotor thrusts through entry 01's matrix **M**.
- **R(q)**, the rotation matrix converting body frame vectors to world frame, written explicitly below.
- **⊗**, quaternion multiplication, Hamilton convention.
---
 
## Conventions, fixed explicitly before any equation
 
Entry 00 committed us to the Hamilton quaternion convention with scalar part first, representing the body-to-world rotation. We restate it here because this entry is where it actually bites, and because a convention mismatch here is the single most common silent error in quadrotor simulation.
 
There are two independent binary choices, giving four combinations, of which only one is ours:
 
1. **Hamilton versus JPL.** These differ in the sign of the quaternion product, equivalently in whether $ij = k$ or $ij = -k$. We use **Hamilton**.
2. **Body-to-world versus world-to-body.** A quaternion can represent either direction of the same rotation. We use **body-to-world**, so that R(q) applied to a body vector yields a world vector, which is what entry 02 needs to rotate thrust.
The consequence that matters: **the rotation matrix and the kinematic equation must be a matched pair.** The R(q) written below and the $\dot{\mathbf{q}} = \tfrac{1}{2}\mathbf{q} \otimes [0, \boldsymbol{\omega}]$ written further down are consistent with each other *and with body-frame* ω. If ω were expressed in the world frame, the multiplication order would flip to $\tfrac{1}{2}[0, \boldsymbol{\omega}] \otimes \mathbf{q}$. Getting this wrong produces a simulator that integrates smoothly, keeps the quaternion normalised, and rotates the wrong way, which is precisely why it survives casual inspection. We therefore check the pairing numerically below rather than trusting it.
 
---
 
## What the quaternion represents (the concept, before the equations)
 
Before evolving the quaternion we state plainly what it is, because it is the one object here with no everyday analogue. Any orientation a rigid body can hold is reachable by a single rotation of some angle θ about some axis n̂. The quaternion packages that single axis and angle into four numbers: q₀ = cos(θ/2) carries the angle, and (q₁, q₂, q₃) = n̂ · sin(θ/2) carries the axis, scaled.
 
We use four numbers for a three degree of freedom quantity on purpose. Orientation genuinely has three degrees of freedom, and it is a mathematical fact that any three number representation of rotation must have a singularity somewhere, a configuration where the representation collapses and two axes fold together. For Euler angles that singularity is gimbal lock near ninety degrees of pitch. It is not an engineering flaw that can be tuned away; it is unavoidable for any three parameter representation. The quaternion escapes it by using four numbers with one constraint, the unit norm condition, which represents every rotation smoothly with no singularity anywhere. The unit norm constraint is the price; freedom from gimbal lock is what it buys.
 
This is precisely why we chose it for this project. We committed in entry 00 to aggressive, realistic trajectories with hard banks and steep climbs, which is exactly the regime that drives orientation toward the configurations where a three parameter representation fails. A gentle path would never approach gimbal lock and Euler angles would suffice, but our chosen flight envelope makes the quaternion a requirement rather than a convenience.
 
### The two consequences of using four numbers for three degrees of freedom
 
The extra number is not free, and it costs us in two specific ways that we must handle rather than ignore. Both will bite later, in the MPC cost and in the learned residual, so we deal with them here.
 
**Consequence 1: the unit norm must be actively maintained.** The kinematic equation below preserves the norm exactly in continuous time, but no finite-step numerical integrator preserves it exactly. Norm error accumulates, and a quaternion of norm 1.01 is no longer a rotation: R(q) built from it is no longer orthogonal, so it stretches vectors, and the thrust magnitude entering entry 02 becomes silently wrong by a percent. Our rule is therefore to **renormalise q after every integration step**, $\mathbf{q} \leftarrow \mathbf{q} / \|\mathbf{q}\|$, and to log the pre-normalisation drift as a health metric. If that drift ever grows rather than staying at the level of rounding, the integrator step is too large and the run is invalid.
 
**Consequence 2: double cover.** The quaternions **q** and **−q** represent the *same physical rotation*. The map from quaternions to orientations is two-to-one. This is not a curiosity; it has three concrete effects on our project:
 
- **Attitude error must not be computed as a plain difference.** For a desired attitude $\mathbf{q}_d$ and an actual attitude $\mathbf{q}$, the naive cost $\|\mathbf{q} - \mathbf{q}_d\|^2$ is wrong: it can be large when the two describe the *same* orientation with opposite signs, which would make the controller command a full rotation to fix an error that does not exist. The correct forms are either the sign-corrected error quaternion $\mathbf{q}_e = \mathbf{q}_d^{-1} \otimes \mathbf{q}$, with its vector part used as the error and its scalar part's sign used to pick the short way around, or the sign-invariant scalar cost $1 - |\mathbf{q}_d \cdot \mathbf{q}|$. We commit to using a sign-corrected error and will state which form the MPC cost uses when we build it.
- **The learned residual sees a discontinuous input unless we canonicalise.** If the quaternion is a feature of the network, a trajectory that passes through a sign flip presents the network with a jump in input while nothing physical happened. We therefore **canonicalise the sign**, enforcing q₀ ≥ 0 on every logged and every fed quaternion, so that the representation is single-valued wherever it can be.
- **Interpolation and averaging of quaternions require the same care**, which matters if we ever average across an ensemble or interpolate a reference attitude.
---
 
## The explicit rotation matrix R(q)
 
Entry 02 used R(q) as a black box, the object that rotates the body frame thrust into the world frame and whose third column is $\mathbf{b}_3$. Since R is built directly from the quaternion we are evolving here, we write it out now. For a unit quaternion q = (q₀, q₁, q₂, q₃):
 
$$
\mathbf{R}(\mathbf{q}) =
\begin{bmatrix}
1 - 2(q_2^2 + q_3^2) & 2(q_1 q_2 - q_0 q_3) & 2(q_1 q_3 + q_0 q_2) \\
2(q_1 q_2 + q_0 q_3) & 1 - 2(q_1^2 + q_3^2) & 2(q_2 q_3 - q_0 q_1) \\
2(q_1 q_3 - q_0 q_2) & 2(q_2 q_3 + q_0 q_1) & 1 - 2(q_1^2 + q_2^2)
\end{bmatrix}
$$
 
Reading off the third column gives the quantity entry 02 cares about most:
 
$$\mathbf{b}_3(\mathbf{q}) = \mathbf{R}(\mathbf{q})\mathbf{e}_3 = \begin{bmatrix} 2(q_1 q_3 + q_0 q_2) \\ 2(q_2 q_3 - q_0 q_1) \\ 1 - 2(q_1^2 + q_2^2) \end{bmatrix}$$
 
Note that $\mathbf{b}_3$ does not depend on rotation about the thrust axis itself, which is the algebraic statement of entry 02's observation that yaw is free as far as translation is concerned.
 
**Sanity check (identity).** For no rotation the quaternion is q = (1, 0, 0, 0). Substituting q₀ = 1 and q₁ = q₂ = q₃ = 0 makes every off diagonal term zero and every diagonal term 1 − 0 = 1, so R = the identity matrix. This is exactly what we expect: with no rotation, a body frame vector is unchanged in the world frame, which is why in entry 02's hover check $\mathbf{b}_3$ reduced to e₃ and the thrust pointed straight up.
 
**Sanity check (orthogonality).** For any unit quaternion, R(q) must satisfy $\mathbf{R}^{\top}\mathbf{R} = \mathbf{I}$ and $\det \mathbf{R} = +1$. These are the conditions that make it a rotation rather than a general linear map, and they are what guarantee that rotating the thrust preserves its magnitude. We verify both numerically as a standing test.
 
---
 
## Orientation kinematics (how the quaternion changes)
 
This is the rotational counterpart of entry 02's position kinematics, where position changed at the rate of the velocity. Here the orientation changes at a rate driven by the angular velocity. For body-frame ω and a body-to-world Hamilton quaternion, the standard result is:
 
$$\dot{\mathbf{q}} = \tfrac{1}{2}\, \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$
 
where the angular velocity ω is written as a quaternion with zero scalar part and multiplied by q using quaternion multiplication. The one half factor traces back to the half-angle in the quaternion's own definition, q₀ = cos(θ/2): the quaternion advances at half the rate of the rotation it encodes. The order of multiplication, q on the *left*, is what encodes that ω is a body-frame quantity; a world-frame ω would multiply from the other side. Together these two details make the update respect the unit norm constraint, keeping q a valid rotation as it evolves. Conceptually it plays the identical role to position changing at the rate of velocity; it simply lives in quaternion algebra because that is the language orientation is written in.
 
**Sanity check (frozen).** If the drone is not rotating, ω = 0, then the bracket is the zero quaternion and q̇ = 0, so the orientation stays frozen. This matches intuition: no rotation rate means no change in which way the drone points.
 
**Sanity check (norm preservation).** Because q̇ is orthogonal to q under this update, the derivative of $\|\mathbf{q}\|^2$ is zero: the norm is conserved in continuous time. This is the property that renormalisation is repairing numerically, not imposing artificially.
 
**Numerical convention check (the one that actually catches errors).** The two sanity checks above pass under *any* of the four convention combinations, so they cannot detect a convention mismatch. This one can. Start level, q = (1, 0, 0, 0), hold a constant body-frame ω = (0, 1, 0), meaning a pure positive pitch rate, integrate for θ seconds, and evaluate. The pairing above is correct if and only if
 
$$\mathbf{b}_3 \to \begin{bmatrix} \sin\theta \\ 0 \\ \cos\theta \end{bmatrix}, \qquad \mathbf{R}\mathbf{e}_1 \to \begin{bmatrix} \cos\theta \\ 0 \\ -\sin\theta \end{bmatrix}$$
 
that is, the thrust axis tilts toward positive world x while the nose drops, which is entry 02's nose-down positive pitch. **We have run this check and it passes**: at θ = 0.3 rad the integration gives a nose direction of (0.9553, 0, −0.2955) and a thrust direction of (0.2955, 0, 0.9553), matching cos and sin of 0.3 to integration tolerance, with orthogonality error at the level of machine epsilon. Any future edit to R(q), to the quaternion product, or to the multiplication order must re-pass this test.
 
---
 
## Rotational Newton's law (how the angular velocity changes)
 
This is the rotational counterpart of entry 02's core equation, but with two differences that make rotation genuinely richer than translation. The law, written in the **body frame**, is:
 
$$\mathbf{I}\, \dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{I}\, \boldsymbol{\omega})$$
 
or, solved for the rate we integrate:
 
$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1}\big(\boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{I}\, \boldsymbol{\omega})\big)$$
 
Three points deserve full attention here.
 
**Why this equation is written in the body frame at all.** This is the deeper reason behind entry 00's choice to keep ω body-frame, and it is worth stating because it is the whole justification for the extra cross-product term. The inertia tensor describes how mass is distributed relative to the airframe. In the body frame that distribution is fixed, so **I** is a matrix of constant numbers. In the world frame it is not: as the drone rotates, its mass distribution sweeps around, so the world-frame inertia is $\mathbf{R}\mathbf{I}\mathbf{R}^{\top}$, a time-varying matrix whose derivative would have to be carried through every equation. We choose the body frame to keep **I** constant. The cross-product term is precisely the price of that choice: it is what appears when Newton's law, which is only valid in an inertial frame, is transported into the rotating body frame.
 
**Inertia is a matrix, not a scalar.** Mass is a single number because a body resists linear acceleration equally in every direction. A drone, however, resists rotation differently about different axes: it is easier to spin about a slim axis than a wide one. So the rotational analogue of mass is a three by three matrix, the inertia tensor. For a symmetric quadrotor the off diagonal terms vanish and it reduces to three numbers, I = diag(I_xx, I_yy, I_zz), one resistance per axis. This is why the same torque produces different angular accelerations on different axes, and it compounds the asymmetry entry 01 found. Roll and pitch are driven strongly, through the effective arm $a = l/\sqrt2$, against the smaller in-plane inertias; yaw is driven weakly, through the much smaller constant $c_{\tau f}$, against I_zz, which for a flat quadrotor is the *largest* of the three, roughly the sum of the other two. Weak actuation against large inertia is why yaw is not merely the slowest axis but slow by a wide margin, and it is the first place we should expect a demanded correction to become unrealizable.
 
**The gyroscopic term has no translational analogue.** The extra term $-\boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega})$ is a coupling that appears only in rotation. When a body is already spinning, its rotation about one axis feeds into the others; this is why spinning objects wobble and precess. Translation has nothing like it, since velocities do not cross couple this way. For gentle flight this term is small and often ignored, but for the aggressive trajectories we target it is real, and keeping it is another reason we chose in entry 00 to model angular velocity honestly as a genuine state rather than assuming it away. Note that this term is *the airframe's own* gyroscopic coupling. The separate gyroscopic and spin-up torques from the spinning rotor discs were considered and excluded in entry 01, with reasons recorded there; the two are easy to conflate and are not the same thing.
 
**Sanity check (no spin).** If the drone is not spinning, ω = 0, the gyroscopic term vanishes and the law reduces to I·ω̇ = τ, the clean rotational Newton's law: torque produces angular acceleration inversely scaled by inertia, exactly parallel to force producing linear acceleration inversely scaled by mass. The coupling term only switches on once the drone is already rotating.
 
**Sanity check (energy, and why it is the sharp one).** With zero torque, the gyroscopic term does no work: taking the dot product of the equation with ω gives $\boldsymbol{\omega}^{\top}\mathbf{I}\dot{\boldsymbol{\omega}} = -\boldsymbol{\omega}^{\top}(\boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega}) = 0$, since a cross product is orthogonal to its own arguments. So rotational kinetic energy $\tfrac12 \boldsymbol{\omega}^{\top}\mathbf{I}\boldsymbol{\omega}$ and the world-frame angular momentum magnitude are both conserved in torque-free motion. This is a standing test for the simulator and it is the sharpest one available for this equation: **if the sign of the cross-product term is wrong, torque-free rotational energy will drift rather than hold**, and it will do so in a way that no visual inspection of a trajectory would reveal.
 
---
 
## The full equations of motion (all thirteen states)
 
With both halves in hand, the complete rate of change of the state is:
 
$$
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v} \\[4pt]
\dot{\mathbf{v}} &= \frac{T}{m}\, \mathbf{R}(\mathbf{q})\mathbf{e}_3 - g\,\mathbf{e}_3 \\[4pt]
\dot{\mathbf{q}} &= \tfrac{1}{2}\, \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} \\[4pt]
\dot{\boldsymbol{\omega}} &= \mathbf{I}^{-1}\big(\boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{I}\, \boldsymbol{\omega})\big)
\end{aligned}
$$
 
subject to the algebraic constraint $\|\mathbf{q}\| = 1$, and driven by inputs $(T, \boldsymbol{\tau}) = \mathbf{M}\mathbf{f}$ with the per-rotor admissible set $0 \le f_i \le f_{\max}$ from entry 01.
 
The first two lines evolve position and velocity in the world frame; the last two evolve orientation and angular velocity, with orientation carried by the quaternion and rotation resolved in the body frame. Everything on the right hand side is either a state or an input, which is exactly the property we need: given the current state and the four rotor thrusts, we can compute every rate and integrate the state forward.
 
**Reading the cascade, which is the point of the whole exercise.** Trace the path from what we command to lateral position, bottom line upward. The rotor thrusts set τ, which drives ω̇; ω drives q̇; q sets $\mathbf{b}_3$; $\mathbf{b}_3$ times T over m sets v̇; v sets ṗ. That is four links from τ to horizontal position, and two from T to vertical position, which is exactly the relative-degree ladder entry 02 tabulated, now visible as the literal structure of the equations rather than as a differentiation count. Every one of those four links carries its own bound: τ is confined to entry 01's polytope, ω is bounded by rate limits, tilt is bounded by the thrust-to-weight ratio through entry 02's $1/\cos\theta$ cost. **Realizability, when we come to define it formally, is the question of whether a requested correction survives the whole cascade within the horizon that matters, and this system of four lines is the object that question is asked about.**
 
---
 
## Standing numerical tests for the simulator
 
We collect the checks scattered above into the list the simulator entry must implement, since these are what will actually establish that the derivation was transcribed into code correctly. They are cheap and they catch the errors that reading cannot.
 
1. **R orthogonality**: $\|\mathbf{R}^{\top}\mathbf{R} - \mathbf{I}\|$ at machine epsilon, $\det\mathbf{R} = +1$, at every step.
2. **Quaternion norm drift** before renormalisation stays at rounding level and does not grow with time.
3. **Convention check**: constant body-rate about +y produces the nose-down tilt and thrust direction recorded above. Passed.
4. **Torque-free energy**: with τ = 0 and ω ≠ 0, both $\tfrac12\boldsymbol{\omega}^{\top}\mathbf{I}\boldsymbol{\omega}$ and $\|\mathbf{R}\mathbf{I}\boldsymbol{\omega}\|$ conserved to integrator tolerance. Catches a sign error in the gyroscopic term.
5. **Ballistic check**: with T = 0 and no drag, the trajectory is an exact parabola and total mechanical energy $\tfrac12 m\|\mathbf{v}\|^2 + mgz$ is conserved. Catches a sign or factor error in the translational equation.
6. **Hover fixed point**: level attitude with T = mg holds position indefinitely, with drift bounded by integrator tolerance.
7. **Integrator order**: halving the step reduces error by roughly sixteen, confirming genuine fourth-order accuracy and catching an accidental Euler step.
8. **Independent cross-check**: a second implementation using Euler-angle attitude must agree with this one in the small-angle regime. Disagreement localises the bug to one of the two.
---
 
## What this entry commits us to, and what is open
 
**Committed.** The full rigid body equations of motion for all thirteen states, comprising the translational pair from entry 02 and the rotational pair derived here, together with the explicit rotation matrix R(q) and its third column $\mathbf{b}_3$. Orientation is carried by a unit quaternion in the Hamilton, scalar-first, body-to-world convention, matched to a body-frame ω in the kinematic equation and verified numerically. Angular velocity is kept as a genuine state. The inertia is a constant matrix *because* the equation is written in the body frame, and the airframe gyroscopic coupling term is retained for aggressive flight. Quaternions are renormalised every step and sign-canonicalised to q₀ ≥ 0 wherever they are logged or fed to a learned model, and attitude error is always computed through a sign-corrected error quaternion, never a plain difference.
 
**Deliberately excluded from the nominal model.** Structural flex, battery and electrical dynamics, and propeller aerodynamics beyond the thrust and reaction torque of entry 01, including the rotor gyroscopic and rotor spin-up torques excluded there with reasons. These are either left in the gap the residual learns or deferred as future sim to real extensions.
 
**Open for the next entry (04, hover equilibrium and linearisation).** We now have the equations of motion. Entry 04 studies their equilibrium, the hover condition, working out what thrust and what rotor speeds hold the drone steady, and linearising about it to expose the local structure: which channels are fast, which are slow, and how much authority sits above hover in each. That linearisation is also where the relative-degree ladder acquires numbers rather than counts. This is the natural bridge from open loop physics toward the controller, since the controller's job is precisely to keep the drone near a desired condition despite the mismatches that push it off equilibrium.
 
*This entry is locked. We do not proceed to 04 until the following can be explained in plain language: why rotation mirrors translation as configuration plus rate, what the quaternion represents and why we chose it, what double cover is and the three places it would bite us, why the equation is written in the body frame and what the cross-product term is the price of, why inertia is a matrix rather than a scalar, why weak yaw actuation against a large I_zz makes yaw the first axis to run out, and what the four-link cascade from rotor thrust to lateral position implies for realizability.*