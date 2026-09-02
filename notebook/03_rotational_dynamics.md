# 03 — Rotational Dynamics

*Derivation notebook, entry 03. Continues the clean-room rebuild from entries 00 to 02. This entry completes the equations of motion by deriving how the drone's orientation and rotation change over time, and writes out the rotation matrix that entry 02 left symbolic.*

---

## What this entry is for

Entry 02 gave us the translational half of the equations of motion: how position and velocity change. This entry gives the rotational half: how the orientation quaternion and the body angular velocity change. Together the two halves are the full rate of change of all thirteen state numbers, which is everything we need to predict the drone's motion forward in time.

The structure of rotation mirrors translation exactly, which is worth keeping in mind because it makes the whole entry feel familiar. In translation we had a configuration (position) whose rate is the velocity, and a velocity whose rate comes from Newton's law with force and mass. In rotation we have a configuration (orientation) whose rate comes from the angular velocity, and an angular velocity whose rate comes from a rotational version of Newton's law with torque and inertia. The correspondence is: force becomes torque, mass becomes inertia, and linear velocity becomes angular velocity.

Two honest notes on how we handle this entry. First, the rotational equations are standard, well established results. Deriving quaternion kinematics from scratch is a long algebraic exercise that teaches little beyond the algebra itself, so we state the standard results, explain what every term means, and verify each with a sanity check, which is exactly how these results are used and defended in practice. Second, we deliberately keep the model a rigid body: no structural flex, no battery or electrical dynamics, no propeller aerodynamics beyond thrust and reaction torque. Those effects are either left in the gap the residual learns or deferred as future sim to real extensions; they do not enter the nominal model.

---

## Notation (added to entries 00 to 02)

- **q** = (q₀, q₁, q₂, q₃), the unit quaternion representing orientation, with q₀² + q₁² + q₂² + q₃² = 1.
- **ω** = (p, q, r)ᵀ, the body frame angular velocity (rate of rotation about the drone's own axes).
- **I**, the inertia tensor, a 3-by-3 matrix. For a symmetric quadrotor it is diagonal, I = diag(Iₓₓ, I_yy, I_zz).
- **τ** = (τ_roll, τ_pitch, τ_yaw)ᵀ, the body torques produced by the motors (from entry 01's motor to wrench map).
- **R(q)**, the rotation matrix converting body frame vectors to world frame, written explicitly below.
- **⊗**, quaternion multiplication.

---

## What the quaternion represents (the concept, before the equations)

Before evolving the quaternion we state plainly what it is, because it is the one object here with no everyday analogue. Any orientation a rigid body can hold is reachable by a single rotation of some angle θ about some axis n̂. The quaternion packages that single axis and angle into four numbers: q₀ = cos(θ/2) carries the angle, and (q₁, q₂, q₃) = n̂ · sin(θ/2) carries the axis, scaled.

We use four numbers for a three degree of freedom quantity on purpose. Orientation genuinely has three degrees of freedom, and it is a mathematical fact that any three number representation of rotation must have a singularity somewhere, a configuration where the representation collapses and two axes fold together. For Euler angles that singularity is gimbal lock near ninety degrees of pitch. It is not an engineering flaw that can be tuned away; it is unavoidable for any three parameter representation. The quaternion escapes it by using four numbers with one constraint, the unit norm condition, which represents every rotation smoothly with no singularity anywhere. The unit norm constraint is the price; freedom from gimbal lock is what it buys.

This is precisely why we chose it for this project. We committed in entry 00 to aggressive, realistic trajectories with hard banks and steep climbs, which is exactly the regime that drives orientation toward the configurations where a three parameter representation fails. A gentle path would never approach gimbal lock and Euler angles would suffice, but our chosen flight envelope makes the quaternion a requirement rather than a convenience.

---

## The explicit rotation matrix R(q)

Entry 02 used R(q) as a black box, the object that rotates the body frame thrust into the world frame. Since R is built directly from the quaternion we are evolving here, we write it out now. For a unit quaternion q = (q₀, q₁, q₂, q₃):

$$
\mathbf{R}(\mathbf{q}) =
\begin{bmatrix}
1 - 2(q_2^2 + q_3^2) & 2(q_1 q_2 - q_0 q_3) & 2(q_1 q_3 + q_0 q_2) \\
2(q_1 q_2 + q_0 q_3) & 1 - 2(q_1^2 + q_3^2) & 2(q_2 q_3 - q_0 q_1) \\
2(q_1 q_3 - q_0 q_2) & 2(q_2 q_3 + q_0 q_1) & 1 - 2(q_1^2 + q_2^2)
\end{bmatrix}
$$

**Sanity check.** For no rotation the quaternion is q = (1, 0, 0, 0). Substituting q₀ = 1 and q₁ = q₂ = q₃ = 0 makes every off diagonal term zero and every diagonal term 1 − 0 = 1, so R = the identity matrix. This is exactly what we expect: with no rotation, a body frame vector is unchanged in the world frame, which is why in entry 02's hover check R(q)·e₃ reduced to e₃ and the thrust pointed straight up.

---

## Orientation kinematics (how the quaternion changes)

This is the rotational counterpart of entry 02's position kinematics, where position changed at the rate of the velocity. Here the orientation changes at a rate driven by the angular velocity. The standard result is:

$$\dot{\mathbf{q}} = \tfrac{1}{2}\, \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$

where the angular velocity ω is written as a quaternion with zero scalar part and multiplied by q using quaternion multiplication. The one half factor and the quaternion multiplication are precisely what make this update respect the unit norm constraint, keeping q a valid rotation as it evolves. Conceptually it plays the identical role to position changing at the rate of velocity; it simply lives in quaternion algebra because that is the language orientation is written in.

**Sanity check.** If the drone is not rotating, ω = 0, then the bracket is the zero quaternion and q̇ = 0, so the orientation stays frozen. This matches intuition: no rotation rate means no change in which way the drone points.

---

## Rotational Newton's law (how the angular velocity changes)

This is the rotational counterpart of entry 02's core equation v̇ = F/m, but with two differences that make rotation genuinely richer than translation. The law is:

$$\mathbf{I}\, \dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{I}\, \boldsymbol{\omega})$$

or, solved for the rate we integrate:

$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1}\big(\boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{I}\, \boldsymbol{\omega})\big)$$

Two points deserve full attention here.

**Inertia is a matrix, not a scalar.** Mass is a single number because a body resists linear acceleration equally in every direction. A drone, however, resists rotation differently about different axes: it is easier to spin about a slim axis than a wide one. So the rotational analogue of mass is a three by three matrix, the inertia tensor. For a symmetric quadrotor the off diagonal terms vanish and it reduces to three numbers, I = diag(Iₓₓ, I_yy, I_zz), one resistance per axis. This is why the same torque produces different angular accelerations on different axes, and it connects back to entry 01: roll and pitch, driven strongly through the arm length, act on axes with their own inertias, while yaw, driven weakly by reaction torque, acts on yet another.

**The gyroscopic term has no translational analogue.** The extra term −ω × (Iω) is a coupling that appears only in rotation. When a body is already spinning, its rotation about one axis feeds into the others; this is why spinning objects wobble and precess. Translation has nothing like it, since velocities do not cross couple this way. For gentle flight this term is small and often ignored, but for the aggressive trajectories we target it is real, and keeping it is another reason we chose in entry 00 to model angular velocity honestly as a genuine state rather than assuming it away.

**Sanity check.** If the drone is not spinning, ω = 0, the gyroscopic term vanishes and the law reduces to I·ω̇ = τ, the clean rotational Newton's law: torque produces angular acceleration inversely scaled by inertia, exactly parallel to force producing linear acceleration inversely scaled by mass. The coupling term only switches on once the drone is already rotating.

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

The first two lines evolve position and velocity in the world frame; the last two evolve orientation and angular velocity, with orientation carried by the quaternion and rotation resolved in the body frame. The inputs entering these equations are the total thrust T and the body torques τ, both produced from the four motor thrusts by entry 01's motor to wrench map. Everything on the right hand side is either a state or an input, which is exactly the property we need: given the current state and the motor commands, we can compute every rate and integrate the state forward.

---

## What this entry commits us to, and what is open

**Committed.** The full rigid body equations of motion for all thirteen states, comprising the translational pair from entry 02 and the rotational pair derived here, together with the explicit rotation matrix R(q). Orientation is carried by a unit quaternion for the singularity free reasons above, angular velocity is kept as a genuine state, and the inertia is a matrix with a gyroscopic coupling term retained for aggressive flight.

**Deliberately excluded from the nominal model.** Structural flex, battery and electrical dynamics, and propeller aerodynamics beyond thrust and reaction torque. These are either left in the gap the residual learns or deferred as future sim to real extensions.

**Open for the next entry (04, hover equilibrium).** We now have the equations of motion. Entry 04 studies their equilibrium, the hover condition, working out what thrust and what motor speeds hold the drone steady, and using it to understand how mismatches such as a wrong mass or added drag push the drone off equilibrium. This is the natural bridge from open loop physics toward the controller, since the controller's job is precisely to keep the drone near a desired condition despite these pushes.

*This entry is locked. We do not proceed to 04 until the following can be explained in plain language: why rotation mirrors translation as configuration plus rate, what the quaternion represents and why we chose it, why inertia is a matrix rather than a scalar, and what the gyroscopic term is and when it matters.*