# 00: State Vector and Reference Frames

*Derivation notebook, entry 00. Part of the clean-room rebuild. Written as we reason through the design, not as a textbook. Every choice here is one we made deliberately and can defend.*

---

## Why we are asking this first

We are building a quadrotor that flies rich, realistic trajectories on its own (figure-eights, banks, climbs, varied speeds), steered by an MPC controller, with a learned residual model that suggests corrections and a supervisor that decides how much of each correction to trust. Before we can control the drone, simulate it, or correct it, we have to answer the most basic question there is: **what numbers fully describe the drone at a single instant?** Everything downstream (the dynamics, the MPC's internal model, the learned residual, the realizability analysis, the trust policy) is built on top of this state. If we get the state wrong or under-specify it, every later stage inherits the error. So we start here, and we do not move past it until we can justify every number.

The test we hold ourselves to for "fully describe" is a prediction test: **the state is the smallest set of quantities such that, given the forces and torques acting on the drone, we could predict its next instant.** If two situations have identical state but different futures, we are missing a state.

---

## Modelling assumptions made in this entry

Three assumptions sit underneath everything below. We state them once so that later entries can refer to them.

1. **Rigid body.** The airframe does not flex, and its mass distribution is fixed relative to the body. The spinning rotors are treated as sources of force and torque, not as separate bodies. The angular momentum of the rotors themselves is neglected in the base model and is listed as an excluded effect in entry 01.
2. **Body origin at the centre of mass.** The position we track is the position of the centre of mass, and the body frame has its origin there. This is what allows translation and rotation to be written as two separate laws in entries 02 and 03.
3. **Flat, non-rotating Earth.** The world frame is treated as inertial, and gravity is a constant vector. For a vehicle that flies a few metres for a few minutes, this is exact to well below any effect we study.

---

## The frame convention, fixed once and for all

Before any state or equation, we fix the convention explicitly. This is not bookkeeping. The quadrotor literature is split roughly evenly between z-up and z-down conventions, and importing a formula written in the other convention produces a sign error that is silent, physically plausible for a few seconds of simulation, and extremely expensive to find. We therefore state our convention here, in entry 00, and every later entry inherits it without restating it.

**World frame (inertial, fixed to the ground), right-handed, z-up:**
- world x: an arbitrary fixed horizontal direction, taken as "forward" for the purposes of describing reference paths.
- world y: horizontal, completing the right-handed set.
- world z: **up**, away from the ground.

Consequently gravity points along **negative** world z. When entry 02 writes the gravitational force it will appear as a minus sign, and that minus sign traces back to this line. We write the world unit vectors as $\mathbf{e}_1, \mathbf{e}_2, \mathbf{e}_3$.

**Body frame (fixed to the airframe, translating and rotating with it), right-handed, z-up:**
- body x: forward, out the nose of the drone.
- body y: to the drone's left.
- body z: **up**, out the top of the airframe, which is the direction the propellers push.

This is the FLU (forward-left-up) body convention paired with a z-up world frame of the ENU (east-north-up) type. Our world axes carry no geographic meaning; only the handedness and the up direction matter. Thrust is always along **positive** body z. We write the body unit vectors, expressed in the world frame, as $\mathbf{b}_1, \mathbf{b}_2, \mathbf{b}_3$.

Note that this is *not* the NED / FRD convention used by PX4, ArduPilot, and a large fraction of the aerospace literature, in which z points down and thrust is negative. Any equation or dataset we import from those sources must be converted before use, and we will flag each such conversion where it happens.

**Rotation sense.** All rotations follow the right-hand rule about the body axes: point the right thumb along the axis, and the fingers curl in the positive direction. This fixes the sign of each angle, and the signs are not all the ones everyday language suggests.

| Rotation | Axis | Positive sense | Effect on the thrust axis $\mathbf{b}_3$ | Resulting acceleration |
|---|---|---|---|---|
| Roll $\phi$ | body x | left side up, right side down | tilts toward **negative** world y | toward negative y (to the right) |
| Pitch $\theta$ | body y | **nose down** | tilts toward **positive** world x | toward positive x (forward) |
| Yaw $\psi$ | body z | nose to the left, counter-clockwise seen from above | unchanged | none |

Each row can be checked from the elementary rotation matrices. Starting from level and rotating by a single angle:

$$
R_x(\phi)\,\mathbf{e}_3 = \begin{bmatrix} 0 \\ -\sin\phi \\ \cos\phi \end{bmatrix}, \qquad
R_y(\theta)\,\mathbf{e}_3 = \begin{bmatrix} \sin\theta \\ 0 \\ \cos\theta \end{bmatrix}, \qquad
R_z(\psi)\,\mathbf{e}_1 = \begin{bmatrix} \cos\psi \\ \sin\psi \\ 0 \end{bmatrix}.
$$

The middle expression is the one that causes trouble. In our convention **positive pitch is nose down, and a drone accelerating forward is at positive pitch.** This is the opposite of the NED / FRD convention, where the y axis points to the right and positive pitch is nose up. The physical motion is the same; only the sign of the number differs, because the pitch axis points the other way. Everyday phrasing ("pitch up") and most aerospace textbooks follow the nose-up sign, so any pitch value we import, plot, or compare against another source must have its convention checked first. Entries 02, 03 and 04 all rely on the nose-down sign, and entry 03 verifies it numerically.

**Units.** SI throughout: metres, seconds, kilograms, radians. Angles are radians everywhere internally; degrees appear only in plots and prose.

---

## The reasoning we followed

**Step 1, where it is and how it is moving (translation).** We clearly need the drone's position in space, so we take position (x, y, z). But position alone fails the prediction test: a drone sitting at a point and a drone rushing through that same point have identical position and completely different next instants. What separates them is how fast they are translating, the linear velocity. So translation needs both a configuration (position) and its rate (velocity), giving 6 numbers so far.

**Step 2, which way it points (orientation).** A quadrotor is not a point; it is a rigid body with an orientation, and orientation matters enormously because thrust always points along the drone's own body z axis. Two drones at the same position, same velocity, but tilted differently will accelerate in different directions on the next instant. So orientation is part of the state.

**Step 3, how fast it is rotating (angular velocity).** Applying the same prediction test to orientation: a drone pointing straight up but spinning fast, and an identical drone pointing straight up but not spinning, have the same orientation now and different orientations a moment later. The thing that distinguishes them is the angular velocity. So, exactly as translation needed both position and velocity, rotation needs both orientation and angular velocity.

This gives us the organizing principle we will rely on throughout the project: **the state pairs every configuration with its rate of change.** Position with linear velocity; orientation with angular velocity. We need both halves of each pair, because the forces and torques set the *accelerations*, and predicting the next instant means integrating twice: acceleration into rate, and rate into configuration.

**Step 4, deciding how to represent orientation.** Orientation has three degrees of freedom, and the obvious choice is to store it as three Euler angles (roll, pitch, yaw). We rejected this for our project. Any three-number description of orientation has a singularity somewhere. For the usual yaw-pitch-roll sequence it is gimbal lock: as the pitch angle approaches 90 degrees, the roll and yaw axes align, one degree of freedom is lost, and the angle rates become unbounded. For a gentle circle this never happens, but we specifically want aggressive trajectories with hard banks and steep climbs, which is exactly the regime where Euler angles fail.

We therefore represent orientation with a **unit quaternion** $\mathbf{q} = (q_0, q_1, q_2, q_3)$: four numbers, subject to the constraint $\|\mathbf{q}\| = 1$, encoding the same three rotational degrees of freedom without any singularity. This moves our count from 12 numbers to 13.

Two consequences of using four numbers for three degrees of freedom need to be stated now, because they affect how we count and how we compare orientations. Entry 03 treats both in detail.

- **The state has 12 degrees of freedom, stored in 13 numbers.** The unit-norm constraint removes one. The quaternion is therefore not a free vector in four dimensions, and any numerical step that treats it as one (adding two quaternions, integrating without renormalizing, subtracting them to form an error) leaves the set of valid orientations.
- **$\mathbf{q}$ and $-\mathbf{q}$ are the same orientation.** An orientation error must therefore never be computed as a plain difference of quaternion components. This matters for the MPC cost in entry 06 and for the tracking error used by the supervisor.

We fix the quaternion convention here as well, since it is the other classic silent-error source. We use the **Hamilton** convention with the **scalar part first**, and the quaternion represents the rotation that maps **body-frame vectors into world-frame vectors**. A rotation by angle $\vartheta$ about a unit axis $\mathbf{n}$ is

$$
\mathbf{q} = \left( \cos\tfrac{\vartheta}{2},\; \mathbf{n}\,\sin\tfrac{\vartheta}{2} \right), \qquad
\mathbf{a}^{W} = \mathbf{q} \otimes \mathbf{a}^{B} \otimes \mathbf{q}^{*} = R(\mathbf{q})\,\mathbf{a}^{B},
$$

where $\mathbf{a}^{B}$ and $\mathbf{a}^{W}$ are the same physical vector written in the body and world frames. Level flight with the nose along world x is $\mathbf{q} = (1, 0, 0, 0)$. Entry 03 writes out $R(\mathbf{q})$ and the kinematic equation, and both must be consistent with this sentence. (The alternative JPL convention, and the alternative world-to-body direction, each flip signs in the kinematics; mixing conventions produces a simulator that integrates smoothly and rotates the wrong way.)

**Step 5, deciding the fidelity of the rotational dynamics.** A common simplification assumes a "perfect inner loop," meaning that whatever body rate the controller commands, the drone achieves it instantly. Under that assumption angular velocity is no longer a state but an input, dropping the count to ten. We deliberately rejected this simplification, for two reasons.

First, it hides the effects our research is about. Motor lag and actuation delay act between the commanded rotor thrusts and the delivered ones, and their damage is done through the rotational dynamics: a late torque becomes a late change in angular velocity, then a late tilt, then a late lateral acceleration. A model in which body rates are achieved instantly has no place for that chain.

Second, our realizability analysis is about what the four rotors can deliver under their thrust and rate limits. That question can only be asked of a model whose inputs are the rotor thrusts themselves. We therefore keep angular velocity as a genuine state, model the rotational dynamics in full, and take the four rotor thrusts as the control input, defined in entry 01.

**Step 6, what we deliberately did NOT add (and why).** We considered enlarging the state further, and made two explicit decisions.

- **Actuator dynamics (delivered rotor thrusts, delayed commands).** These are real states: the delivered thrust at one instant depends on what was commanded earlier, which is memory in exactly the sense of the prediction test. We keep them out of the *base* rigid-body state and add them as a separate actuator state when we model motor lag and delay. The full plant state is then the pair of the rigid-body state defined here and the actuator state, as in the research proposal. Keeping the two separate is deliberate: the nominal MPC model uses only the rigid-body state with ideal actuators, and the actuator state is precisely what it does not know about.
- **Sensor biases and estimator states (for example Extended Kalman Filter states).** These belong to *state estimation*, meaning recovering the state from noisy sensors. Our simulator provides the true state directly; we have no sensors, no measurement noise, and therefore no estimation problem. Adding these would be modelling a problem we do not have. We exclude them, and note sensor noise and estimation as a requirement of the later hardware phase only.

Following this reasoning, we finalized a **13-number rigid-body state**, with the actuator state as a planned, separate addition.

---

## The finalized state

$$
\mathbf{x} = (\mathbf{p},\ \mathbf{v},\ \mathbf{q},\ \boldsymbol{\omega}) \in \mathbb{R}^3 \times \mathbb{R}^3 \times S^3 \times \mathbb{R}^3
$$

| # | Quantity | Symbol | What it means | Frame |
|---|----------|--------|---------------|-------|
| 1 to 3 | Position | $\mathbf{p} = (x, y, z)$ | where the centre of mass is | **World** |
| 4 to 6 | Linear velocity | $\mathbf{v} = (v_x, v_y, v_z)$ | how fast the centre of mass is translating | **World** |
| 7 to 10 | Orientation (unit quaternion) | $\mathbf{q} = (q_0, q_1, q_2, q_3)$ | which way the drone points | **Body to world rotation** |
| 11 to 13 | Angular velocity | $\boldsymbol{\omega} = (\omega_x, \omega_y, \omega_z)$ | how fast the body rotates relative to the world, measured about its own axes | **Body** |

Here $S^3$ denotes the set of unit quaternions. The state is stored as 13 numbers and has 12 degrees of freedom.

**A note on symbols.** Three collisions are possible, and we resolve each one here.

- **Angular velocity.** The aerospace convention writes body angular velocity as (p, q, r). We do not use it, because $\mathbf{p}$ is already the position vector and $\mathbf{q}$ is already the quaternion. We write the components as $(\omega_x, \omega_y, \omega_z)$, the rotation rates about body x, body y and body z.
- **Roll, pitch and yaw rates.** Where prose says "roll rate," "pitch rate," and "yaw rate," it means $\omega_x$, $\omega_y$ and $\omega_z$. These equal the time derivatives of the roll, pitch and yaw angles only near level flight. Away from level flight the two differ, which is one more reason we do not use Euler angles as states.
- **The state vector and the x coordinate.** Bold $\mathbf{x}$ is the full state vector, as in the research proposal. Italic $x$ is the first component of position. The two never appear in the same role.

---

## Why each quantity lives in the frame it does

We use two frames throughout, the fixed **world frame** and the **body frame** attached to the drone, both defined above. Choosing the right frame for each quantity is not cosmetic; it determines how cleanly the dynamics are written, and getting a frame wrong is a classic source of subtle bugs.

- **Position, world.** "Where am I" only has meaning relative to a fixed ground reference. In the body frame the drone is always at its own origin, so body-frame position carries no information. Position is inherently world-frame.
- **Linear velocity, world.** This was a genuine choice, not an automatic one. Velocity can be expressed in the world frame ("moving 2 m/s along world x, regardless of which way the nose points") or in the body frame ("moving 2 m/s forward relative to the nose"). We chose the world frame for three connected reasons. First, the reference paths are specified in world coordinates, so the tracking error is naturally a world-frame quantity. Second, the two dominant forces live in different frames: gravity always points along negative world z, while thrust always points along positive body z, and to sum them we must express both in one common frame. We adopt the world frame as that common frame and rotate the body-frame thrust into it using the orientation. Third, with world-frame velocity the position kinematics are simply $\dot{\mathbf{p}} = \mathbf{v}$ and Newton's law carries no extra term. Body-frame velocity would add a $\boldsymbol{\omega} \times \mathbf{v}$ term to the translational equation, because the frame in which the velocity is written is itself rotating.
- **One consequence to carry forward.** Aerodynamic drag on a quadrotor is most naturally described in the body frame, since it acts on the rotors and the airframe. When we add drag as a mismatch, the velocity will be rotated into the body frame as $R(\mathbf{q})^{\top}\mathbf{v}$, the drag computed there, and the result rotated back. This is a cost of the world-frame choice, and we accept it because drag is a perturbation and gravity and thrust are the baseline.
- **Orientation, the bridge between frames.** The quaternion is not "in" a frame; it *is* the rotation that converts body-frame vectors into world-frame vectors (and, inverted, back again). It is precisely the object we use above to rotate body-frame thrust into the world frame. This is why orientation sits at the centre of the dynamics: every coupling between how the drone points and how it moves passes through it.
- **Angular velocity, body.** Rotation is naturally measured about the drone's own axes: a gyroscope reads rotation about the body frame, and the torques the four rotors produce are simplest when written about the body axes. There is a second, deeper reason that entry 03 will make explicit: the inertia tensor is constant only when expressed in the body frame, since the mass distribution is fixed relative to the airframe but sweeps around in the world frame as the drone rotates. Writing the rotational dynamics in the body frame keeps that matrix a set of fixed numbers. The price is the gyroscopic term $\boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega})$ in entry 03, which is the rotational counterpart of the term we avoided for translation by choosing the world frame.

---

## Matching the simulator and the vehicle

The conventions above were chosen on their merits, but they must also match the tools we use, and a mismatch here would be the silent kind. We record the expected correspondence now and verify it in Phase 2 before trusting it.

- **MuJoCo.** A free body in MuJoCo is expected to store position in the world frame, orientation as a scalar-first unit quaternion, linear velocity in the world frame, and angular velocity in the body frame, with world z up. If that holds, our state maps onto the simulator state with no conversion. The Phase 2 validation must confirm each of the four, in particular the frame of the angular velocity, with a test that would fail under the wrong assumption.
- **Crazyflie.** The vehicle we model uses a forward-left-up body frame, which matches ours. Its firmware and logging interfaces, however, have historically reported the pitch angle with the nose-up sign, which is the opposite of the right-hand rule in that frame. Any pitch value that comes from Crazyflie firmware, logs, or published Crazyflie parameters must have its sign checked before use.

---

## What this entry commits us to, and what remains open

**Committed:**
- A rigid body with the body origin at the centre of mass, in a flat, non-rotating world.
- A right-handed, z-up world frame with gravity along negative world z, and a right-handed, FLU body frame with thrust along positive body z.
- Right-hand rule for all rotations. Positive roll is left side up and tilts thrust toward negative y. **Positive pitch is nose down** and tilts thrust toward positive x. Positive yaw is counter-clockwise seen from above.
- SI units, radians internally.
- A rigid-body state $\mathbf{x} = (\mathbf{p}, \mathbf{v}, \mathbf{q}, \boldsymbol{\omega})$ of 13 numbers and 12 degrees of freedom: world-frame position and velocity, a unit-quaternion orientation, and body-frame angular velocity, with angular velocity kept as a real state (no perfect-inner-loop shortcut).
- Hamilton quaternion convention, scalar part first, representing the body-to-world rotation, with $\mathbf{q}$ and $-\mathbf{q}$ denoting the same orientation.
- Body angular velocity written as $(\omega_x, \omega_y, \omega_z)$, not (p, q, r).

**Deferred, noted, not forgotten:**
- The actuator state (delivered rotor thrusts and delayed commands) is added as a separate state when we model motor lag and delay.
- Sensor noise and estimator states are out of scope for the simulation study and return in the hardware phase.
- The simulator and vehicle conventions listed above are expectations until Phase 2 verifies them.

**Open for the next entry (01, forces and torques):** we established that thrust lives in the body frame and gravity in the world frame, connected by the orientation. Entry 01 makes this concrete: what forces and torques act on the drone, how the four rotors produce them, and what limits the rotors impose.

*This entry is locked. We do not proceed to 01 until every choice above can be explained in plain language, in particular: why the frame and quaternion conventions had to be fixed before any equation was written, why positive pitch is nose down in our frame while it is nose up in the NED / FRD convention, and why the state has 12 degrees of freedom although it is stored in 13 numbers.*