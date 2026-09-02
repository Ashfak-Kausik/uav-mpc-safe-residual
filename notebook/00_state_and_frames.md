# 00 — State Vector and Reference Frames

*Derivation notebook, entry 00. Part of the clean-room rebuild. Written as we reason through the design, not as a textbook — every choice here is one we made deliberately and can defend.*

---

## Why we are asking this first

We are building a quadrotor that flies rich, realistic trajectories on its own — figure-eights, banks, climbs, varied speeds — steered by an MPC controller, with a learned "helper" that suggests corrections and a supervisor that decides when to trust it. Before we can control the drone, simulate it, or correct it, we have to answer the most basic question there is: **what numbers fully describe the drone at a single instant?** Everything downstream — the dynamics, the MPC's internal model, the learned residual, the realizability analysis — is built on top of this state. If we get the state wrong or under-specify it, every later stage inherits the error. So we start here, and we do not move past it until we can justify every number.

The test we hold ourselves to for "fully describe" is a prediction test: **the state is the minimal set of numbers such that, given the forces and torques acting on the drone, we could predict its next instant.** If two situations have identical state but different futures, we are missing a state.

---

## The reasoning we followed

**Step 1 — where it is and how it is moving (translation).** We clearly need the drone's position in space, so we take position (x, y, z). But position alone fails the prediction test: a drone sitting at a point and a drone rushing through that same point have identical position and completely different next instants. What separates them is how fast they are translating — the linear velocity. So translation needs both a configuration (position) and its rate (velocity): 6 numbers so far.

**Step 2 — which way it points (orientation).** A quadrotor is not a point; it is a rigid body with an orientation, and orientation matters enormously because thrust always points along the drone's own "up" axis. Two drones at the same position, same velocity, but tilted differently will accelerate in different directions on the next instant. So orientation is part of the state.

**Step 3 — how fast it is rotating (angular velocity).** Applying the same prediction test to orientation: a drone pointing straight up but spinning fast, and an identical drone pointing straight up but not spinning, have the same orientation now and different orientations a moment later. The thing that distinguishes them is the angular velocity — the rate of change of orientation. So, exactly as translation needed both position and velocity, rotation needs both orientation and angular velocity.

This gives us the organizing principle we will rely on throughout the project: **the state pairs every configuration with its rate of change.** Position ↔ linear velocity; orientation ↔ angular velocity. We need both halves of each pair, because predicting the next instant means integrating rates forward, and we cannot integrate a configuration whose rate we do not have.

**Step 4 — deciding how to represent orientation.** Orientation has three degrees of freedom (roll, pitch, yaw), and the obvious choice is to store it as three Euler angles. We rejected this for our project. Euler angles suffer a singularity called gimbal lock: as the drone pitches toward 90° (straight up), two of the rotation axes align and the representation breaks down mathematically. For a gentle circle this never happens, but we specifically want aggressive, realistic trajectories — hard banks and steep climbs — which is exactly the regime where Euler angles fail. We therefore represent orientation with a **unit quaternion** (q₀, q₁, q₂, q₃): four numbers, subject to a unit-norm constraint, encoding the same three rotational degrees of freedom without any singularity. This costs us one extra number but buys us a representation that never breaks, which is the correct trade for agile flight. This decision moves our count from 12 to 13.

**Step 5 — deciding the fidelity of the rotational dynamics.** A common simplification assumes a "perfect inner loop": that whatever body-rate the controller commands, the drone achieves it instantly. Under that assumption angular velocity is no longer a state but an input, dropping the count to nine or ten. We deliberately rejected this simplification. Assuming instantaneous rotation hides precisely the effects our research is about — most directly motor lag, which is *defined* by the drone not instantly achieving a commanded rotation. Since aggressive trajectories depend heavily on how fast the drone can actually rotate, and since our learned helper exists to correct exactly these rotational and dynamic shortfalls, we keep angular velocity as a genuine state and model the rotational dynamics honestly.

**Step 6 — what we deliberately did NOT add (and why).** We considered enlarging the state further to match real-life conditions, and made two explicit decisions:
- **Actuator/motor dynamics (e.g. motor lag as a state):** relevant and honest, but we add it *when we model that specific mismatch* (in the perturbation phase), not preemptively in the base drone. Our rule is to add a state only when the system has memory that affects the future *and* we have chosen to model that effect. The base drone does not yet include it; the note is carried forward.
- **Sensor biases / estimator states (e.g. Extended Kalman Filter states):** these belong to *state estimation* — recovering the state from noisy sensors. Our simulator provides the true state directly; we have no sensors, no measurement noise, and therefore no estimation problem. Adding these would be modeling a problem we do not have and drifting toward a different (perception/navigation) project. We exclude them, and note sensor-noise/estimation as a possible future extension only.

Following this reasoning, we finalized a **13-state base drone**, with two effects noted as deferred additions rather than baked in now.

---

## The finalized state (13 states)

| # | Quantity | Symbol | What it means | Natural frame |
|---|----------|--------|---------------|---------------|
| 1–3 | Position | (x, y, z) | where the drone is in space | **World** |
| 4–6 | Linear velocity | (vₓ, v_y, v_z) | how fast it is translating | **World** |
| 7–10 | Orientation (unit quaternion) | (q₀, q₁, q₂, q₃) | which way the drone points | **World ↔ Body relationship** |
| 11–13 | Angular velocity | (p, q, r) | how fast it is rotating about its own axes | **Body** |

---

## Why each quantity lives in the frame it does

We use two frames throughout: a fixed **world frame** (attached to the ground) and a **body frame** (attached to the drone, moving and rotating with it). Choosing the right frame for each quantity is not cosmetic — it determines how cleanly the dynamics are written, and getting a frame wrong is a classic source of subtle bugs.

- **Position → world.** "Where am I" only has meaning relative to a fixed ground reference. In the body frame the drone is always at its own origin, so body-frame position carries no information. Position is inherently world-frame.

- **Linear velocity → world.** This was a genuine choice, not an automatic one. Velocity can be expressed in the world frame ("moving 2 m/s North, regardless of heading") or the body frame ("moving 2 m/s forward relative to the nose"). We chose the world frame for two connected reasons. First, the reference paths we want the drone to follow are specified in world coordinates, so the tracking error is naturally a world-frame quantity. Second, the two dominant forces live in different frames — gravity always points down in the world frame, while thrust always points along the body "up" axis — and to sum them via "acceleration = total force / mass" we must express both in one common frame. We adopt the world frame as that common frame and rotate the body-frame thrust into it using the orientation. This makes the orientation the hinge that connects the body-frame forces to the world-frame motion.

- **Orientation → the bridge between frames.** The quaternion is not "in" a frame; it *is* the rotation that converts body-frame vectors into world-frame vectors (and back). It is precisely the object we use in the previous point to rotate body-frame thrust into the world frame. This is why orientation sits at the center of the dynamics: every coupling between how the drone points and how it moves passes through it.

- **Angular velocity → body.** Rotation is naturally measured about the drone's own axes: a gyroscope reads rotation about the body frame, and the rotational dynamics (the torques the four motors produce) are cleanest when written about the body axes. So angular velocity is inherently a body-frame quantity.

---

## What this entry commits us to, and what remains open

**Committed:** a 13-state base drone — world-frame position and velocity, a unit-quaternion orientation, and body-frame angular velocity — with angular velocity kept as a real state (no perfect-inner-loop shortcut).

**Deferred, noted, not forgotten:**
- Motor/actuator dynamics (e.g. motor lag) will be added as extra state(s) when we model that mismatch, in the perturbation phase.
- Sensor noise and estimator states are out of scope for this paper; a possible future extension only.

**Open for the next entry (01 — forces and torques):** we established that thrust lives in the body frame and gravity in the world frame, connected by the orientation. Entry 01 makes this concrete: what forces and torques act on the drone, how the four motors produce them, and how the quaternion rotates body-frame thrust into the world frame so we can finally write the equations of motion.

*This entry is locked. We do not proceed to 01 until every choice above can be explained in plain language.*