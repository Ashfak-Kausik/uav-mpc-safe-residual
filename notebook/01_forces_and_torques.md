# 01 — Forces and Torques

*Derivation notebook, entry 01. Continues the clean-room rebuild from entry 00. Written as we reason through the physics and decide what enters our model, not as a textbook.*

---

## What this entry is for

Entry 00 told us what numbers describe the drone at an instant. This entry tells us what changes those numbers. Everything that moves the drone is either a force, which changes how it translates (position and velocity), or a torque, which changes how it rotates (orientation and angular velocity). So our job here is to list every force and every torque acting on the drone, decide which of them belong in our nominal model and which we deliberately leave out, and work out how the one thing we can actually command, the four motors, produces those forces and torques. Once we have that, entry 02 can write the equations of motion.

A note on why frames matter here: entry 00 ended on the observation that the two dominant forces live in different frames, thrust in the body frame and gravity in the world frame, connected by the orientation. This entry makes that concrete, because getting a force into the wrong frame is one of the classic sources of silent error in quadrotor work.

---

## The forces (what makes the drone translate)

We began by listing every force we could think of acting on a quadrotor in flight, then sorted them into what belongs in the nominal model and what does not.

**Gravity.** Always points straight down, and it is naturally a world frame quantity because "down" is defined relative to the ground, not the drone. It is constant. It belongs in the nominal model.

**Thrust.** The four propellers push the drone along its own "up" direction, so total thrust always points along the body +z axis. This is a body frame quantity, and it is the single most important consequence of orientation: when the drone tilts, its thrust tilts with it, so a tilted thrust gains a horizontal component. This is the entire reason a quadrotor can move sideways at all, since it has no way to push sideways directly. Thrust belongs in the nominal model, and the orientation is what we will use to rotate it from the body frame into the world frame so it can be added to gravity.

**Drag.** Air resistance opposes motion, so it points opposite to the drone's velocity and is naturally expressed in the world frame, where the velocity lives. Drag is real and we care about it, but we make a deliberate choice: it does not enter the nominal model. Instead we introduce it later as one of the perturbations we study, so that the nominal model stays clean and the drag becomes part of the mismatch our controller and learned helper must deal with.

**Higher order aerodynamic effects (blade flapping, ground effect, and similar).** These are real and we acknowledge them, but we deliberately leave them out of the nominal model. They are small, difficult to model precisely, and, importantly, they are exactly the kind of unmodeled mismatch our learned residual exists to capture. In other words, leaving them out is not an oversight; the gap they create is part of what the research is about. We note them here so they are on record, and we let them live in the residual rather than in the analytical model.

So the nominal force model reduces to two forces: **gravity, pointing down in the world frame, and thrust, pointing up in the body frame.** Drag is added as a perturbation. Flapping, ground effect, and their relatives are left in the gap that the residual learns.

---

## The torques (what makes the drone rotate)

A quadrotor has no flaps, rudder, or moving control surfaces. It has only four spinning propellers. So the interesting question is how four propellers, purely by spinning at different speeds, make the drone roll, pitch, and yaw. Working through it, we found that roll and pitch use one mechanism and yaw uses a completely different one, which is a distinction worth stating clearly because it explains a lot about how a quadrotor behaves.

**Roll and pitch come from thrust differences across the frame.** If the motors on one side of the drone push harder than the motors on the other side, that side lifts more and the drone tips. Making the left side push harder than the right rolls the drone; making the front push harder than the back pitches it. So roll and pitch are produced by an imbalance in thrust between opposite sides, acting through the physical arm of the frame.

**Yaw comes from a reaction torque, not from any sideways push.** Every spinning propeller drags the surrounding air, and by Newton's third law the air drags back on the drone, producing a reaction torque that tries to spin the body the opposite way to the propeller. If all four propellers spun the same direction, the drone would spin uncontrollably, so the standard layout spins two propellers clockwise and two counterclockwise. At equal speeds their reaction torques cancel and there is no net yaw. To yaw on purpose, we deliberately break that balance: speeding up the clockwise pair while slowing the counterclockwise pair lets the clockwise reaction torques dominate, turning the drone one way, and reversing it turns the drone the other way. Yaw is therefore driven by imbalancing an aerodynamic reaction, which is why it is the weakest and slowest of the three rotational axes.

The key insight is that these are two genuinely different mechanisms. Roll and pitch are thrust differences acting through a lever. Yaw is a difference in aerodynamic reaction torques from the direction of spin.

---

## The motor to wrench map (the heart of this entry)

The one thing we can actually command is the four motor thrusts. The dynamics, however, care about the total force and the three torques. So there is a fixed conversion from four motor commands to one total thrust plus three torques, and working out that conversion is the central result of this entry.

**Total thrust is a sum.** All four thrusts point the same way, along the body up axis, so the total upward force is simply their sum. There is no leverage involved in adding forces that all point the same direction.

**Roll and pitch torques are differences multiplied by the arm length.** A torque is a force times a lever arm, so the roll torque is the difference in thrust between the left and right sides multiplied by how far those motors sit from the center, and the pitch torque is the same idea between front and back. This is where the arm length enters: it is the lever arm that converts a thrust difference into a torque. A motor far from the center produces more torque for the same thrust difference, because a longer lever gives more leverage, and a motor near the center produces less.

**Yaw torque is a difference in reaction torques and does not involve the arm length.** Yaw is produced by imbalancing the clockwise and counterclockwise reaction torques, which is a direct aerodynamic effect of spinning, not a lever effect. It therefore scales with a motor constant relating each propeller's spin to its reaction drag, not with the arm length.

That last contrast is a deep and useful one. Roll and pitch torque scale with the arm length, a geometric leverage, while yaw torque scales with an aerodynamic reaction constant. This is precisely why a real quadrotor has strong, quick roll and pitch authority but comparatively weak, slow yaw authority, and it is a fact we expect to matter later when we ask which corrections are realizable on which axis.

Collecting all of this gives the motor to wrench map, the fixed relationship that turns four motor thrusts into the total force and the three torques that actually drive the drone.

**Table: The motor to wrench map, how the four motors' thrusts convert into the total force and the three torques that drive the drone's motion.**

| Output | Comes from | Mechanism | Scales with |
|--------|-----------|-----------|-------------|
| Total thrust | sum of the four motor thrusts | all push along body up | (a direct sum) |
| Roll torque | thrust difference, left versus right | lever across the frame | arm length |
| Pitch torque | thrust difference, front versus back | lever across the frame | arm length |
| Yaw torque | reaction difference, clockwise versus counterclockwise | aerodynamic reaction from spin direction | motor drag constant |

---

## What this entry commits us to, and what is open

**Committed.** The nominal force model is gravity, pointing down in the world frame, plus thrust, pointing up in the body frame. The rotational actuation is captured by the motor to wrench map above, in which roll and pitch come from thrust differences acting through the arm length and yaw comes from an imbalance in the propellers' reaction torques.

**Deliberately left out of the nominal model.** Drag, which we add later as a perturbation. Blade flapping, ground effect, and similar higher order effects, which we leave in the gap that the learned residual is meant to capture.

**Open for the next entry (02, equations of motion).** We now have the forces and torques and we know they live in mixed frames, thrust in the body frame and gravity in the world frame, joined by the orientation. Entry 02 uses this to write the actual equations of motion: how the total force produces linear acceleration in the world frame once the body thrust is rotated in, and how the torques produce angular acceleration about the body axes, giving us the rates we need to integrate the 13 state numbers forward in time.

*This entry is locked. We do not proceed to 02 until every choice above can be explained in plain language, in particular the two different torque mechanisms and why the arm length matters for roll and pitch but not for yaw.*