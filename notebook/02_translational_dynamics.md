# 02 — Translational Dynamics

*Derivation notebook, entry 02. Continues the clean-room rebuild from entries 00 and 01. Written as we reason through the physics, deriving the equations that tell us how the drone moves through space.*

---

## What this entry is for

Entry 00 gave us the 13 numbers that describe the drone, and entry 01 gave us the forces and torques that act on it. This entry takes the translational half of that and writes the actual equations of motion: given the forces, how do position and velocity change over time. In other words, we are writing the rate of change of the translational state, which is exactly what we need in order to predict the drone's motion forward in time. The rotational half, how the orientation and angular velocity change, is handled next in entry 03.

We work in the world frame for position and velocity, as decided in entry 00, and we carry over the central fact from entry 01: gravity lives in the world frame and thrust lives in the body frame, joined by the orientation. This entry is where that fact stops being a note and becomes an equation.

---

## Notation

To keep the equations unambiguous, we fix the following symbols:

- **p** = [x, y, z]ᵀ, the position, expressed in the world frame.
- **v** = [vₓ, v_y, v_z]ᵀ, the linear velocity, expressed in the world frame.
- m, the mass of the drone; g, the magnitude of gravitational acceleration (9.81 m/s²).
- **R(q)**, the rotation matrix built from the orientation quaternion q. It converts a vector written in the body frame into the world frame. We keep it symbolic here; the explicit 3-by-3 matrix in terms of the quaternion components is derived in entry 03, where we handle orientation properly.
- T, the total thrust magnitude, that is, the sum of the four motor thrusts from entry 01.
- **e₃** = [0, 0, 1]ᵀ, the unit vector pointing "up".

---

## Position kinematics (the first equation)

The first equation is the simplest, and it is exact. The rate of change of position is, by definition, the velocity. There is nothing to derive:

$$\dot{\mathbf{p}} = \mathbf{v}$$

This is the instantaneous form of the familiar idea that distance equals velocity times time; here we simply state that at every instant the position is changing at the rate given by the current velocity. Integrating this equation forward is how we move the drone's position through space once we know its velocity.

---

## The core equation: linear acceleration

The second equation is the heart of this entry. It comes from Newton's second law, that acceleration equals total force divided by mass:

$$m\dot{\mathbf{v}} = \sum \mathbf{F} \qquad\Longrightarrow\qquad \dot{\mathbf{v}} = \frac{1}{m}\sum \mathbf{F}$$

Our nominal model has two forces, and the only subtlety, the one that must be handled carefully, is that they live in different frames.

**Gravity** is naturally a world frame quantity, pointing straight down:

$$\mathbf{F}_{\text{gravity}} = -mg\,\mathbf{e}_3 = \begin{bmatrix} 0 \\ 0 \\ -mg \end{bmatrix}$$

**Thrust** is naturally a body frame quantity. The four propellers push the drone along its own "up" axis, so in body coordinates the thrust is T·e₃. But we cannot add a body frame vector to a world frame vector directly. We must first rotate the thrust into the world frame using the rotation matrix R(q):

$$\mathbf{F}_{\text{thrust}} = \mathbf{R}(\mathbf{q})\,(T\mathbf{e}_3) = T\,\mathbf{R}(\mathbf{q})\mathbf{e}_3$$

The quantity R(q)·e₃ is the direction of "body up" expressed in the world frame, that is, the actual direction the thrust points once the drone has tilted. This is precisely where the orientation earns its place in the dynamics: it is what connects the direction the drone is pointing to the direction it accelerates.

Combining the two forces gives the core translational equation:

$$\boxed{\;\dot{\mathbf{v}} = \frac{1}{m}\,T\,\mathbf{R}(\mathbf{q})\mathbf{e}_3 \;-\; g\,\mathbf{e}_3\;}$$

In words, the linear acceleration equals the thrust, rotated into the world frame and divided by mass, minus gravity. The single thing that is easy to get wrong here, and a classic source of silent error, is forgetting to rotate the thrust; without R(q) one would be adding a body frame vector to a world frame vector, which is meaningless.

---

## Physical sanity checks

Having written the equation, we verify it against two situations we understand physically, which both confirms the equation and shows what it is telling us.

**Hover, level and stationary.** Flying level means there is no rotation, so R(q) is the identity matrix and R(q)·e₃ = e₃, that is, thrust points straight up. Hovering means the drone is not accelerating, so the left side is zero. Substituting:

$$\mathbf{0} = \frac{T}{m}\mathbf{e}_3 - g\,\mathbf{e}_3 \qquad\Longrightarrow\qquad \frac{T}{m} = g \qquad\Longrightarrow\qquad \boxed{T = mg}$$

The thrust must equal the weight. This matches the intuition that to hover the drone must exactly overcome gravity, and it also tells us something we will use later: this is exactly why a mass mismatch matters. If the true mass differs from the value the controller assumes, the thrust it computes for hover will be wrong, and the drone will slowly sink or climb. We have, in effect, derived the reason mass mismatch is a perturbation worth studying.

**Tilt producing sideways motion.** Suppose the drone pitches forward by a small angle θ. Then thrust no longer points straight up; its world frame direction gains a horizontal component. Schematically, for a pitch θ,

$$\mathbf{R}(\mathbf{q})\mathbf{e}_3 \approx \begin{bmatrix} \sin\theta \\ 0 \\ \cos\theta \end{bmatrix}$$

Substituting into the core equation:

$$\dot{\mathbf{v}} \approx \frac{T}{m}\begin{bmatrix}\sin\theta \\ 0 \\ \cos\theta\end{bmatrix} - \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix} = \begin{bmatrix} \dfrac{T}{m}\sin\theta \\[6pt] 0 \\[6pt] \dfrac{T}{m}\cos\theta - g \end{bmatrix}$$

The top row is the key result: the horizontal acceleration equals (T/m)·sin θ. When the drone is level (θ = 0) it is zero, and there is no sideways motion. When the drone tilts (θ ≠ 0) it becomes nonzero, and the drone accelerates sideways; tilting more accelerates it faster. This is the entire mechanism by which a quadrotor moves horizontally, now falling directly out of the equation rather than being asserted.

There is an important structural observation hidden in this result, and it connects to the core of our later research. To move sideways, the drone must first tilt, which means it must first rotate, which takes time. Horizontal motion is therefore not something we can command directly; it is one step removed from what we control, because it requires the orientation to change first. This chain, control acts on rotation, rotation produces tilt, tilt produces horizontal acceleration, is the reason horizontal effects are structurally harder to influence quickly than vertical ones, and we expect it to matter a great deal when we later ask which corrections are realizable on which axis and over what horizon.

---

## What this entry commits us to, and what is open

**Committed.** The translational dynamics of the drone are given by two equations: the position changes at the rate of the velocity, and the velocity changes according to the thrust rotated into the world frame and divided by mass, minus gravity.

$$\dot{\mathbf{p}} = \mathbf{v}, \qquad \dot{\mathbf{v}} = \frac{T}{m}\,\mathbf{R}(\mathbf{q})\mathbf{e}_3 - g\,\mathbf{e}_3$$

These describe how six of the thirteen state numbers, position and velocity, evolve in time.

**Left symbolic for now.** The explicit form of the rotation matrix R(q) in terms of the quaternion components. Its structure is all we need here; its full expression belongs with the orientation, in entry 03.

**Open for the next entry (03, rotational dynamics).** We have the translational half. Entry 03 completes the equations of motion by deriving how the orientation quaternion and the body angular velocity change over time, using the torques from entry 01. That entry is also where the rotation matrix R(q) is written out explicitly, since it is built directly from the quaternion we will be evolving there. With both halves in hand we will have the full rate of change of all thirteen states, which is everything needed to predict the drone's motion.

*This entry is locked. We do not proceed to 03 until the two translational equations can be explained in plain language, in particular why the thrust must be rotated by R(q) while gravity is not, and why the equation forces T = mg at hover.*