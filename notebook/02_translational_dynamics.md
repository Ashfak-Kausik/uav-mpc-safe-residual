# 02 — Translational Dynamics
 
*Derivation notebook, entry 02. Continues the clean-room rebuild from entries 00 and 01. Written as we reason through the physics, deriving the equations that tell us how the drone moves through space.*
 
---
 
## What this entry is for
 
Entry 00 gave us the 13 numbers that describe the drone, and entry 01 gave us the forces and torques that act on it. This entry takes the translational half of that and writes the actual equations of motion: given the forces, how do position and velocity change over time. In other words, we are writing the rate of change of the translational state, which is exactly what we need in order to predict the drone's motion forward in time. The rotational half, how the orientation and angular velocity change, is handled next in entry 03.
 
We work in the world frame for position and velocity, as decided in entry 00, and we carry over the central fact from entry 01: gravity lives in the world frame and thrust lives in the body frame, joined by the orientation. This entry is where that fact stops being a note and becomes an equation.
 
This entry also carries more weight for the project than its length suggests. Two of the results below, the inversion that recovers the required thrust and tilt from a desired acceleration, and the relative-degree count, are the analytical backbone of our realizability criterion. They are derived here because this is where the structure first appears, not later where it is used.
 
---
 
## Notation
 
To keep the equations unambiguous, we fix the following symbols, extending entry 00:
 
- **p** = [x, y, z]ᵀ, the position, expressed in the world frame.
- **v** = [vₓ, v_y, v_z]ᵀ, the linear velocity, expressed in the world frame.
- m, the mass of the drone; g, the magnitude of gravitational acceleration, 9.81 m/s².
- **R(q)**, the rotation matrix built from the orientation quaternion q. It converts a vector written in the body frame into the world frame, which is the direction fixed in entry 00. We keep it symbolic here; the explicit 3-by-3 matrix in terms of the quaternion components is written out in entry 03, where we handle orientation properly.
- T, the total thrust magnitude, that is, the sum of the four rotor thrusts from entry 01, with $T = \sum_i f_i$ and $0 \le T \le 4 f_{\max}$.
- **e₃** = [0, 0, 1]ᵀ, the third basis vector. In the world frame it points up; in the body frame it points along the thrust axis. Which of the two is meant is always clear from whether it sits inside R(q) or outside it.
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
 
**Gravity** is naturally a world frame quantity, pointing along negative world z as fixed in entry 00:
 
$$\mathbf{F}_{\text{gravity}} = -mg\,\mathbf{e}_3 = \begin{bmatrix} 0 \\ 0 \\ -mg \end{bmatrix}$$
 
**Thrust** is naturally a body frame quantity. The four propellers push the drone along its own body z axis, so in body coordinates the thrust is T·e₃. But we cannot add a body frame vector to a world frame vector directly. We must first rotate the thrust into the world frame using the rotation matrix R(q):
 
$$\mathbf{F}_{\text{thrust}} = \mathbf{R}(\mathbf{q})\,(T\mathbf{e}_3) = T\,\mathbf{R}(\mathbf{q})\mathbf{e}_3$$
 
The quantity R(q)·e₃ is the third column of R(q); it is the direction of "body up" expressed in the world frame, that is, the actual direction the thrust points once the drone has tilted. This is precisely where the orientation earns its place in the dynamics: it is what connects the direction the drone is pointing to the direction it accelerates. We will refer to this world-frame unit vector often enough that it is worth a name:
 
$$\mathbf{b}_3(\mathbf{q}) \;\equiv\; \mathbf{R}(\mathbf{q})\,\mathbf{e}_3, \qquad \|\mathbf{b}_3\| = 1$$
 
Combining the two forces gives the core translational equation:
 
$$\boxed{\;\dot{\mathbf{v}} = \frac{T}{m}\,\mathbf{b}_3(\mathbf{q}) \;-\; g\,\mathbf{e}_3\;}$$
 
In words, the linear acceleration equals the thrust, rotated into the world frame and divided by mass, minus gravity. The single thing that is easy to get wrong here, and a classic source of silent error, is forgetting to rotate the thrust; without R(q) one would be adding a body frame vector to a world frame vector, which is meaningless.
 
**The structural reading of this equation.** Look at what the drone actually gets to choose. It chooses a scalar, T, and it chooses a direction, $\mathbf{b}_3$, but the direction is not a free variable at this instant: it is a function of the orientation, which is a state, which can only be changed by rotating, which takes time. So the translational dynamics offer **one instantaneous degree of freedom (magnitude) and two delayed ones (direction)**. Every asymmetry in this project, why vertical corrections behave differently from lateral ones, why the same residual is realizable at one injection point and not another, traces back to this single sentence.
 
---
 
## Physical sanity checks
 
Having written the equation, we verify it against two situations we understand physically, which both confirms the equation and shows what it is telling us.
 
**Hover, level and stationary.** Flying level means there is no rotation, so R(q) is the identity matrix and $\mathbf{b}_3 = \mathbf{e}_3$, that is, thrust points straight up. Hovering means the drone is not accelerating, so the left side is zero. Substituting:
 
$$\mathbf{0} = \frac{T}{m}\mathbf{e}_3 - g\,\mathbf{e}_3 \qquad\Longrightarrow\qquad \frac{T}{m} = g \qquad\Longrightarrow\qquad \boxed{T = mg}$$
 
The thrust must equal the weight. This matches the intuition that to hover the drone must exactly overcome gravity, and it also tells us something we will use later: this is exactly why a mass mismatch matters. If the true mass differs from the value the controller assumes, the thrust it computes for hover will be wrong. Open loop, the drone would steadily sink or climb. Under feedback, what happens depends on the controller: a controller without integral action settles at a constant altitude offset, and one with integral action removes it (entry 04). We have, in effect, derived the reason mass mismatch is a perturbation worth studying. Note also what this says about the correction channel: a mass error is an error in a *scalar multiplying the one instantaneous degree of freedom*, which is the most favourable structure a mismatch can possibly have. We should expect, before running anything, that mass is the easiest of our perturbations to correct.
 
**Tilt producing sideways motion, with the sign done carefully.** Consider a rotation by angle θ about the body y axis. The sign here needs care, and it is worth spelling out because the FLU convention we adopted in entry 00 makes it the opposite of the aerospace habit. In FLU the body y axis points **left**, so by the right-hand rule a positive rotation about it carries the body z axis toward positive world x and carries the nose toward negative world z. **Positive pitch in our convention is therefore nose down**, and it tilts the thrust *forward*. (In the NED/FRD convention used by most autopilots, body y points right and positive pitch is nose up. The physics is identical; only the label flips. We record this here because it is exactly the kind of thing that gets silently imported wrong.)
 
With that fixed, for a pitch of θ:
 
$$\mathbf{b}_3 = \mathbf{R}(\mathbf{q})\mathbf{e}_3 = \begin{bmatrix} \sin\theta \\ 0 \\ \cos\theta \end{bmatrix}$$
 
Substituting into the core equation:
 
$$\dot{\mathbf{v}} = \frac{T}{m}\begin{bmatrix}\sin\theta \\ 0 \\ \cos\theta\end{bmatrix} - \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix} = \begin{bmatrix} \dfrac{T}{m}\sin\theta \\[6pt] 0 \\[6pt] \dfrac{T}{m}\cos\theta - g \end{bmatrix}$$
 
The top row is the key result: the horizontal acceleration equals (T/m)·sin θ. When the drone is level, θ = 0, it is zero and there is no sideways motion. When the drone tilts it becomes nonzero and the drone accelerates forward; tilting more accelerates it faster. This is the entire mechanism by which a quadrotor moves horizontally, now falling directly out of the equation rather than being asserted.
 
**A useful corollary: coordinated tilt.** If we want to tilt and *not* lose altitude, the bottom row must stay zero, which forces $T = mg/\cos\theta$. Thrust must increase as the secant of the tilt angle. Substituting back into the top row gives the maximum lateral acceleration available at a given tilt:
 
$$a_{\text{lat}} = g\tan\theta$$
 
Two things fall out of this that we will use directly. First, lateral acceleration is capped by the maximum tilt the vehicle can hold, and holding that tilt while maintaining altitude costs a factor $1/\cos\theta$ in thrust, which means the tilt limit is really set by the thrust-to-weight ratio: a vehicle with TWR of 2 runs out of thrust at 60 degrees of tilt, giving at most $g\tan 60° \approx 1.7g$ of lateral acceleration, and that is before any margin is left over for corrections. Second, this couples the two channels: **demanding lateral correction consumes vertical authority.** A realizability test that treats the lateral and vertical channels independently is therefore wrong twice over, once at the rotor level through entry 01's polytope, and once here at the tilt level.
 
---
 
## Inverting the equation: what a desired acceleration demands
 
This is the result that entry 01's constraint polytope was waiting for, and it is short.
 
Suppose something, a trajectory generator, an MPC solve, or a learned residual injected at the reference, asks for a desired world-frame acceleration $\mathbf{a}_{\text{des}}$. What does the vehicle have to do to produce it? Rearranging the core equation:
 
$$T\,\mathbf{b}_3 = m\big(\mathbf{a}_{\text{des}} + g\,\mathbf{e}_3\big)$$
 
The right-hand side is a fully known vector. Since $\mathbf{b}_3$ is a unit vector, taking the norm and then the direction separates cleanly:
 
$$\boxed{\;T_{\text{req}} = m\,\big\|\mathbf{a}_{\text{des}} + g\,\mathbf{e}_3\big\|, \qquad \mathbf{b}_3^{\text{req}} = \frac{\mathbf{a}_{\text{des}} + g\,\mathbf{e}_3}{\big\|\mathbf{a}_{\text{des}} + g\,\mathbf{e}_3\big\|}\;}$$
 
Any desired acceleration determines the required thrust magnitude and the required thrust *direction* uniquely, with one exception. At $\mathbf{a}_{\text{des}} = -g\,\mathbf{e}_3$ (free fall) the thrust is zero and the direction is undefined. Near it the direction becomes ill-conditioned: a small change in the request swings $\mathbf{b}_3^{\text{req}}$ through a large angle. A reference correction that asks for near-free-fall is therefore a badly posed request, independently of the actuator limits. The remaining rotational freedom, yaw about the thrust direction, is unconstrained by translation, which is why yaw is specified separately in a trajectory.
 
This is a small piece of algebra with outsized consequences for our project:
 
1. It is the **exact map from a reference-level correction to the actuation it implicitly demands.** Our chosen injection point is the reference supplied to the MPC. This inversion is how we find out what that reference offset is really asking the vehicle to do.
2. Composed with the thrust bound it gives an immediate, closed-form **magnitude check**: the requested acceleration is achievable in the vertical sense only if $T_{\text{req}} \le 4 f_{\max}$, and it demands a tilt of $\arccos\!\big((\mathbf{b}_3^{\text{req}})_z\big)$ from level, which must be within the vehicle's tilt envelope.
3. It shows the structural difference between the two channels in one line. The magnitude $T_{\text{req}}$ can be delivered essentially at once, subject only to rotor rate limits. The direction $\mathbf{b}_3^{\text{req}}$ cannot: the vehicle must first rotate to it, which is a dynamic process governed by entry 03 and bounded by entry 01's torque polytope. **A correction is not one request; it is a magnitude request and a direction request, with completely different feasibility characters.**
4. It is the germ of **differential flatness**. The full statement, that every state and input of a quadrotor is an algebraic function of the position and yaw outputs and their derivatives, is exactly this inversion carried two derivatives further, through $\dot{\mathbf{b}}_3$ to the required body rates and through $\ddot{\mathbf{b}}_3$ to the required torques. We do not derive that here, but we flag it as the tool we will need for two things: generating dynamically feasible aggressive trajectories rather than geometric curves the vehicle cannot fly, and turning the qualitative realizability argument into a computable one.
---
 
## How far is horizontal motion from the controls? Plant structure versus injection point

Entry 00 and the tilt check both gestured at the idea that horizontal motion is "one step removed" from what we control. It is worth counting the steps precisely. Two different questions are easy to blur here, and keeping them apart is the point of this section:

1. **Plant structure.** How many integrators sit between a *plant input* and an output? This is a property of the vehicle.
2. **Injection point.** Where does a correction enter the control loop, and what does it pass through before it reaches the plant input? This is a property of the control architecture.

### (a) Plant structure: integrators from each plant input

Differentiate a world-frame position component and see how many derivatives it takes before an input appears.

**Vertical channel, from collective thrust.** $z \to \dot z = v_z \to \dot v_z$ contains $T$. Two differentiations.

**Lateral channel, from body torque.** $x \to v_x \to \dot v_x$ contains $\mathbf{b}_3$, which is orientation, not an input. Differentiate again: $\dot{\mathbf{b}}_3$ depends on the angular velocity $\boldsymbol{\omega}$, still not an input. Differentiate once more: $\dot{\boldsymbol{\omega}}$ contains the torque $\boldsymbol{\tau}$. Four differentiations.

**A precision we owe the reader.** The count "4 from torque" is exact in the **linearization about hover**, which is where entry 04 computes it: there $\ddot x = g\,\delta\theta$ does not involve $\delta T$ at all. In the full nonlinear model it needs a qualification. With the rotor thrusts as inputs, $(T, \boldsymbol\tau)$ is an invertible function of the input, and $\ddot x = (T/m)\,b_{3,x}(\mathbf q)$ already contains the input $T$ at the **second** derivative. Its coefficient $b_{3,x}/m$ vanishes at hover but not when tilted. The matrix that multiplies the inputs at the second derivative of $(x, y, z)$ has rank 1 (only the $T$ column is non-zero). So the nonlinear system with thrust inputs has **no well-defined vector relative degree**. The standard remedy is dynamic extension: treat $T$ as a state and $\ddot T$ as an input. Then the outputs $(x, y, z, \psi)$ have vector relative degree $(4, 4, 4, 2)$, and the decoupling matrix is invertible whenever $T > 0$. Mistler, Benallegue and M'Sirdi (IEEE RO-MAN 2001, DOI 10.1109/ROMAN.2001.981968) give this construction for the quadrotor, and [checks/check_02_translational.py](checks/check_02_translational.py) verifies it. For our purposes the conclusion is the same: lateral position is four integrations from the torque. But the statement is a property of the *linearized* chain, and the nonlinear version goes through the extended system.

**Motor lag adds one more step.** Entry 01 places the motor lag in the nominal model, so the true plant input is the *commanded* thrust. A first-order lag is not an integrator, but it does add one order (one pole) between the command and the delivered thrust. From the commanded thrusts, the linearized chain to lateral position has order 5, and to height order 3.

| Plant input | Order to lateral position (hover linearization) | Order to height |
|---|---|---|
| Commanded rotor thrusts (lag in the nominal model) | 5 (4 integrators + motor lag) | 3 (2 integrators + motor lag) |
| Delivered rotor thrusts / body torque and collective thrust | 4 | 2 |
| Body-rate command, *if* a perfect rate loop is assumed | 3 | 2 |
| Attitude / thrust-direction command, *if* a perfect attitude loop is assumed | 2 | 2 |
| Acceleration command, *if* a perfect acceleration loop is assumed | 2 | 2 |

The last three rows are idealizations: each assumes a loop that delivers its command instantly. Our model assumes none of them. They are listed because the earlier study injected a correction at the body-rate level (the third row), and because published residual methods often inject at the attitude or acceleration level.

### (b) Injection points: what the correction passes through

A **reference is not a plant input.** Whatever reference the MPC receives, the vehicle still realizes it through the same commanded thrusts and the same chain in table (a). A reference-level correction therefore does not "skip" any integrators. An earlier version of this entry listed it at "0 integrations", and that was wrong. What changes between injection points is not the plant's integrator count. It is three other things:

1. **Who computes the plant input.** A correction added *after* the optimizer (a post-solve feedforward on thrust, torque or body rate) reaches the rotors directly. It bypasses the input constraints, and it ignores what the MPC knows about the chain. A correction added to the *reference* is turned into inputs *by the constraint-aware MPC*, which plans the whole chain under the rotor box and rate limits.
2. **Preview.** A reference correction is visible over the whole prediction horizon, so the MPC can start the rotate-then-translate manoeuvre early. A post-solve feedforward acts only at the current step. *(Qualified by entry 06: the primary MPC holds the reference constant over its horizon, so it sees the current correction over the horizon, but not future changes of it. Full preview of a changing correction needs the trajectory formulation deferred to entry 08.)*
3. **The transfer to the output.** A post-solve correction reaches the output through the plant, $G(s)$, with no feedback shaping. A reference correction reaches it through the closed loop's reference-to-output map $T_{ry}$. That map has finite bandwidth; its DC gain is close to 1, and exactly 1 if the MPC is offset-free. Under saturation it becomes a nonlinear map. The realizability ratio (entry 09) and the loop-gain bound (entry 10) are computed on this transfer, not on an integrator count.

| Injection point | Input computed by | Input constraints respected? | Preview over horizon? | Transfer from correction to output |
|---|---|---|---|---|
| Post-solve thrust / torque feedforward | adder after the MPC | no | no | plant $G$, open loop |
| Post-solve body-rate feedforward (earlier study) | adder after the MPC (through a rate loop) | no | no | rate loop then plant, open loop |
| Learned residual inside the MPC's prediction model | the MPC | yes | yes | depends on the learned model; changes the controller itself |
| Shift of the MPC's reference (**this project**) | the MPC (unchanged) | yes | yes | closed-loop $T_{ry}$ |

So the thesis sentence is this: **every injection point uses the same plant chain. What differs is whether the correction passes through the constraint-aware optimizer, with preview, and through which closed-loop transfer.** Accuracy of the learned model does not enter this table at all. That is why the same accurate correction can help at one injection point and hurt at another. It is also why a reference-level correction can still hurt: $T_{ry}$ has limited bandwidth and limited phase margin, and a state-dependent correction closes a loop around it (entry 10).

Two caveats we record honestly. First, the chain order is a necessary structural quantity, not a sufficient one: a short chain does not make a correction realizable if the required wrench falls outside entry 01's polytope. Realizability needs both the structure and the bound. Second, the orders are structural and do not change with tilt, but the *gains* along each step do, which is why the criterion has to be evaluated online rather than tabulated once.
 
---
 
## What this entry commits us to, and what is open
 
**Committed.** The translational dynamics of the drone are given by two equations: the position changes at the rate of the velocity, and the velocity changes according to the thrust rotated into the world frame and divided by mass, minus gravity.
 
$$\dot{\mathbf{p}} = \mathbf{v}, \qquad \dot{\mathbf{v}} = \frac{T}{m}\,\mathbf{b}_3(\mathbf{q}) - g\,\mathbf{e}_3, \qquad \mathbf{b}_3(\mathbf{q}) = \mathbf{R}(\mathbf{q})\mathbf{e}_3$$
 
These describe how six of the thirteen state numbers, position and velocity, evolve in time. Also committed: the inversion giving $T_{\text{req}}$ and $\mathbf{b}_3^{\text{req}}$ from a desired acceleration, the coordinated-tilt relation $a_{\text{lat}} = g\tan\theta$ with its $1/\cos\theta$ thrust cost, and the two tables above. These are (a) the plant's chain orders in the hover linearization, with the dynamic-extension qualification for the nonlinear model, and (b) the injection-point comparison, in which a reference shift leaves the plant chain unchanged and differs only in who computes the input, in preview, and in the closed-loop transfer.
 
**Sign convention recorded.** Positive pitch about body y is nose down and tilts thrust toward positive world x, which is the FLU consequence of entry 00's convention and the opposite of the NED/FRD habit.
 
**Left symbolic for now.** The explicit form of the rotation matrix R(q) in terms of the quaternion components. Its structure is all we need here; its full expression belongs with the orientation, in entry 03.
 
**Deferred, noted, not forgotten.**
- Drag enters this equation, when we add it, as a third term on the right-hand side, $-\tfrac{1}{m}\mathbf{R}\mathbf{D}\mathbf{R}^{\top}\mathbf{v}$ for rotor drag or a speed-dependent term for body drag. We record the slot now so that the perturbation phase is an addition to a known equation rather than a rewrite of it.
- Full differential flatness, which is this entry's inversion carried further, to be derived when we need feasible aggressive trajectories.
- **Integration and discretization hygiene**, the subject of entry 05, flagged here because it will contaminate our experiments if left implicit. The plant will be integrated with a fixed-step RK4 at a small step; the MPC will carry its own, coarser discrete model. The gap between the two is itself a form of model mismatch, and it adds to every perturbation we inject. We will measure the pure-discretization tracking error at zero perturbation and report it as the noise floor, so that no perturbation effect smaller than it is ever claimed as a result. We will also verify the integrator numerically by halving the step and confirming the error falls by roughly a factor of sixteen, which is the signature of genuine fourth-order accuracy and the cheapest way to catch an accidental Euler step.
**Open for the next entry (03, rotational dynamics).** We have the translational half, and we now know precisely what it demands of the rotational half: a thrust direction $\mathbf{b}_3^{\text{req}}$ that the orientation must be steered to. Entry 03 completes the equations of motion by deriving how the orientation quaternion and the body angular velocity change over time, using the torques from entry 01, and writes out R(q) explicitly since it is built directly from the quaternion evolved there. With both halves in hand we will have the full rate of change of all thirteen states, which is everything needed to predict the drone's motion.
 
*This entry is locked. We do not proceed to 03 until the following can be explained in plain language: why the thrust must be rotated by R(q) while gravity is not, why the equation forces T = mg at hover, why positive pitch is nose down in our convention, why a desired acceleration splits into a magnitude request and a direction request with different feasibility characters, what the chain-order table counts and why it holds exactly only in the hover linearization, and why a reference-level correction does not shorten the plant chain but changes who computes the input, the preview, and the transfer to the output.*