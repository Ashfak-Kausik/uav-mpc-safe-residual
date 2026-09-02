# 04 — Hover Equilibrium and Linearisation
 
*Derivation notebook, entry 04. Continues the clean-room rebuild from entries 00 to 03. This entry finds the drone's equilibrium, measures how much authority sits around it, and uses it to work out precisely how each of our four mismatches pushes the vehicle off balance. It is the bridge from "here is the physics" toward "here is the controller."*
 
---
 
## What this entry is for
 
Entries 00 to 03 gave us a complete nonlinear model: thirteen states, four rotor thrusts, and the equations that connect them. That model can be simulated, but it cannot yet be *reasoned about*, because nonlinear equations do not readily tell you which channels are fast, how much room is left over for corrections, or what a given mismatch will actually do to the loop. This entry supplies that reasoning by doing the one thing that always comes next: find the equilibrium, and look at the dynamics in its neighbourhood.
 
Three things come out of it, and all three are things the rest of the project needs.
 
1. **The equilibrium itself**, which tells us the trim rotor thrusts and, importantly, that the equilibrium is not a single point but a family.
2. **The authority margin around it**, meaning how much torque and thrust are actually available above trim. This is the first place entry 01's constraint polytope produces a number rather than a shape, and those numbers are what our realizability criterion will be normalised against.
3. **A structural classification of our four mismatches** into those that *move the equilibrium* and those that *change the dynamics about it*. This turns out to be the sharpest thing in the entry. It is, we believe, the reason our prior study found that some mismatches were correctable and one was destabilising, and it lets us write down predictions before we run anything.
This entry is deliberately shorter and more intuitive than entry 03. The hard representational work is done; this is where it starts paying.
 
---
 
## Finding the equilibrium
 
An equilibrium is a state at which every rate is zero, so that the vehicle left alone stays exactly where it is. Setting the left-hand sides of entry 03's four equations to zero and reading them from the bottom up:
 
**From $\dot{\boldsymbol{\omega}} = \mathbf{0}$:** with ω = 0 the gyroscopic term vanishes, so the condition is simply $\boldsymbol{\tau} = \mathbf{0}$. No net torque.
 
**From $\dot{\mathbf{q}} = \mathbf{0}$:** the quaternion kinematics give zero only when ω = 0, which we already have. So this equation imposes nothing new; it is automatically satisfied and, crucially, it does **not** pin down the orientation.
 
**From $\dot{\mathbf{v}} = \mathbf{0}$:** we need $\tfrac{T}{m}\mathbf{b}_3 = g\mathbf{e}_3$. Since $\mathbf{b}_3$ is a unit vector and the right-hand side points along world z, this forces $\mathbf{b}_3 = \mathbf{e}_3$, meaning the thrust axis must point straight up, and then $T = mg$.
 
**From $\dot{\mathbf{p}} = \mathbf{0}$:** we need $\mathbf{v} = \mathbf{0}$, the vehicle is not translating. The position itself is unconstrained.
 
Inverting entry 01's mixer at $T = mg$, $\boldsymbol{\tau} = \mathbf{0}$ gives the trim rotor thrusts:
 
$$\boxed{\;f_i^{\text{hover}} = \frac{mg}{4}, \quad i = 1,\dots,4\;}$$
 
Every rotor carries a quarter of the weight. This is the answer we expected, and getting it out of the machinery rather than assuming it is the point.
 
### The equilibrium is a family, not a point
 
Two of the thirteen states were left free by the conditions above, and both matter.
 
**Position is free.** Any point in space is an equilibrium, which is the formal statement that the vehicle has no preferred location and therefore no natural restoring force toward a reference. This is exactly the job we are handing to the controller.
 
**Yaw is free.** The condition was $\mathbf{b}_3 = \mathbf{e}_3$, which fixes only two of the three rotational degrees of freedom. Rotation about the thrust axis is unconstrained, so the equilibrium set includes a full circle of yaw angles. We already saw this algebraically in entry 03, where $\mathbf{b}_3$ turned out to be independent of rotation about itself. Here it acquires its physical meaning: **yaw is dynamically decoupled from equilibrium**, which is why it is specified separately in a trajectory and why an error in it produces no restoring tendency whatsoever.
 
### What is not an equilibrium, and why that is informative
 
Level flight at a constant nonzero velocity is *not* an equilibrium of the full state, since position keeps changing. But it is what is usually called a trim or relative equilibrium, and in our nominal model it has a striking property: because the nominal model contains no velocity-dependent force, **the thrust and attitude required to hold constant velocity are identical to those required to hover**, at any speed, in any direction. The vehicle cannot tell, from its own actuation, whether it is stationary or cruising at 10 m/s.
 
Record that, because it is precisely what drag destroys, and noticing what a perturbation destroys is how we will classify the perturbations below.
 
---
 
## How much authority is there around hover?
 
At trim each rotor sits at $mg/4$, with a ceiling at $f_{\max}$ and a floor at zero. The gap between trim and those bounds is the entire budget from which every manoeuvre and every correction must be paid. Entry 01 gave the constraint set; here we turn it into numbers.
 
Define the thrust-to-weight ratio $\text{TWR} = 4f_{\max}/(mg)$, so that trim sits at a fraction $1/\text{TWR}$ of each rotor's range.
 
**Vertical authority** is the easy case. Maximum upward acceleration is $g(\text{TWR} - 1)$, obtained by commanding all rotors to $f_{\max}$. Maximum downward acceleration is $g$, obtained by cutting thrust entirely, and it cannot exceed $g$ because the rotors cannot pull. **Vertical authority is asymmetric**, generously so upward on a high-TWR vehicle and hard-capped at free fall downward. A correction demanding rapid descent is therefore in a materially worse position than one demanding rapid climb, which is not obvious and which we should carry forward.
 
**Roll and pitch authority** requires holding $T = mg$ while spreading the rotor thrusts apart. Maximising $\tau_x = a(f_1 + f_2 - f_3 - f_4)$ subject to $\sum f_i = mg$ and $0 \le f_i \le f_{\max}$ pushes one pair up and the other down until something saturates. Working through the two cases gives a compact result:
 
$$\tau_x^{\max}\big|_{\text{hover}} = a\,mg \cdot \min\!\big(1,\; \text{TWR} - 1\big)$$
 
If $\text{TWR} \ge 2$ the binding limit is the *floor*: the descending pair reaches zero thrust before the ascending pair reaches its ceiling, and extra TWR buys no further torque at hover. If $\text{TWR} < 2$ the ceiling binds first and authority scales linearly with the margin above hover. Two consequences worth stating. First, **torque authority at hover saturates at TWR of 2**, so beyond that point a higher-TWR vehicle is not a more agile one in this respect, only a faster-climbing one. Second, this whole quantity is evaluated *at hover*; away from hover the available torque shrinks, exactly as entry 01's polytope warned, and a vehicle already commanding high thrust for a climb has less left for attitude.
 
**Yaw authority** has the identical structure with $c_{\tau f}$ in place of $a$:
 
$$\tau_z^{\max}\big|_{\text{hover}} = c_{\tau f}\,mg \cdot \min\!\big(1,\; \text{TWR} - 1\big), \qquad \frac{\tau_z^{\max}}{\tau_x^{\max}} = \frac{c_{\tau f}}{a} \approx \frac{1}{15}$$
 
Converting torques to angular accelerations divides by the relevant inertia, and here the two effects compound in the same direction: yaw is driven by the smaller constant *and* resisted by the larger inertia, since $I_{zz} \approx I_{xx} + I_{yy}$ for a flat airframe. Entry 03 predicted yaw would be the first axis to run out; this is the number behind that prediction.
 
**The three normalised groups we will use.** These authority expressions are the reason our cross-vehicle claim can be stated without per-vehicle tuning. The quantities that actually govern behaviour are dimensionless: the thrust-to-weight ratio TWR, the torque-to-inertia group $a\,mg/I_{xx}$ which sets the maximum angular acceleration and hence how fast the vehicle can reorient, and the yaw penalty $c_{\tau f}/a$. Any threshold in our supervisor should be expressed in these, not in newtons and metres.
 
---
 
## Linearising about hover: where the relative-degree ladder gets numbers
 
Entry 02 counted the steps from each injection point to each output. Entry 03 showed the same count as the literal cascade of the equations. Here we attach gains to it, which is what turns a structural claim into a computable one.
 
Take a small perturbation about hover. Write the quaternion in its small-angle form, $\mathbf{q} \approx (1, \delta\phi/2, \delta\theta/2, \delta\psi/2)$, where $\delta\phi, \delta\theta, \delta\psi$ are small roll, pitch and yaw deviations. Substituting into entry 03's expression for $\mathbf{b}_3$ and keeping first order terms:
 
$$\mathbf{b}_3 \approx \begin{bmatrix} \delta\theta \\ -\delta\phi \\ 1 \end{bmatrix}$$
 
Substituting that into entry 02's core equation and writing $T = mg + \delta T$, the three translational channels decouple to first order:
 
$$\ddot{x} \approx g\,\delta\theta, \qquad \ddot{y} \approx -g\,\delta\phi, \qquad \ddot{z} \approx \frac{\delta T}{m}$$
 
Read the signs against our conventions and they check out: positive pitch is nose down, which tilts thrust forward and accelerates along positive x; positive roll lifts the left side, which tilts thrust to the right and accelerates along negative y. These are the same signs entry 01's mixer produces, arrived at from the opposite direction, which is a useful consistency check on the whole chain.
 
Now push through the attitude dynamics, $\delta\ddot{\theta} = \tau_y / I_{yy}$, to get the full input-to-output gains:
 
| Channel | Linearised relation | Order | Gain |
|---|---|---|---|
| Vertical, from thrust | $\ddot z = \delta T / m$ | 2 | $1/m$ |
| Lateral, from attitude deviation | $\ddot x = g\,\delta\theta$ | 2 | $g$ |
| Lateral, from body torque | $x^{(4)} = (g/I_{yy})\,\tau_y$ | 4 | $g/I_{yy}$ |
| Yaw, from torque | $\ddot\psi = \tau_z / I_{zz}$ | 2 | $1/I_{zz}$ |
 
Three observations we will use directly.
 
**The lateral channel is a chain of four integrators.** Four integrators in series is a notoriously delicate object to control: it contributes 360 degrees of phase lag on its own, so all of the phase margin has to be manufactured by the controller. Anything that adds *further* lag between the command and the torque, which is exactly what motor lag and delay do, eats directly into a margin that was never generous. This is the mechanism we expect to be behind the destabilisation we saw in the prior study, and stating it here, before any experiment, is the point of doing this entry.
 
**The vertical channel is a chain of two.** Two integrators is a benign, textbook object with plenty of margin. Vertical and lateral are not merely different in degree; they are in different difficulty classes.
 
**The lateral gain is $g/I_{yy}$, and $g$ is not adjustable.** Lateral acceleration is bought entirely by tilting against gravity. A vehicle in a weaker gravitational field would be a worse lateral tracker at the same tilt. This is a small point but it explains why the tilt angle, rather than any actuator quantity, is the natural currency for lateral authority.
 
---
 
## How each mismatch attacks the equilibrium
 
This is the section the entry exists for. We take our four perturbations and ask a single question of each: *does it move the equilibrium, or does it change the dynamics about the equilibrium?* The answer sorts them cleanly, and the sorting predicts their behaviour.
 
**Mass error: moves the equilibrium, in the best possible channel.** If the true mass is $m' = m + \Delta m$ while the controller believes $m$, the true hover thrust is $m'g$ but the controller computes $mg$. The result is a constant residual acceleration
 
$$\Delta\ddot{z} = \frac{mg}{m'} - g = -g\,\frac{\Delta m}{m + \Delta m}$$
 
This is a **constant bias on the vertical channel**, which is the one channel with an instantaneous input and relative degree 2. It is the most benign structure a mismatch can have: a constant error, on the fastest channel, requiring no rotation to correct. We predict, before running, that mass will be the easiest of our four to correct and that a residual for it will help at essentially any injection point.
 
**Drag: does not move the hover equilibrium at all, but destroys the trim family.** Drag is proportional to velocity, and at hover the velocity is zero, so the hover equilibrium is untouched: trim thrust is still $mg$ and trim attitude is still level. What drag destroys is the velocity-invariance we noticed above. Holding a constant velocity $v$ now requires a steady tilt to generate an opposing force, approximately
 
$$\delta\theta_{\text{trim}} \approx \frac{d_x v}{mg}$$
 
So drag is a **velocity-dependent, lateral-channel** mismatch, and it acts at relative degree 2 from attitude and 4 from torque. Two predictions follow. First, drag will be invisible in any hover or low-speed test and will grow with speed, so our aggressive trajectories are the right testbed for it and the old 1.5 m circle was not. Second, and more interestingly, drag is a *smooth function of a measured state*. A well-tuned MPC with integral-like action already rejects smooth state-dependent disturbances reasonably well on its own. We therefore predict that a drag residual will be accurate and will nonetheless **change little**, because it is largely predicting a correction the nominal controller was already making. That is our candidate explanation for the "does nothing" regime, and it is a falsifiable one.
 
**Motor lag: does not move the equilibrium at all, and that is exactly why it is dangerous.** A first-order lag between commanded and delivered rotor thrust, $\tau_m \dot f = f_{\text{cmd}} - f$, has *no* effect at equilibrium: in steady state $f = f_{\text{cmd}}$ and every condition above is unchanged. An equilibrium analysis is entirely blind to it.
 
What it changes is the dynamics. It inserts an additional pole in the innermost, fastest loop, contributing phase lag that grows with frequency, and it does so at the entrance to the four-integrator lateral chain identified above. Since that chain has no margin to spare, the lag consumes phase margin directly, and there is a value of $\tau_m$ at which the margin reaches zero. That value is the **cliff**. Its location is not a property of $\tau_m$ alone but of the ratio of $\tau_m$ to the controller's own timescales, which is why it must be expressed as a dimensionless group, $\tau_m / T_{\text{horizon}}$ or $\tau_m \omega_c$, if it is to transfer across vehicles.
 
This produces our sharpest prediction, and the one most worth being wrong about: **a residual that accurately predicts the effect of motor lag can still destabilise the loop**, because the correction it requests must travel through the very channel the lag has degraded, and requesting more aggressive action through a phase-deficient loop is the standard recipe for driving it unstable. Accuracy is irrelevant to this failure mode. This is the "hurts" regime, and if the rebuild reproduces it, this paragraph is the explanation.
 
**Delay: the same character as lag, but worse.** A pure input delay also leaves the equilibrium untouched and also attacks phase margin, but its phase lag grows linearly with frequency without bound rather than saturating at 90 degrees. It is the more aggressive member of the same family. We expect it to behave like motor lag with an earlier cliff.
 
### The classification, and what it buys us
 
| Mismatch | Moves equilibrium? | Channel | Relative degree from torque | Predicted regime |
|---|---|---|---|---|
| Mass | Yes, constant bias | Vertical | 2 | Helps |
| Drag | No, but breaks velocity trim | Lateral, state-dependent | 4 | Does nothing |
| Motor lag | No, attacks phase margin | Lateral, at the chain entrance | 4 | Hurts, past a cliff |
| Delay | No, attacks phase margin harder | Lateral, at the chain entrance | 4 | Hurts, earlier cliff |
 
The column that predicts the regime is not an accuracy column. Nothing here refers to how well a model could learn the mismatch; all four are learnable to high accuracy. What differs is *where the mismatch sits relative to the actuation chain*, and that is our thesis, arriving here as a consequence of equilibrium analysis rather than as an assertion.
 
We record these four predictions now, before building the controller, so that the later experiments can confirm or refute them rather than be fitted to them. **If the rebuilt experiments contradict this table, the table is what changes, and we investigate why.** That is the discipline; a prediction recorded after the fact is not a prediction.
 
---
 
## What this entry commits us to, and what is open
 
**Committed.**
- The hover equilibrium: $\mathbf{v} = \mathbf{0}$, $\boldsymbol{\omega} = \mathbf{0}$, $\mathbf{b}_3 = \mathbf{e}_3$, $T = mg$, $\boldsymbol{\tau} = \mathbf{0}$, $f_i = mg/4$. Position and yaw are free, so the equilibrium is a family, not a point.
- In the nominal model, constant-velocity level flight requires exactly the hover actuation at any speed. Drag is what destroys this.
- Authority at hover: vertical acceleration in $[-g,\; g(\text{TWR}-1)]$, asymmetric; roll and pitch torque $a\,mg\min(1, \text{TWR}-1)$, saturating at TWR of 2; yaw smaller by $c_{\tau f}/a$ and further penalised by the larger $I_{zz}$.
- The normalising groups for the cross-vehicle study: TWR, $a\,mg/I_{xx}$, $c_{\tau f}/a$, and later $\tau_m/T_{\text{horizon}}$.
- The linearised gains: $\ddot z = \delta T/m$, $\ddot x = g\,\delta\theta$, $x^{(4)} = (g/I_{yy})\tau_y$, confirming entry 02's relative-degree ladder with numbers and confirming the sign conventions from the opposite direction.
- The mismatch classification table above, recorded as a set of pre-registered predictions.
**Deferred, noted, not forgotten.**
- The formal stability-margin calculation that locates the motor-lag cliff. We have identified the mechanism, four integrators plus an added pole, but we have not computed where the margin actually vanishes, because that depends on the controller we have not yet built. This is the natural first analysis to run once the MPC exists.
- Linearisation away from hover. Everything above is local to the hover point, and our trajectories are deliberately not local to it. The counts are structural and hold everywhere; the gains and the authority numbers are not, and must be evaluated online. This is the reason our realizability criterion has to be a run-time quantity rather than a lookup table, and it is worth remembering that this entry is where that necessity became visible.
- Actuator lag and delay as explicit extra states, to be added in the perturbation phase along with the decision of whether the lag acts on $f_i$ or on $\Omega_i$.
**Open for the next entry (05, the nominal MPC).** We now know where the vehicle wants to sit, how much authority it has around that point, how fast each channel is, and how each mismatch pushes it off. That is the full brief for a controller: what it must hold, what it has to hold it with, and what it will be fighting. Entry 05 builds the nominal MPC from these ingredients, justifying each term of the cost, each constraint, the horizon length, and the discretisation, with the horizon in particular chosen against the timescales this entry has just measured rather than picked by habit.
 
*This entry is locked. We do not proceed to 05 until the following can be explained in plain language: why the equilibrium is a family rather than a point and which two states are free, why vertical authority is asymmetric and capped at $g$ downward, why roll authority stops improving above a thrust-to-weight ratio of 2, why the lateral channel being four integrators is a control problem rather than merely a bookkeeping fact, and why an equilibrium analysis is completely blind to motor lag even though motor lag is the mismatch we expect to be most dangerous.*