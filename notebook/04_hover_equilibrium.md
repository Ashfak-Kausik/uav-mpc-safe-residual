# 04 — Hover Equilibrium and Linearisation

*Derivation notebook, entry 04. Continues the clean-room rebuild from entries 00 to 03. This entry finds the drone's equilibrium, measures how much authority sits around it, and uses it to work out how each of our four mismatches pushes the vehicle off balance. It is the bridge from "here is the physics" toward "here is the controller."*

*Revised after the audit ([99_audit.md](99_audit.md)) and decisions Q1–Q5 ([decisions.md](decisions.md)). Numbers are for the committed Crazyflie 2.1+ parameter set of entry 01 and are computed in [checks/params.py](checks/params.py) and [checks/check_04_hover.py](checks/check_04_hover.py).*

---

## What this entry is for

Entries 00 to 03 gave us a complete nonlinear model. It has thirteen rigid-body states, four delivered rotor thrusts behind a motor lag, four commanded thrusts as the input, and the equations that connect them. That model can be simulated, but it cannot yet be *reasoned about*, because nonlinear equations do not readily tell you which channels are fast, how much room is left over for corrections, or what a given mismatch will actually do to the loop. This entry supplies that reasoning by doing the one thing that always comes next: find the equilibrium, and look at the dynamics in its neighbourhood.

Three things come out of it, and all three are things the rest of the project needs.

1. **The equilibrium itself**, which tells us the trim rotor thrusts and, importantly, that the equilibrium is not a single point but a family.
2. **The authority margin around it**, meaning how much torque and thrust are actually available above trim. This is the first place entry 01's constraint polytope produces a number rather than a shape, and those numbers are what our realizability criterion will be normalised against.
3. **A structural classification of our four mismatches** into those that *move the equilibrium* and those that *change the dynamics about it*. This is the sharpest thing in the entry. It is, we believe, the core of the reason our prior study found some mismatches correctable and one destabilising. It lets us write down predictions before we run anything, with one honest caveat: the predictions also depend on controller choices that entry 06 makes, so they are stated conditionally.

---

## Finding the equilibrium

An equilibrium is a state at which every rate is zero, so that the vehicle left alone stays exactly where it is. Setting the left-hand sides of entry 03's four equations, and of entry 01's motor lag, to zero and reading them from the bottom up:

**From the motor lag, $\tau_m\dot{\mathbf f} = \mathbf f^{\text{cmd}} - \mathbf f = \mathbf 0$:** the delivered thrusts equal the commanded ones. At equilibrium the lag is invisible.

**From $\dot{\boldsymbol{\omega}} = \mathbf{0}$:** with ω = 0 the gyroscopic term vanishes, so the condition is simply $\boldsymbol{\tau} = \mathbf{0}$. No net torque.

**From $\dot{\mathbf{q}} = \mathbf{0}$:** the quaternion kinematics give zero only when ω = 0, which we already have. So this equation imposes nothing new; it is automatically satisfied and, crucially, it does **not** pin down the orientation.

**From $\dot{\mathbf{v}} = \mathbf{0}$:** we need $\tfrac{T}{m}\mathbf{b}_3 = g\mathbf{e}_3$. Since $\mathbf{b}_3$ is a unit vector and the right-hand side points along world z, this forces $\mathbf{b}_3 = \mathbf{e}_3$, meaning the thrust axis must point straight up, and then $T = mg$.

**From $\dot{\mathbf{p}} = \mathbf{0}$:** we need $\mathbf{v} = \mathbf{0}$, the vehicle is not translating. The position itself is unconstrained.

Inverting entry 01's mixer at $T = mg$, $\boldsymbol{\tau} = \mathbf{0}$ gives the trim rotor thrusts, commanded and delivered alike:

$$\boxed{\;f_i^{\text{hover}} = f_i^{\text{cmd,hover}} = \frac{mg}{4}, \quad i = 1,\dots,4\;}$$

Every rotor carries a quarter of the weight. This is the answer we expected, and getting it out of the machinery rather than assuming it is the point.

### The equilibrium is a family, not a point

Two of the thirteen rigid-body states were left free by the conditions above, and both matter.

**Position is free.** Any point in space is an equilibrium, which is the formal statement that the vehicle has no preferred location and therefore no natural restoring force toward a reference. This is exactly the job we are handing to the controller.

**Yaw is free.** The condition was $\mathbf{b}_3 = \mathbf{e}_3$, which fixes only two of the three rotational degrees of freedom. Rotation about the thrust axis is unconstrained, so the equilibrium set includes a full circle of yaw angles. We already saw this algebraically in entry 03, where $\mathbf{b}_3$ turned out to be independent of rotation about itself. Here it acquires its physical meaning: **yaw is dynamically decoupled from equilibrium**, which is why it is specified separately in a trajectory and why an error in it produces no restoring tendency whatsoever.

### What is not an equilibrium, and why that is informative

Level flight at a constant nonzero velocity is *not* an equilibrium of the full state, since position keeps changing. But it is what is usually called a trim or relative equilibrium, and in our nominal model it has a striking property: because the nominal model contains no velocity-dependent force, **the thrust and attitude required to hold constant velocity are identical to those required to hover**, at any speed, in any direction. The vehicle cannot tell, from its own actuation, whether it is stationary or cruising at 10 m/s.

Record that, because it is precisely what drag destroys, and noticing what a perturbation destroys is how we will classify the perturbations below.

---

## How much authority is there around hover?

At trim each rotor sits at $mg/4$, with a ceiling at $f_{\max}$ and a floor at $f_{\min}$. The gap between trim and those bounds is the entire budget from which every manoeuvre and every correction must be paid. Entry 01 gave the constraint set; here we turn it into numbers.

Define the thrust-to-weight ratio $\text{TWR} = 4f_{\max}/(mg)$, so that trim sits at a fraction $1/\text{TWR}$ of each rotor's range above the floor (for $f_{\min} = 0$). For the committed Crazyflie 2.1+ parameters, **TWR = 1.69**.

**Vertical authority** is the easy case. Maximum upward acceleration is $g(\text{TWR} - 1)$, obtained by commanding all rotors to $f_{\max}$: 6.7 m/s² for our vehicle. Maximum downward acceleration is $g$, obtained by cutting thrust entirely, and it cannot exceed $g$ because the rotors cannot pull. **Vertical authority is asymmetric.** With TWR below 2 it is now asymmetric in the *opposite* direction from a high-TWR vehicle: our vehicle can fall faster (up to $g$) than it can climb (up to $0.69g$). Both limits are tight enough that vertical corrections will meet them.

**Roll and pitch authority** requires holding $T = mg$ while spreading the rotor thrusts apart. Maximising $\tau_x = a(f_1 + f_2 - f_3 - f_4)$ subject to $\sum f_i = mg$ and $f_{\min} \le f_i \le f_{\max}$ pushes one pair up and the other down until something saturates. Since $\tau_x = a\,(2(f_1+f_2) - mg)$ and $f_1 + f_2$ is capped both by $2f_{\max}$ and by $mg - 2f_{\min}$ (the other pair cannot go below the floor), the result is

$$\tau_x^{\max}\big|_{\text{hover}} = a\,\min\!\big(4f_{\max} - mg,\; mg - 4f_{\min}\big) \;\overset{f_{\min}=0}{=}\; a\,mg \cdot \min\!\big(1,\; \text{TWR} - 1\big).$$

[checks/check_01_mixer.py](checks/check_01_mixer.py) checks both forms against an exact linear program. If $\text{TWR} \ge 2$ (with $f_{\min} = 0$), the binding limit is the *floor*: the descending pair reaches zero thrust before the ascending pair reaches its ceiling, and extra TWR buys no further torque at hover. If $\text{TWR} < 2$, the *ceiling* binds first and authority scales with the margin above hover.

**Our vehicle is on the ceiling-limited side.** With TWR = 1.69, $\tau_x^{\max} = a(4f_{\max} - mg) = 6.4\times10^{-3}$ N·m, which is 69 percent of the $a\,mg = 9.3\times10^{-3}$ N·m a floor-limited vehicle of the same size would have. Three consequences follow, and they are recorded here because they shape every later entry:
- **Saturation and realizability limits will be active more often.** Any request that pushes a rotor up runs into the ceiling before any request that pushes a rotor down reaches the floor. The realizability ratio of entry 09 will therefore be informative over much more of the flight envelope than on a high-TWR vehicle, and the aggressive trajectories will live close to it.
- **Upward thrust and attitude torque compete for the same headroom.** Climbing, or holding a tilt (which costs $1/\cos\theta$ in thrust, entry 02), uses up the ceiling margin that torque needs. The maximum coordinated tilt is $\arccos(1/\text{TWR}) = 54°$, and at that tilt no torque authority is left at all.
- **The committed TWR is the least certain number in the model.** The cross-source spread is 1.69 to about 2.5 (entry 01). Above 2 the vehicle would be floor-limited instead, so the bench re-identification of $f_{\max}$ can move us across the knee. Every result that depends on it is reported together with the TWR it was computed for.

The whole calculation is evaluated *at hover*. Away from hover the available torque shrinks further, exactly as entry 01's polytope warned.

**Yaw authority** has the identical structure with $c_{\tau f}$ in place of $a$:

$$\tau_z^{\max}\big|_{\text{hover}} = c_{\tau f}\,\min\!\big(4f_{\max} - mg,\; mg - 4f_{\min}\big), \qquad \frac{\tau_z^{\max}}{\tau_x^{\max}} = \frac{c_{\tau f}}{a} = 0.215 .$$

Converting torques to angular accelerations divides by the relevant inertia, and here the two effects compound in the same direction: yaw is driven by the smaller constant *and* resisted by the larger inertia ($I_{zz} = 1.77\,I_{xx}$). The available yaw angular acceleration is $0.215/1.77 = 0.12$ of the roll angular acceleration. Entry 03 predicted yaw would be the first axis to run out; this is the number behind that prediction.

### The normalising quantities

These authority expressions are the reason our cross-vehicle claim can be stated without per-vehicle tuning. The quantities that actually govern behaviour are dimensionless:

- **TWR** = $4f_{\max}/(mg)$ (1.69), together with $f_{\min}/f_{\text{hover}}$ (0 here).
- **The yaw penalty** $c_{\tau f}/a$ (0.215) and the inertia ratio $I_{zz}/I_{xx}$ (1.77).
- **Time ratios against the attitude time scale.** The maximum angular acceleration at hover, $a\,mg/I_{xx}$, is *not* dimensionless: its units are s⁻². An earlier version of this entry listed it as a dimensionless group, which was wrong. Its inverse square root is a time,

$$t_{\text{att}} = \sqrt{\frac{I_{xx}}{a\,m\,g}} \approx 42\ \text{ms},$$

the time scale on which the vehicle can reorient. Every time in the problem is normalised by it, or by the closed-loop crossover frequency $\omega_c$ once entry 06 fixes the controller: the motor time constant ($\tau_m/t_{\text{att}} = 1.53$), the delay, the control period $\Delta t/t_{\text{att}}$, and the horizon $T_{\text{horizon}}/t_{\text{att}}$.

The value $\tau_m/t_{\text{att}} = 1.53$ is the quantitative reason behind decision Q1. On this vehicle the motor is slower than the attitude dynamics it drives. A model that ignored it would be mismatched before anything is injected. Any threshold in our supervisor should be expressed in these quantities, not in newtons and metres.

---

## Linearising about hover: where the chain gets numbers

Entry 02 counted the steps from each plant input to each output. Entry 03 showed the same count as the literal cascade of the equations. Here we attach gains to it, which is what turns a structural claim into a computable one.

Take a small perturbation about hover. Write the quaternion in its small-angle form, $\mathbf{q} \approx (1, \delta\phi/2, \delta\theta/2, \delta\psi/2)$, where $\delta\phi, \delta\theta, \delta\psi$ are small roll, pitch and yaw deviations. Substituting into entry 03's expression for $\mathbf{b}_3$ and keeping first order terms:

$$\mathbf{b}_3 \approx \begin{bmatrix} \delta\theta \\ -\delta\phi \\ 1 \end{bmatrix}$$

Substituting that into entry 02's core equation and writing $T = mg + \delta T$, the three translational channels decouple to first order:

$$\ddot{x} \approx g\,\delta\theta, \qquad \ddot{y} \approx -g\,\delta\phi, \qquad \ddot{z} \approx \frac{\delta T}{m}$$

Read the signs against our conventions and they check out: positive pitch is nose down, which tilts thrust forward and accelerates along positive x; positive roll lifts the left side, which tilts thrust to the right and accelerates along negative y. These are the same signs entry 01's mixer produces, arrived at from the opposite direction, which is a useful consistency check on the whole chain.

Now push through the attitude dynamics, $\delta\ddot{\theta} = \tau_y / I_{yy}$ (with the MPC's diagonal inertia), and through the motor lag, $\tau_m\,\delta\dot{f}_i = \delta f_i^{\text{cmd}} - \delta f_i$, to get the full input-to-output transfer functions:

| Channel | Linearised relation | Order | Transfer function |
|---|---|---|---|
| Vertical, from delivered thrust | $\ddot z = \delta T / m$ | 2 | $\dfrac{1}{m s^2}$ |
| Lateral, from attitude deviation | $\ddot x = g\,\delta\theta$ | 2 | $\dfrac{g}{s^2}$ |
| Lateral, from body torque | $x^{(4)} = (g/I_{yy})\,\tau_y$ | 4 | $\dfrac{g}{I_{yy}s^4}$ |
| Lateral, from *commanded* pitch torque (lag in the nominal model) | — | 5 | $\dfrac{g}{I_{yy}s^4(\tau_m s + 1)}$ |
| Vertical, from *commanded* collective thrust | — | 3 | $\dfrac{1}{m s^2(\tau_m s + 1)}$ |
| Yaw, from torque | $\ddot\psi = \tau_z / I_{zz}$ | 2 | $\dfrac{1}{I_{zz}s^2}$ |

Three observations we will use directly.

**The lateral channel is a chain of four integrators, and on this vehicle a lag sits in front of it.** Four integrators in series contribute 360 degrees of phase lag on their own, so all of the phase margin has to be manufactured by the controller. The motor lag adds up to 90 degrees more, $\arctan(\tau_m\omega)$ at frequency ω. With $\tau_m/t_{\text{att}} = 1.53$, it does so already at the frequencies where the attitude loop must act. This lag is now *in the nominal model*, so the MPC accounts for it. What remains dangerous is any lag or delay the MPC does *not* know about: an error in $\tau_m$, and the actuation delay.

**The vertical channel is a chain of two (three with the lag).** Two integrators is a benign, textbook object with plenty of margin. Vertical and lateral are not merely different in degree; they are in different difficulty classes.

**The lateral gain is $g/I_{yy}$, and $g$ is not adjustable.** Lateral acceleration is bought entirely by tilting against gravity. This explains why the tilt angle, rather than any actuator quantity, is the natural currency for lateral authority.

---

## How each mismatch attacks the equilibrium

This is the section the entry exists for. We take our four perturbations and ask a single question of each: *does it move the equilibrium, or does it change the dynamics about the equilibrium?* The answer sorts them cleanly. A second question then decides what a correction can do: **does the nominal MPC already reject a constant disturbance by itself?** Decision Q3 fixes the primary nominal MPC **without** a disturbance model (no integral action). Offset-free MPC is kept as a separate, strong baseline. Every prediction below is therefore stated for both cases, and the primary case is marked.

**Mass error: moves the equilibrium, in the best possible channel.** If the true mass is $m' = m + \Delta m$ while the controller believes $m$, then at the controller's hover thrust $mg$ the vehicle accelerates by

$$\Delta\ddot{z} = \frac{mg}{m'} - g = -g\,\frac{\Delta m}{m + \Delta m}.$$

This is an **open-loop** statement: it is what would happen if the controller kept applying exactly $mg$. A feedback controller does not do that. It raises thrust in proportion to the altitude error, so the closed loop settles where the extra thrust balances the extra weight:
- **Without a disturbance model (primary):** a constant altitude offset, roughly $\Delta m\,g$ divided by the vertical feedback gain of the MPC.
- **With one (offset-free baseline):** zero offset.

In both cases the mismatch is a constant bias on the vertical channel, which has the shortest chain (two integrators) and requires no rotation to correct. It is the most benign structure a mismatch can have. With TWR = 1.69, a *heavier* vehicle also eats into the already small upward margin, so large mass errors will meet the ceiling.

**Drag: does not move the hover equilibrium at all, but destroys the trim family.** Drag is proportional to velocity, and at hover the velocity is zero, so the hover equilibrium is untouched: trim thrust is still $mg$ and trim attitude is still level. What drag destroys is the velocity-invariance we noticed above. Holding a constant velocity $v$ now requires a steady tilt to generate an opposing force, approximately

$$\delta\theta_{\text{trim}} \approx \frac{d_x v}{mg}.$$

With the order-of-magnitude drag of entry 01 this is about 0.025 rad (1.4°) per m/s. The exact trim with the full $\mathbf R\mathbf D\mathbf R^\top\mathbf v$ model agrees to 0.2 percent up to 2 m/s ([checks/check_04_hover.py](checks/check_04_hover.py)). So drag is a **velocity-dependent, lateral-channel** mismatch acting at the end of the four-integrator chain, and it is **small** on this vehicle at moderate speed. Its severity is normalised as $d\,v_{\max}/(mg)$, the trim tilt at the fastest point of the trajectory.

**Motor lag: does not move the equilibrium; on this vehicle the mismatch is an error in the time constant.** A first-order lag has *no* effect at equilibrium: in steady state $f = f_{\text{cmd}}$ and every condition above is unchanged. An equilibrium analysis is entirely blind to it. Because the nominal model now contains the identified lag (decision Q1), the mismatch we study is an **error in $\tau_m$**: the true time constant differs from the modelled one, for example by ±30–50 percent. A *slower* true motor than modelled adds unmodelled phase lag at the entrance to the lateral chain. A faster one removes some, and is expected to be benign.

The unmodelled phase lag at a frequency ω is $\arctan(\tau_m^{\text{true}}\omega) - \arctan(\tau_m^{\text{nom}}\omega)$. Since the lateral chain has no margin to spare, this consumes phase margin directly, and there is a value of $\tau_m^{\text{true}}$ at which the margin reaches zero. That value is the **cliff**. The physically meaningful group is $\tau_m\,\omega_c$, the lag measured against the closed-loop crossover ω_c, which sets the phase loss. The ratio to the horizon, $\tau_m/T_{\text{horizon}}$, plays a different, secondary role: it measures how much of the lag's response the MPC's prediction covers. An earlier version of this entry used the horizon ratio as the cliff's location, and that mixed the two.

A residual that accurately predicts the effect of an unmodelled lag **can still destabilise the loop**. The correction it requests must travel through the very channel the lag has degraded, and requesting more aggressive action through a phase-deficient loop is the standard recipe for driving it unstable. Accuracy is irrelevant to this failure mode. This was the "hurts" regime of the earlier study. Two things now weaken the prediction, and we record them rather than hide them. First, with the identified lag in the nominal model, the remaining unmodelled lag is smaller than the full lag the earlier study faced. Second, with injection at the reference, the correction passes through the constraint-aware MPC and the closed-loop transfer $T_{ry}$ (entry 02, table b), not around them. **The "hurts" regime may therefore weaken or vanish for lag errors.** If it does, that is a finding, not a failure.

**Delay: the same character as lag, but worse, and still a pure mismatch.** A pure input delay also leaves the equilibrium untouched and also attacks phase margin, but its phase lag $\omega T_d$ grows linearly with frequency without bound, rather than saturating at 90 degrees. The nominal model does not contain it (decision Q1). We expect it to behave like an unmodelled lag with an earlier cliff, and it is now the more likely of the two to show the "hurts" regime.

**A further caveat on lag and delay corrections.** Neither moves the equilibrium, so neither has a steady-state effect. A correction computed as "the reference offset that cancels the mismatch at steady state" therefore has nothing to cancel for them. The definition of the correct reference correction for a zero-DC mismatch is an open problem, and it is the critical one for entry 08 (see [decisions.md](decisions.md), "Critical open problem F-2").

### The classification, and what it buys us

| Mismatch | Moves equilibrium? | Channel (chain order from its input) | Nominal MPC **without** disturbance model (primary) | Nominal MPC **with** disturbance model (offset-free baseline) |
|---|---|---|---|---|
| Mass | Yes, constant bias | Vertical (2 from thrust) | Constant altitude offset; a residual removes it: **helps** | Offset removed by the MPC; a residual is largely redundant: **little change** |
| Drag | No, but breaks velocity trim | Lateral, state-dependent (4 from torque) | Position lag on constant-velocity segments, small on this vehicle; a residual reduces it: **helps a little**, growing with speed | Steady lag removed, transients remain: **little change** |
| Motor-lag error ($\tau_m$ wrong) | No, changes phase | Lateral, at the chain entrance | Slower-than-modelled motor loses phase margin; a state-dependent residual adds loop gain: **hurts past a threshold**, possibly weakened or absent under reference injection | Same, with the observer's dynamics added: the threshold moves (earlier with a fast observer) |
| Delay | No, changes phase faster | Lateral, at the chain entrance | As lag error, **with an earlier threshold** | As lag error |

The columns that predict the regime are not accuracy columns. Nothing here refers to how well a model could learn the mismatch; all four are learnable to high accuracy. What differs is *where the mismatch sits relative to the actuation chain*, and *whether the nominal controller already compensates it*. That is our thesis, arriving here as a consequence of equilibrium analysis rather than as an assertion.

We record these predictions now, before building the controller, so that later experiments can confirm or refute them rather than be fitted to them. They are **provisional**: entry 06 fixes the controller (horizon, weights, crossover), and the predictions are **re-registered after entry 06** with the closed loop's actual margins. **If the experiments contradict the re-registered table, the table is what changes, and we investigate why.** That is the discipline; a prediction recorded after the fact is not a prediction.

---

## What this entry commits us to, and what is open

**Committed.**
- The hover equilibrium: $\mathbf{v} = \mathbf{0}$, $\boldsymbol{\omega} = \mathbf{0}$, $\mathbf{b}_3 = \mathbf{e}_3$, $T = mg$, $\boldsymbol{\tau} = \mathbf{0}$, $f_i = f_i^{\text{cmd}} = mg/4$. Position and yaw are free, so the equilibrium is a family, not a point.
- In the nominal model, constant-velocity level flight requires exactly the hover actuation at any speed. Drag is what destroys this.
- Authority at hover, for general $f_{\min}$:
  - vertical acceleration in $[-g,\; g(\text{TWR}-1)]$;
  - roll and pitch torque $a\min(4f_{\max} - mg,\ mg - 4f_{\min})$;
  - yaw smaller by $c_{\tau f}/a$ and further penalised by the larger $I_{zz}$.
- For the committed Crazyflie 2.1+: TWR = 1.69, so the vehicle is **ceiling-limited**. Torque authority is 69 percent of the floor-limited value, the maximum coordinated tilt is 54°, and saturation and realizability limits will be active often.
- The normalising quantities: TWR, $f_{\min}/f_{\text{hover}}$, $c_{\tau f}/a$, $I_{zz}/I_{xx}$, and every time measured against $t_{\text{att}} = \sqrt{I_{xx}/(a m g)} \approx 42$ ms or against the crossover $\omega_c$ (in particular $\tau_m\omega_c$, delay/$t_{\text{att}}$, $\Delta t/t_{\text{att}}$). The quantity $a\,mg/I_{xx}$ is an angular acceleration, not a dimensionless group.
- The linearised transfer functions in the table above, including the motor-lag pole in front of each channel.
- The mismatch classification table above, with both disturbance-model columns, as provisional pre-registered predictions; the primary nominal MPC has no disturbance model (Q3).

**Deferred, noted, not forgotten.**
- The formal stability-margin calculation that locates the lag-error and delay cliffs. The mechanism is identified, but the location depends on the controller. It is the first analysis to run once entry 06 exists, together with the re-registration of the predictions.
- Linearisation away from hover. Everything above is local to the hover point, and our trajectories are deliberately not local to it. The orders are structural and hold everywhere; the gains and the authority numbers are not, and must be evaluated online. This is why the realizability criterion has to be a run-time quantity rather than a lookup table.
- The plant's representation of the motor lag (on thrust or on rotor speed) and of the delay buffer: entry 07.

**Open for the next entry (05, discretisation).** Before building a controller we must say how continuous time becomes discrete time, twice. The plant is integrated finely (RK4 with quaternion renormalisation) and the MPC predicts with a coarser discrete model. Entry 05 defines both, measures the gap between them as the noise floor, and checks the integrator's order. Entry 06 then builds the nominal MPC, with the horizon chosen against the timescales measured here ($t_{\text{att}}$, $\tau_m$) rather than picked by habit.

*This entry is locked. We do not proceed to 05 until the following can be explained in plain language: why the equilibrium is a family rather than a point and which two states are free; why vertical authority is asymmetric and capped at $g$ downward; why roll authority is ceiling-limited for our vehicle (TWR < 2) and what that means for saturation; why $a\,mg/I_{xx}$ is not a dimensionless group and what $t_{\text{att}}$ is; why the lateral channel being four integrators behind a motor lag is a control problem rather than a bookkeeping fact; why an equilibrium analysis is blind to motor lag and delay; and why every prediction depends on whether the nominal MPC has a disturbance model.*
