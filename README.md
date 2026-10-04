# Safe Learning-Augmented Model Predictive Control for Quadrotor UAVs

A clean-room research project on **when a controller should trust a learned correction**. We study quadrotor trajectory tracking under model mismatch, where a nominal model predictive controller (MPC) is augmented by a learned residual correction, and we build a run-time **supervisor** that decides, moment by moment, how much of that learned correction to apply, with a guaranteed fallback to the nominal controller. The learned component is allowed to improve tracking when it is trustworthy, but it can never destabilise the loop.

This repository is built from first principles. The nonlinear dynamics, the MPC, the learned residual, and the supervisor are each derived and validated independently, with a derivation notebook recording the reasoning behind every modelling choice.

---

## 1. Summary

Model predictive control is a standard tool for quadrotor trajectory tracking because it plans over a finite horizon while respecting actuator limits. Its accuracy, however, depends on how well its internal model matches the real vehicle, and the two rarely match exactly. Mass changes, aerodynamic drag, motor lag, and sensing or actuation delay all push the plant away from the nominal model and degrade tracking.

A common remedy is to learn the residual between the nominal prediction and the observed motion and feed it back into the controller. In practice, applying such a learned correction unconditionally is not safe: depending on the type of mismatch and on where the correction is injected, the same accurate residual can improve tracking, have no useful effect, or actively destabilise the closed loop.

This project treats that observation as the central research problem. We make the decision explicit: a supervisory layer scales the learned correction by a continuous factor between zero and one, based on whether the correction is physically realisable within the control horizon and whether it is trustworthy and safe to apply. When either condition is threatened, the supervisor shrinks the correction to zero and the system reverts to the unmodified nominal MPC. The nominal controller is never changed; the learned correction only ever modifies an external reference supplied to it.

---

## 2. Problem statement

Learned residual corrections are widely added to quadrotor MPC to compensate for model mismatch, and are usually reported to help. There is, however, no principled and checkable way to predict, before deployment, whether a given learned correction will help, do nothing, or harm the closed loop, and no run-time mechanism that lets the learned correction improve the controller while guaranteeing it cannot break it.

Our prior empirical work indicated that the deciding factor is not the accuracy of the learned model but the structural relationship between what the correction predicts and what the controller can physically act on within the available horizon and actuator limits. A correction can be accurately predicted, and the vehicle can be fully controllable, yet the correction can still be unrealisable at a given injection point over a short horizon. This project formalises that relationship and builds a supervisory controller around it.

---

## 3. Related work and research gap

The project sits at the intersection of learning-augmented MPC, uncertainty-aware control, and safe learning-based control with guarantees. The table below summarises representative prior work, what each contributes, and the gap it leaves open relative to our problem.

| # | Representative work | What it does | Main contribution | Gap relative to this project |
|---|---------------------|--------------|-------------------|------------------------------|
| 1 | Torrente et al., Data-driven MPC for quadrotors [1] | Learns a Gaussian-process residual and embeds it inside the MPC optimisation | Shows learned residuals inside the optimiser improve agile tracking | The correction is applied unconditionally; no run-time decision on when it should or should not be trusted |
| 2 | Salzmann et al., Real-time neural MPC [2] | Integrates neural network dynamics into MPC and solves it in real time | Makes neural-augmented MPC real-time feasible on hardware | Assumes the learned term is beneficial; does not study regimes where it fails or a fallback |
| 3 | Saviolo and Loianno, Learning quadrotor dynamics [3] | Surveys and develops learned dynamics models for precise, safe, agile flight | Consolidates learned-dynamics approaches for quadrotors | Focuses on model accuracy, not on whether a learned correction is realisable or safe to apply |
| 4 | Hewing et al., Cautious MPC using Gaussian process regression [4] | Propagates model uncertainty into the MPC constraints | Uncertainty-aware, cautious control under learned model error | Uncertainty tightens constraints but does not gate a residual by realisability or provide a scalable fallback |
| 5 | Hewing et al., Learning-based MPC: toward safe learning in control [5] | Reviews learning-based MPC and safe-learning formulations | Frames the field and its safety considerations | A survey framing; no specific realisability criterion or supervisory architecture |
| 6 | Brunke et al., Safe learning in robotics [6] | Reviews safe learning from learning-based control to safe RL | Maps the safe-learning landscape and its guarantees | Positions the area; does not provide a run-time trust decision for learned corrections |
| 7 | Shi et al., Neural Lander [7] | Learns ground-effect residual dynamics for stable drone landing | Demonstrates learned residual with closed-loop stability | Residual is always active in its regime; no graded, realisability-aware supervision |
| 8 | Dawson et al., Safe control with learned certificates [8] | Surveys neural Lyapunov, barrier, and contraction certificates | Toolbox for attaching guarantees to learned controllers | Certifies controllers, not a supervisory layer that decides when to trust a learned residual |
| 9 | Faessler et al., Differential flatness subject to rotor drag [9] | Models rotor drag and uses flatness for accurate high-speed tracking | Canonical treatment of the dominant modellable aerodynamic mismatch | Compensates a specific effect; does not generalise to deciding realisability across mismatch types |
| 10 | Garone et al., Reference and command governors [10] | Shrinks a reference to maintain constraint feasibility | Mature theory for safely modifying references online | Shrinks toward a known-safe steady reference for feasibility, not a learned correction scaled by realisability and uncertainty |

**Gap we address (in general terms).** Prior work either applies the learned correction unconditionally, or treats uncertainty and safety at the level of the nominal controller, or supplies certificates for controllers rather than for the decision to trust a learned residual. What is missing is a supervisory layer that decides, at run time, whether a learned residual is both **realisable** within the control horizon under real actuator limits and **safe and trustworthy** to apply, scales it accordingly, and falls back to the nominal controller before harm can occur. Filling this gap is the purpose of the project.

---

## 4. Objectives

1. Derive, from first principles, a complete and validated nonlinear quadrotor model, and build a nominal MPC whose every term can be justified against the vehicle's measured timescales and authority.
2. Characterise how the nominal controller degrades under the four studied mismatches (mass, drag, motor lag, delay) and their combinations, recording qualitative predictions before running the experiments.
3. Define precisely what quantity the learned residual predicts and why predicting it should help control, and build a residual network that also reports a calibrated uncertainty estimate.
4. Formalise a **finite-horizon correction realisability** criterion: whether a requested correction can be produced within the control horizon, at the chosen injection point, under true actuator magnitude and rate limits.
5. Build a two-layer supervisor that scales the learned correction by a continuous factor, combining a hard safety envelope with a soft performance layer, and that reverts to the nominal controller when safety or realisability is threatened.
6. Validate the full system in high-fidelity simulation across rich trajectories and multiple vehicles, and show that the supervisor preserves beneficial correction while removing harmful correction.

---

## 5. Possible contributions

- **C1.** An explicit, computable, injection-point-aware notion of finite-horizon correction realisability that distinguishes model accuracy, controllability, realisability, and effective injection, and that predicts whether a given mismatch is correctable.
- **C2.** A two-layer run-time supervisor (hard safety envelope and soft performance layer) that scales a learned residual continuously and guarantees reversion to the unmodified nominal MPC, so the learned component can help but cannot break the controller.
- **C3.** A first-principles empirical study across rich trajectories and multiple vehicles demonstrating the criterion and the supervisor, including a regime map (supervised versus unsupervised), an adversarial stress test, and cross-vehicle generalisation with normalised thresholds.

These may be refined as experiments land; predictions recorded before experiments are treated as pre-registered and are revised only with evidence.

---

## 6. Roadmap

The project is built in dependency order. Each phase has an understanding gate (the reasoning can be explained in plain language) and a validation gate (a concrete check passes) before the next phase begins.

| Phase | Focus | Key deliverables | Horizon |
|-------|-------|------------------|---------|
| A | Mathematical foundation | Rigid-body dynamics and frames, 13-state model, actuator mixer and constraint polytope, finite-horizon realisability criterion | In progress |
| B | Nominal MPC | MPC built from the derived model; validation on hover, line, circle, figure-eight; mismatch characterisation (mass, drag, lag, delay) | Planned |
| C | Learned residual | Definition of the learned quantity; residual network with ensemble-based uncertainty; oracle test before trusting the network; training with clean holdouts; integration via reference shaping | Planned |
| D | Safety supervisor and theory | Realisability veto and uncertainty gating; barrier and Lyapunov based safety envelope; two-layer scaling with run-time-assurance fallback; do-no-harm guarantee; supervisor calibration | Planned |
| E | Simulation validation | Regime map (supervised versus unsupervised); adversarial stress rollout; cross-vehicle generalisation; criterion ablations | Planned |
| F | Sim-to-real deployment | System identification of a physical quadrotor; on-board deployment of MPC, residual, and supervisor; indoor then outdoor flight tests; sim-to-real gap analysis | Future direction |
| G | Multi-agent extension | A fleet of supervised UAVs; distributed and learning-based cooperative correction; cooperative safety and fault tolerance | Future direction |

Phases A to E constitute the core study. Phases F and G are planned future directions that extend the validated framework to hardware and to cooperative multi-vehicle settings.

---

## 7. Repository structure
uav-mpc-safe-residual/
notebook/ Derivation notebook: dynamics, forces and torques, equations of motion,
actuator model, equilibrium and realisability, each entry recording the
reasoning behind every modelling choice
src/ Dynamics, simulator interface, MPC, reference shaping, perturbations,
realisability primitive, residual network, supervisor, experiment harness
configs/ One configuration file per reproducible run
experiments/ Sweep specifications (regime map, oracle tests, stress rollouts)
tests/ Unit and property-based tests (dynamics conservation laws, supervisor invariants)
README.md This file


---

## 8. Status

The project is in **Phase A**. The full rigid-body equations of motion have been derived and recorded in the notebook, and the actuator model, constraint polytope, and finite-horizon realisability criterion are being formalised. No controller, network, or supervisor code has been written yet; the discipline of the rebuild is to derive and understand each layer before implementing it, and to validate the physics before building control on top of it.

---

## 9. Tooling

Python, with PyTorch for the learned components, CasADi and IPOPT for optimisation-based control, and MuJoCo for simulation. All runs are deterministic (seeded) and carry provenance (configuration plus commit hash) so that any result can be traced to the code that produced it. The project is CPU-only at this stage.

---

## 10. References

[1] G. Torrente, E. Kaufmann, P. Foehn, and D. Scaramuzza, "Data-Driven MPC for Quadrotors," *IEEE Robotics and Automation Letters*, vol. 6, no. 2, 2021.

[2] T. Salzmann, E. Kaufmann, J. Arrizabalaga, M. Pavone, D. Scaramuzza, and M. Ryll, "Real-Time Neural MPC: Deep Learning Model Predictive Control for Quadrotors and Agile Robotic Platforms," *IEEE Robotics and Automation Letters*, vol. 8, no. 4, 2023.

[3] A. Saviolo and G. Loianno, "Learning Quadrotor Dynamics for Precise, Safe, and Agile Flight Control," *Annual Reviews in Control*, vol. 55, 2023.

[4] L. Hewing, J. Kabzan, and M. N. Zeilinger, "Cautious Model Predictive Control Using Gaussian Process Regression," *IEEE Transactions on Control Systems Technology*, vol. 28, no. 6, 2020.

[5] L. Hewing, K. P. Wabersich, M. Menner, and M. N. Zeilinger, "Learning-Based Model Predictive Control: Toward Safe Learning in Control," *Annual Review of Control, Robotics, and Autonomous Systems*, vol. 3, 2020.

[6] L. Brunke, M. Greeff, A. W. Hall, Z. Yuan, S. Zhou, J. Panerati, and A. P. Schoellig, "Safe Learning in Robotics: From Learning-Based Control to Safe Reinforcement Learning," *Annual Review of Control, Robotics, and Autonomous Systems*, vol. 5, 2022.

[7] G. Shi, X. Shi, M. O'Connell, R. Yu, K. Azizzadenesheli, A. Anandkumar, Y. Yue, and S.-J. Chung, "Neural Lander: Stable Drone Landing Control Using Learned Dynamics," *IEEE International Conference on Robotics and Automation (ICRA)*, 2019.

[8] C. Dawson, S. Gao, and C. Fan, "Safe Control With Learned Certificates: A Survey of Neural Lyapunov, Barrier, and Contraction Methods for Robotics and Control," *IEEE Transactions on Robotics*, vol. 39, no. 3, 2023.

[9] M. Faessler, A. Franchi, and D. Scaramuzza, "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories," *IEEE Robotics and Automation Letters*, vol. 3, no. 2, 2018.

[10] E. Garone, S. Di Cairano, and I. Kolmanovsky, "Reference and Command Governors for Systems with Constraints: A Survey on Theory and Applications," *Automatica*, vol. 75, 2017.

[11] S. Berkenkamp, M. Turchetta, A. P. Schoellig, and A. Krause, "Safe Model-Based Reinforcement Learning with Stability Guarantees," *Advances in Neural Information Processing Systems (NeurIPS)*, 2017.

[12] A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, and P. Tabuada, "Control Barrier Functions: Theory and Applications," *European Control Conference (ECC)*, 2019.

[13] L. Sha, "Using Simplicity to Control Complexity," *IEEE Software*, vol. 18, no. 4, 2001.

[14] R. Mahony, V. Kumar, and P. Corke, "Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor," *IEEE Robotics and Automation Magazine*, vol. 19, no. 3, 2012.

[15] D. Mellinger and V. Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors," *IEEE International Conference on Robotics and Automation (ICRA)*, 2011.

[16] J. Sola, "Quaternion Kinematics for the Error-State Kalman Filter," *arXiv:1711.02508*, 2017.

[17] J. B. Rawlings, D. Q. Mayne, and M. M. Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill Publishing, 2017.

[18] J. A. E. Andersson, J. Gillis, G. Horn, J. B. Rawlings, and M. Diehl, "CasADi: A Software Framework for Nonlinear Optimization and Optimal Control," *Mathematical Programming Computation*, vol. 11, no. 1, 2019.

[19] A. Waechter and L. T. Biegler, "On the Implementation of an Interior-Point Filter Line-Search Algorithm for Large-Scale Nonlinear Programming," *Mathematical Programming*, vol. 106, no. 1, 2006.

[20] E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A Physics Engine for Model-Based Control," *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2012.