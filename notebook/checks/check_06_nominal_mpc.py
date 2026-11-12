"""Entry 06: the nominal MPC for tracking. Every number quoted in 06_nominal_mpc.md is computed here."""
import numpy as np
import params as P
from quad_sym import Report, qmul, _R_np
from linmodel import continuous, zoh, NX, NU, NY, MIX, C, G, f_hover
from mpc06 import (DT, N, Q, R, HEADROOM, U_MIN, U_MAX, DU_MAX, TILT_MAX, Ad, Bd, Pm, K, T_OFF, Kx, Kr, A_LAG,
                   CondensedQP, solve_daqp, OSQPSolver, closed_loop, spectral_radius, channel_margins, cliff)

rep = Report("check_06_nominal_mpc")
D = P.derived()
t_att, tau = D["t_att [s]"], P.tau_m
rng = np.random.default_rng(6)

# ---------------------------------------------------------------- model, horizon, weights
ctrb = np.hstack([np.linalg.matrix_power(Ad, k) @ Bd for k in range(NX)])
rep.check("16-state lagged hover model is controllable from the 4 commanded thrusts", np.linalg.matrix_rank(ctrb) == NX)
rep.check("horizon N = 20 (200 ms) = 4.7 t_att = 3.1 tau_m", N == 20 and abs(N * DT / t_att - 4.73) < 0.01 and abs(N * DT / tau - 3.09) < 0.01)
rep.check("input weight from rotor headroom about hover: min(f_max - f_h, f_h - f_min) = 0.0489 N (ceiling side)",
          abs(HEADROOM - 0.0489) < 5e-5 and abs(HEADROOM - (P.f_max - f_hover)) < 1e-15)
rep.check("terminal cost P (DARE) is positive definite", np.linalg.eigvalsh(Pm).min() > 0)
rep.check("input box in deviation coordinates: [-0.0583, +0.0489] N; rate limit 0.0268 N per period",
          abs(U_MIN + 0.0583) < 5e-5 and abs(U_MAX - 0.0489) < 5e-5 and abs(DU_MAX - 0.0268) < 5e-5)

# ---------------------------------------------------------------- the QP
qp = CondensedQP()
rep.check("condensed QP: 86 decision variables (80 inputs, 4 artificial steady state, 2 slacks), 180 general rows",
          qp.nz == 86 and qp.Acon.shape == (180, 86))
eigH = np.linalg.eigvalsh(qp.H)
rep.check("QP Hessian is positive definite (strictly convex)", eigH.min() > 0, f"min eig {eigH.min():.1f}, cond {eigH.max()/eigH.min():.1e}")
# sparse (non-condensed) form, for the record
n_sparse = NX * N + NU * N + NY + 2
rep.check("equivalent sparse form: 406 variables with 320 dynamics equalities", n_sparse == 406 and NX * N == 320)

# hard constraints are inputs only, so holding the previous command is always feasible
feasible = True
for _ in range(50):
    x0 = rng.normal(size=NX) * np.r_[[2] * 3, [3] * 3, [0.8] * 3, [10] * 3, [0.03] * 4]
    up = rng.uniform(U_MIN, U_MAX, size=NU)
    r = rng.normal(size=NY) * 2
    zc = np.r_[np.tile(up, N), r, 1e3, 1e3]                          # hold u_prev, theta = r, big slacks
    f, bl, bu = qp.vectors(x0, up, r)
    lhs = np.r_[zc, qp.Acon @ zc]
    feasible &= bool(np.all(lhs >= bl - 1e-9) and np.all(lhs <= bu + 1e-9))
    z, flag, _ = solve_daqp(qp, x0, up, r)
    feasible &= (flag == 1)
rep.check("feasibility is trivial: 'hold the previous command' satisfies every hard constraint, and DAQP solves all 50 random cases",
          feasible)

# unconstrained regime: QP solution equals the affine law (Target Result 1 needs this)
worst = 0
for _ in range(20):
    x0 = rng.normal(size=NX) * 1e-3
    x0[2] += 1.0                      # flying at 1 m, away from the soft altitude floor at 0 m
    r = rng.normal(size=NY) * 1e-2
    r[2] += 1.0
    z, flag, _ = solve_daqp(qp, x0, np.zeros(NU), r)
    worst = max(worst, np.abs(z[:NU] - (Kx @ x0 + Kr @ r)).max())
rep.check("with no constraint active the QP's first move equals the affine law u = Kx x + Kr r", worst < 1e-10, f"{worst:.1e}")

# ---------------------------------------------------------------- nominal loop: stability, offset, margins
Acl, Bcl = closed_loop()
rho = spectral_radius(Acl)
rep.check("Assumption 2 (nominal): closed loop with the on-board lag model is Schur stable, spectral radius 0.9587",
          abs(rho - 0.9587) < 1e-4 and rho < 1)
dc = np.linalg.solve(np.eye(Acl.shape[0]) - Acl, Bcl)
rep.check("zero steady-state offset for constant references (DC gain r -> (p, psi) = I)",
          np.allclose(dc[[0, 1, 2, 8]], np.eye(4), atol=1e-9))
ws = np.linspace(1e-3, np.pi / DT, 6000)
hinf = max(np.linalg.svd(np.linalg.solve(np.exp(1j * w * DT) * np.eye(Acl.shape[0]) - Acl, Bcl)[[0, 1, 2]][:, :3],
                         compute_uv=False)[0] for w in ws)
rep.check("finite reference-to-position gain; frequency-sweep estimate of its H-infinity norm = 1.00 (no peaking)",
          abs(hinf - 1.0) < 1e-3, f"{hinf:.6f}")

mg = channel_margins(20000)
for k, v in mg.items():
    print(f"   {k:10s} w_c = {v['wc']:6.2f} rad/s = {v['wc']*t_att:.2f}/t_att   PM = {v['pm']:5.1f} deg   GM = {v['gm']:6.2f}"
          f"   PM/w_c = {np.radians(v['pm'])/v['wc']*1e3:6.1f} ms")
rp = mg["roll"]
rep.check("roll/pitch loop: w_c = 28.5 rad/s = 1.21/t_att, PM = 41.5 deg, GM = 5.8 (15.3 dB)",
          abs(rp["wc"] - 28.5) < 0.3 and abs(rp["wc"] * t_att - 1.21) < 0.02 and abs(rp["pm"] - 41.5) < 0.5
          and abs(rp["gm"] - 5.83) < 0.05 and abs(mg["pitch"]["pm"] - rp["pm"]) < 0.2)
rep.check("collective loop: w_c = 7.6 rad/s, PM = 52 deg; yaw loop: w_c = 9.2 rad/s, PM = 54 deg",
          abs(mg["collective"]["wc"] - 7.6) < 0.1 and abs(mg["collective"]["pm"] - 52.2) < 0.5
          and abs(mg["yaw"]["wc"] - 9.25) < 0.1 and abs(mg["yaw"]["pm"] - 54.0) < 0.5)
rep.check("each loop has a single gain crossover", all(v["n_cross"] == 1 for v in mg.values()))
rep.check("sampling: w_c dt = 0.29 rad (crossover far below Nyquist, 314 rad/s)", abs(rp["wc"] * DT - 0.285) < 0.005)

# ---------------------------------------------------------------- re-registered cliffs
Dc = cliff("D", 0.0, 0.08)
dm_est = np.radians(rp["pm"]) / rp["wc"]
rep.check("delay cliff (exact, eigenvalues of the delayed loop): D = 25.4 ms = 2.54 dt = 0.60 t_att",
          abs(Dc - 0.02537) < 2e-4 and abs(Dc / DT - 2.54) < 0.02 and abs(Dc / t_att - 0.60) < 0.01)
rep.check("loop-margin estimate PM/w_c agrees with the exact delay cliff to within 0.2 ms", abs(dm_est - Dc) < 2e-4,
          f"{dm_est*1e3:.2f} vs {Dc*1e3:.2f} ms")
rep.check("effective loop delay at the cliff, D + dt/2 = 30.4 ms = 0.72 t_att", abs(Dc + DT / 2 - 0.0304) < 2e-4)
Lc = cliff("tau_true", 3 * tau, 5 * tau)
rep.check("motor-lag cliff: true tau_m = 3.28 x nominal = 0.212 s = 5.0 t_att", abs(Lc / tau - 3.28) < 0.01 and abs(Lc / t_att - 5.01) < 0.02)
ok = all(spectral_radius(closed_loop(tau_true=s * tau)[0]) < 1 for s in (0.1, 0.5, 0.7, 1.3, 1.5, 2.0))
rep.check("lag errors of -90 %, -50 %, -30 %, +30 %, +50 %, +100 % all leave the nominal loop stable", ok)


# mass error: steady altitude offset (no disturbance model)
def mass_offset(ratio):
    from scipy.linalg import expm
    mp = P.m * ratio
    A, B = continuous()
    A[5, 12:16] = 1 / mp
    Mx = np.zeros((21, 21)); Mx[:16, :16] = A; Mx[5, 16] = P.g * (P.m / mp - 1); Mx[:16, 17:21] = B
    Ex = expm(Mx * DT)
    Phi, dvec, Gam = Ex[:16, :16], Ex[:16, 16], Ex[:16, 17:21]
    A0, _ = closed_loop()
    n = A0.shape[0]
    Fz = np.zeros((NU, n)); Fz[:, :12] = Kx[:, :12]; Fz[:, NX:NX + NU] = Kx[:, 12:16]
    A0[:16] = np.hstack([Phi, np.zeros((16, n - 16))]) + Gam @ Fz
    rhs = np.zeros(n); rhs[:16] = dvec
    return np.linalg.solve(np.eye(n) - A0, rhs)[2]


o10, o20 = mass_offset(1.1), mass_offset(1.2)
rep.check("mass +10 %: steady altitude offset -49 mm (below the reference); +20 %: -99 mm (linear in the error)",
          abs(o10 * 1e3 + 49.3) < 0.5 and abs(o20 / o10 - 2.0) < 0.01)


# ramp reference (piecewise-constant setpoint formulation): steady lag, and the share due to drag
def ramp_lag(drag):
    A, B = continuous()
    A[3, 3] = -drag / P.m
    Phi, Gam = zoh(A, B, DT)
    A0, B0 = closed_loop()
    n = A0.shape[0]
    Fz = np.zeros((NU, n)); Fz[:, :12] = Kx[:, :12]; Fz[:, NX:NX + NU] = Kx[:, 12:16]
    A0[:16] = np.hstack([Phi, np.zeros((16, n - 16))]) + Gam @ Fz
    z = np.zeros(n)
    for k in range(1500):
        z = A0 @ z + B0 @ np.array([k * DT, 0, 0, 0])
    return 1499 * DT - z[0]


l0, l1 = ramp_lag(0.0), ramp_lag(D["d_xy ~ K_aero*4*w_hover [N s/m]"])
rep.check("setpoint-only reference on a 1 m/s ramp: steady lag 0.47 m; drag adds only 0.023 m",
          abs(l0 - 0.466) < 0.002 and abs((l1 - l0) - 0.0227) < 0.001)

# ---------------------------------------------------------------- heading frame: the hover model is yaw-invariant
from scipy.optimize import approx_fprime


def plant_f(x, u, I):
    p, v, q, w, f = x[0:3], x[3:6], x[6:10], x[10:13], x[13:17]
    T, tx, ty, tz = MIX @ f
    Rm = _R_np(q)
    vdot = T / P.m * Rm[:, 2] - np.array([0, 0, P.g])
    qd = 0.5 * np.array(qmul(q, [0, *w]))
    wd = np.linalg.solve(I, np.array([tx, ty, tz]) - np.cross(w, I @ w))
    return np.concatenate([v, vdot, qd, wd, (u - f) / tau])


def qexp(v):
    th = np.linalg.norm(v)
    return np.array([1.0, 0, 0, 0]) if th < 1e-15 else np.r_[np.cos(th / 2), np.sin(th / 2) * v / th]


def error_dyn(xe, psi0):
    """16-state error dynamics about hover at yaw psi0, positions/velocities in the heading frame."""
    Rz = np.array([[np.cos(psi0), -np.sin(psi0), 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]])
    q = np.array(qmul(qexp(np.array([0, 0, psi0])), qexp(xe[6:9])))
    x = np.r_[Rz @ xe[0:3], Rz @ xe[3:6], q, xe[9:12], f_hover + xe[12:16]]
    dx = plant_f(x, np.full(4, f_hover), P.I_diag)
    # attitude-error rate = w to first order at hover; position/velocity rates back in the heading frame
    return np.r_[Rz.T @ dx[0:3], Rz.T @ dx[3:6], xe[9:12], dx[10:13], dx[13:17]]


A_c, _ = continuous()
worst = 0
for psi0 in (0.0, 0.7, 2.0, -2.8):
    J = np.array([approx_fprime(np.zeros(16), lambda e, i=i: error_dyn(e, psi0)[i], 1e-7) for i in range(16)])
    worst = max(worst, np.abs(J - A_c).max())
rep.check("in the heading frame the hover linearisation is identical at any yaw (one LTI model serves all headings)",
          worst < 1e-4, f"max |dA| = {worst:.1e}")

# ---------------------------------------------------------------- nonlinear closed loop with the constrained QP
xs = np.zeros(17); xs[6] = 1; xs[2] = 1.0; xs[13:17] = f_hover
fhat = np.zeros(NU); up = np.zeros(NU)
r = np.array([1.0, 0.0, 1.5, 0.3])
osq = OSQPSolver(qp)
U, X, flags, dif, st, it, ost = [], [], [], [], [], [], []


def logq(q):
    q = q if q[0] >= 0 else -q
    vn = np.linalg.norm(q[1:])
    return np.zeros(3) if vn < 1e-12 else 2 * np.arctan2(vn, q[0]) * q[1:] / vn


def step(x, u, h):
    f = lambda s: plant_f(s, u, P.I_full)
    k1 = f(x); k2 = f(x + h / 2 * k1); k3 = f(x + h / 2 * k2); k4 = f(x + h * k3)
    y = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4); y[6:10] /= np.linalg.norm(y[6:10])
    return y


for k in range(400):
    xe = np.r_[xs[0:6], logq(xs[6:10]), xs[10:13], fhat]
    z, flag, info = solve_daqp(qp, xe, up, r)
    zo, status, oinfo = osq.solve(xe, up, r)
    u = z[:NU]
    U.append(u); X.append(xs.copy()); flags.append(flag); dif.append(np.abs(zo[:NU] - u).max())
    st.append(info["solve_time"]); it.append(info["iterations"]); ost.append(oinfo.solve_time)
    for _ in range(10):
        xs = step(xs, f_hover + u, 0.001)
    fhat = A_LAG * fhat + (1 - A_LAG) * u
    up = u
U, X = np.array(U), np.array(X)
tilt = np.degrees(max(np.arccos(np.clip(_R_np(x[6:10])[2, 2], -1, 1)) for x in X))
rep.check("nonlinear plant (full inertia, RK4 1 ms) + constrained QP: step of 1 m in x, 0.5 m in z, 0.3 rad in yaw converges",
          np.abs(xs[0:3] - r[0:3]).max() < 1e-4 and abs(logq(xs[6:10])[2] - 0.3) < 1e-4)
rep.check("commanded thrusts stay inside [f_min, f_max] and the rate limit, with the ceiling reached during the step",
          U.min() + f_hover >= P.f_min - 1e-9 and U.max() + f_hover <= P.f_max + 1e-9
          and np.abs(np.diff(U, axis=0)).max() <= DU_MAX + 1e-9 and abs(U.max() + f_hover - P.f_max) < 1e-9)
rep.check("soft tilt limit (20 deg) held to within 0.2 deg; altitude never below its start", tilt < np.degrees(TILT_MAX) + 0.2 and X[:, 2].min() > 0.999,
          f"max tilt {tilt:.2f} deg")
rep.check("DAQP and OSQP return the same first move (to 1e-5) on all 400 solves; DAQP always optimal",
          max(dif) < 1e-5 and set(flags) == {1})
st, it, ost = np.array(st) * 1e3, np.array(it), np.array(ost) * 1e3
print(f"   PC solve time, DAQP internal: median {np.median(st):.3f} ms, p99 {np.percentile(st, 99):.3f} ms, max {st.max():.3f} ms;"
      f" iterations median {np.median(it):.0f}, max {it.max()}")
print(f"   PC solve time, OSQP internal: median {np.median(ost):.3f} ms, p99 {np.percentile(ost, 99):.3f} ms, max {ost.max():.3f} ms")
rep.check("PC: DAQP solves every step in under 10 ms (worst case recorded for the on-board estimate)", st.max() < 10)
rep.check("active-set worst case comes from saturated transients (max iterations >> median)", it.max() > 10 * np.median(it))

# ---------------------------------------------------------------- arithmetic quoted in the entry
rep.check("rate limit: full thrust range in 4 periods (40 ms) = 0.62 tau_m", abs(4 * DU_MAX - (P.f_max - P.f_min)) < 1e-15 and abs(0.04 / tau - 0.62) < 0.005)
rep.check("20 deg tilt: sin is 2.0 % below its argument, cos is 6.0 % below 1",
          abs(1 - np.sin(TILT_MAX) / TILT_MAX - 0.020) < 0.001 and abs(1 - np.cos(TILT_MAX) - 0.060) < 0.001)
rep.check("gain margins in dB: 5.83 -> 15.3, 24.4 -> 27.8, 19.9 -> 26.0",
          abs(20 * np.log10(rp["gm"]) - 15.3) < 0.05 and abs(20 * np.log10(mg["collective"]["gm"]) - 27.8) < 0.05
          and abs(20 * np.log10(mg["yaw"]["gm"]) - 26.0) < 0.05)
rep.check("N = 15 would give 66 variables", NU * 15 + NY + 2 == 66)
rep.check("single-precision storage: Hessian 86x86 ~ 30 kB, constraint matrix 180x86 ~ 62 kB",
          abs(86 * 86 * 4 / 1000 - 29.6) < 0.1 and abs(180 * 86 * 4 / 1000 - 61.9) < 0.1)
rep.check("on-board estimate arithmetic: (86/60)^2 = 2.05; 1.375 ms x 2.05 = 2.8 ms; x3 iterations = 8.5 ms",
          abs((86 / 60) ** 2 - 2.05) < 0.01 and abs(1.375 * (86 / 60) ** 2 - 2.82) < 0.01 and abs(3 * 1.375 * (86 / 60) ** 2 - 8.47) < 0.01)
rep.check("offset weight T = 100 G'PG", np.allclose(T_OFF, 100 * G.T @ Pm @ G))

raise SystemExit(rep.finish())
