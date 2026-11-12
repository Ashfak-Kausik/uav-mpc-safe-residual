"""Entry 06: the nominal MPC for tracking. Every number quoted in 06_nominal_mpc.md is computed here.

Two implementations are analysed (amendment of 2026-11-12):
  a = apply-on-solve (zero modelled computation delay; jitter up to the solve time is unmodelled)
  b = fixed one-step computation delay, modelled by predicting x_{k+1} with the command in flight (chosen)
"""
import numpy as np
from scipy.linalg import expm
import params as P
from quad_sym import Report, qmul, _R_np
from linmodel import continuous, zoh, NX, NU, NY, MIX, C, G, f_hover
from mpc06 import (DT, N, Q, R, HEADROOM, U_MIN, U_MAX, DU_MAX, TILT_MAX, Ad, Bd, Pm, K, T_OFF, Kx, Kr, A_LAG, IMPL,
                   CondensedQP, solve_daqp, kkt_residuals, OSQPSolver, closed_loop, spectral_radius,
                   channel_margins, cliff)

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
rep.check("chosen implementation is (b), modelled one-step computation delay", IMPL == "b")

# ---------------------------------------------------------------- the QP
qp = CondensedQP()
rep.check("condensed QP: 86 decision variables (80 inputs, 4 artificial steady state, 2 slacks), 180 general rows",
          qp.nz == 86 and qp.Acon.shape == (180, 86))
eigH = np.linalg.eigvalsh(qp.H)
rep.check("QP Hessian is positive definite (strictly convex)", eigH.min() > 0, f"min eig {eigH.min():.1f}, cond {eigH.max()/eigH.min():.1e}")
n_sparse = NX * N + NU * N + NY + 2
rep.check("equivalent sparse form: 406 variables with 320 dynamics equalities", n_sparse == 406 and NX * N == 320)

feasible = True
for _ in range(50):
    x0 = rng.normal(size=NX) * np.r_[[2] * 3, [3] * 3, [0.8] * 3, [10] * 3, [0.03] * 4]
    up = rng.uniform(U_MIN, U_MAX, size=NU)
    r = rng.normal(size=NY) * 2
    zc = np.r_[np.tile(up, N), r, 1e3, 1e3]
    f, bl, bu = qp.vectors(x0, up, r)
    lhs = np.r_[zc, qp.Acon @ zc]
    feasible &= bool(np.all(lhs >= bl - 1e-9) and np.all(lhs <= bu + 1e-9))
    z, flag, _ = solve_daqp(qp, x0, up, r)
    feasible &= (flag == 1)
rep.check("feasibility is trivial: 'hold the previous command' satisfies every hard constraint, and DAQP solves all 50 random cases",
          feasible)

worst = 0
for _ in range(20):
    x0 = rng.normal(size=NX) * 1e-3
    x0[2] += 1.0                      # flying at 1 m, away from the soft altitude floor at 0 m
    r = rng.normal(size=NY) * 1e-2
    r[2] += 1.0
    z, flag, _ = solve_daqp(qp, x0, np.zeros(NU), r)
    worst = max(worst, np.abs(z[:NU] - (Kx @ x0 + Kr @ r)).max())
rep.check("with no constraint active the QP's first move equals the affine law u = Kx x + Kr r", worst < 1e-10, f"{worst:.1e}")


# ---------------------------------------------------------------- closed-loop helpers (both implementations)
def offset_and_lag(impl):
    """(altitude offset for +10 % and +20 % mass, ramp lag at 1 m/s without and with drag) for an implementation."""
    out = []
    for ratio in (1.1, 1.2):
        mp = P.m * ratio
        A, B = continuous()
        A[5, 12:16] = 1 / mp
        Mx = np.zeros((21, 21)); Mx[:16, :16] = A; Mx[5, 16] = P.g * (P.m / mp - 1); Mx[:16, 17:21] = B
        Ex = expm(Mx * DT)
        A0, _ = closed_loop(impl=impl, extra=(Ex[:16, :16], Ex[:16, 17:21]))
        rhs = np.zeros(A0.shape[0]); rhs[:16] = Ex[:16, 16]
        out.append(np.linalg.solve(np.eye(A0.shape[0]) - A0, rhs)[2])
    for drag in (0.0, D["d_xy ~ K_aero*4*w_hover [N s/m]"]):
        A, B = continuous()
        A[3, 3] = -drag / P.m
        A0, B0 = closed_loop(impl=impl, extra=zoh(A, B, DT))
        z = np.zeros(A0.shape[0])
        for k in range(1500):
            z = A0 @ z + B0 @ np.array([k * DT, 0, 0, 0])
        out.append(1499 * DT - z[0])
    return out


def hinf_r_to_p(impl):
    Acl, Bcl = closed_loop(impl=impl)
    ws = np.linspace(1e-3, np.pi / DT, 6000)
    return max(np.linalg.svd(np.linalg.solve(np.exp(1j * w * DT) * np.eye(Acl.shape[0]) - Acl, Bcl)[[0, 1, 2]][:, :3],
                             compute_uv=False)[0] for w in ws)


res = {}
for impl in ("a", "b"):
    Acl, Bcl = closed_loop(impl=impl)
    mg = channel_margins(20000, impl=impl)
    res[impl] = dict(rho=spectral_radius(Acl), mg=mg, dc=np.linalg.solve(np.eye(Acl.shape[0]) - Acl, Bcl),
                     Dc=cliff("D", 0.0, 0.08, impl=impl), Lc=cliff("tau_true", 1.5 * tau, 6 * tau, impl=impl),
                     hinf=hinf_r_to_p(impl), ol=offset_and_lag(impl))
    print(f"   implementation ({impl}):")
    for k, v in mg.items():
        print(f"     {k:10s} w_c = {v['wc']:6.2f} rad/s = {v['wc']*t_att:.2f}/t_att   PM = {v['pm']:5.1f} deg   GM = {v['gm']:6.2f}"
              f" ({20*np.log10(v['gm']):4.1f} dB)   PM/w_c = {np.radians(v['pm'])/v['wc']*1e3:6.1f} ms")
    print(f"     delay cliff {res[impl]['Dc']*1e3:.2f} ms, lag cliff {res[impl]['Lc']/tau:.3f} x tau_m, rho {res[impl]['rho']:.4f}")

a, b = res["a"], res["b"]
# ---- implementation (a): the original entry-06 numbers (kept visible)
rp = a["mg"]["roll"]
rep.check("(a) nominal closed loop Schur stable, spectral radius 0.9587", abs(a["rho"] - 0.9587) < 1e-4)
rep.check("(a) roll/pitch: w_c = 28.5 rad/s = 1.21/t_att, PM = 41.5 deg, GM = 5.8 (15.3 dB)",
          abs(rp["wc"] - 28.5) < 0.3 and abs(rp["wc"] * t_att - 1.21) < 0.02 and abs(rp["pm"] - 41.5) < 0.5
          and abs(rp["gm"] - 5.83) < 0.05 and abs(a["mg"]["pitch"]["pm"] - rp["pm"]) < 0.2)
rep.check("(a) collective: w_c = 7.6 rad/s, PM = 52 deg, GM 24.4; yaw: w_c = 9.2 rad/s, PM = 54 deg, GM 19.9",
          abs(a["mg"]["collective"]["wc"] - 7.6) < 0.1 and abs(a["mg"]["collective"]["pm"] - 52.2) < 0.5
          and abs(a["mg"]["yaw"]["wc"] - 9.25) < 0.1 and abs(a["mg"]["yaw"]["pm"] - 54.0) < 0.5
          and abs(a["mg"]["collective"]["gm"] - 24.4) < 0.1 and abs(a["mg"]["yaw"]["gm"] - 19.9) < 0.1)
rep.check("(a) delay cliff D = 25.4 ms = 2.54 dt = 0.60 t_att; PM/w_c agrees to 0.2 ms",
          abs(a["Dc"] - 0.02537) < 2e-4 and abs(np.radians(rp["pm"]) / rp["wc"] - a["Dc"]) < 2e-4)
rep.check("(a) lag cliff: true tau_m = 3.28 x nominal", abs(a["Lc"] / tau - 3.28) < 0.01)

# ---- implementation (b): chosen
rb = b["mg"]["roll"]
rep.check("(b) the predictor is exact: nominal closed-loop spectral radius unchanged (0.9587)", abs(b["rho"] - a["rho"]) < 1e-9)
rep.check("(b) roll/pitch: w_c = 25.2 rad/s = 1.07/t_att, PM = 35.6 deg, GM = 2.66 (8.5 dB)",
          abs(rb["wc"] - 25.2) < 0.2 and abs(rb["wc"] * t_att - 1.07) < 0.02 and abs(rb["pm"] - 35.6) < 0.3
          and abs(rb["gm"] - 2.66) < 0.03 and abs(20 * np.log10(rb["gm"]) - 8.5) < 0.1)
rep.check("(b) collective: w_c = 7.3 rad/s, PM = 49.8 deg, GM = 8.9; yaw: w_c = 8.8 rad/s, PM = 51.3 deg, GM = 7.4",
          abs(b["mg"]["collective"]["wc"] - 7.34) < 0.1 and abs(b["mg"]["collective"]["pm"] - 49.8) < 0.5
          and abs(b["mg"]["collective"]["gm"] - 8.87) < 0.1 and abs(b["mg"]["yaw"]["wc"] - 8.8) < 0.1
          and abs(b["mg"]["yaw"]["pm"] - 51.3) < 0.5 and abs(b["mg"]["yaw"]["gm"] - 7.4) < 0.1)
rep.check("(b) each loop has a single gain crossover", all(v["n_cross"] == 1 for v in b["mg"].values()))
rep.check("(b) additional-delay cliff D = 24.6 ms = 2.46 dt = 0.58 t_att (beyond the modelled period); PM/w_c agrees to 0.2 ms",
          abs(b["Dc"] - 0.02462) < 2e-4 and abs(b["Dc"] / DT - 2.46) < 0.02 and abs(b["Dc"] / t_att - 0.58) < 0.01
          and abs(np.radians(rb["pm"]) / rb["wc"] - b["Dc"]) < 2e-4)
rep.check("(b) lag cliff: true tau_m = 2.86 x nominal (inside a 4x sweep, outside +-50 %)", abs(b["Lc"] / tau - 2.86) < 0.01 and 1.5 < b["Lc"] / tau < 4)
ok = all(spectral_radius(closed_loop(tau_true=s * tau, impl="b")[0]) < 1 for s in (0.3, 0.5, 0.7, 1.3, 1.5, 2.0))
rep.check("(b) lag errors of -70, -50, -30, +30, +50, +100 % all leave the nominal loop stable", ok)
Lc_lo = cliff("tau_true", 0.1 * tau, 0.3 * tau, impl="b")
rep.check("(b) LOWER lag cliff: a true motor faster than 0.186 tau_m (12 ms = 1.2 dt) destabilizes the predictor-based loop",
          abs(Lc_lo / tau - 0.186) < 0.002 and abs(Lc_lo / DT - 1.20) < 0.02
          and spectral_radius(closed_loop(tau_true=0.1 * tau, impl="b")[0]) > 1)
rep.check("(a) has no lower lag cliff: true motors down to 0.01 tau_m stay stable",
          all(spectral_radius(closed_loop(tau_true=s * tau, impl="a")[0]) < 1 for s in (0.01, 0.05, 0.1)))
rep.check("(a) and (b): zero steady-state offset for constant references (DC gain r -> (p, psi) = I)",
          np.allclose(a["dc"][[0, 1, 2, 8]], np.eye(4), atol=1e-9) and np.allclose(b["dc"][[0, 1, 2, 8]], np.eye(4), atol=1e-9))
rep.check("(a) and (b): reference-to-position gain finite, frequency-sweep estimate of the H-infinity norm 1.00",
          abs(a["hinf"] - 1) < 1e-3 and abs(b["hinf"] - 1) < 1e-3, f"{a['hinf']:.6f}, {b['hinf']:.6f}")
rep.check("mass +10 % -> altitude offset -49 mm (a) / -53 mm (b, the predictor uses the nominal mass); linear in the error",
          abs(a["ol"][0] * 1e3 + 49.3) < 0.5 and abs(b["ol"][0] * 1e3 + 53.0) < 0.5
          and all(abs(r_["ol"][1] / r_["ol"][0] - 2.0) < 0.01 for r_ in (a, b)))
rep.check("ramp 1 m/s: setpoint lag 0.466 m (a) / 0.476 m (b, one period more); drag adds 0.023 m (a) / 0.024 m (b)",
          abs(a["ol"][2] - 0.466) < 0.002 and abs(b["ol"][2] - 0.476) < 0.002
          and abs((a["ol"][3] - a["ol"][2]) - 0.0227) < 0.001 and abs((b["ol"][3] - b["ol"][2]) - 0.0239) < 0.001)

# ---- delay budget: total tolerable effective delay minus hold, computation and estimation
T_SOLVE_WORST_EST = 0.009          # on-board worst-case solve estimate (entry 06, section 10) -- an estimate
budget_a = (a["Dc"] + DT / 2) - DT / 2 - T_SOLVE_WORST_EST
budget_b = (b["Dc"] + DT / 2 + DT) - DT / 2 - DT
rep.check("budget left for injected delay + estimation latency: (a) 25.4 - 9 = 16.4 ms (if the 9 ms worst-case solve holds), (b) 24.6 ms",
          abs(budget_a - 0.0164) < 3e-4 and abs(budget_b - 0.0246) < 3e-4 and budget_b > budget_a)
rep.check("total tolerable effective delay at the cliff: (a) 30.4 ms; (b) 39.6 ms (hold 5 + computation 10 + 24.6)",
          abs(a["Dc"] + DT / 2 - 0.0304) < 3e-4 and abs(b["Dc"] + DT / 2 + DT - 0.0396) < 3e-4)

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
    Rz = np.array([[np.cos(psi0), -np.sin(psi0), 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]])
    q = np.array(qmul(qexp(np.array([0, 0, psi0])), qexp(xe[6:9])))
    x = np.r_[Rz @ xe[0:3], Rz @ xe[3:6], q, xe[9:12], f_hover + xe[12:16]]
    dx = plant_f(x, np.full(4, f_hover), P.I_diag)
    return np.r_[Rz.T @ dx[0:3], Rz.T @ dx[3:6], xe[9:12], dx[10:13], dx[13:17]]


A_c, _ = continuous()
worst = 0
for psi0 in (0.0, 0.7, 2.0, -2.8):
    J = np.array([approx_fprime(np.zeros(16), lambda e, i=i: error_dyn(e, psi0)[i], 1e-7) for i in range(16)])
    worst = max(worst, np.abs(J - A_c).max())
rep.check("in the heading frame the hover linearisation is identical at any yaw (one LTI model serves all headings)",
          worst < 1e-4, f"max |dA| = {worst:.1e}")


# ---------------------------------------------------------------- nonlinear closed loop with the constrained QP
def logq(q):
    q = q if q[0] >= 0 else -q
    vn = np.linalg.norm(q[1:])
    return np.zeros(3) if vn < 1e-12 else 2 * np.arctan2(vn, q[0]) * q[1:] / vn


def step(x, u, h):
    f = lambda s: plant_f(s, u, P.I_full)
    k1 = f(x); k2 = f(x + h / 2 * k1); k3 = f(x + h / 2 * k2); k4 = f(x + h * k3)
    y = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4); y[6:10] /= np.linalg.norm(y[6:10])
    return y


def run(impl, steps=400):
    xs = np.zeros(17); xs[6] = 1; xs[2] = 1.0; xs[13:17] = f_hover
    fhat = np.zeros(NU); u_prev = np.zeros(NU)
    r = np.array([1.0, 0.0, 1.5, 0.3])
    osq = OSQPSolver(qp)
    log = dict(U=[], X=[], flag=[], kkt=[], osqp=[], st=[], it=[], ost=[])
    for k in range(steps):
        xe = np.r_[xs[0:6], logq(xs[6:10]), xs[10:13], fhat]
        x0 = xe if impl == "a" else Ad @ xe + Bd @ u_prev          # (b): predict x_{k+1} with the command in flight
        z, flag, info = solve_daqp(qp, x0, u_prev, r)
        zo, status, oinfo = osq.solve(x0, u_prev, r)
        u = z[:NU]
        log["U"].append(u); log["X"].append(xs.copy()); log["flag"].append(flag)
        log["kkt"].append(kkt_residuals(qp, z, info["lam"], x0, u_prev, r))
        log["osqp"].append(np.abs(zo[:NU] - u).max())
        log["st"].append(info["solve_time"]); log["it"].append(info["iterations"]); log["ost"].append(oinfo.solve_time)
        applied = u if impl == "a" else u_prev                     # (b): u_k is applied from t_{k+1}
        for _ in range(10):
            xs = step(xs, f_hover + applied, 0.001)
        fhat = A_LAG * fhat + (1 - A_LAG) * applied
        u_prev = u
    log = {k: np.array(v) for k, v in log.items()}
    return xs, r, log


for impl in ("a", "b"):
    xs, r, L = run(impl)
    U, X = L["U"], L["X"]
    tilt = np.degrees(max(np.arccos(np.clip(_R_np(x[6:10])[2, 2], -1, 1)) for x in X))
    rep.check(f"({impl}) nonlinear plant + constrained QP: 1 m x, 0.5 m z, 0.3 rad yaw step converges to 1e-4",
              np.abs(xs[0:3] - r[0:3]).max() < 1e-4 and abs(logq(xs[6:10])[2] - 0.3) < 1e-4)
    rep.check(f"({impl}) commands inside [f_min, f_max] and the rate limit, ceiling reached during the step",
              U.min() + f_hover >= P.f_min - 1e-9 and U.max() + f_hover <= P.f_max + 1e-9
              and np.abs(np.diff(U, axis=0)).max() <= DU_MAX + 1e-9 and abs(U.max() + f_hover - P.f_max) < 1e-9)
    rep.check(f"({impl}) soft tilt limit (20 deg) held to within 0.3 deg; altitude never below its start",
              tilt < np.degrees(TILT_MAX) + 0.3 and X[:, 2].min() > 0.999, f"max tilt {tilt:.2f} deg")
    kk = L["kkt"]
    rep.check(f"({impl}) every DAQP solution (400) is optimal and satisfies KKT to 1e-8 "
              "(primal feasibility, relative stationarity, complementarity)",
              set(L["flag"]) == {1} and kk[:, 0].max() < 1e-8 and kk[:, 1].max() < 1e-8 and kk[:, 2].max() < 1e-8,
              f"max primal {kk[:, 0].max():.1e}, stationarity {kk[:, 1].max():.1e}, complementarity {kk[:, 2].max():.1e}")
    rep.check(f"({impl}) OSQP sanity check: first moves agree with DAQP to 1e-3 f_max (= 1.2e-4 N)",
              L["osqp"].max() < 1e-3 * P.f_max, f"observed max difference {L['osqp'].max():.1e} N")
    st, it, ost = L["st"] * 1e3, L["it"], L["ost"] * 1e3
    print(f"   ({impl}) PC solve time, DAQP internal: median {np.median(st):.3f} ms, p99 {np.percentile(st, 99):.3f} ms,"
          f" max {st.max():.3f} ms; iterations median {np.median(it):.0f}, max {it.max()};"
          f" OSQP internal median {np.median(ost):.3f} ms")
    rep.check(f"({impl}) PC: DAQP solves every step in under 10 ms; worst case from saturated transients (max iter >> median)",
              st.max() < 10 and it.max() > 10 * np.median(it))

# ---------------------------------------------------------------- arithmetic quoted in the entry
rep.check("rate limit: full thrust range in 4 periods (40 ms) = 0.62 tau_m", abs(4 * DU_MAX - (P.f_max - P.f_min)) < 1e-15 and abs(0.04 / tau - 0.62) < 0.005)
rep.check("20 deg tilt: sin is 2.0 % below its argument, cos is 6.0 % below 1",
          abs(1 - np.sin(TILT_MAX) / TILT_MAX - 0.020) < 0.001 and abs(1 - np.cos(TILT_MAX) - 0.060) < 0.001)
rep.check("N = 15 would give 66 variables", NU * 15 + NY + 2 == 66)
rep.check("single-precision storage: Hessian 86x86 ~ 30 kB, constraint matrix 180x86 ~ 62 kB",
          abs(86 * 86 * 4 / 1000 - 29.6) < 0.1 and abs(180 * 86 * 4 / 1000 - 61.9) < 0.1)
rep.check("on-board estimate arithmetic: (86/60)^2 = 2.05; 1.375 ms x 2.05 = 2.8 ms; x3 iterations = 8.5 ms",
          abs((86 / 60) ** 2 - 2.05) < 0.01 and abs(1.375 * (86 / 60) ** 2 - 2.82) < 0.01 and abs(3 * 1.375 * (86 / 60) ** 2 - 8.47) < 0.01)
rep.check("offset weight T = 100 G'PG", np.allclose(T_OFF, 100 * G.T @ Pm @ G))

raise SystemExit(rep.finish())
