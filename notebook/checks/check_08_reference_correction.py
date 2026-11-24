"""Entry 08: the reference correction (resolves F-2 and F-7). Every number quoted in 08_reference_correction.md
is computed here. Linear closed loop, implementation (b), trajectory-reference MPC unless stated."""
import numpy as np
import params as P
from quad_sym import Report
from linmodel import NX, NU
from mpc06 import DT, N, K, U_MIN, U_MAX, DU_MAX, TILT_MAX
from ref08 import (Fx, Fflat, Fpos, NWS, LAMBDA, simulate, closed_loop_traj, rho, bisect, oracle, traj_step,
                   traj_ramp, traj_fig8, interior, rms_int)

rep = Report("check_08_reference_correction")

# ---------------------------------------------------------------- 0. the trajectory-reference law
rep.check("trajectory MPC: the feedback part of the unconstrained law equals the LQR gain K (Fx = K)", np.abs(Fx - K).max() < 1e-9,
          f"{np.abs(Fx - K).max():.1e}")
A0 = closed_loop_traj()
x0 = np.zeros(NX); x0[0] = 0.01; x0[7] = 0.02
Yf = simulate(np.zeros((6, 60, 4)), x0=x0)
zs = [np.r_[x0, np.zeros(A0.shape[0] - NX)]]
for _ in range(59):
    zs.append(A0 @ zs[-1])
rep.check("closed-loop matrix reproduces the simulated free response (60 steps)", np.abs(np.array(zs)[:, 0] - Yf[:, 0]).max() < 1e-12)
rd = bisect(lambda D: rho(closed_loop_traj(D=D)) - 1, 0.0, 0.04)
rl_hi = bisect(lambda s: rho(closed_loop_traj(tau_ratio=s)) - 1, 1.5, 4.0)
rl_lo = bisect(lambda s: rho(closed_loop_traj(tau_ratio=s)) - 1, 0.1, 0.3)
rep.check("trajectory law: spectral radius 0.9582; delay cliff 24.6 ms; lag cliffs 0.186 and 2.86 (entry 06 values to 0.1 ms / 0.005)",
          abs(rho(A0) - 0.9582) < 1e-4 and abs(rd - 0.02459) < 1e-4 and abs(rl_lo - 0.186) < 0.002 and abs(rl_hi - 2.857) < 0.003,
          f"{rd*1e3:.2f} ms, {rl_lo:.3f}, {rl_hi:.3f}")

# ---------------------------------------------------------------- 1. reference content: trajectory vs constant set-point
ramp, f8, step = traj_ramp(), traj_fig8(), traj_step()
k4 = int(4.0 / DT)
lag_traj = ramp[0, k4, 0] - simulate(ramp)[k4, 0]
lag_const = ramp[0, k4, 0] - simulate(ramp, law="const")[k4, 0]
rep.check("1 m/s ramp: the trajectory reference removes the steady lag (< 1 um); the constant-reference ablation lags 0.476 m",
          abs(lag_traj) < 1e-6 and abs(lag_const - 0.476) < 0.002, f"{lag_traj:.1e} m vs {lag_const:.3f} m")
acc = np.linalg.norm(f8[2, :, :2], axis=1).max()
spd = np.linalg.norm(f8[1, :, :2], axis=1).max()
rep.check("figure-8 (A = 0.28 m, period 3 s): peak speed 0.83 m/s, peak acceleration 3.38 m/s^2 = 19.0 deg of tilt",
          abs(spd - 0.829) < 0.005 and abs(acc - 3.38) < 0.02 and abs(np.degrees(np.arctan(acc / P.g)) - 19.0) < 0.15,
          f"{spd:.3f} m/s, {acc:.3f} m/s^2")
e_traj, e_const = rms_int(f8[0], simulate(f8)), rms_int(f8[0], simulate(f8, law="const"))
print(f"   figure-8 interior RMS: trajectory {e_traj*1e3:.4f} mm, constant set-point {e_const*1e3:.1f} mm")
rep.check("figure-8, nominal plant: trajectory reference tracks to < 0.05 mm RMS (interior); constant set-point ablation 232 mm",
          e_traj < 5e-5 and abs(e_const - 0.232) < 0.003, f"{e_traj*1e3:.4f} mm")
cons = []
for name, pl in (("step", step), ("ramp", ramp), ("fig8", f8)):
    for mm in (dict(), dict(mass_ratio=1.3), dict(D=0.02), dict(tau_ratio=2.0)):
        Y, U, X = simulate(pl, full=True, **mm)
        cons.append((name, str(mm), U.min(), U.max(), np.abs(np.diff(U, axis=0)).max(), np.abs(X[:, 6:8]).max()))
for cc in cons:
    print(f"   {cc[0]:5s} {cc[1]:22s} u in [{cc[2]:+.4f}, {cc[3]:+.4f}] N, max step {cc[4]:.4f} N, max tilt {np.degrees(cc[5]):5.1f} deg")
rep.check("step 0.1 m, ramp 1 m/s and figure-8, nominal and at mass +30 %, delay 20 ms, lag x2: box, rate and tilt constraints inactive",
          all(U_MIN < c[2] and c[3] < U_MAX and c[4] < DU_MAX and c[5] < TILT_MAX for c in cons))

# ---------------------------------------------------------------- 2. taxonomy: feedforward vs feedback corrections
dr_any = np.zeros((f8.shape[1], 4)); dr_any[:, 0] = 0.01 * np.sin(np.arange(f8.shape[1]) * 0.05)
xp = np.zeros(NX); xp[0] = 0.001
d1 = simulate(f8, dr=dr_any, x0=xp, D=0.02) - simulate(f8, dr=dr_any, D=0.02)
d2 = simulate(f8, x0=xp, D=0.02) - simulate(f8, D=0.02)
rep.check("feedforward correction: the response to an initial perturbation is identical with and without it (eigenvalues unchanged)",
          np.abs(d1 - d2).max() < 1e-14)


def Kc_form(form, c):
    Kc = np.zeros((4, 12))
    if form == "vel":
        Kc[0, 3] = Kc[1, 4] = c * DT
    elif form == "pos":
        Kc[0, 0] = Kc[1, 1] = Kc[2, 2] = -c
    elif form == "tilt":
        Kc[0, 7] = c; Kc[1, 6] = -c
    return Kc


def dcliff(Kc):
    return bisect(lambda D: rho(closed_loop_traj(Kc=Kc, D=D)) - 1, 0.0, 0.06)


fb = {(f, c): dcliff(Kc_form(f, c)) for f, c in (("vel", -2), ("vel", 4), ("pos", -0.5), ("pos", 1.0),
                                                ("tilt", -0.2), ("tilt", -0.1), ("tilt", 0.1), ("tilt", 0.2))}
print("   delay cliff [ms] under feedback corrections:", {k: round(v * 1e3, 2) for k, v in fb.items()})
rep.check("feedback on measured velocity (lead of -2 ... +4 periods) moves the delay cliff by < 0.1 ms",
          abs(fb[("vel", -2)] - rd) < 1e-4 and abs(fb[("vel", 4)] - rd) < 1e-4)
rep.check("feedback on position (gain -0.5 ... +1) moves it by < 0.35 ms",
          abs(fb[("pos", -0.5)] - rd) < 3.5e-4 and abs(fb[("pos", 1.0)] - rd) < 3.5e-4)
rep.check("feedback on TILT moves it strongly both ways: -0.2 m/rad -> 20.8 ms (narrows 15 %), +0.2 m/rad -> 29.0 ms (widens 18 %)",
          abs(fb[("tilt", -0.2)] - 0.0208) < 2e-4 and abs(fb[("tilt", 0.2)] - 0.0290) < 2e-4
          and fb[("tilt", -0.1)] < rd < fb[("tilt", 0.1)])

# ---------------------------------------------------------------- 3. the oracle
hold = np.zeros((6, 700, 4))
dr, y0, G, S, cols = oracle(hold, mass_ratio=1.2)
sl = slice(250, 400)
steady_off = -y0[sl, 2].mean()
print(f"   mass +20 % on hover: oracle shift {dr[sl, 2].mean()*1e3:.2f} mm, steady offset {steady_off*1e3:.2f} mm")
rep.check("(a) mass +20 % on hover (7 s): oracle = constant altitude shift = steady offset / (1 + lambda) = 105.07 / 1.01 = 104.03 mm",
          abs(dr[sl, 2].mean() - steady_off / (1 + LAMBDA)) < 1e-5 and np.std(dr[sl, 2]) < 1e-6)
hold = np.zeros((6, 300, 4))
dr, y0, *_ = oracle(ramp, drag_scale=4)
lag_d = ramp[0, k4, 0] - y0[k4, 0]
print(f"   drag x4 on the ramp: oracle lead {dr[k4, 0]*1e3:.2f} mm, steady lag {lag_d*1e3:.2f} mm")
rep.check("(a) drag x4 on the 1 m/s ramp: oracle = constant lead matching the steady lag (95 mm) to 1.5 %",
          abs(dr[k4, 0] / lag_d - 1) < 0.015)
zero_hold = all(np.abs(oracle(hold, **mm)[0]).max() == 0.0 for mm in (dict(D=0.02), dict(tau_ratio=1.5), dict(tau_ratio=0.5)))
rep.check("(b) delay 20 ms and lag x1.5 / x0.5 on a held set-point: the oracle correction is exactly zero (F-2)", zero_hold)
dr20, y020, *_ = oracle(f8, D=0.02)
si = interior(f8.shape[1])
mx20 = np.abs(dr20[si]).max()
rep.check("(b) ... and nonzero on the figure-8 (delay 20 ms)", mx20 > 1e-5, f"max |dr| {mx20*1e3:.3f} mm")
# (c) is the delay oracle a time advance by D?  Tested: no.
dr10 = oracle(f8, D=0.01)[0]
mx10 = np.abs(dr10[si]).max()
T8 = f8.shape[1]


def best_advance(drD):
    tgt = drD[si, :2].reshape(-1)
    best = None
    for tau_ in np.arange(-0.05, 0.15, 0.001):
        idx = np.arange(T8)[si] + tau_ / DT
        adv = np.column_stack([np.interp(idx, np.arange(T8), f8[0][:, a]) - f8[0][si, a] for a in (0, 1)])
        res = np.linalg.norm(tgt - adv.reshape(-1)) / np.linalg.norm(tgt)
        if best is None or res < best[1]:
            best = (tau_, res)
    return best


ba = best_advance(dr20)
tgt = dr20[si, :2].reshape(-1)
Xd = np.column_stack([f8[o][si, :2].reshape(-1) for o in range(1, 6)])
c_, *_ = np.linalg.lstsq(Xd, tgt, rcond=None)
r2_joint = 1 - np.sum((tgt - Xd @ c_) ** 2) / np.sum(tgt ** 2)
r2_single = []
for o in range(1, 6):
    x1 = f8[o][si, :2].reshape(-1, 1)
    c1, *_ = np.linalg.lstsq(x1, tgt, rcond=None)
    r2_single.append(1 - np.sum((tgt - x1 @ c1) ** 2) / np.sum(tgt ** 2))
print(f"   delay 20 ms oracle: best pure time advance {ba[0]*1e3:.1f} ms (residual {ba[1]:.3f}); R^2 vs single derivatives 1..5",
      [round(v, 3) for v in r2_single], f"; joint {r2_joint:.3f}")
rep.check("(c) the delay oracle is NOT a time advance by D: the best pure advance is ~1 ms and leaves 85 % of it unexplained",
          abs(ba[0]) <= 0.002 and ba[1] > 0.8, f"{ba[0]*1e3:.1f} ms, residual {ba[1]:.3f}")
rep.check("(c) no single derivative explains it (R^2 <= 0.56); jointly the plan's 1st-5th derivatives explain 95 % (R^2 = 0.948)",
          max(r2_single) < 0.56 and abs(r2_joint - 0.948) < 0.01)
rep.check("(c) it is linear in D (max 1.45 mm at 10 ms, 2.96 mm at 20 ms) and 5.6x smaller than a naive advance D v_max = 16.6 mm",
          abs(mx10 - 0.00145) < 3e-5 and abs(mx20 - 0.00296) < 5e-5 and abs(mx20 / mx10 - 2.04) < 0.05
          and abs(0.02 * spd / mx20 - 5.6) < 0.1)
dr30, *_ = oracle(f8, D=0.030)
grow = simulate(f8, dr=dr30, x0=xp, D=0.030) - simulate(f8, dr=dr30, D=0.030)
decay = simulate(f8, dr=dr20, x0=xp, D=0.020) - simulate(f8, dr=dr20, D=0.020)
rep.check("(d) inside the set (20 ms) a 1 mm perturbation decays under the oracle; beyond the cliff (30 ms) it grows > 100x",
          np.abs(decay[-100:]).max() < 1e-4 and np.abs(grow[-100:]).max() > 0.1)
prev, trunc = {}, {}
for name, mm in (("delay", dict(D=0.02)), ("mass", dict(mass_ratio=1.2)), ("drag", dict(drag_scale=4)), ("lag", dict(tau_ratio=1.5))):
    drm, y0m, Gm, _, cl = oracle(f8, **mm)
    c0 = len(cl) // 2; j = cl[c0]
    w = np.abs(Gm[3 * c0].reshape(f8.shape[1], 3)[:, 0])
    cum = np.cumsum(w[j:]) / w[j:].sum()
    prev[name] = (int(np.argmax(cum >= 0.95)), int(np.argmax(cum >= 0.99)), round(w[:j].sum() / w.sum(), 3))
    # performance of the oracle when it may only use errors up to L samples ahead (weights masked, not re-optimised)
    e = (f8[0][:, 0:3] - y0m).reshape(-1)
    before = rms_int(f8[0], y0m)
    tt = np.repeat(np.arange(T8), 3); cj = np.repeat(np.array(cl), 3)
    for L in (0, NWS, 50):
        z = (Gm * ((tt[None, :] - cj[:, None]) <= L)) @ e
        d = np.zeros((T8, 4))
        for ci, jj in enumerate(cl):
            d[jj, :3] = z[3 * ci:3 * ci + 3]
        trunc[(name, L)] = 1 - rms_int(f8[0], simulate(f8, dr=d, **mm)) / before
print("   oracle kernel (future samples holding 95 %, 99 % of the weight; share on past errors):", prev)
print("   fraction of the error removed with preview limited to L samples:", {k: round(v, 3) for k, v in trunc.items()})
rep.check("preview: the full oracle's kernel is long (95 % of its future weight within 112 samples = 1.1 s for delay, 38-46 for the others)",
          prev["delay"][0] == 112 and prev["mass"][0] == 46 and prev["lag"][0] == 38)
rep.check("with preview limited to the 21-step horizon window the oracle still removes 72-93 % of the error; with 0.5 s, 95-98 %",
          all(0.72 <= trunc[(n, NWS)] <= 0.93 for n in prev) and all(0.94 <= trunc[(n, 50)] <= 0.985 for n in prev))
rep.check("with NO preview (past errors only) the same correction makes tracking WORSE at every case (-63 % to -90 %)",
          all(trunc[(n, 0)] < -0.6 for n in prev))

# ---------------------------------------------------------------- 4. tests over the admissible set (figure-8)
grid = ([dict(mass_ratio=s) for s in (0.8, 0.9, 1.1, 1.2, 1.3)] + [dict(drag_scale=s) for s in (1, 2, 4, 8, 16)]
        + [dict(tau_ratio=s) for s in (0.3, 0.5, 0.7, 1.3, 1.5, 2.0)] + [dict(D=k * DT) for k in (0.5, 1, 1.5, 2)])
table = []
for mm in grid:
    drm, y0m, *_ = oracle(f8, **mm)
    before, after = rms_int(f8[0], y0m), rms_int(f8[0], simulate(f8, dr=drm, **mm))
    table.append((list(mm.items())[0], before, after))
    print(f"   {str(mm):22s} figure-8 interior RMS: nominal MPC {before*1e3:8.3f} mm -> with oracle {after*1e3:7.3f} mm")
rep.check("oracle improves tracking at every one of the 20 single-mismatch points inside the admissible set",
          all(a < b for _, b, a in table))
TB = {k: (b, a) for k, b, a in table}
rep.check("oracle cuts the error by > 90 % at every point", all(a / b < 0.1 for (b, a) in TB.values()),
          f"worst remaining fraction {max(a / b for (b, a) in TB.values()):.3f}")
rep.check("... and by >= 95 % at every point except drag x16 (94.9 %)",
          all(a / b < 0.05 for k, (b, a) in TB.items() if k != ("drag_scale", 16)) and abs(1 - TB[("drag_scale", 16)][1] / TB[("drag_scale", 16)][0] - 0.949) < 0.002)
rep.check("table spot values: mass 1.2 105.1 -> 1.03 mm; drag x16 103.6 -> 5.29 mm; lag x2 2.08 -> 0.069 mm; delay 20 ms 0.678 -> 0.023 mm",
          abs(TB[("mass_ratio", 1.2)][0] - 0.1051) < 3e-4 and abs(TB[("mass_ratio", 1.2)][1] - 0.00103) < 3e-5
          and abs(TB[("drag_scale", 16)][0] - 0.1036) < 3e-4 and abs(TB[("drag_scale", 16)][1] - 0.00529) < 5e-5
          and abs(TB[("tau_ratio", 2.0)][0] - 0.00208) < 2e-5 and abs(TB[("tau_ratio", 2.0)][1] - 0.000069) < 3e-6
          and abs(TB[("D", 0.02)][0] - 0.000678) < 5e-6 and abs(TB[("D", 0.02)][1] - 0.000023) < 2e-6)
rep.check("about half of the oracle's weight lies on past errors (0.45-0.54)", all(0.45 <= v[2] <= 0.54 for v in prev.values()))
rep.check("why the correction must never be differentiated: the 5th-order central difference weights samples by up to 2.5/dt^5 = 2.5e10",
          abs(2.5 / DT**5 - 2.5e10) < 1e3)

raise SystemExit(rep.finish())
