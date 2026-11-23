"""Entry 09: the realizability ratio and Target Result 1. Every number quoted in 09_realizability.md is computed
here. Trajectory MPC of entry 08 with the constraints of entry 06 (implementation b), linear plant."""
import time
import numpy as np
import params as P
from quad_sym import Report
from linmodel import NX, NU, NY
from mpc06 import DT, N, Ad, Bd, U_MAX, U_MIN, DU_MAX
from ref08 import Fx, Fflat, traj_fig8, plant_disc, NWS
from realiz09 import (TrajQP, affine_U, full_window, ratio_test, pwa_ratio, lp_ratio, h_vec, G, ROW_KIND, DU_DR,
                      POS_COLS, Xi, Su, YROWS, nU, lin_step, G_tilt, G_alt)

rep = Report("check_09_realizability")
rng = np.random.default_rng(9)
qp = TrajQP()

# ---------------------------------------------------------------- the QP and its affine law
rep.check("constrained trajectory QP: 82 variables (80 inputs + 2 slacks); 420 inequality rows with slacks at zero "
          "(160 box, 160 rate, 80 tilt, 20 altitude)",
          qp.nz == 82 and G.shape == (420, 80) and ROW_KIND.count("box+") == 80 and ROW_KIND.count("tilt+") == 40
          and ROW_KIND.count("alt") == 20)
plan = traj_fig8(); plan[0, :, 2] += 1.0                                 # flown at 1 m
Fw = full_window(plan, np.zeros((plan.shape[1], 4)), 300)
x0 = Xi[:NX] @ Fw
U = affine_U(x0, Fw)
rep.check("the first move of the affine law equals the entry-08 trajectory law (Fx x + Fflat window)",
          np.abs(U[:NU] - (Fx @ x0 + Fflat @ Fw)).max() < 1e-12)
rep.check("the correction-to-input sensitivity dU/dDelta_r is a constant matrix (LTI MPC): 80 x 84", DU_DR.shape == (80, 84))

# ---------------------------------------------------------------- Target Result 1, numerically on random cases
n_cases, ok_inside, ok_outside, n_lim = 0, True, True, 0
for trial in range(40):
    k = int(rng.integers(150, 700))
    Fw = full_window(plan, np.zeros((plan.shape[1], 4)), k)
    x0 = Xi[:NX] @ Fw + rng.normal(size=NX) * np.r_[[0.01] * 3, [0.02] * 3, [0.01] * 3, [0.05] * 3, [0.001] * 4]
    u_prev = affine_U(x0, Fw)[:NU] + rng.normal(size=NU) * 1e-3     # previous command near the current one
    Uq = affine_U(x0, Fw)
    if np.any(h_vec(x0, u_prev) - G @ Uq <= 1e-9):
        continue                                                     # premise of Result 1: no constraint active
    n_cases += 1
    dR = np.tile(rng.normal(size=4) * np.r_[0.3, 0.3, 0.3, 0.3], NWS)
    dF = np.zeros_like(Fw); dF[POS_COLS] = dR
    r, arg = ratio_test(Uq, DU_DR @ dR, x0, u_prev)
    for beta in (0.25 * r, 0.75 * r, 0.999 * r):
        z, flag = qp.solve(x0, u_prev, Fw + beta * dF)
        ok_inside &= flag == 1 and np.abs(z[:nU] - affine_U(x0, Fw + beta * dF)).max() < 1e-9 and z[nU:].max() < 1e-9
    if r < 1:
        n_lim += 1
        z, flag = qp.solve(x0, u_prev, Fw + min(1.0, 1.05 * r) * dF)
        ok_outside &= np.abs(z[:nU] - affine_U(x0, Fw + min(1.0, 1.05 * r) * dF)).max() > 1e-7
print(f"   random cases with the premise satisfied: {n_cases}; of which the correction is limited (r_H < 1): {n_lim}")
rep.check("Result 1: for beta <= r_H the constrained QP's solution equals the affine law (to 1e-9), slacks zero",
          ok_inside and n_cases >= 25)
rep.check("... and just beyond r_H (1.05 r_H) it departs from the affine law: r_H is the exact boundary, not a margin",
          ok_outside and n_lim >= 10)
Ua = Uq.copy(); Ua[0] = U_MAX
r_sat, arg_sat = ratio_test(Ua, np.eye(nU)[0], x0, u_prev)
rep.check("a rotor already at its ceiling and pushed further gives r_H = 0 (limited by its box row)",
          r_sat == 0.0 and ROW_KIND[arg_sat] == "box+")

# ---------------------------------------------------------------- closed loop linear in alpha (inside r_H)
plan3 = traj_fig8(T_s=5.0); plan3[0, :, 2] += 1.0
Phi, G0, G1, d, c = plant_disc()


def run(alpha, corr):
    x = np.zeros(NX); x[2] = 1.0; fhat = np.zeros(NU); hist = [np.zeros(NU)] * (d + 3)
    Y, rmin = [], 1.0
    for k in range(plan3.shape[1] - 1):
        u_prev = hist[0]
        xh = Ad @ np.r_[x[:12], fhat] + Bd @ u_prev
        Fw = full_window(plan3, alpha * corr, k + 1)
        z, flag = qp.solve(xh, u_prev, Fw)
        dRw = (corr[np.clip(np.arange(k + 1, k + 1 + NWS), 0, plan3.shape[1] - 1)]).reshape(-1)
        rmin = min(rmin, ratio_test(z[:nU], DU_DR @ dRw, xh, u_prev)[0])
        x, fhat, hist = lin_step(x, fhat, hist, z[:NU], Phi, G0, G1, d, c)
        Y.append(x[:3].copy())
    return np.array(Y), rmin


corr = np.zeros((plan3.shape[1], 4)); corr[:, 2] = 0.01 * np.clip((np.arange(plan3.shape[1]) - 200) / 100, 0, 1)
y0, _ = run(0.0, corr); yh, _ = run(0.5, corr); y1, rmin = run(1.0, corr)
rep.check("closed loop with a correction inside r_H at every step: outputs are affine in alpha (y(0.5) = (y(0) + y(1))/2 to 1e-12)",
          rmin >= 1.0 - 1e-12 and np.abs(yh - 0.5 * (y0 + y1)).max() < 1e-12, f"min r_H along the run {rmin:.3f}")

# ---------------------------------------------------------------- r_H against the exact PWA ratio and the LP
res = {}
t_rh = t_pwa = t_lp = 0.0
for period, mr in ((3.0, 1.0), (2.2, 1.3), (2.0, 1.3)):
    pl = traj_fig8(T_s=6.0, period=period); pl[0, :, 2] += 1.0
    Phi, G0, G1, d, c = plant_disc(mass_ratio=mr)
    x = np.zeros(NX); x[2] = 1.0; fhat = np.zeros(NU); hist = [np.zeros(NU)] * (d + 3)
    S = []
    for k in range(pl.shape[1] - 1):
        u_prev = hist[0]
        xh = Ad @ np.r_[x[:12], fhat] + Bd @ u_prev
        Fw = full_window(pl, np.zeros((pl.shape[1], 4)), k + 1)
        z, flag = qp.solve(xh, u_prev, Fw)
        Uq = z[:nU]
        if k > 150 and k % 3 == 0:
            for kind in ("vert", "lat"):
                if kind == "vert":
                    vec = np.array([0, 0, 0.16, 0])                      # mass-type correction (entry 08: +30 % mass)
                else:
                    v = pl[1, k + 1, :2]
                    vec = np.r_[0.05 * v / max(np.linalg.norm(v), 1e-9), 0, 0]   # drag-type lateral lead
                dR = np.tile(vec, NWS); dF = np.zeros_like(Fw); dF[POS_COLS] = dR; dU = DU_DR @ dR
                t0 = time.perf_counter(); rh, _ = ratio_test(Uq, dU, xh, u_prev); t_rh += time.perf_counter() - t0
                t0 = time.perf_counter(); rp, *_ = pwa_ratio(Uq, xh, u_prev, Fw, dF); t_pwa += time.perf_counter() - t0
                t0 = time.perf_counter(); rl, st = lp_ratio(Uq, xh, u_prev, dU); t_lp += time.perf_counter() - t0
                act = int(np.sum(h_vec(xh, u_prev) - G @ Uq <= 1e-9))
                S.append((rh, rp, rl, act, z[nU:].max(), st))
        x, fhat, hist = lin_step(x, fhat, hist, Uq[:NU], Phi, G0, G1, d, c)
    res[(period, mr)] = np.array(S, dtype=float)
for key, S in res.items():
    na = S[:, 3] == 0
    print(f"   period {key[0]} s, mass x{key[1]}: {len(S)} evaluations, active constraints in {np.mean(~na)*100:.0f} %;"
          f" r_H < 1 in {np.mean(S[:, 0] < 1)*100:.0f} %; r_H > r_PWA in {np.sum(S[:, 0] > S[:, 1] + 1e-9)};"
          f" r_LP - r_H max {np.max(S[:, 2] - S[:, 0]):.3f}, mean {np.mean(S[:, 2] - S[:, 0]):.4f}")
allS = np.vstack(list(res.values()))
na = allS[:, 3] == 0
rep.check("all LPs solved; no soft constraint used its slack in these runs", np.all(allS[:, 5] == 0) and np.all(allS[:, 4] < 1e-9))
rep.check("no active constraint (premise of Result 1): r_H = r_PWA to 1e-12 in all 452 such evaluations",
          na.sum() == 452 and np.abs(allS[na, 0] - allS[na, 1]).max() < 1e-12, f"{na.sum()} evaluations")
rep.check("r_H never exceeds the physical (LP) ratio, and is within 0.032 of it everywhere (mean gap <= 0.002)",
          np.all(allS[:, 2] >= allS[:, 0] - 1e-7) and np.max(allS[:, 2] - allS[:, 0]) < 0.033 and np.mean(allS[:, 2] - allS[:, 0]) < 0.0021)
act = ~na
rep.check("with constraints already active (69-79 % of the aggressive runs), r_H OVER-states the exact transmission "
          "ratio r_PWA in 135 of 596 evaluations: r_H certifies Result 1 only without active constraints",
          np.sum(allS[act, 0] > allS[act, 1] + 1e-9) == 135 and len(res[(2.2, 1.3)]) + len(res[(2.0, 1.3)]) == 596
          and abs(np.mean(res[(2.2, 1.3)][:, 3] > 0) - 0.69) < 0.01 and abs(np.mean(res[(2.0, 1.3)][:, 3] > 0) - 0.79) < 0.01)
nev = len(allS)
print(f"   time per evaluation (Python, PC): r_H {t_rh/nev*1e3:.3f} ms, r_PWA {t_pwa/nev*1e3:.3f} ms, LP {t_lp/nev*1e3:.2f} ms")
rep.check("cost ordering on the PC: r_H is >= 5x cheaper than r_PWA and >= 50x cheaper than the LP",
          t_pwa / t_rh > 5 and t_lp / t_rh > 50)

# ---------------------------------------------------------------- operation count of r_H (on-board estimate)
macs_box_rate = 2 * (4 * nU)                         # G U and G dU on the structured box/rate rows
macs_state = 2 * (G_tilt.shape[0] + G_alt.shape[0]) * nU     # dense predicted-tilt/altitude rows
divs = G.shape[0]
rep.check("ratio test cost: 640 MACs on box/rate rows (O(N m)), 9600 MACs on tilt/altitude rows (O(N^2 m)), at most 420 divisions",
          macs_box_rate == 640 and macs_state == 9600 and divs == 420)

raise SystemExit(rep.finish())
