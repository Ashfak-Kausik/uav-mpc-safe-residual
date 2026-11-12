"""Entry 07: perturbation models, severities, and the admissible mismatch set (Assumption 2).
Every number quoted in 07_perturbation_models.md is computed here. Implementation (b) throughout."""
import itertools
import numpy as np
import params as P
from quad_sym import Report
from linmodel import continuous, zoh, f_hover
from mpc06 import DT, N, closed_loop, spectral_radius, cliff
from plant07 import Plant, run_closed_loop, H, D_HOVER, DZ_OVER_DXY

rep = Report("check_07_perturbations")
D = P.derived()
t_att, tau = D["t_att [s]"], P.tau_m


# ---------------------------------------------------------------- linear perturbed plants
def lin_plant(mass_ratio=1.0, drag_scale=0.0, tau_ratio=1.0):
    """Hover linearisation of the perturbed plant about ITS OWN trim (thrust m'g): the input gain becomes
    1/m', the tilt coupling stays g, drag adds -d/m' on the velocities, the lag uses the true tau."""
    A, B = continuous(tau=tau * tau_ratio)
    mp = P.m * mass_ratio
    A[5, 12:16] = 1 / mp
    d = drag_scale * D_HOVER
    A[3, 3] = A[4, 4] = -d / mp
    A[5, 5] = -d * DZ_OVER_DXY / mp
    return zoh(A, B, DT)


def rho(mass_ratio=1.0, drag_scale=0.0, tau_ratio=1.0, D=0.0):
    if mass_ratio == 1.0 and drag_scale == 0.0:
        return spectral_radius(closed_loop(tau_true=tau * tau_ratio, D=D)[0])
    return spectral_radius(closed_loop(D=D, extra=lin_plant(mass_ratio, drag_scale, tau_ratio))[0])


def gains(n=2000, **kw):
    """Frequency-sweep estimates of the peak gains from the position reference to (position, tilt)."""
    Acl, Bcl = closed_loop(tau_true=tau * kw.get("tau_ratio", 1.0), D=kw.get("D", 0.0))
    ws = np.linspace(1e-3, np.pi / DT, n)
    gp = gt = 0.0
    for w in ws:
        Hm = np.linalg.solve(np.exp(1j * w * DT) * np.eye(Acl.shape[0]) - Acl, Bcl)
        gp = max(gp, np.linalg.svd(Hm[[0, 1, 2]][:, :3], compute_uv=False)[0])
        gt = max(gt, np.linalg.svd(Hm[[6, 7]][:, :3], compute_uv=False)[0])
    return gp, gt


# ---------------------------------------------------------------- 1. motor-lag form
small = {}
for form in ("thrust", "speed"):
    pl = Plant(lag_form=form); pl.reset()
    f1 = 1.01 * f_hover
    traj = []
    for _ in range(300):
        pl.lag_step(np.full(4, f1)); traj.append(pl.f[0])
    small[form] = np.array(traj)
rel = np.abs(small["speed"] - small["thrust"]).max() / (0.01 * f_hover)
rep.check("small signal: for a 1 % thrust step the speed-form and thrust-form responses agree to within 0.3 % of the step",
          rel < 0.003, f"max difference {rel*100:.3f} % of the step")
times = {}
for f1, name in ((P.f_max, "up"), (P.f_min, "down")):
    for form in ("thrust", "speed"):
        pl = Plant(lag_form=form); pl.reset(); t = 0
        target = f_hover + 0.632 * (f1 - f_hover)
        while (pl.f[0] - target) * np.sign(f1 - f_hover) < 0:
            pl.lag_step(np.full(4, f1)); t += H
        times[(name, form)] = t
print("   63 % step times [ms]:", {k: round(v * 1e3, 1) for k, v in times.items()})
rep.check("large steps: thrust form 65 ms both ways; speed form 70 ms up to the ceiling, 50 ms down to the floor",
          abs(times[("up", "thrust")] - 0.065) < 0.0015 and abs(times[("down", "thrust")] - 0.065) < 0.0015
          and abs(times[("up", "speed")] - 0.070) < 0.0015 and abs(times[("down", "speed")] - 0.050) < 0.0015)

# ---------------------------------------------------------------- 2. severities, normalised
tw = lambda mr: 4 * P.f_max / (P.m * mr * P.g)
rep.check("hover is feasible inside the box only for mass ratios in [0.18, 1.69] (f_min <= m'g/4 <= f_max)",
          abs(4 * P.f_min / (P.m * P.g) - 0.180) < 0.001 and abs(4 * P.f_max / (P.m * P.g) - 1.687) < 0.001)
rep.check("mass +30 %: effective TWR 1.30; ceiling headroom at hover 0.0275 N = 56 % of nominal",
          abs(tw(1.3) - 1.30) < 0.005 and abs((P.f_max - 1.3 * f_hover) - 0.0275) < 2e-4
          and abs((P.f_max - 1.3 * f_hover) / (P.f_max - f_hover) - 0.56) < 0.01)
rep.check("drag: m/d = 4.1 s = 96 t_att (very slow); trim tilt 0.025 rad per m/s per unit drag scale",
          abs(P.m / D_HOVER - 4.07) < 0.02 and abs(P.m / D_HOVER / t_att - 96) < 1.5 and abs(D_HOVER / (P.m * P.g) - 0.025) < 0.001)
rep.check("tau-ratio grid in t_att and horizon units: 4 tau_m = 0.259 s = 6.1 t_att = 1.29 T_horizon",
          abs(4 * tau - 0.259) < 0.001 and abs(4 * tau / t_att - 6.11) < 0.02 and abs(4 * tau / (N * DT) - 1.29) < 0.01)

# normalisation table quoted in the entry
twr_tab = {mr: round(tw(mr), 2) for mr in (0.8, 0.9, 1.1, 1.2, 1.3)}
head_tab = {mr: round((P.f_max - mr * f_hover) / (P.f_max - f_hover), 2) for mr in (0.8, 0.9, 1.1, 1.2, 1.3)}
print("   effective TWR:", twr_tab, " ceiling headroom / nominal:", head_tab)
rep.check("mass grid -> effective TWR 2.11, 1.87, 1.53, 1.41, 1.30 and ceiling headroom 1.29, 1.15, 0.85, 0.71, 0.56 of nominal",
          list(twr_tab.values()) == [2.11, 1.87, 1.53, 1.41, 1.30] and list(head_tab.values()) == [1.29, 1.15, 0.85, 0.71, 0.56])
lag_tab = {s: (round(s * tau * 1e3), round(s * tau / t_att, 2), round(s * tau / (N * DT), 2)) for s in (0.15, 0.3, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0)}
print("   lag grid (ms, /t_att, /T_horizon):", lag_tab)
rep.check("lag grid: 0.15x = 10 ms = 0.23 t_att; 1x = 65 ms = 1.53 t_att = 0.32 T_h; 4x = 259 ms = 6.11 t_att = 1.29 T_h",
          lag_tab[0.15] == (10, 0.23, 0.05) and lag_tab[1.0] == (65, 1.53, 0.32) and lag_tab[4.0] == (259, 6.11, 1.29))
cliff_b = 0.02462
del_tab = {k: round(k * DT / cliff_b, 2) for k in (0.5, 1, 1.5, 2, 3, 4, 6, 8)}
print("   delay grid as a fraction of the 24.6 ms cliff:", del_tab)
rep.check("delay grid as a fraction of its cliff: 2 dt = 0.81, 3 dt = 1.22, 8 dt = 3.25",
          del_tab[2] == 0.81 and del_tab[3] == 1.22 and del_tab[8] == 3.25)

# ---------------------------------------------------------------- 3. Assumption 2 over the single-mismatch grids (linear, impl b)
GRID = dict(
    mass_ratio=[0.8, 0.9, 1.1, 1.2, 1.3],
    drag_scale=[1, 2, 4, 8, 16],
    tau_ratio=[0.15, 0.3, 0.5, 0.7, 1.3, 1.5, 2.0, 3.0, 4.0],
    D=[k * DT for k in (0.5, 1, 1.5, 2, 3, 4, 6, 8)],
)
stab = {k: {v: rho(**{k: v}) < 1 for v in vals} for k, vals in GRID.items()}
for k, s in stab.items():
    print(f"   {k:11s} stable: {[v for v, ok in s.items() if ok]}   unstable: {[v for v, ok in s.items() if not ok]}")
rep.check("mass -20 ... +30 %: all stable", all(stab["mass_ratio"].values()))
rep.check("drag x1 ... x16: all stable", all(stab["drag_scale"].values()))
rep.check("lag: stable for 0.3 ... 2.0 tau_m; unstable at 0.15, 3 and 4 tau_m (cliffs 0.186 and 2.86)",
          [v for v, ok in stab["tau_ratio"].items() if not ok] == [0.15, 3.0, 4.0])
rep.check("delay: stable up to 2 dt; unstable from 3 dt (cliff 2.46 dt)",
          [v for v, ok in stab["D"].items() if not ok] == [k * DT for k in (3, 4, 6, 8)])

# gains from the position reference as a cliff is approached (input to entry 10)
g = {k: gains(D=k) for k in (0.0, 0.010, 0.020, 0.024)}
g_lag = {s: gains(tau_ratio=s) for s in (1.5, 2.0, 2.8)}
print("   peak gains r->(p, tilt):", {f"D={k*1e3:.0f}ms": tuple(round(x, 2) for x in v) for k, v in g.items()},
      {f"tau x{s}": tuple(round(x, 2) for x in v) for s, v in g_lag.items()})
rep.check("reference-to-position peak gain stays 1.00 up to 24 ms of delay (no position peaking)",
          all(abs(v[0] - 1.0) < 1e-3 for v in g.values()))
rep.check("reference-to-tilt peak gain [rad/m] grows toward the delay cliff: 1.75 (0) -> 1.74 (10 ms) -> 4.0 (20 ms) -> 31.7 (24 ms)",
          abs(g[0.0][1] - 1.745) < 0.01 and abs(g[0.010][1] - 1.737) < 0.01 and abs(g[0.020][1] - 4.02) < 0.05
          and abs(g[0.024][1] - 31.7) < 0.5)
rep.check("... and toward the lag cliff: 1.86 (x1.5) -> 3.16 (x2) -> 45.6 (x2.8)",
          abs(g_lag[1.5][1] - 1.858) < 0.01 and abs(g_lag[2.0][1] - 3.163) < 0.02 and abs(g_lag[2.8][1] - 45.6) < 0.5)

# ---------------------------------------------------------------- 4. combinations
combos = list(itertools.product([1.0, 1.2], [1, 4], [0.5, 1.0, 1.5], [0, 1, 2]))
res = {c: rho(mass_ratio=c[0], drag_scale=c[1], tau_ratio=c[2], D=c[3] * DT) for c in combos}
unst = sorted(c for c, v in res.items() if v >= 1)
print(f"   combinations (mass, drag, tau, delay/dt): {len(combos)} points, unstable: {unst}")
rep.check("36 combinations: mass {1, 1.2} x drag {1, 4} x lag {0.5, 1, 1.5} x delay {0, 1, 2} dt", len(combos) == 36)
rep.check("combinations: unstable exactly where a FASTER-than-modelled motor (x0.5) meets 2 dt of delay (4 of 36)",
          len(unst) == 4 and all(c[2] == 0.5 and c[3] == 2 for c in unst))
cl = {s: cliff("D", 0.0, 0.03 if s != 0.5 else 0.04, tau_true=s * tau) for s in (0.5, 1.5, 2.0)}
rep.check("combined delay cliffs: 15.8 ms with a x0.5 lag, 22.7 ms with x1.5, 16.2 ms with x2 (24.6 ms with none)",
          abs(cl[0.5] - 0.01577) < 2e-4 and abs(cl[1.5] - 0.02274) < 2e-4 and abs(cl[2.0] - 0.01620) < 2e-4,
          ", ".join(f"x{s}: {v*1e3:.2f} ms" for s, v in cl.items()))
rep.check("a +-50 % lag error narrows the delay margin by up to 36 % (x0.5: 15.8 / 24.6)", abs(1 - cl[0.5] / 0.02462 - 0.36) < 0.01)

# ---------------------------------------------------------------- 5. nonlinear spot checks (impl b, constrained QP)
r = np.array([0.5, 0.0, 1.2, 0.0])


def nl(**kw):
    X, U, flags = run_closed_loop(Plant(**kw), r, steps=300)
    return X, set(flags)


def tail(X, n=100):
    return np.abs(X[-n:, 0:3] - r[0:3]).max()


X, fl = nl()
rep.check("nonlinear, no mismatch: converges (noise floor: last-second error 0.2 mm)", tail(X) < 5e-4 and fl == {1}, f"{tail(X)*1e3:.2f} mm")
X, fl = nl(mass_ratio=1.2)
rep.check("nonlinear, mass +20 %: altitude settles 106 mm low, as the linear prediction (2 x 53 mm)",
          abs((r[2] - X[-1, 2]) - 0.106) < 0.002 and np.abs(X[-1, 0:2] - r[0:2]).max() < 1e-3)
X, fl = nl(drag_scale=4)
rep.check("nonlinear, drag x4: set-point reached (no steady effect), last-second error < 5 mm", tail(X) < 5e-3)
for form in ("thrust", "speed"):
    for s in (0.5, 1.5):
        X, fl = nl(tau_ratio=s, lag_form=form)
        rep.check(f"nonlinear, lag x{s} ({form} form): converges", tail(X) < 1e-3)
X, fl = nl(D=0.020)
rep.check("nonlinear, delay 20 ms (inside the set, near the cliff): converges, with tilt overshoot past the soft limit",
          tail(X) < 1e-3)
X, fl = nl(D=0.030)
rep.check("nonlinear, delay 30 ms (beyond the 24.6 ms cliff): diverges", tail(X) > 1.0)
X, fl = nl(tau_ratio=3.0)
rep.check("nonlinear, lag x3 (beyond the 2.86 cliff): does not settle within 3 s", tail(X) > 0.01)

raise SystemExit(rep.finish())
