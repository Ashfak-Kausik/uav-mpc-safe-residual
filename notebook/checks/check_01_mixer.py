"""Entry 01: rotor torques, mixer M, its closed-form inverse, and the wrench polytope.
Entry 04: hover authority tau_max = a*m*g*min(1, TWR-1) checked against an exact LP.
"""
import numpy as np
import sympy as sp
from scipy.optimize import linprog
from quad_sym import Report, mixer

rep = Report("check_01_mixer")
a, c, T, tx, ty, tz, f = sp.symbols("a c T tau_x tau_y tau_z f", positive=True)

# r_i x (0,0,f_i) for the four rotor positions (FL, BL, BR, FR)
pos = [(a, a), (-a, a), (-a, -a), (a, -a)]
s = [1, -1, 1, -1]  # reaction-torque sign about body z (1,3 clockwise seen from above)
fs = sp.symbols("f1:5")
tau = sp.zeros(3, 1)
for (x, y), fi in zip(pos, fs):
    tau += sp.Matrix([x, y, 0]).cross(sp.Matrix([0, 0, fi]))
M = mixer(a, c)
W = M * sp.Matrix(fs)
rep.check("roll row = sum y_i f_i (cross product)", sp.expand(W[1] - tau[0]) == 0)
rep.check("pitch row = -sum x_i f_i (cross product)", sp.expand(W[2] - tau[1]) == 0)
rep.check("yaw row = c * sum s_i f_i", sp.expand(W[3] - c * sum(si * fi for si, fi in zip(s, fs))) == 0)
rep.check("rotor 1 alone: +roll, -pitch (lifts left side and nose)",
          W[1].subs({fs[0]: 1, fs[1]: 0, fs[2]: 0, fs[3]: 0}) == a and W[2].subs({fs[0]: 1, fs[1]: 0, fs[2]: 0, fs[3]: 0}) == -a)

# Closed-form inverse from entry 01
Minv_text = sp.Matrix([
    [sp.Rational(1, 4), 1 / (4 * a), -1 / (4 * a), 1 / (4 * c)],
    [sp.Rational(1, 4), 1 / (4 * a), 1 / (4 * a), -1 / (4 * c)],
    [sp.Rational(1, 4), -1 / (4 * a), 1 / (4 * a), 1 / (4 * c)],
    [sp.Rational(1, 4), -1 / (4 * a), -1 / (4 * a), -1 / (4 * c)],
])
rep.check("closed-form inverse in entry 01 equals M^-1", sp.simplify(M * Minv_text - sp.eye(4)) == sp.zeros(4, 4))
G = sp.simplify(M * M.T)
rep.check("rows of M mutually orthogonal (M M^T diagonal)", G == sp.diag(4, 4 * a**2, 4 * a**2, 4 * c**2))
rep.check("det M != 0 for a, c > 0", sp.simplify(M.det()) != 0, f"det = {sp.factor(M.det())}")

# Yaw/roll authority ratio for equal rotor spread
rep.check("yaw/roll torque per unit rotor spread = c/a",
          sp.simplify((Minv_text[0, 1] / Minv_text[0, 3]) - c / a) == 0)


# Hover authority: exact LP vs closed form, over a TWR grid, with f_min = 0 and f_min > 0
def tau_max_lp(m, g, fmax, fmin, a_, row):
    Mn = np.array(mixer(a_, 0.006).evalf(), dtype=float)
    cost = -Mn[row]
    A_eq = Mn[[r for r in range(4) if r != row]]  # T = mg, other torques zero
    b_eq = np.zeros(3)
    b_eq[0] = m * g
    res = linprog(cost, A_eq=A_eq, b_eq=b_eq, bounds=[(fmin, fmax)] * 4, method="highs")
    return -res.fun if res.success else np.nan


m, g, a_ = 0.027, 9.81, 0.046 / np.sqrt(2)
worst = 0.0
worst_gen = 0.0
for twr in np.linspace(1.05, 4.0, 60):
    fmax = twr * m * g / 4
    lp = tau_max_lp(m, g, fmax, 0.0, a_, 1)
    cf = a_ * m * g * min(1.0, twr - 1.0)
    worst = max(worst, abs(lp - cf) / cf)
    fmin = 0.1 * m * g / 4  # 10 % of hover thrust as idle floor
    lpg = tau_max_lp(m, g, fmax, fmin, a_, 1)
    cfg = a_ * min(4 * fmax - m * g, m * g - 4 * fmin)
    worst_gen = max(worst_gen, abs(lpg - cfg) / cfg)
rep.check("tau_x,max(hover) = a m g min(1, TWR-1) matches LP (f_min=0)", worst < 1e-6, f"max rel err {worst:.1e}")
rep.check("generalised: a*min(4 f_max - m g, m g - 4 f_min) matches LP (f_min>0)", worst_gen < 1e-6, f"max rel err {worst_gen:.1e}")

# Yaw authority has the same structure (row 3), with c in place of a
fmax = 2.2 * m * g / 4
Mn = np.array(mixer(a_, 0.006).evalf(), dtype=float)
res = linprog(-Mn[3], A_eq=Mn[[0, 1, 2]], b_eq=[m * g, 0, 0], bounds=[(0, fmax)] * 4, method="highs")
rep.check("tau_z,max(hover) = c m g min(1, TWR-1) matches LP", abs(-res.fun - 0.006 * m * g * 1.0) < 1e-9)

# Committed yaw penalty c_tf/a (entries 01 and 04) vs the verified sources.
# Regression for audit B-2: the old value 'c_tf/a ~ 1/15' matched no source.
import params as P
ratio = P.c_tf / P.a
src = {"Foerster 2.0": 0.005964552 / (0.046 / np.sqrt(2)),
       "FW legacy props": 0.007350862856566459 / (0.046 / np.sqrt(2))}
print(f"   committed c_tf/a = {ratio:.3f}; sources: " + ", ".join(f"{k} {v:.3f}" for k, v in src.items()))
rep.check("committed c_tf/a = 0.215 (as quoted in entries 01 and 04)", abs(ratio - 0.215) < 5e-4)
rep.check("committed c_tf/a lies within the verified FO/FW source spread", min(src.values()) <= ratio <= max(src.values()))
rep.check("old entry-04 value 1/15 is outside the verified spread (audit B-2 stays fixed)", not (min(src.values()) <= 1 / 15 <= max(src.values())))

# Committed vehicle with the firmware floor f_min = THRUST_MIN: roll and yaw authority via LP
Mc = np.array(mixer(P.a, P.c_tf).evalf(), dtype=float)
for row, k, name in [(1, P.a, "roll"), (3, P.c_tf, "yaw")]:
    others = [r for r in range(4) if r != row]
    b = np.zeros(3)
    b[0] = P.m * P.g
    res = linprog(-Mc[row], A_eq=Mc[others], b_eq=b, bounds=[(P.f_min, P.f_max)] * 4, method="highs")
    cf = k * min(4 * P.f_max - P.m * P.g, P.m * P.g - 4 * P.f_min)
    rep.check(f"committed vehicle: {name} authority = {name == 'roll' and 'a' or 'c_tf'}*min(4f_max - mg, mg - 4f_min) (LP, f_min = THRUST_MIN)",
              abs(-res.fun - cf) < 1e-12, f"{-res.fun:.4e} N m")

raise SystemExit(rep.finish())
