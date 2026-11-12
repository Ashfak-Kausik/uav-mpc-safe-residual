"""Entry 04: equilibrium, linearisation, mass bias, drag trim, normalising groups."""
import numpy as np
import sympy as sp
from scipy.optimize import fsolve
from quad_sym import Report, R_entry03, qmul, mixer

rep = Report("check_04_hover")
m, g, a, c, Ixx, Iyy, Izz = sp.symbols("m g a c I_xx I_yy I_zz", positive=True)
p = sp.Matrix(sp.symbols("x y z", real=True))
v = sp.Matrix(sp.symbols("vx vy vz", real=True))
q = sp.Matrix(sp.symbols("q0:4", real=True))
w = sp.Matrix(sp.symbols("wx wy wz", real=True))
f = sp.Matrix(sp.symbols("f1:5", real=True))
I = sp.diag(Ixx, Iyy, Izz)
M = mixer(a, c)
wr = M * f
T, tau = wr[0], sp.Matrix(wr[1:])
e3 = sp.Matrix([0, 0, 1])
R = R_entry03(list(q))
xdot = sp.Matrix.vstack(
    v,
    T / m * R * e3 - g * e3,
    sp.Matrix([sp.Rational(1, 2) * e for e in qmul(list(q), [0, *w])]),
    I.inv() * (tau - w.cross(I * w)),
)
X = sp.Matrix.vstack(p, v, q, w)
hover = {**{s: 0 for s in list(p) + list(v) + list(w)}, q[0]: 1, q[1]: 0, q[2]: 0, q[3]: 0,
         **{fi: m * g / 4 for fi in f}}
rep.check("hover (f_i = m g/4, level, at rest) is an equilibrium", sp.simplify(xdot.subs(hover)) == sp.zeros(13, 1))
# Yaw family: any q = (cos psi/2, 0, 0, sin psi/2) is also an equilibrium
psi = sp.symbols("psi", real=True)
hov_psi = {**hover, q[0]: sp.cos(psi / 2), q[3]: sp.sin(psi / 2)}
rep.check("equilibrium is a yaw family", sp.simplify(xdot.subs(hov_psi)) == sp.zeros(13, 1))

# Linearisation (Jacobian at hover)
A = xdot.jacobian(X).subs(hover)
B = xdot.jacobian(f).subs(hover)
# small-angle: q ~ (1, dphi/2, dth/2, dpsi/2) => vdot_x = 2 g dq2 = g dth
rep.check("d(vdot_x)/d(q2) = 2g  (=> xddot = g dtheta)", sp.simplify(A[3, 8] - 2 * g) == 0)
rep.check("d(vdot_y)/d(q1) = -2g (=> yddot = -g dphi)", sp.simplify(A[4, 7] + 2 * g) == 0)
rep.check("d(vdot_z)/d(f_i) = 1/m (=> zddot = dT/m)", all(sp.simplify(B[5, i] - 1 / m) == 0 for i in range(4)))
rep.check("d(q2dot)/d(wy) = 1/2 (=> dtheta_dot = w_y)", sp.simplify(A[8, 11] - sp.Rational(1, 2)) == 0)
rep.check("d(wy_dot)/d f = a(-1,1,1,-1)/Iyy (=> w_y_dot = tau_y/Iyy)",
          sp.simplify(B[11, :] - sp.Matrix([[-a, a, a, -a]]) / Iyy) == sp.zeros(1, 4))
# Transfer tau_y -> x is g/(Iyy s^4): chain x <- vx <- q2 <- wy
s = sp.symbols("s")
Gx = (A[0, 3] / s) * (A[3, 8] / s) * (A[8, 11] / s) * (1 / (Iyy * s))
rep.check("tau_y -> x transfer = g/(Iyy s^4)", sp.simplify(Gx - g / (Iyy * s**4)) == 0)
# Lateral position is decoupled from collective thrust at hover (relative degree 4 in the linearisation)
rep.check("d(vdot_x)/d f = 0 at hover (lateral decoupled from collective thrust)", B[3, :] == sp.zeros(1, 4))

# Mass bias: open-loop residual acceleration if controller applies m g with true mass m'
dm = sp.symbols("Delta_m", real=True)
rep.check("open-loop mass bias zddot = -g dm/(m+dm)", sp.simplify(m * g / (m + dm) - g + g * dm / (m + dm)) == 0)

# Drag trim: exact pitch for constant velocity v along x vs approximation d_x v/(m g)
import params as P
D = P.derived()
mm, gg = P.m, P.g
dx = D["d_xy ~ K_aero*4*w_hover [N s/m]"]
dz = P.K_aero[2] / P.K_aero[0] * dx  # same rotor-speed scaling on the z axis


def trim(vel):
    def eqs(z):
        Tt, th = z
        Rp = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])
        Fd = -Rp @ np.diag([dx, dx, dz]) @ Rp.T @ np.array([vel, 0, 0])
        acc = Tt / mm * Rp[:, 2] - gg * np.array([0, 0, 1]) + Fd / mm
        return [acc[0], acc[2]]
    return fsolve(eqs, [mm * gg, 0.0])


worst = 0
for vel in (0.5, 1.0, 2.0):
    Tt, th = trim(vel)
    approx = dx * vel / (mm * gg)
    worst = max(worst, abs(th - approx) / abs(th))
    print(f"   v = {vel} m/s: exact trim pitch {th:.5f} rad, approx d_x v/(m g) = {approx:.5f} rad")
rep.check("drag trim pitch ~ d_x v/(m g) to 0.2 % up to 2 m/s, positive (nose down into the motion)", worst < 0.002, f"max rel err {worst:.1e}")
rep.check("drag trim tilt ~ 0.025 rad per m/s (entry 04)", abs(D["drag trim tilt per m/s [rad]"] - 0.025) < 0.001)

# Regression for audit A-04-1: a m g / Ixx has units 1/s^2; t_att = sqrt(Ixx/(a m g)) is a time,
# and tau_m / t_att is the dimensionless group.
L_, M_, T_ = sp.symbols("L M Tm", positive=True)  # length, mass, time dimensions
dims = (L_ * M_ * L_ / T_**2) / (M_ * L_**2)
rep.check("a m g / I_xx has units 1/s^2 (an angular acceleration, not a dimensionless group)", sp.simplify(dims * T_**2) == 1)
rep.check("t_att = sqrt(I_xx/(a m g)) has units of time, so tau_m/t_att is dimensionless",
          sp.simplify(sp.sqrt(1 / dims) / T_) == 1)

# Committed numbers quoted in entry 04 (Crazyflie 2.1+, entry 01 / decision Q5)
quoted = {
    "TWR": (1.69, 0.005),
    "c_tf/a": (0.215, 0.0005),
    "Izz/Ixx": (1.77, 0.005),
    "t_att [s]": (0.042, 0.0005),
    "tau_m/t_att": (1.53, 0.005),
    "a_up,max [m/s^2]": (6.7, 0.05),
    "max coordinated tilt [deg]": (54, 0.5),
    "yaw/roll angular-accel ratio": (0.12, 0.005),
}
for k, (val, tol) in quoted.items():
    rep.check(f"entry 04 quotes {k} = {val}", abs(D[k] - val) <= tol, f"computed {D[k]:.4g}")
# Authority with the firmware thrust floor f_min (fix after STEP 2 review), checked against exact LPs
from scipy.optimize import linprog
Mn = np.array(mixer(P.a, P.c_tf).evalf(), dtype=float)


def lp_tau_x(fmax, fmin, mass):
    r = linprog(-Mn[1], A_eq=Mn[[0, 2, 3]], b_eq=[mass * gg, 0, 0], bounds=[(fmin, fmax)] * 4, method="highs")
    return -r.fun


tau_lp = lp_tau_x(P.f_max, P.f_min, P.m)
tau_floor = lp_tau_x(1e3, P.f_min, P.m)      # ceiling removed: floor-limited value
rep.check("LP: hover roll authority = a(4 f_max - m g) (ceiling-limited) with f_min = THRUST_MIN",
          abs(tau_lp - P.a * (4 * P.f_max - P.m * gg)) < 1e-12 and abs(tau_lp - D["tau_x,max hover [N m]"]) < 1e-12)
rep.check("LP: authority is 84 % of the floor-limited value a(m g - 4 f_min)",
          abs(tau_lp / tau_floor - 0.84) < 0.005, f"{tau_lp / tau_floor:.3f}")
# knee: the TWR at which ceiling and floor bind together
knee = None
for twr in np.linspace(1.5, 2.2, 7001):
    fmax_t = twr * P.m * gg / 4
    if lp_tau_x(fmax_t, P.f_min, P.m) >= P.a * (P.m * gg - 4 * P.f_min) - 1e-12:
        knee = twr
        break
rep.check("LP: authority knee at TWR = 2 - f_min/f_hover = 1.82 (not 2)",
          knee is not None and abs(knee - 1.82) < 0.002 and abs(D["TWR knee = 2 - f_min/f_hover"] - 1.82) < 0.002, f"LP knee {knee:.4f}")
rep.check("vehicle is ceiling-limited but within 0.13 of the knee (TWR 1.69 < 1.82)",
          D["TWR"] < knee and knee - D["TWR"] < 0.14)
# max downward acceleration inside the box: all rotors at f_min (LP: minimise total thrust)
r = linprog(Mn[0], A_eq=Mn[[1, 2, 3]], b_eq=[0, 0, 0], bounds=[(P.f_min, P.f_max)] * 4, method="highs")
a_down = gg - r.fun / P.m
rep.check("LP: max downward acceleration inside the box = g - 4 f_min/m = 8.0 m/s^2 (not g)",
          abs(a_down - 8.04) < 0.005 and abs(a_down - D["a_down,max in box [m/s^2]"]) < 1e-9, f"{a_down:.3f}")

# Motor lag in the nominal model: transfer functions with the lag pole (entry 04 table)
tau = sp.symbols("tau_m", positive=True)
lag = 1 / (tau * s + 1)
Gx_cmd = Gx * lag      # commanded pitch torque -> x  (tau_y is a linear combination of delivered thrusts)
Gz_cmd = (1 / (m * s**2)) * lag
rep.check("commanded pitch torque -> x = g/(Iyy s^4 (tau_m s + 1))  (order 5)",
          sp.simplify(Gx_cmd - g / (Iyy * s**4 * (tau * s + 1))) == 0 and sp.degree(sp.denom(sp.together(Gx_cmd)), s) == 5)
rep.check("commanded collective thrust -> z = 1/(m s^2 (tau_m s + 1))  (order 3)",
          sp.degree(sp.denom(sp.together(Gz_cmd)), s) == 3)
# equilibrium with the lag state: fdot = (fcmd - f)/tau = 0  <=>  f = fcmd = m g / 4
fcmd = sp.symbols("fcmd", real=True)
rep.check("lag state at equilibrium: delivered = commanded = m g/4",
          sp.solve(sp.Eq((fcmd - m * g / 4) / tau, 0), fcmd) == [m * g / 4])

raise SystemExit(rep.finish())
