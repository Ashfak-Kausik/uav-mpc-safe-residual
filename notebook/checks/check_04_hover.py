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
mm, gg, dx, dz = 0.027, 9.81, 9.1e-3, 1.0e-2  # placeholder magnitudes only, NOT parameters


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
rep.check("drag trim pitch ~ d_x v/(m g), positive (nose down into the motion)", worst < 0.02, f"max rel err {worst:.1e}")

# Units of the 'dimensionless' group a m g / Ixx
L_, M_, T_ = sp.symbols("L M Tm", positive=True)  # length, mass, time dimensions
dims = (L_ * M_ * L_ / T_**2) / (M_ * L_**2)
rep.check("a m g / I_xx is dimensionless (entry 04 claim)", sp.simplify(dims) == 1, f"actual dimension = {sp.simplify(dims)} (1/s^2)")

raise SystemExit(rep.finish())
