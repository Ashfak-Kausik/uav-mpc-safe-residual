"""Entry 02: translational dynamics, inversion, coordinated tilt, relative degree."""
import numpy as np
import sympy as sp
from quad_sym import Report, R_entry03

rep = Report("check_02_translational")
m, g, T = sp.symbols("m g T", positive=True)
th = sp.symbols("theta", real=True)
e3 = sp.Matrix([0, 0, 1])

# Pitch-only attitude: q = (cos th/2, 0, sin th/2, 0)
q = [sp.cos(th / 2), 0, sp.sin(th / 2), 0]
b3 = sp.simplify(R_entry03(q) * e3)
rep.check("b3 for pitch theta = (sin th, 0, cos th)", sp.simplify(b3 - sp.Matrix([sp.sin(th), 0, sp.cos(th)])) == sp.zeros(3, 1))
vdot = T / m * b3 - g * e3
rep.check("hover: vdot = 0 at level iff T = m g", sp.solve(vdot.subs(th, 0)[2], T) == [m * g])
Tc = m * g / sp.cos(th)
rep.check("coordinated tilt T = m g / cos th keeps vdot_z = 0", sp.simplify(vdot[2].subs(T, Tc)) == 0)
rep.check("a_lat = g tan th at coordinated tilt", sp.simplify(vdot[0].subs(T, Tc) - g * sp.tan(th)) == 0)
twr = 2
th_max = np.arccos(1 / twr)
rep.check("TWR = 2 -> max coordinated tilt 60 deg, a_lat = 1.73 g",
          abs(np.degrees(th_max) - 60) < 1e-9 and abs(np.tan(th_max) - np.sqrt(3)) < 1e-12)

# Inversion: T_req, b3_req reproduce a_des
rng = np.random.default_rng(0)
ok = True
for _ in range(1000):
    ad = rng.normal(size=3) * 5
    mm, gg = 0.03, 9.81
    w = ad + gg * np.array([0, 0, 1])
    Treq = mm * np.linalg.norm(w)
    b = w / np.linalg.norm(w)
    ok &= np.allclose(Treq / mm * b - gg * np.array([0, 0, 1]), ad)
rep.check("inversion T_req = m||a+g e3||, b3_req = (a+g e3)/||.|| reproduces a_des", ok)
# free fall: a_des = -g e3 gives T_req = 0 and an undefined direction; near it the direction is ill-conditioned
gg_ = 9.81
wff = np.array([0.0, 0.0, -gg_]) + gg_ * np.array([0, 0, 1])
eps_ = 1e-3
b_a = (np.array([eps_, 0, -gg_]) + gg_ * np.array([0, 0, 1])); b_a /= np.linalg.norm(b_a)
b_b = (np.array([-eps_, 0, -gg_]) + gg_ * np.array([0, 0, 1])); b_b /= np.linalg.norm(b_b)
rep.check("inversion singular at a_des = -g e3: T_req = 0, and a 2e-3 m/s^2 change in the request flips b3_req by 180 deg",
          np.linalg.norm(wff) == 0 and np.isclose(b_a @ b_b, -1.0))

# Relative degree with rotor thrusts as inputs.
# With u = (T, tau), xddot = (T/m) b3_x : T appears at the 2nd derivative with coefficient b3_x/m,
# which vanishes at hover. Hence the 'relative degree 4 from torque' is exact only in the
# hover linearization; in the nonlinear model the decoupling matrix of (x,y,z,psi) is singular.
q0, q1, q2, q3 = sp.symbols("q0:4", real=True)
R = R_entry03([q0, q1, q2, q3])
xdd = (T / m * R * e3)
coef_T_in_xdd = sp.diff(xdd[0], T)
rep.check("d(xddot)/dT = b3_x/m is nonzero away from hover",
          sp.simplify(coef_T_in_xdd - R[0, 2] / m) == 0 and coef_T_in_xdd.subs({q0: 1, q1: 0, q2: 0, q3: 0}) == 0)
# Decoupling matrix at the 2nd derivative of (x, y, z) w.r.t. (T, tau): only the T column is nonzero -> rank 1
Dec = sp.Matrix([[sp.diff(xdd[i], T), 0, 0, 0] for i in range(3)])
rep.check("2nd-derivative decoupling matrix of (x,y,z) has rank 1 -> no vector relative degree without dynamic extension",
          Dec.subs({q0: 1, q1: 0, q2: 0, q3: 0}).rank() == 1)

# Dynamic extension (entry 02, Mistler et al. 2001): with T a state and Tddot an input, the 4th derivative
# of position and the 2nd derivative of yaw depend on (Tddot, tau) through an invertible matrix when T > 0.
# p'''' = (Tddot b3 + 2 Tdot b3dot + T b3ddot)/m,  b3ddot = R(w x (w x e3)) + R(wdot x e3),  wdot = I^-1(tau - w x Iw)
# Near hover yaw psi has psi'' = wz_dot (+ terms without inputs).
Ixx, Iyy, Izz = sp.symbols("I_xx I_yy I_zz", positive=True)
Tdd, tx, ty, tz = sp.symbols("Tdd tau_x tau_y tau_z", real=True)
wv = sp.Matrix(sp.symbols("w1:4", real=True))
Tdot = sp.symbols("Tdot", real=True)
Iin = sp.diag(Ixx, Iyy, Izz)
wdot = Iin.inv() * (sp.Matrix([tx, ty, tz]) - wv.cross(Iin * wv))
b3dd = R * (wv.cross(wv.cross(e3))) + R * (wdot.cross(e3))
b3d = R * (wv.cross(e3))
p4 = (Tdd * (R * e3) + 2 * Tdot * b3d + T * b3dd) / m
yaw2 = wdot[2]
outs = sp.Matrix([p4[0], p4[1], p4[2], yaw2])
Dmat = outs.jacobian(sp.Matrix([Tdd, tx, ty, tz]))
at_hover = {q0: 1, q1: 0, q2: 0, q3: 0, wv[0]: 0, wv[1]: 0, wv[2]: 0}
D0 = sp.simplify(Dmat.subs(at_hover))
rep.check("dynamic extension: decoupling matrix of (x,y,z,psi) wrt (Tddot, tau) is invertible at hover for T>0",
          sp.simplify(D0.det()) != 0, f"det = {sp.factor(D0.det())}")

raise SystemExit(rep.finish())
