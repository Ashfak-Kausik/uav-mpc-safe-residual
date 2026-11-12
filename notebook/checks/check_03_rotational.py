"""Entry 03: R(q), quaternion kinematics, rotational dynamics, standing tests."""
import numpy as np
import sympy as sp
from quad_sym import Report, R_entry03, _R_np, qmul, qconj, rk4

rep = Report("check_03_rotational")
q0, q1, q2, q3 = sp.symbols("q0:4", real=True)
q = [q0, q1, q2, q3]
R = R_entry03(q)

# R(q) is the matrix of v -> q (x) [0,v] (x) q*, i.e. Hamilton, body-to-world
v = sp.symbols("v1:4", real=True)
rot = qmul(qmul(q, [0, *v]), qconj(q))
Rv = R * sp.Matrix(v)
unit = {q0: sp.sqrt(1 - q1**2 - q2**2 - q3**2)}
diff_rot = [sp.simplify((rot[i + 1] - Rv[i]).subs(unit)) for i in range(3)]
rep.check("R(q) in entry 03 equals q (x) v (x) q* (Hamilton, body->world) on unit q", all(d == 0 for d in diff_rot))
rep.check("R^T R = I on unit quaternions", sp.simplify((R.T * R - sp.eye(3)).subs(unit)) == sp.zeros(3, 3))
rep.check("det R = 1 on unit quaternions", sp.simplify(R.det().subs(unit)) == 1)
b3 = R.col(2)
rep.check("b3 = third column as written", sp.simplify(b3 - sp.Matrix([2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1**2 + q2**2)])) == sp.zeros(3, 1))
# b3 independent of rotation about b3 itself: R(q (x) qz(psi)) e3 = R(q) e3
psi = sp.symbols("psi", real=True)
qz = [sp.cos(psi / 2), 0, 0, sp.sin(psi / 2)]
rep.check("b3 invariant under body-z rotation (yaw freedom)",
          sp.simplify(R_entry03(qmul(q, qz)).col(2) - b3) == sp.zeros(3, 1))

# Kinematics: qdot = 1/2 q (x) [0, w] is consistent with Rdot = R skew(w) (body w)
w = sp.symbols("w1:4", real=True)
qdot = [sp.Rational(1, 2) * e for e in qmul(q, [0, *w])]
Rdot = sp.Matrix(3, 3, lambda i, j: sum(sp.diff(R[i, j], q[k]) * qdot[k] for k in range(4)))
W = sp.Matrix([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
rep.check("qdot = 1/2 q(x)[0,w] gives Rdot = R [w]x  (body-frame w)", sp.simplify((Rdot - R * W).subs(unit)) == sp.zeros(3, 3))
rep.check("q . qdot = 0 (norm preserved in continuous time)", sp.simplify(sum(q[k] * qdot[k] for k in range(4))) == 0)


# Numerical convention check from entry 03 (constant body rate about +y for 0.3 s)
def f_att(x, u):
    qq, ww = x[:4], x[4:]
    return np.concatenate([0.5 * np.array(qmul(qq, [0, *ww])), np.zeros(3)])


x = np.array([1, 0, 0, 0, 0, 1.0, 0])
h = 1e-3
for _ in range(300):
    x = rk4(f_att, x, None, h)
    x[:4] /= np.linalg.norm(x[:4])
Rn = _R_np(x[:4])
rep.check("pitch-rate test: nose -> (cos .3, 0, -sin .3), b3 -> (sin .3, 0, cos .3)",
          np.allclose(Rn[:, 0], [np.cos(.3), 0, -np.sin(.3)], atol=1e-9) and np.allclose(Rn[:, 2], [np.sin(.3), 0, np.cos(.3)], atol=1e-9),
          f"nose={Rn[:, 0].round(4)}, b3={Rn[:, 2].round(4)}")

# Gyroscopic sign test. Entry 03 claims a wrong sign makes torque-free energy drift.
I = np.diag([1.395e-5, 1.436e-5, 2.173e-5])  # order of magnitude only; not a parameter commitment
Iinv = np.linalg.inv(I)


def f_rb(x, sign):
    qq, ww = x[:4], x[4:]
    wdot = Iinv @ (-sign * np.cross(ww, I @ ww))
    return np.concatenate([0.5 * np.array(qmul(qq, [0, *ww])), wdot])


res = {}
for sign, label in [(1, "correct"), (-1, "flipped")]:
    x = np.array([1, 0, 0, 0, 3.0, 0.2, 1.0])
    E0 = 0.5 * x[4:] @ I @ x[4:]
    L0 = _R_np(x[:4]) @ I @ x[4:]
    Lb0 = np.linalg.norm(I @ x[4:])
    for _ in range(4000):
        x = rk4(lambda s, u: f_rb(s, sign), x, None, 1e-3)
        x[:4] /= np.linalg.norm(x[:4])
    E = 0.5 * x[4:] @ I @ x[4:]
    L = _R_np(x[:4]) @ I @ x[4:]
    res[label] = (abs(E - E0) / E0, abs(np.linalg.norm(I @ x[4:]) - Lb0) / Lb0, np.linalg.norm(L - L0) / np.linalg.norm(L0))
for k, (dE, dLmag, dL) in res.items():
    print(f"   {k:8s}: rel. energy drift {dE:.1e}, rel. |I w| drift {dLmag:.1e}, rel. world-L vector drift {dL:.1e}")
rep.check("energy is conserved with the CORRECT gyroscopic sign", res["correct"][0] < 1e-8)
rep.check("entry 03 claim 'wrong sign -> energy drifts' holds", res["flipped"][0] > 1e-6,
          "FALSE: energy and |Iw| are conserved under either sign (w.(w x Iw) = 0)")
rep.check("world-frame angular momentum VECTOR R I w detects the flipped sign", res["flipped"][2] > 1e-3 and res["correct"][2] < 1e-8)

raise SystemExit(rep.finish())
