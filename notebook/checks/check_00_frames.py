"""Entry 00: elementary rotations and the sign table (roll/pitch/yaw effects)."""
import sympy as sp
from quad_sym import Report

rep = Report("check_00_frames")
phi, th, psi = sp.symbols("phi theta psi", real=True)
e1, e2, e3 = sp.eye(3).col(0), sp.eye(3).col(1), sp.eye(3).col(2)

Rx = sp.Matrix([[1, 0, 0], [0, sp.cos(phi), -sp.sin(phi)], [0, sp.sin(phi), sp.cos(phi)]])
Ry = sp.Matrix([[sp.cos(th), 0, sp.sin(th)], [0, 1, 0], [-sp.sin(th), 0, sp.cos(th)]])
Rz = sp.Matrix([[sp.cos(psi), -sp.sin(psi), 0], [sp.sin(psi), sp.cos(psi), 0], [0, 0, 1]])

rep.check("Rx(phi) e3 = (0, -sin phi, cos phi)", sp.simplify(Rx * e3 - sp.Matrix([0, -sp.sin(phi), sp.cos(phi)])) == sp.zeros(3, 1))
rep.check("Ry(theta) e3 = (sin th, 0, cos th)", sp.simplify(Ry * e3 - sp.Matrix([sp.sin(th), 0, sp.cos(th)])) == sp.zeros(3, 1))
rep.check("Rz(psi) e1 = (cos psi, sin psi, 0)", sp.simplify(Rz * e1 - sp.Matrix([sp.cos(psi), sp.sin(psi), 0])) == sp.zeros(3, 1))

# Physical readings of the table
rep.check("positive roll lifts the left side (body y gets +z)", sp.simplify((Rx * e2)[2] - sp.sin(phi)) == 0)
rep.check("positive pitch is nose down (body x gets -z)", sp.simplify((Ry * e1)[2] + sp.sin(th)) == 0)
rep.check("positive yaw turns nose left (+y)", sp.simplify((Rz * e1)[1] - sp.sin(psi)) == 0)

# Each elementary matrix is the exponential of the right-hand-rule generator
for name, Rm, ax, ang in [("x", Rx, e1, phi), ("y", Ry, e2, th), ("z", Rz, e3, psi)]:
    K = sp.Matrix([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    dR0 = sp.diff(Rm, ang).subs(ang, 0)
    rep.check(f"R_{name} is a right-hand rotation (dR/dangle at 0 = skew(axis))", sp.simplify(dR0 - K) == sp.zeros(3, 3))

raise SystemExit(rep.finish())
