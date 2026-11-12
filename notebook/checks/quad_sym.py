"""Shared symbolic/numeric helpers for the notebook check scripts.

Conventions (entry 00, fixed): z-up world, FLU body, Hamilton scalar-first
quaternion mapping body vectors to world vectors, body-frame angular velocity.
"""
import sympy as sp
import numpy as np


def qmul(p, q):
    """Hamilton product p (x) q, scalar first. Works for sympy or numpy 4-vectors."""
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q
    return [
        p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
        p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
        p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
        p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0,
    ]


def qconj(q):
    return [q[0], -q[1], -q[2], -q[3]]


def R_entry03(q):
    """Rotation matrix exactly as written in entry 03."""
    q0, q1, q2, q3 = q
    return sp.Matrix([
        [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1**2 + q2**2)],
    ])


def R_num(q):
    return np.array(R_entry03(list(q)).evalf(), dtype=float) if isinstance(q[0], sp.Basic) else _R_np(q)


def _R_np(q):
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1**2 + q2**2)],
    ])


def mixer(a, c):
    """Entry 01 rotor-to-wrench map M: (T, tau_x, tau_y, tau_z) = M f."""
    return sp.Matrix([
        [1, 1, 1, 1],
        [a, a, -a, -a],
        [-a, a, a, -a],
        [c, -c, c, -c],
    ])


def rk4(f, x, u, h):
    k1 = f(x, u)
    k2 = f(x + 0.5 * h * k1, u)
    k3 = f(x + 0.5 * h * k2, u)
    k4 = f(x + h * k3, u)
    return x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


class Report:
    """Tiny pass/fail collector so every script ends with a clear summary."""

    def __init__(self, name):
        self.name = name
        self.rows = []

    def check(self, label, ok, detail=""):
        self.rows.append((label, bool(ok), detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""))

    def finish(self):
        n_fail = sum(1 for _, ok, _ in self.rows if not ok)
        print(f"\n{self.name}: {len(self.rows) - n_fail}/{len(self.rows)} checks passed")
        return n_fail
