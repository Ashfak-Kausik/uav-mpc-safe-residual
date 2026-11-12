"""Shared linear hover model and MPC-for-tracking design (entries 05-06, reused by later entries).

State x (16, error form of entry 05): p(0:3), v(3:6), th = (phi, theta, psi)(6:9), w(9:12), f(12:16),
all deviations from hover; f = delivered rotor thrust deviation. Input u (4): commanded rotor thrust
deviation, u_abs = f_hover + u. The 17-state quaternion model of entry 05 differs only by the
redundant q0 direction, which is identically constant to first order and is dropped here.
"""
import numpy as np
from scipy.linalg import expm, solve_discrete_are
import params as P

NX, NU, NY = 16, 4, 4
g, m, a, c = P.g, P.m, P.a, P.c_tf
f_hover = m * g / 4
MIX = np.array([[1, 1, 1, 1], [a, a, -a, -a], [-a, a, a, -a], [c, -c, c, -c]])


def continuous(tau=P.tau_m, I=P.I_diag):
    A = np.zeros((NX, NX))
    B = np.zeros((NX, NU))
    A[0:3, 3:6] = np.eye(3)
    A[3, 7] = g
    A[4, 6] = -g
    A[5, 12:16] = 1 / m
    A[6:9, 9:12] = np.eye(3)
    A[9:12, 12:16] = np.linalg.inv(I) @ MIX[1:4]
    A[12:16, 12:16] = -np.eye(4) / tau
    B[12:16, :] = np.eye(4) / tau
    return A, B


def zoh(A, B, dt):
    n, k = B.shape
    M = np.zeros((n + k, n + k))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n:]


# outputs tracked: position and yaw
C = np.zeros((NY, NX))
C[0:3, 0:3] = np.eye(3)
C[3, 8] = 1.0
# steady-state manifold: x_s = G theta, u_s = 0 (hover at any position and yaw)
G = C.T.copy()


def lqr(Ad, Bd, Q, R):
    Pm = solve_discrete_are(Ad, Bd, Q, R)
    K = -np.linalg.solve(R + Bd.T @ Pm @ Bd, Bd.T @ Pm @ Ad)   # u = K (x - x_s)
    return Pm, K


def tracking_law(Pm, K, T):
    """Unconstrained MPC-for-tracking law u = Kx x + Kr r (terminal cost = DARE solution, so the
    finite-horizon optimum equals the LQR from x to x_s; the artificial steady state minimises
    (x - G th)' P (x - G th) + (th - r)' T (th - r))."""
    S = np.linalg.inv(G.T @ Pm @ G + T)
    Kx = K @ (np.eye(NX) - G @ S @ G.T @ Pm)
    Kr = -K @ G @ S @ T
    return Kx, Kr, S
