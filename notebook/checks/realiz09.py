"""Entry 09: the realizability ratio. The constrained trajectory QP (entry 08 law + entry 06 constraints), the
closed-form ratio test r_H, the exact piecewise-affine (PWA) transmission ratio r_PWA, and the output-realizability
LP ratio r_LP.

All inequality rows of the QP, with the slacks at zero, are written as  G U <= h(x0, u_prev):
  input box (upper, lower), input rate (upper, lower, first row relative to u_prev),
  soft tilt |phi_k|, |theta_k| <= TILT_MAX and soft altitude z_k >= Z_MIN (k = 1 ... N).
"""
import numpy as np
from scipy.optimize import linprog
import params as P
from linmodel import NX, NU, NY
from mpc06 import (DT, N, Q, R, Ad, Bd, Pm, A_LAG, U_MIN, U_MAX, DU_MAX, TILT_MAX, Z_MIN, RHO1, RHO2,
                   prediction_matrices)
from ref08 import flat_operators, NF, NWS, flat_window, pos_window, plant_disc

Sx, Su = prediction_matrices()
Qbar = np.kron(np.eye(N + 1), Q); Qbar[-NX:, -NX:] = Pm
Rbar = np.kron(np.eye(N), R)
Xi, Ups = flat_operators()
POS_COLS = np.concatenate([np.arange(k * NF, k * NF + NY) for k in range(N + 1)])
HU = 2 * (Su.T @ Qbar @ Su + Rbar)                          # Hessian in U (slacks at zero)
HU_inv = np.linalg.inv(HU)
FU_x = 2 * Su.T @ Qbar @ Sx                                  # f = FU_x x0 + FU_F F,  F = flat window (+ correction)
FU_F = -2 * (Su.T @ Qbar @ Xi + Rbar @ Ups)
DU_DR = -HU_inv @ FU_F[:, POS_COLS]                          # dU*/d(position-yaw correction window): 4N x 4(N+1)

# ------------------------------------------------------------------ inequality rows G U <= h
nU = NU * N
I_U = np.eye(nU)
D_rate = np.eye(nU) - np.eye(nU, k=-NU)                       # u_k - u_{k-1} (first block: u_0)
TILT_IDX = [(k, i) for k in range(1, N + 1) for i in (6, 7)]
G_tilt = np.array([Su[k * NX + i] for k, i in TILT_IDX])
S_tilt = np.array([Sx[k * NX + i] for k, i in TILT_IDX])
G_alt = np.array([Su[k * NX + 2] for k in range(1, N + 1)])
S_alt = np.array([Sx[k * NX + 2] for k in range(1, N + 1)])
G = np.vstack([I_U, -I_U, D_rate, -D_rate, G_tilt, -G_tilt, -G_alt])
ROW_KIND = (["box+"] * nU + ["box-"] * nU + ["rate+"] * nU + ["rate-"] * nU
            + ["tilt+"] * len(TILT_IDX) + ["tilt-"] * len(TILT_IDX) + ["alt"] * N)
YROWS = np.array([k * NX + i for k in range(1, N + 1) for i in range(3)])       # predicted positions k = 1..N


def h_vec(x0, u_prev):
    rate_off = np.zeros(nU); rate_off[:NU] = u_prev
    return np.r_[np.full(nU, U_MAX), np.full(nU, -U_MIN), DU_MAX + rate_off, DU_MAX - rate_off,
                 TILT_MAX - S_tilt @ x0, TILT_MAX + S_tilt @ x0, -(Z_MIN - S_alt @ x0)]


def f_vec(x0, Fwin):
    return FU_x @ x0 + FU_F @ Fwin


def affine_U(x0, Fwin):
    """Unconstrained minimiser (slacks zero): the affine law over the whole horizon."""
    return -HU_inv @ f_vec(x0, Fwin)


def full_window(plan, dr, k):
    """Flat window from the plan starting at k, with the position/yaw correction added to the position entries."""
    Fw = flat_window(plan, k).copy()
    Fw[POS_COLS] += pos_window(dr, k)
    return Fw


# ------------------------------------------------------------------ the constrained QP (DAQP), U + 2 slacks
class TrajQP:
    def __init__(self):
        n_s = 2
        self.nz = nU + n_s
        H = np.zeros((self.nz, self.nz)); H[:nU, :nU] = HU; H[nU:, nU:] = 2 * RHO2 * np.eye(n_s)
        self.H = H
        # general rows: rate (two-sided), tilt (one-sided pairs, slack 0), altitude (slack 1)
        Gt = np.hstack([np.vstack([G_tilt, -G_tilt]), -np.ones((2 * len(TILT_IDX), 1)), np.zeros((2 * len(TILT_IDX), 1))])
        Ga = np.hstack([G_alt, np.zeros((N, 1)), np.ones((N, 1))])
        self.A = np.vstack([np.hstack([D_rate, np.zeros((nU, n_s))]), Gt, Ga])

    def solve(self, x0, u_prev, Fwin):
        import daqp
        f = np.r_[f_vec(x0, Fwin), RHO1, RHO1]
        bl_s = np.r_[np.full(nU, U_MIN), 0.0, 0.0]
        bu_s = np.r_[np.full(nU, U_MAX), 1e30, 1e30]
        rate_l = np.full(nU, -DU_MAX); rate_u = np.full(nU, DU_MAX)
        rate_l[:NU] += u_prev; rate_u[:NU] += u_prev
        tu = np.r_[TILT_MAX - S_tilt @ x0, TILT_MAX + S_tilt @ x0]
        bl = np.r_[bl_s, rate_l, np.full(len(tu), -1e30), Z_MIN - S_alt @ x0]
        bu = np.r_[bu_s, rate_u, tu, np.full(N, 1e30)]
        z, fval, flag, info = daqp.solve(self.H, f, self.A, bu, bl, primal_tol=1e-9)
        return np.asarray(z), flag


# ------------------------------------------------------------------ the three ratios
def ratio_test(U, dU, x0, u_prev, tol=1e-12):
    """r_H: largest beta in [0, 1] with G (U + beta dU) <= h. Returns (r, index of the limiting row or None).
    A row already at or beyond its bound that the correction pushes further gives r_H = 0."""
    slack = np.maximum(h_vec(x0, u_prev) - G @ U, 0.0)
    rate = G @ dU
    pos = rate > tol
    if not pos.any():
        return 1.0, None
    b = slack[pos] / rate[pos]
    i = int(np.argmin(b))
    r = float(min(1.0, b[i]))
    return r, (int(np.flatnonzero(pos)[i]) if r < 1.0 else None)


def pwa_ratio(U, x0, u_prev, Fwin, dF, act_tol=1e-9):
    """r_PWA: largest beta in [0, 1] for which the QP's solution stays in its current critical region (same active
    set) as the correction beta * dF is added. KKT sensitivity of the current active set: exact for the QP.
    Returns (r, dU, lam, active set)."""
    h = h_vec(x0, u_prev)
    slack = h - G @ U
    Aset = np.flatnonzero(slack <= act_tol)
    GA = G[Aset]
    nA = len(Aset)
    f = f_vec(x0, Fwin)
    lam = np.linalg.lstsq(GA.T, -(HU @ U + f), rcond=None)[0] if nA else np.zeros(0)
    KKT = np.block([[HU, GA.T], [GA, np.zeros((nA, nA))]])
    rhs = np.r_[-FU_F @ dF, np.zeros(nA)]
    sol = np.linalg.lstsq(KKT, rhs, rcond=None)[0]
    dU, dlam = sol[:nU], sol[nU:]
    r = 1.0
    rate = G @ dU
    inact = np.setdiff1d(np.arange(G.shape[0]), Aset)
    for i in inact:
        if rate[i] > 1e-12:
            r = min(r, max(slack[i], 0.0) / rate[i])
    for j in range(nA):
        if dlam[j] < -1e-12:
            r = min(r, max(lam[j], 0.0) / -dlam[j])
    return r, dU, lam, Aset


def lp_ratio(U, x0, u_prev, dU):
    """r_LP: largest beta in [0, 1] such that SOME admissible input sequence delivers the predicted position
    trajectory Y(U) + beta * Cy dU (positions k = 1..N) under all QP rows with slacks at zero.
    Written around the ratio-test candidate: U' = U + beta dU + w with Cy w = 0 (w changes the inputs without
    changing the requested positions), and every row normalised, so that the LP is well conditioned.
    By construction r_LP >= r_H whenever U itself satisfies all rows (w = 0, beta = r_H is feasible)."""
    Cy = Su[YROWS]
    h = np.maximum(h_vec(x0, u_prev), G @ U)        # soft rows already violated by the QP may stay as violated
    slack = h - G @ U
    Gd = G @ dU
    # rows: G w + beta Gd <= slack ; normalise each row by its coefficient norm
    A_ub = np.hstack([G, Gd.reshape(-1, 1)])
    sc = np.maximum(np.linalg.norm(A_ub, axis=1), 1e-12)
    A_ub, b_ub = A_ub / sc[:, None], slack / sc
    # equality Cy w = 0 through an orthonormal basis of its null space: w = Z v  (removes the equality entirely)
    _, s_, Vt = np.linalg.svd(Cy)
    Z = Vt[np.sum(s_ > 1e-12 * s_[0]):].T
    A_v = np.hstack([A_ub[:, :nU] @ Z, A_ub[:, nU:]])
    c = np.r_[np.zeros(Z.shape[1]), -1.0]
    res = linprog(c, A_ub=A_v, b_ub=b_ub, bounds=[(None, None)] * Z.shape[1] + [(0, 1)], method="highs")
    return (res.x[-1] if res.success else np.nan), res.status


def lin_step(x, fhat, hist, u, Phi, G0, G1, d, c):
    """One period of the linear perturbed plant with implementation (b) bookkeeping (as ref08.simulate)."""
    u_prev = hist[0]
    hist = [u] + hist[:-1]
    x = Phi @ x + G0 @ hist[d] + G1 @ hist[d + 1] + c
    fhat = A_LAG * fhat + (1 - A_LAG) * u_prev
    return x, fhat, hist
