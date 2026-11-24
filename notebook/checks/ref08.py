"""Entry 08: trajectory reference, linear closed-loop simulation under mismatch, and the oracle correction.

Reference. A PLAN supplies, at every sample, the reference r = (x, y, z, psi) and its analytic derivatives up to
order 5 (as a trajectory planner does). Over the horizon k = 0 ... N the MPC builds, by linear (small-angle)
differential flatness of the lagged hover model (entries 02-04):
  position, yaw                       <- r
  velocity, yaw rate                  <- r'
  roll = -y''/g, pitch = x''/g        <- r''
  body rates  w_x = -y'''/g, w_y = x'''/g
  delivered thrust f = M^-1 (m z'', -Ixx y''''/g, Iyy x''''/g, Izz psi'')
  commanded thrust u_ref = f + tau_m f'            (inverts the motor lag; uses r''''')
Cost: sum ||x_k - xi_k||_Q + ||u_k - u_ref,k||_R + ||x_N - xi_N||_P; no artificial steady state, no terminal set.

Correction. A reference correction dr = (dx, dy, dz, dpsi) shifts ONLY the position/yaw entries of xi over the
horizon. It is never differentiated: a finite-difference derivative of an arbitrary correction would be amplified
by 1/dt^order (dt^-5 = 1e10), which is what an earlier version of this module did and why it was replaced.
Unconstrained first move: u = Fx x0 + Fflat * flat_window + Fpos * dr_window.
"""
import numpy as np
import sympy as sp
from scipy.linalg import expm
import params as P
from linmodel import NX, NU, NY, MIX, continuous, zoh, f_hover
from mpc06 import DT, N, Q, R, Ad, Bd, Pm, K, Kx, Kr, A_LAG, prediction_matrices, frac_delay_gammas

NF = 6 * NY                     # flat vector per horizon step: r, r', ..., r''''' (4 components each)
NWS = N + 1                     # horizon steps in the window (k = 0 ... N)


def flat_operators(n=N):
    """Xi ((n+1)*16 x (n+1)*NF) and Ups (n*4 x (n+1)*NF) from the stacked flat vectors to (xi, u_ref)."""
    g, m = P.g, P.m
    Ixx, Iyy, Izz = np.diag(P.I_diag)
    Minv = np.linalg.inv(MIX)
    Xi = np.zeros(((n + 1) * NX, (n + 1) * NF))
    Ups = np.zeros((n * NU, (n + 1) * NF))
    for k in range(n + 1):
        col = lambda o, c: k * NF + o * NY + c          # derivative order o of component c at step k
        r = k * NX
        for i in range(3):
            Xi[r + i, col(0, i)] = 1.0                   # position
            Xi[r + 3 + i, col(1, i)] = 1.0               # velocity
        Xi[r + 8, col(0, 3)] = 1.0                       # yaw
        Xi[r + 6, col(2, 1)] = -1 / g                    # roll  = -y''/g
        Xi[r + 7, col(2, 0)] = 1 / g                     # pitch =  x''/g
        Xi[r + 9, col(3, 1)] = -1 / g                    # w_x   = -y'''/g
        Xi[r + 10, col(3, 0)] = 1 / g                    # w_y   =  x'''/g
        Xi[r + 11, col(1, 3)] = 1.0                      # w_z   =  psi'
        W = np.zeros((4, (n + 1) * NF)); Wd = np.zeros((4, (n + 1) * NF))
        W[0, col(2, 2)] = m;  W[1, col(4, 1)] = -Ixx / g;  W[2, col(4, 0)] = Iyy / g;  W[3, col(2, 3)] = Izz
        Wd[0, col(3, 2)] = m; Wd[1, col(5, 1)] = -Ixx / g; Wd[2, col(5, 0)] = Iyy / g; Wd[3, col(3, 3)] = Izz
        Xi[r + 12:r + 16] = Minv @ W
        if k < n:
            Ups[k * NU:(k + 1) * NU] = Minv @ (W + P.tau_m * Wd)
    return Xi, Ups


def trajectory_law(n=N):
    Sx, Su = prediction_matrices(n=n)
    Qbar = np.kron(np.eye(n + 1), Q)
    Qbar[-NX:, -NX:] = Pm
    Rbar = np.kron(np.eye(n), R)
    Mi = np.linalg.inv(Su.T @ Qbar @ Su + Rbar)
    Xi, Ups = flat_operators(n)
    Fx = -(Mi @ Su.T @ Qbar @ Sx)[:NU]
    Fflat = (Mi @ (Su.T @ Qbar @ Xi + Rbar @ Ups))[:NU]
    pos_cols = np.concatenate([np.arange(k * NF, k * NF + NY) for k in range(n + 1)])
    Fpos = Fflat[:, pos_cols]                                    # gain on the position/yaw entries (4 x 4(n+1))
    return Fx, Fflat, Fpos


Fx, Fflat, Fpos = trajectory_law()
FPOS_SUM = Fpos.reshape(NU, NWS, NY).sum(axis=1)                # gain of a horizon-constant shift (4 x 4)


# ------------------------------------------------------------------ plans (analytic derivatives)
t_sym = sp.symbols("t", real=True)


def _eval(expr, t):
    f = sp.lambdify(t_sym, expr, "numpy")
    v = f(t)
    return np.broadcast_to(v, t.shape).astype(float)


def plan_from(pieces, T_s):
    """pieces: list of (t_start, [expr_x, expr_y, expr_z, expr_psi]); returns D (6 x T x 4) of derivatives."""
    t = np.arange(int(round(T_s / DT))) * DT
    D = np.zeros((6, len(t), 4))
    for i, (t0, exprs) in enumerate(pieces):
        t1 = pieces[i + 1][0] if i + 1 < len(pieces) else np.inf
        mask = (t >= t0) & (t < t1)
        for c, e in enumerate(exprs):
            for o in range(6):
                D[o, mask, c] = _eval(sp.diff(e, t_sym, o), t[mask])
    return D


def S7(s):
    return s**4 * (35 - 84 * s + 70 * s**2 - 20 * s**3)     # smoothstep, zero 1st-3rd derivatives at 0 and 1


def traj_step(T_s=4.0, h=0.1, t_step=0.5):
    """Set-point change: the plan has no derivatives (a planner would not provide any for a step)."""
    t = np.arange(int(round(T_s / DT))) * DT
    D = np.zeros((6, len(t), 4)); D[0, t >= t_step, 0] = h
    return D


def traj_ramp(T_s=6.0, V=1.0, tb=1.0):
    s = t_sym / tb
    before = V * tb * (sp.Rational(7) * s**5 - 14 * s**6 + 10 * s**7 - sp.Rational(5, 2) * s**8)   # integral of S7
    after = V * tb * sp.Rational(1, 2) + V * (t_sym - tb)
    z = sp.Integer(0)
    return plan_from([(0.0, [before, z, z, z]), (tb, [after, z, z, z])], T_s)


def traj_fig8(T_s=9.0, A=0.28, period=3.0, tb=1.5):
    w = 2 * sp.pi / period
    fade = S7(t_sym / tb)
    z = sp.Integer(0)
    x8, y8 = A * sp.sin(w * t_sym), (A / 2) * sp.sin(2 * w * t_sym)
    return plan_from([(0.0, [fade * x8, fade * y8, z, z]), (tb, [x8, y8, z, z])], T_s)


def ref_of(D):
    return D[0]


# ------------------------------------------------------------------ perturbed linear plant
def plant_disc(mass_ratio=1.0, drag_scale=0.0, tau_ratio=1.0, D=0.0):
    """Discrete perturbed plant (about its own trim), with the injected delay D plus the modelled one-period
    computation delay: x+ = Phi x + G0 u_{k-d} + G1 u_{k-d-1} + c (c = mass bias)."""
    from plant07 import D_HOVER, DZ_OVER_DXY
    A, B = continuous(tau=P.tau_m * tau_ratio)
    mp = P.m * mass_ratio
    A[5, 12:16] = 1 / mp
    d_ = drag_scale * D_HOVER
    A[3, 3] = A[4, 4] = -d_ / mp
    A[5, 5] = -d_ * DZ_OVER_DXY / mp
    Phi, G0, G1, d = frac_delay_gammas(A, B, D + DT)
    E = np.zeros((NX + 1, NX + 1)); E[:NX, :NX] = A; E[5, NX] = P.g * (P.m / mp - 1)
    c = expm(E * DT)[:NX, NX]
    return Phi, G0, G1, d, c


def flat_window(Dp, k):
    T = Dp.shape[1]
    idx = np.clip(np.arange(k, k + NWS), 0, T - 1)
    return np.transpose(Dp[:, idx, :], (1, 0, 2)).reshape(-1)          # (k, order, comp) -> stacked


def pos_window(dr, k):
    T = dr.shape[0]
    idx = np.clip(np.arange(k, k + NWS), 0, T - 1)
    return dr[idx].reshape(-1)


def simulate(plan, law="traj", dr=None, x0=None, Kc=None, full=False, **mm):
    """Linear closed loop, implementation (b): at step k the controller predicts x_{k+1|k} with the command in
    flight and computes u_k for time k+1 from the plan window starting at k+1 (+ the correction window).
    law = "const": entry-06 MPC for tracking with the set-point r_{k+1} (ablation; no feedforward).
    dr: reference correction (T x 4), position/yaw only. Kc: state-feedback correction dr_k = Kc x_rigid,k."""
    Phi, G0, G1, d, c = plant_disc(**mm)
    T = plan.shape[1]
    ref = plan[0]
    dr = np.zeros((T, 4)) if dr is None else dr
    x = np.zeros(NX) if x0 is None else x0.copy()
    fhat = np.zeros(NU)
    hist = [np.zeros(NU)] * (d + 3)
    Y = np.zeros((T, 4)); Uh = np.zeros((T, NU)); Xh = np.zeros((T, NX))
    for k in range(T):
        Y[k] = [x[0], x[1], x[2], x[8]]; Xh[k] = x
        u_prev = hist[0]
        xhat = Ad @ np.r_[x[:12], fhat] + Bd @ u_prev
        fb = np.zeros(4) if Kc is None else Kc @ x[:12]
        if law == "traj":
            u = Fx @ xhat + Fflat @ flat_window(plan, k + 1) + Fpos @ pos_window(dr, k + 1) + FPOS_SUM @ fb
        else:
            kk = min(k + 1, T - 1)
            u = Kx @ xhat + Kr @ (ref[kk] + dr[kk] + fb)
        Uh[k] = u
        hist = [u] + hist[:-1]
        x = Phi @ x + G0 @ hist[d] + G1 @ hist[d + 1] + c
        fhat = A_LAG * fhat + (1 - A_LAG) * u_prev
    return (Y, Uh, Xh) if full else Y


def closed_loop_traj(Kc=None, **mm):
    """Closed-loop matrix of the trajectory law (implementation b, thrust estimate) with an optional
    state-feedback correction dr = Kc x_rigid applied over the whole horizon.
    State z = [x (16), fhat (4), u_{k-1}, ..., u_{k-d-2}]."""
    Phi, G0, G1, d, c = plant_disc(**mm)
    nb = d + 3
    n = NX + NU + NU * nb
    Sel = np.zeros((NX, n)); Sel[:12, :12] = np.eye(12); Sel[12:16, NX:NX + NU] = np.eye(NU)
    U0 = np.zeros((NU, n)); U0[:, NX + NU:NX + 2 * NU] = np.eye(NU)
    Fz = Fx @ (Ad @ Sel + Bd @ U0)
    if Kc is not None:
        Fz[:, :12] += FPOS_SUM @ Kc
    A = np.zeros((n, n))
    slot = lambda i: slice(NX + NU + NU * i, NX + NU + NU * (i + 1))
    Ud = np.zeros((NU, n)); Ud[:, slot(d - 1)] = np.eye(NU)
    Ud1 = np.zeros((NU, n)); Ud1[:, slot(d)] = np.eye(NU)
    A[:NX] = np.hstack([Phi, np.zeros((NX, n - NX))]) + G0 @ Ud + G1 @ Ud1
    A[NX:NX + NU, NX:NX + NU] = A_LAG * np.eye(NU)
    A[NX:NX + NU, slot(0)] += (1 - A_LAG) * np.eye(NU)
    A[slot(0)] = Fz
    for i in range(1, nb):
        A[slot(i), slot(i - 1)] = np.eye(NU)
    return A


def rho(A):
    return float(np.max(np.abs(np.linalg.eigvals(A))))


def bisect(f, lo, hi, tol=1e-6):
    flo = f(lo)
    assert flo * f(hi) < 0
    while abs(hi - lo) > tol:
        mid = 0.5 * (lo + hi)
        if (f(mid) < 0) == (flo < 0):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def interior(T, head=1.5, tail=1.0):
    return slice(int(round(head / DT)), T - int(round(tail / DT)))


def rms_int(ref, y, head=1.5, tail=1.0):
    sl = interior(ref.shape[0], head, tail)
    return float(np.sqrt(np.mean(np.sum((y[sl, 0:3] - ref[sl, 0:3]) ** 2, axis=1))))


# ------------------------------------------------------------------ the oracle
LAMBDA = 0.01      # invert the closed loop only where its gain exceeds sqrt(lambda) = 0.1 (-20 dB)
SKIP = NWS         # first window left uncorrected
TAIL = 50          # last 0.5 s left uncorrected: their effect would fall outside the window


def response_matrix(T, skip=SKIP, tail=TAIL, **mm):
    """y = y0 + S dr for position corrections (x, y, z), built from impulse responses (the loop is LTI)."""
    L = 2 * T
    zero_plan = np.zeros((6, L, 4))
    base = simulate(zero_plan, **mm)[:, 0:3]
    resp = {}
    for a in range(3):
        dr = np.zeros((L, 4)); dr[T, a] = 1.0
        resp[a] = simulate(zero_plan, dr=dr, **mm)[:, 0:3] - base
    cols = list(range(skip, T - tail))
    S = np.zeros((3 * T, 3 * len(cols)))
    for ci, j in enumerate(cols):
        for a in range(3):
            S[:, 3 * ci + a] = resp[a][T - j:2 * T - j].reshape(-1)
    return S, cols


def oracle(plan, lam=LAMBDA, **mm):
    """Regularised least-squares correction over the whole window: argmin ||y(r + dr) - r||^2 + lam ||dr||^2."""
    T = plan.shape[1]
    ref = plan[0]
    y0 = simulate(plan, **mm)[:, 0:3]
    S, cols = response_matrix(T, **mm)
    e = (ref[:, 0:3] - y0).reshape(-1)
    G = np.linalg.solve(S.T @ S + lam * np.eye(S.shape[1]), S.T)
    z = G @ e
    dr = np.zeros((T, 4))
    for ci, j in enumerate(cols):
        dr[j, 0:3] = z[3 * ci:3 * ci + 3]
    return dr, y0, G, S, cols
