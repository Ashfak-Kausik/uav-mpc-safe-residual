"""Entry 10: the loop-gain bound. Closed loop under mismatch as an LTI map G_theta from the (horizon-constant)
reference correction to the signal the learned map reads; certified H-infinity norm; small-gain bound on trust.

Scaling (Bryson deviations of entry 06): every signal is divided by its acceptable deviation, so that gains and
Lipschitz constants are dimensionless and computed in the same norm.
  read signal y_k = (x_rigid,k / dev, u_{k-1} / headroom)  (16)    correction delta_k = (dx, dy, dz, dpsi) / S_DELTA
"""
import numpy as np
from mpc06 import DT, DEV, HEADROOM
from ref08 import closed_loop_traj, FPOS_SUM, plant_disc
from linmodel import NX, NU

DEV_RIGID = np.r_[[DEV["p"]] * 3, [DEV["v"]] * 3, [DEV["tilt"]] * 2, DEV["yaw"], [DEV["w"]] * 2, DEV["wz"]]
S_DELTA = np.r_[0.1, 0.1, 0.1, 0.5]          # correction scale: 0.1 m in position, 0.5 rad in yaw
NY_READ = 16


def lti(read="full", **mm):
    """(A, B, C) of the closed loop (trajectory law, implementation b) from the scaled correction delta_k (applied
    as a shift of the whole horizon) to the scaled read signal. read: "full" (16), "position" (3), "tilt" (2)."""
    A = closed_loop_traj(**mm)
    n = A.shape[0]
    B = np.zeros((n, 4))
    slot0 = slice(NX + NU, NX + 2 * NU)        # u_k enters buffer slot 0 (it is u_{k-1} at the next step)
    B[slot0] = FPOS_SUM @ np.diag(S_DELTA)
    Cx = np.zeros((12, n)); Cx[:, :12] = np.diag(1 / DEV_RIGID)
    Cu = np.zeros((NU, n)); Cu[:, slot0] = np.eye(NU) / HEADROOM
    if read == "full":
        C = np.vstack([Cx, Cu])
    elif read == "position":
        C = Cx[0:3]
    elif read == "tilt":
        C = Cx[6:8]
    else:
        C = read(n)
    return A, B, C


def freq_resp(A, B, C, w):
    z = np.exp(1j * w * DT)
    return C @ np.linalg.solve(z * np.eye(A.shape[0]) - A, B)


def hinf_sweep(A, B, C, n=4000):
    """Lower bound: maximum singular value over a dense frequency grid (plus the grid frequency of the peak)."""
    ws = np.linspace(1e-4, np.pi / DT, n)
    vals = [np.linalg.svd(freq_resp(A, B, C, w), compute_uv=False)[0] for w in ws]
    i = int(np.argmax(vals))
    return vals[i], ws[i]


def hinf_certified(A, B, C, D=None, rtol=1e-6):
    """Upper bound to relative tolerance rtol: bisection on the Hamiltonian test of Boyd, Balakrishnan and Kabamba
    (1989) after the bilinear map z = (1 + s)/(1 - s), which maps the unit circle onto the imaginary axis and
    preserves the H-infinity norm. gamma > ||G||_inf  iff  the Hamiltonian H(gamma) has no imaginary eigenvalue."""
    n, m = B.shape
    p = C.shape[0]
    D = np.zeros((p, m)) if D is None else D
    Ip = np.linalg.inv(A + np.eye(n))
    Ac = Ip @ (A - np.eye(n))
    Bc = np.sqrt(2) * Ip @ B
    Cc = np.sqrt(2) * C @ Ip
    Dc = D - C @ Ip @ B
    assert np.max(np.linalg.eigvals(Ac).real) < 0, "closed loop must be stable"

    def has_imag(g):
        R = g**2 * np.eye(m) - Dc.T @ Dc
        Ri = np.linalg.inv(R)
        Ah = Ac + Bc @ Ri @ Dc.T @ Cc
        Hm = np.block([[Ah, Bc @ Ri @ Bc.T],
                       [-Cc.T @ (np.eye(p) + Dc @ Ri @ Dc.T) @ Cc, -Ah.T]])
        ev = np.linalg.eigvals(Hm)
        return np.any(np.abs(ev.real) < 1e-9 * np.maximum(1.0, np.abs(ev)))

    lo = max(hinf_sweep(A, B, C, n=600)[0], np.linalg.svd(Dc, compute_uv=False)[0] if Dc.size else 0.0)
    hi = 2 * lo + 1e-12
    while has_imag(hi):
        hi *= 2
    while (hi - lo) > rtol * hi:
        mid = 0.5 * (lo + hi)
        if has_imag(mid):
            lo = mid
        else:
            hi = mid
    return hi


def first_crossing(Acl_of_alpha, hi=1e4, tol=1e-5, a0=1e-3, step=1.25):
    """Smallest alpha > 0 at which the spectral radius of Acl_of_alpha(alpha) reaches 1 (inf if none up to hi).
    Geometric bracketing from a0 (ratio `step`, so a crossing narrower than that ratio could be missed), then
    bisection inside the first bracket."""
    rho = lambda a: np.max(np.abs(np.linalg.eigvals(Acl_of_alpha(a))))
    lo, a = 0.0, a0
    while a <= hi:
        if rho(a) >= 1:
            break
        lo, a = a, a * step
    else:
        return np.inf
    hi = a
    while hi - lo > tol * hi:
        mid = 0.5 * (lo + hi)
        if rho(mid) >= 1:
            hi = mid
        else:
            lo = mid
    return hi
