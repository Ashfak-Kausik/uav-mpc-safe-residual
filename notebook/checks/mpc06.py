"""Entry 06: the nominal MPC for tracking (design constants, condensed QP, closed-loop analysis).

Shared by check_06_nominal_mpc.py and later entries (08-10). All parameters come from params.py
and linmodel.py; the design choices are justified in 06_nominal_mpc.md.
"""
import numpy as np
from scipy.linalg import expm
import params as P
from linmodel import NX, NU, NY, MIX, C, G, f_hover, continuous, zoh, lqr, tracking_law

# ------------------------------------------------------------------ design constants (entry 06)
DT = 0.010                        # control period, entry 05
N = 20                            # horizon steps: 200 ms = 4.7 t_att = 3.1 tau_m
# Bryson-rule weights: 1 / (acceptable deviation)^2
DEV = dict(p=0.10, v=0.50, tilt=0.35, yaw=0.50, w=2.0, wz=2.0)
HEADROOM = min(P.f_max - f_hover, f_hover - P.f_min)   # rotor thrust headroom about hover [N]
Q = np.diag(np.r_[[1 / DEV["p"]**2] * 3, [1 / DEV["v"]**2] * 3, [1 / DEV["tilt"]**2] * 2, 1 / DEV["yaw"]**2,
                  [1 / DEV["w"]**2] * 2, 1 / DEV["wz"]**2, [0.0] * 4])
R = np.eye(NU) / HEADROOM**2
U_MIN, U_MAX = P.f_min - f_hover, P.f_max - f_hover     # box on commanded-thrust deviations
DU_MAX = 0.25 * (P.f_max - P.f_min)                      # per-step command change limit
TILT_MAX = 0.35                                          # soft tilt limit [rad] (20 deg)
Z_MIN = 0.0                                              # soft altitude floor [m]
RHO1, RHO2 = 1e4, 1e2                                    # exact (L1) and quadratic slack penalties

A_c, B_c = continuous()
Ad, Bd = zoh(A_c, B_c, DT)
Pm, K = lqr(Ad, Bd, Q, R)
T_OFF = 100.0 * (G.T @ Pm @ G)                           # offset cost on the artificial steady state
Kx, Kr, S_th = tracking_law(Pm, K, T_OFF)
A_LAG = np.exp(-DT / P.tau_m)                            # nominal lag pole per period


# ------------------------------------------------------------------ condensed QP
def prediction_matrices(Ad_=Ad, Bd_=Bd, n=N):
    """X_all (k = 0..n) = Sx x0 + Su U."""
    Sx = np.zeros(((n + 1) * NX, NX))
    Su = np.zeros(((n + 1) * NX, n * NU))
    Ak = np.eye(NX)
    for k in range(n + 1):
        Sx[k * NX:(k + 1) * NX] = Ak
        Ak = Ad_ @ Ak
    for k in range(1, n + 1):
        for j in range(k):
            Su[k * NX:(k + 1) * NX, j * NU:(j + 1) * NU] = np.linalg.matrix_power(Ad_, k - 1 - j) @ Bd_
    return Sx, Su


class CondensedQP:
    """Decision z = [U (4N), theta (4), s (2)]; s = (s_tilt, s_alt): ONE slack per soft-constraint family,
    shared over the horizon (infinity-norm softening, exact penalty for RHO1 large enough).
    minimise 1/2 z'Hz + f'z  s.t.  bl <= Acon z <= bu  (first n_z rows of the bounds are simple bounds)."""

    def __init__(self, n=N):
        self.n = n
        Sx, Su = prediction_matrices(n=n)
        self.Sx, self.Su = Sx, Su
        nU, nth, ns = NU * n, NY, 2
        self.nU, self.nth, self.ns = nU, nth, ns
        self.nz = nU + nth + ns
        Qbar = np.kron(np.eye(n + 1), Q)
        Qbar[-NX:, -NX:] = Pm
        Gall = np.kron(np.ones((n + 1, 1)), G)
        M = np.hstack([Su, -Gall])                      # E = M w + Sx x0, w = (U, theta)
        Hw = 2 * (M.T @ Qbar @ M)
        Hw[:nU, :nU] += 2 * np.kron(np.eye(n), R)
        Hw[nU:, nU:] += 2 * T_OFF
        H = np.zeros((self.nz, self.nz))
        H[:nU + nth, :nU + nth] = Hw
        H[nU + nth:, nU + nth:] = 2 * RHO2 * np.eye(ns)
        self.H = 0.5 * (H + H.T)
        self.M, self.Qbar = M, Qbar
        # general constraints
        rows = []
        # rate: u_0 - u_prev, u_k - u_{k-1}
        Drate = np.zeros((nU, self.nz))
        for k in range(n):
            Drate[k * NU:(k + 1) * NU, k * NU:(k + 1) * NU] = np.eye(NU)
            if k > 0:
                Drate[k * NU:(k + 1) * NU, (k - 1) * NU:k * NU] = -np.eye(NU)
        self.Drate = Drate
        # soft tilt and altitude, k = 1..n: phi_k - s <= TILT, -phi_k - s <= TILT, same for theta; z_k + s >= Z_MIN
        Tilt = np.zeros((4 * n, self.nz))
        Alt = np.zeros((n, self.nz))
        self.tilt_x0 = np.zeros((4 * n, NX))
        self.alt_x0 = np.zeros((n, NX))
        for k in range(1, n + 1):
            r0 = (k - 1) * 4
            for j, idx in enumerate((6, 7)):            # phi, theta state indices
                row_u = Su[k * NX + idx, :]
                row_x = Sx[k * NX + idx, :]
                sidx = nU + nth + 0
                Tilt[r0 + 2 * j, :nU] = row_u;  Tilt[r0 + 2 * j, sidx] = -1;  self.tilt_x0[r0 + 2 * j] = row_x
                Tilt[r0 + 2 * j + 1, :nU] = -row_u; Tilt[r0 + 2 * j + 1, sidx] = -1; self.tilt_x0[r0 + 2 * j + 1] = -row_x
            Alt[k - 1, :nU] = Su[k * NX + 2, :]
            Alt[k - 1, nU + nth + 1] = 1
            self.alt_x0[k - 1] = Sx[k * NX + 2, :]
        self.Acon = np.vstack([Drate, Tilt, Alt])
        self.n_rate, self.n_tilt, self.n_alt = nU, 4 * n, n

    def vectors(self, x0, u_prev, r):
        nU, nth = self.nU, self.nth
        fw = 2 * (self.M.T @ self.Qbar @ (self.Sx @ x0))
        fw[nU:] += -2 * T_OFF @ r
        f = np.r_[fw, RHO1 * np.ones(self.ns)]
        lb_simple = np.r_[np.full(nU, U_MIN), np.full(nth, -1e30), np.zeros(self.ns)]
        ub_simple = np.r_[np.full(nU, U_MAX), np.full(nth, 1e30), np.full(self.ns, 1e30)]
        rate_l = np.full(nU, -DU_MAX); rate_u = np.full(nU, DU_MAX)
        rate_l[:NU] += u_prev; rate_u[:NU] += u_prev
        tilt_u = TILT_MAX - self.tilt_x0 @ x0
        tilt_l = np.full(self.n_tilt, -1e30)
        alt_l = Z_MIN - self.alt_x0 @ x0
        alt_u = np.full(self.n_alt, 1e30)
        bl = np.r_[lb_simple, rate_l, tilt_l, alt_l]
        bu = np.r_[ub_simple, rate_u, tilt_u, alt_u]
        return f, bl, bu


def solve_daqp(qp, x0, u_prev, r):
    import daqp
    f, bl, bu = qp.vectors(x0, u_prev, r)
    z, fval, flag, info = daqp.solve(qp.H, f, qp.Acon, bu, bl)
    return np.asarray(z), flag, info


class OSQPSolver:
    def __init__(self, qp):
        import osqp
        import scipy.sparse as sp
        self.qp = qp
        A_full = sp.vstack([sp.eye(qp.nz), sp.csc_matrix(qp.Acon)]).tocsc()
        f, bl, bu = qp.vectors(np.zeros(NX), np.zeros(NU), np.zeros(NY))
        self.s = osqp.OSQP()
        self.s.setup(P=sp.triu(sp.csc_matrix(qp.H)).tocsc(), q=f, A=A_full, l=bl, u=bu,
                     verbose=False, eps_abs=1e-7, eps_rel=1e-7, polishing=True, max_iter=20000)

    def solve(self, x0, u_prev, r):
        f, bl, bu = self.qp.vectors(x0, u_prev, r)
        self.s.update(q=f, l=bl, u=bu)
        res = self.s.solve()
        return res.x, res.info.status, res.info


# ------------------------------------------------------------------ closed-loop analysis (unconstrained law)
def plant_matrices(tau_true=P.tau_m, I=P.I_diag):
    A, B = continuous(tau=tau_true, I=I)
    return A, B


def frac_delay_gammas(A, B, D):
    """Delay D = (d + eps) DT: x+ = Phi x + G0 u_{k-d} + G1 u_{k-d-1}."""
    d = int(np.floor(D / DT + 1e-12))
    eps = D / DT - d
    Phi, Gam = zoh(A, B, DT)
    if eps < 1e-12:
        return Phi, Gam, np.zeros_like(Gam), d
    _, Ga = zoh(A, B, (1 - eps) * DT)
    _, Gb = zoh(A, B, eps * DT)
    return Phi, Ga, expm(A * (1 - eps) * DT) @ Gb, d


def closed_loop(tau_true=P.tau_m, D=0.0, I=P.I_diag):
    """Linear closed loop: plant (true lag, delay D) + MPC law u = Kx [x_rigid; f_hat] + Kr r, where
    f_hat is the controller's open-loop lag model driven by its own (undelayed) commands (thrust is not
    measured on board). State: [x (16), f_hat (4), u_{k-1} ... u_{k-d-1} (4(d+1))]. Returns (Acl, Bcl_r)."""
    A, B = plant_matrices(tau_true, I)
    Phi, G0, G1, d = frac_delay_gammas(A, B, D)
    nb = d + 1
    n = NX + NU + NU * nb
    Kxr, Kxf = Kx[:, :12], Kx[:, 12:16]

    def u_of(state_row_block):
        pass

    # u_k as a linear function of the closed-loop state z = [x, fhat, buf]: u_k = Fz z + Kr r
    Fz = np.zeros((NU, n))
    Fz[:, :12] = Kxr
    Fz[:, NX:NX + NU] = Kxf
    Acl = np.zeros((n, n))
    Bcl = np.zeros((n, NY))
    # plant: x+ = Phi x + G0 u_{k-d} + G1 u_{k-d-1}; buffer slot i holds u_{k-1-i}
    def u_delayed(lag):
        """row operator for u_{k-lag}: lag = 0 -> current u (function of state), lag >= 1 -> buffer slot lag-1."""
        if lag == 0:
            return Fz, Kr
        Z = np.zeros((NU, n))
        Z[:, NX + NU + NU * (lag - 1):NX + NU + NU * lag] = np.eye(NU)
        return Z, np.zeros((NU, NY))
    Ua, Ra = u_delayed(d)
    Ub, Rb = u_delayed(d + 1)
    Acl[:NX] = np.hstack([Phi, np.zeros((NX, n - NX))]) + G0 @ Ua + G1 @ Ub
    Bcl[:NX] = G0 @ Ra + G1 @ Rb
    # controller lag model: fhat+ = a fhat + (1 - a) u_k
    Acl[NX:NX + NU, NX:NX + NU] = A_LAG * np.eye(NU)
    Acl[NX:NX + NU] += (1 - A_LAG) * Fz
    Bcl[NX:NX + NU] = (1 - A_LAG) * Kr
    # buffer shift: slot0+ = u_k, slot i+ = slot i-1
    Acl[NX + NU:NX + 2 * NU] = Fz
    Bcl[NX + NU:NX + 2 * NU] = Kr
    for i in range(1, nb):
        Acl[NX + NU + NU * i:NX + NU + NU * (i + 1), NX + NU + NU * (i - 1):NX + NU + NU * i] = np.eye(NU)
    return Acl, Bcl


def spectral_radius(Acl):
    return float(np.max(np.abs(np.linalg.eigvals(Acl))))


def loop_at_plant_input(w, tau_true=P.tau_m):
    """Loop gain L(e^{jw dt}) broken at the plant input (commanded thrusts), with the controller's
    internal lag model: u = (I - Kxf F(z))^{-1} Kxr x_rigid, F(z) = (1 - a)/(z - a). Returned in
    wrench coordinates (T, tau_x, tau_y, tau_z): L_w = MIX L MIX^{-1}; negative-feedback convention."""
    A, B = plant_matrices(tau_true)
    Phi, Gam = zoh(A, B, DT)
    z = np.exp(1j * w * DT)
    X = np.linalg.solve(z * np.eye(NX) - Phi, Gam)           # x per unit command
    F = (1 - A_LAG) / (z - A_LAG) * np.eye(NU)
    Cz = np.linalg.solve(np.eye(NU) - Kx[:, 12:16] @ F, Kx[:, :12])
    L = -(Cz @ X[:12])
    return MIX @ L @ np.linalg.inv(MIX)


def channel_margins(n_freq=40000):
    ws = np.logspace(-1, np.log10(np.pi / DT * 0.9999), n_freq)
    Ls = np.array([np.diag(loop_at_plant_input(w)) for w in ws])   # (n_freq, 4)
    out = {}
    for ch, name in enumerate(["collective", "roll", "pitch", "yaw"]):
        L = Ls[:, ch]
        mag, ph = np.abs(L), np.unwrap(np.angle(L))
        cross = np.where(np.diff(np.sign(mag - 1)) != 0)[0]
        pms = [(ws[i], (np.degrees(np.angle(L[i])) + 180) % 360) for i in cross]
        # phase crossovers (angle = -180 mod 360) where |L| < 1 -> gain margin
        ang = np.angle(L)
        pc = np.where((np.abs(np.diff(np.sign(ang))) > 0) & (np.abs(ang[:-1]) > np.pi / 2))[0]
        gms = [1 / mag[i] for i in pc if mag[i] < 1]
        wc, pm = min(pms, key=lambda t: t[1]) if pms else (np.nan, np.nan)
        out[name] = dict(wc=wc, pm=pm, gm=min(gms) if gms else np.inf, n_cross=len(pms))
    return out


def cliff(param, lo, hi, tol=1e-6):
    """Bisection on spectral radius = 1 for closed_loop(**{param: value})."""
    f = lambda v: spectral_radius(closed_loop(**{param: v})[0]) - 1
    assert f(lo) < 0 < f(hi), (f(lo), f(hi))
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) < 0 else (lo, mid)
    return 0.5 * (lo + hi)
