"""Entry 07: the perturbed plant (nonlinear) and its closed loop with the nominal MPC (implementation b).

Actuator chain at the plant step h = 1 ms (entry 05):
  command (clipped to [f_min, f_max] by the MPC) -> computation delay of one period (implementation b,
  modelled by the MPC) -> injected delay D (plant only, resolution h) -> motor lag (thrust form or
  rotor-speed form, time constant tau_ratio * tau_m) -> mixer -> wrench on the rigid body.
Mismatches: mass_ratio (point mass at the centre of mass: inertia unchanged), drag_scale (multiple of the
Foerster rotor-drag model, scaled with the sum of rotor speeds), tau_ratio, lag_form, delay D.
"""
import numpy as np
import params as P
from quad_sym import qmul, _R_np
from linmodel import MIX, f_hover, NX, NU
from mpc06 import DT, Ad, Bd, A_LAG, CondensedQP, solve_daqp

H = 0.001
SUB = int(round(DT / H))
D_HOVER = P.derived()["d_xy ~ K_aero*4*w_hover [N s/m]"]           # N s/m, in-plane, at hover
DZ_OVER_DXY = P.K_aero[2] / P.K_aero[0]


class Plant:
    def __init__(self, mass_ratio=1.0, drag_scale=0.0, tau_ratio=1.0, lag_form="thrust", D=0.0, I=None):
        self.m = P.m * mass_ratio
        self.I = P.I_full if I is None else I
        self.drag_scale = drag_scale
        self.tau = P.tau_m * tau_ratio
        self.lag_form = lag_form
        self.nd = int(round(D / H))
        assert abs(self.nd * H - D) < 1e-12, "injected delay must be a multiple of the plant step"

    def reset(self, p0=(0, 0, 1.0)):
        self.x = np.zeros(13); self.x[0:3] = p0; self.x[6] = 1.0
        self.f = np.full(4, f_hover)                       # delivered thrust (equals f_hover at start)
        self.buf = [np.full(4, f_hover)] * self.nd         # injected-delay buffer (plant only)

    def drag_force(self, R, v, f):
        if self.drag_scale == 0:
            return np.zeros(3)
        speed_ratio = np.sum(np.sqrt(np.maximum(f, 0) / f_hover)) / 4    # sum of rotor speeds / its hover value
        d = self.drag_scale * D_HOVER * speed_ratio
        Dm = np.diag([d, d, d * DZ_OVER_DXY])
        return -R @ Dm @ R.T @ v

    def rigid_f(self, x, f):
        p, v, q, w = x[0:3], x[3:6], x[6:10], x[10:13]
        T, tx, ty, tz = MIX @ f
        R = _R_np(q)
        vdot = (T * R[:, 2] + self.drag_force(R, v, f)) / self.m - np.array([0, 0, P.g])
        qd = 0.5 * np.array(qmul(q, [0, *w]))
        wd = np.linalg.solve(self.I, np.array([tx, ty, tz]) - np.cross(w, self.I @ w))
        return np.concatenate([v, vdot, qd, wd])

    def lag_step(self, fcmd):
        """Exact over one plant step for a held command (entry 05)."""
        k = 1 - np.exp(-H / self.tau)
        if self.lag_form == "thrust":
            self.f = self.f + k * (fcmd - self.f)
        else:   # rotor-speed form: tau dOmega/dt = Omega_cmd - Omega, f = f_hover (Omega/Omega_hover)^2
            w = np.sqrt(self.f / f_hover)
            wc = np.sqrt(np.maximum(fcmd, 0) / f_hover)
            w = w + k * (wc - w)
            self.f = f_hover * w**2

    def step_period(self, fcmd):
        """Advance one control period with the command fcmd entering the injected-delay buffer."""
        for _ in range(SUB):
            if self.nd:
                self.buf = self.buf[1:] + [fcmd]
                applied = self.buf[0]
            else:
                applied = fcmd
            self.lag_step(applied)
            f = self.f
            g_ = lambda s: self.rigid_f(s, f)
            x = self.x
            k1 = g_(x); k2 = g_(x + H / 2 * k1); k3 = g_(x + H / 2 * k2); k4 = g_(x + H * k3)
            x = x + H / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            x[6:10] /= np.linalg.norm(x[6:10])
            self.x = x


def logq(q):
    q = q if q[0] >= 0 else -q
    vn = np.linalg.norm(q[1:])
    return np.zeros(3) if vn < 1e-12 else 2 * np.arctan2(vn, q[0]) * q[1:] / vn


_QP = None


def run_closed_loop(plant, r, steps=300, p0=(0, 0, 1.0), r_fn=None):
    """Nominal MPC, implementation (b): solve from the predicted state x_{k+1|k}, apply the command one period later."""
    global _QP
    if _QP is None:
        _QP = CondensedQP()
    plant.reset(p0)
    fhat = np.zeros(NU); u_prev = np.zeros(NU)
    X, U, flags = [], [], []
    for k in range(steps):
        rk = r if r_fn is None else r_fn(k)
        xe = np.r_[plant.x[0:6], logq(plant.x[6:10]), plant.x[10:13], fhat]
        x0 = Ad @ xe + Bd @ u_prev
        z, flag, _ = solve_daqp(_QP, x0, u_prev, rk)
        u = z[:NU]
        X.append(plant.x.copy()); U.append(u); flags.append(flag)
        plant.step_period(f_hover + u_prev)            # the command in flight is applied this period
        fhat = A_LAG * fhat + (1 - A_LAG) * u_prev
        u_prev = u
    return np.array(X), np.array(U), flags
