"""Entry 05: discretization. Every number quoted in 05_discretization.md is computed here.

Conventions and parameters from entries 00-04 (params.py). Linear models are the hover
linearisation of entry 04 with the motor lag in the nominal model (decision Q1).
"""
import numpy as np
from scipy.linalg import expm
from quad_sym import Report, qmul, _R_np
import params as P

rep = Report("check_05_discretization")
D = P.derived()
g, m, a, c, tau = P.g, P.m, P.a, P.c_tf, P.tau_m
Inom = P.I_diag
t_att = D["t_att [s]"]
DT = 0.010          # chosen control period (entry 05)
H = 0.001           # chosen plant step (entry 05)
Mmix = np.array([[1, 1, 1, 1], [a, a, -a, -a], [-a, a, a, -a], [c, -c, c, -c]])


# ---------------------------------------------------------------- 1. timescale ratios for the candidate periods
print("   Control period candidates:")
for hz in (500, 200, 100, 50, 25):
    dt = 1 / hz
    print(f"     {hz:4d} Hz  dt = {dt*1e3:5.1f} ms   dt/t_att = {dt/t_att:.3f}   dt/tau_m = {dt/tau:.3f}"
          f"   ZOH phase at 1/t_att = {np.degrees(dt/(2*t_att)):.1f} deg")
rep.check("chosen dt = 10 ms: dt/t_att = 0.236 and dt/tau_m = 0.155",
          abs(DT / t_att - 0.236) < 0.001 and abs(DT / tau - 0.155) < 0.001)
rep.check("ZOH phase loss at w = 1/t_att: 6.8 deg at 100 Hz, 13.5 deg at 50 Hz",
          abs(np.degrees(DT / (2 * t_att)) - 6.8) < 0.05 and abs(np.degrees(0.02 / (2 * t_att)) - 13.5) < 0.05)
rep.check("plant step h = 1 ms divides dt (10 substeps) and h/t_att, h/tau_m < 0.025",
          abs(DT / H - round(DT / H)) < 1e-12 and H / t_att < 0.025 and H / tau < 0.025)


# ---------------------------------------------------------------- 2. linear hover models (16-state error, 17-state quaternion)
def lin16():
    """x = [p(3), v(3), th(3) small-angle rotation vector, w(3), f(4) delivered thrust deviations], u = f_cmd deviations."""
    A = np.zeros((16, 16))
    B = np.zeros((16, 4))
    A[0:3, 3:6] = np.eye(3)
    A[3, 7] = g           # xdd = g dtheta (pitch)
    A[4, 6] = -g          # ydd = -g dphi (roll)
    A[5, 12:16] = 1 / m   # zdd = sum df / m
    A[6:9, 9:12] = np.eye(3)
    A[9:12, 12:16] = np.linalg.inv(Inom) @ Mmix[1:4]
    A[12:16, 12:16] = -np.eye(4) / tau
    B[12:16, :] = np.eye(4) / tau
    return A, B


def lin17():
    """x = [p(3), v(3), q0..q3, w(3), f(4)]; quaternion linearised at (1,0,0,0): q0dot = 0, q_{1..3}dot = w/2."""
    A = np.zeros((17, 17))
    B = np.zeros((17, 4))
    A[0:3, 3:6] = np.eye(3)
    A[3, 8] = 2 * g       # vdot_x = 2 g q2
    A[4, 7] = -2 * g      # vdot_y = -2 g q1
    A[5, 13:17] = 1 / m
    A[7:10, 10:13] = 0.5 * np.eye(3)
    A[10:13, 13:17] = np.linalg.inv(Inom) @ Mmix[1:4]
    A[13:17, 13:17] = -np.eye(4) / tau
    B[13:17, :] = np.eye(4) / tau
    return A, B


def zoh(A, B, dt):
    n, k = B.shape
    Mx = np.zeros((n + k, n + k))
    Mx[:n, :n] = A
    Mx[:n, n:] = B
    E = expm(Mx * dt)
    return E[:n, :n], E[:n, n:]


def rk4_lin(A, B, x, u, h):
    f = lambda z: A @ z + B @ u
    k1 = f(x); k2 = f(x + h / 2 * k1); k3 = f(x + h / 2 * k2); k4 = f(x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


A16, B16 = lin16()
A17, B17 = lin17()
Ad, Bd = zoh(A16, B16, DT)
Ad17, Bd17 = zoh(A17, B17, DT)
rep.check("17-state quaternion model: q0 row and column are identically zero at hover (redundant direction)",
          np.all(A17[6] == 0) and np.all(A17[:, 6] == 0))

# ZOH is exact: compare with a fine RK4 integration of the continuous model under a held input
rng = np.random.default_rng(1)
x0 = rng.normal(size=16) * 0.1
u0 = rng.normal(size=4) * 0.01
x = x0.copy()
for _ in range(10000):
    x = rk4_lin(A16, B16, x, u0, DT / 10000)
err_zoh = np.max(np.abs(Ad @ x0 + Bd @ u0 - x))
rep.check("exact ZOH (matrix exponential) matches the continuous solution under held input", err_zoh < 1e-12, f"{err_zoh:.1e}")
# 16- and 17-state models give identical translational predictions (q = th/2 small-angle map)
x17 = np.zeros(17); x17[0:6] = x0[0:6]; x17[6] = 0; x17[7:10] = x0[6:9] / 2; x17[10:17] = x0[9:16]
y16 = Ad @ x0 + Bd @ u0
y17 = Ad17 @ x17 + Bd17 @ u0
rep.check("16-state error model and 17-state quaternion model agree (q_vec = th/2)",
          np.allclose(y16[0:6], y17[0:6], atol=1e-14) and np.allclose(y16[6:9] / 2, y17[7:10], atol=1e-14))
rep.check("ZOH lag mode is exp(-dt/tau_m) = 0.8567 at dt = 10 ms",
          abs(Ad[12, 12] - np.exp(-DT / tau)) < 1e-14 and abs(Ad[12, 12] - 0.8567) < 5e-5)

# ---------------------------------------------------------------- 3. forward Euler: error and where it breaks
lag_err = ((1 - np.exp(-DT / tau)) - DT / tau) / (1 - np.exp(-DT / tau))
print(f"   Euler lag decrement dt/tau = {DT/tau:.4f} vs exact 1-exp(-dt/tau) = {1-np.exp(-DT/tau):.4f} ({-lag_err*100:.1f} % too large)")
rep.check("Euler overstates the per-step lag decrement by 7.9 % at dt = 10 ms", abs(-lag_err - 0.079) < 0.001)

# One-step input effect: exact ZOH reaches every state through the chain, Euler reaches only the lag states.
reach_exact = np.flatnonzero(np.abs(Bd).sum(1) > 0)
reach_euler = np.flatnonzero(np.abs(B16 * DT).sum(1) > 0)
rep.check("one Euler step moves only the 4 lag states; exact ZOH moves the whole chain",
          len(reach_euler) == 4 and len(reach_exact) > 4)


def horizon_error(dt, T=0.5):
    """Relative error of the predicted x-trajectory for a commanded pitch-torque step held over T."""
    n = int(round(T / dt))
    u = 1e-3 * np.array([-1, 1, 1, -1.0])  # pure pitch torque command
    Ade, Bde = zoh(A16, B16, dt)
    Aeu, Beu = np.eye(16) + A16 * dt, B16 * dt
    xe = np.zeros(16); xu = np.zeros(16)
    pe, pu = [], []
    for _ in range(n):
        xe = Ade @ xe + Bde @ u; xu = Aeu @ xu + Beu @ u
        pe.append(xe[0]); pu.append(xu[0])
    pe, pu = np.array(pe), np.array(pu)
    return np.max(np.abs(pe - pu)) / np.max(np.abs(pe))


print("   Forward-Euler horizon error (lateral position, pitch-torque step, T = 0.5 s):")
herr = {}
for ms in (1, 2, 5, 10, 20, 40):
    herr[ms] = horizon_error(ms / 1000)
    print(f"     dt = {ms:2d} ms ({ms/1000/t_att:.3f} t_att): relative error {herr[ms]*100:5.1f} %")
rep.check("Euler horizon error grows ~linearly in dt: 1.5 % at 1 ms, 14 % at 10 ms, 27 % at 20 ms",
          abs(herr[1] - 0.015) < 0.002 and abs(herr[10] - 0.14) < 0.005 and abs(herr[20] - 0.267) < 0.005
          and herr[40] > herr[20] > herr[10] > herr[1])


# Euler stability of a mode lambda: |1 + lambda dt| < 1  <=>  dt < -2 Re(lambda) / |lambda|^2
def dt_crit(lam):
    return -2 * lam.real / abs(lam) ** 2


rep.check("Euler-stable for the lag pole iff dt < 2 tau_m = 129 ms", abs(dt_crit(-1 / tau + 0j) - 2 * tau) < 1e-15 and abs(2 * tau - 0.129) < 0.001)
print("   Euler stability limit for a closed-loop mode w_n, zeta (dt_crit = 2 zeta / w_n):")
for wn_scale in (0.5, 1, 2, 4):
    for zeta in (0.3, 0.7):
        wn = wn_scale / t_att
        lam = -zeta * wn + 1j * wn * np.sqrt(1 - zeta**2)
        print(f"     w_n = {wn_scale}/t_att, zeta = {zeta}: dt_crit = {dt_crit(lam)*1e3:6.1f} ms = {dt_crit(lam)/t_att:.2f} t_att")
# numerical confirmation of the criterion on a 2nd-order mode
wn, zeta = 2 / t_att, 0.3
lam = -zeta * wn + 1j * wn * np.sqrt(1 - zeta**2)
dc = dt_crit(lam)
rep.check("criterion confirmed: Euler of a mode at w_n = 2/t_att, zeta = 0.3 is stable just below dt_crit (12.7 ms) and unstable just above",
          abs(1 + lam * 0.99 * dc) < 1 < abs(1 + lam * 1.01 * dc) and abs(dc - 0.0127) < 0.0001)
# (the closed-loop speeds are fixed in entry 06; the next check records the formula's value at dt = 10 ms)
wn_lim = 2 * 0.7 / DT
rep.check("at dt = 10 ms Euler loses stability for a zeta = 0.7 mode faster than 140 rad/s = 5.9 / t_att",
          abs(wn_lim - 140) < 1e-9 and abs(wn_lim * t_att - 5.93) < 0.01)
rep.check("... and for a zeta = 0.3 mode faster than 60 rad/s = 2.5 / t_att",
          abs(2 * 0.3 / DT - 60) < 1e-9 and abs(2 * 0.3 / DT * t_att - 2.54) < 0.01)
rep.check("lag closes 14.3 % of its gap per period; ~4 periods per t_att, ~6 per tau_m",
          abs((1 - np.exp(-DT / tau)) - 0.143) < 0.001 and int(t_att / DT) == 4 and int(tau / DT) == 6)

# RK4 of the linear model over one dt vs exact ZOH (what a nonlinear MPC with one RK4 step per dt would incur, linear part)
Phi4 = np.eye(16); term = np.eye(16)
for k in range(1, 5):
    term = term @ (A16 * DT) / k
    Phi4 = Phi4 + term
rk4_err = np.linalg.norm(Phi4 - Ad, 2) / np.linalg.norm(Ad - np.eye(16), 2)
rep.check("one RK4 step per dt reproduces the exact transition matrix to < 1e-4 (relative)", rk4_err < 1e-4, f"{rk4_err:.1e}")


# ---------------------------------------------------------------- 4. quaternion integration on the unit sphere
def qexp(w, h):
    """Exact quaternion increment for a constant body rate w over h: exp([0, w h / 2])."""
    th = np.linalg.norm(w) * h
    if th < 1e-14:
        return np.array([1.0, 0, 0, 0])
    n = w / np.linalg.norm(w)
    return np.concatenate([[np.cos(th / 2)], np.sin(th / 2) * n])


w = np.array([3.0, -2.0, 5.0])
q = np.array([1.0, 0, 0, 0])
for _ in range(1000):
    q = np.array(qmul(q, qexp(w, DT)))
# exact answer: rotation about w/|w| by |w| * 10 s
q_true = qexp(w, 1000 * DT)
rep.check("exponential-map update q <- q (x) exp(w dt/2) is exact for constant body rate (1000 steps)",
          min(np.linalg.norm(q - q_true), np.linalg.norm(q + q_true)) < 1e-10)
rep.check("exponential-map update keeps |q| = 1 to machine precision", abs(np.linalg.norm(q) - 1) < 1e-13)

# RK4 on the quaternion kinematics WITHOUT renormalisation: norm drift over an MPC horizon
qdot = lambda qq, ww: 0.5 * np.array(qmul(qq, [0, *ww]))
drift = {}
for wmag in (5.0, 10.0, 20.0):
    ww = wmag * np.array([1.0, 1.0, 0.0]) / np.sqrt(2)
    q = np.array([1.0, 0, 0, 0])
    for _ in range(50):  # 50 steps of 10 ms = 0.5 s horizon
        k1 = qdot(q, ww); k2 = qdot(q + DT / 2 * k1, ww); k3 = qdot(q + DT / 2 * k2, ww); k4 = qdot(q + DT * k3, ww)
        q = q + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    drift[wmag] = abs(np.linalg.norm(q) - 1)
    print(f"   RK4 quaternion, no renormalisation, 50 x 10 ms at |w| = {wmag:4.1f} rad/s: |q| - 1 = {drift[wmag]:.1e}")
rep.check("RK4 quaternion norm drift over a 0.5 s horizon stays below 1e-5 up to 20 rad/s", max(drift.values()) < 1e-5)


# ---------------------------------------------------------------- 5. nonlinear plant: RK4 + renormalisation, order check
def plant_f(x, u, I=P.I_full):
    p, v, q, w, f = x[0:3], x[3:6], x[6:10], x[10:13], x[13:17]
    T, tx, ty, tz = Mmix @ f
    R = _R_np(q)
    vdot = T / m * R[:, 2] - np.array([0, 0, g])
    qd = 0.5 * np.array(qmul(q, [0, *w]))
    wd = np.linalg.solve(I, np.array([tx, ty, tz]) - np.cross(w, I @ w))
    fd = (u - f) / tau
    return np.concatenate([v, vdot, qd, wd, fd])


def plant_step(x, u, h, renorm=True):
    k1 = plant_f(x, u); k2 = plant_f(x + h / 2 * k1, u); k3 = plant_f(x + h / 2 * k2, u); k4 = plant_f(x + h * k3, u)
    y = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    if renorm:
        y[6:10] /= np.linalg.norm(y[6:10])
    return y


def run(h, T=0.2):
    x = np.zeros(17); x[6] = 1.0; x[10:13] = [2.0, -1.5, 3.0]; x[13:17] = m * g / 4
    u = m * g / 4 + 0.01 * np.array([1.0, -0.5, 0.8, -0.3])
    for _ in range(int(round(T / h))):
        x = plant_step(x, u, h)
    return x


ref = run(1e-5)
e1 = np.linalg.norm(run(4e-3) - ref)
e2 = np.linalg.norm(run(2e-3) - ref)
e3 = np.linalg.norm(run(1e-3) - ref)
print(f"   RK4 + renormalisation, global error at T = 0.2 s: h = 4 ms {e1:.2e}, 2 ms {e2:.2e}, 1 ms {e3:.2e}; ratios {e1/e2:.1f}, {e2/e3:.1f}")
rep.check("plant integrator is 4th order with renormalisation (halving h cuts the error ~16x)",
          12 < e1 / e2 < 20 and 12 < e2 / e3 < 20)
rep.check("plant global error at h = 1 ms over 0.2 s is below 1e-9", e3 < 1e-9, f"{e3:.1e}")

# motor lag inside the plant: exact per-substep update equals the continuous lag for a held command
f0, fc = 0.05, 0.11
f = f0
for _ in range(int(DT / H)):
    f = f + (1 - np.exp(-H / tau)) * (fc - f)
rep.check("plant lag update f <- f + (1 - e^{-h/tau})(f_cmd - f) is exact for a held command",
          abs(f - (fc + (f0 - fc) * np.exp(-DT / tau))) < 1e-15)
# identical first-order lags commute with the (linear) mixer: lag on rotor thrusts == lag on the wrench
fv = np.array([0.05, 0.07, 0.06, 0.08]); fcv = np.array([0.11, 0.02, 0.09, 0.04])
k = 1 - np.exp(-H / tau)
lag_rotor = Mmix @ (fv + k * (fcv - fv))
lag_wrench = Mmix @ fv + k * (Mmix @ fcv - Mmix @ fv)
rep.check("with identical tau per rotor and clipping before the lag, lag on rotors == lag on the wrench",
          np.allclose(lag_rotor, lag_wrench, atol=1e-16))


# ---------------------------------------------------------------- 6. delay: shift register and fractional delay
def delayed_aug(Ad_, Bd_, d):
    n, k = Bd_.shape
    N = n + k * d
    Aa = np.zeros((N, N)); Ba = np.zeros((N, k))
    Aa[:n, :n] = Ad_
    Aa[:n, n + k * (d - 1):] = Bd_          # oldest buffered command drives the plant
    for i in range(d - 1, 0, -1):            # shift register
        Aa[n + k * i:n + k * (i + 1), n + k * (i - 1):n + k * i] = np.eye(k)
    Ba[n:n + k] = np.eye(k)                  # newest command enters slot 0
    return Aa, Ba


d = 3
Aa, Ba = delayed_aug(Ad, Bd, d)
U = rng.normal(size=(20, 4)) * 0.01
xa = np.zeros(16 + 4 * d)
x = np.zeros(16)
ok = True
for kk in range(20):
    xa = Aa @ xa + Ba @ U[kk]
    ud = U[kk - d] if kk - d >= 0 else np.zeros(4)
    x = Ad @ x + Bd @ ud
    ok &= np.allclose(xa[:16], x, atol=1e-15)
rep.check("integer delay d*dt as a shift register of 4d states reproduces the delayed system exactly (d = 3)", ok)


def frac_gamma(A, B, dt, eps):
    """Delay (d + eps) dt, 0 < eps < 1: x+ = Phi x + G0 u_{k-d} + G1 u_{k-d-1}."""
    _, G_a = zoh(A, B, (1 - eps) * dt)            # int_0^{(1-eps)dt} e^{As} ds B
    _, G_b = zoh(A, B, eps * dt)                   # int_0^{eps dt} e^{As} ds B
    G1 = expm(A * (1 - eps) * dt) @ G_b
    return G_a, G1


eps = 0.3
G0, G1 = frac_gamma(A16, B16, DT, eps)
rep.check("fractional-delay gains add up to the ZOH input matrix (G0 + G1 = Bd)", np.allclose(G0 + G1, Bd, atol=1e-15))
# verify against a fine continuous simulation with delay (d + eps) dt, d = 1, piecewise-constant commands
d = 1
ufine = lambda t: U[int(np.floor((t - (d + eps) * DT) / DT + 1e-9))] if t >= (d + eps) * DT - 1e-12 else np.zeros(4)
xc = np.zeros(16); hh = DT / 1000
xs = np.zeros(16); ok = True
for kk in range(12):
    for j in range(1000):
        t = kk * DT + j * hh
        xc = rk4_lin(A16, B16, xc, ufine(t + hh / 2), hh)
    u_d = U[kk - d] if kk - d >= 0 else np.zeros(4)
    u_d1 = U[kk - d - 1] if kk - d - 1 >= 0 else np.zeros(4)
    xs = Ad @ xs + G0 @ u_d + G1 @ u_d1
    ok &= np.allclose(xs, xc, atol=1e-9 * max(1, np.abs(xc).max()))
rep.check("modified-z (fractional) delay model matches a fine continuous simulation, delay 1.3 dt", ok)


# Pade(1,1) phase accuracy for the delay grid, at w = 1/t_att and at w = 2/t_att
def pade_phase(Dl, w):
    return -2 * np.arctan(w * Dl / 2)


grid_dt = [0, 0.5, 1, 1.5, 2, 3, 4, 6, 8]
print("   Delay sweep grid:")
worst = 0
for gd in grid_dt:
    Dl = gd * DT
    perr = abs(pade_phase(Dl, 1 / t_att) - (-Dl / t_att))
    worst = max(worst, np.degrees(perr) if Dl <= 4 * DT else 0)
    print(f"     {gd:3.1f} dt = {Dl*1e3:4.0f} ms = {Dl/t_att:.2f} t_att = {Dl/tau:.2f} tau_m;  "
          f"Pade(1,1) phase error at 1/t_att {np.degrees(perr):.2f} deg")
p11 = {gd: np.degrees(abs(pade_phase(gd * DT, 1 / t_att) + gd * DT / t_att)) for gd in grid_dt}
rep.check("Pade(1,1) phase error at w = 1/t_att: < 0.5 deg up to 2 dt, but 3.6 deg at 4 dt and 21.5 deg at 8 dt (not usable on the full grid)",
          p11[2] < 0.5 and abs(p11[4] - 3.57) < 0.05 and abs(p11[8] - 21.54) < 0.05)


def pade22_phase(Dl, w):
    x = w * Dl
    return -2 * np.arctan2(x / 2, 1 - x**2 / 12)


p22 = max(np.degrees(abs(pade22_phase(gd * DT, 1 / t_att) + gd * DT / t_att)) for gd in grid_dt)
rep.check("Pade(2,2) phase error at w = 1/t_att is at most 1.6 deg over the whole grid (1.53 deg at 8 dt)",
          p22 <= 1.6 and abs(p22 - 1.53) < 0.01, f"{p22:.2f} deg")
rep.check("grid in t_att units: 8 dt = 80 ms = 1.89 t_att", abs(8 * DT / t_att - 1.89) < 0.005)

# ---------------------------------------------------------------- 7. sample-and-hold as delay dt/2
ws = np.linspace(0.1, 0.9, 9) * np.pi / DT  # below Nyquist
H_zoh = (1 - np.exp(-1j * ws * DT)) / (1j * ws * DT)
rep.check("ZOH frequency response = exp(-j w dt/2) * sinc: phase exactly -w dt/2 below Nyquist",
          np.allclose(np.angle(H_zoh), -ws * DT / 2, atol=1e-12))
rep.check("sample-and-hold acts as a delay of dt/2 = 5 ms = 0.12 t_att = 0.077 tau_m",
          abs(DT / 2 / t_att - 0.118) < 0.001 and abs(DT / 2 / tau - 0.077) < 0.001)
mag = abs(H_zoh[0]) if False else abs((1 - np.exp(-1j * (1 / t_att) * DT)) / (1j * (1 / t_att) * DT))
rep.check("ZOH magnitude loss at w = 1/t_att is below 0.25 %", 1 - mag < 0.0025, f"{(1-mag)*100:.3f} %")

raise SystemExit(rep.finish())
