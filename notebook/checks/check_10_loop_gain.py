"""Entry 10: the loop-gain bound. Every number quoted in 10_loop_gain.md is computed here.
(Runtime: a few minutes; the exact stability boundaries of the averaging estimator need large eigenvalue solves.)"""
import itertools
import numpy as np
import params as P
from quad_sym import Report
from linmodel import NX, NU
from mpc06 import DT, DEV, HEADROOM
from ref08 import oracle
from loop10 import lti, hinf_sweep, hinf_certified, freq_resp, first_crossing, S_DELTA, DEV_RIGID

rep = Report("check_10_loop_gain")
rng = np.random.default_rng(10)
EPS = 0.1

# ---------------------------------------------------------------- 1. certified H-infinity norm
A, B, C = lti()
gs, wpk = hinf_sweep(A, B, C)
gc = hinf_certified(A, B, C)
rep.check("certified H-inf (Hamiltonian bisection) agrees with the frequency-sweep lower bound: sweep <= certified <= sweep (1 + 1e-5)",
          gs <= gc * (1 + 1e-9) and gc <= gs * (1 + 1e-5), f"{gs:.6f} vs {gc:.6f}")
At, Bt, Ct = lti(read="tilt")
gt = hinf_certified(At, Bt, Ct)
rep.check("nominal gain from a scaled correction to the read signal = 1.000 (position tracking); to tilt alone 0.504 = 1.76 rad/m (entry 07: 1.75)",
          abs(gc - 1.0) < 1e-3 and abs(gt - 0.504) < 1e-3 and abs(gt * DEV_RIGID[6] / S_DELTA[0] - 1.76) < 0.01)
# window stacking: ||[G; z^-1 G; ...; z^-L G]|| = sqrt(L+1) ||G||  (delays do not change magnitudes)
n = A.shape[0]
L = 2
Aa = np.zeros((n + 2 * 16, n + 2 * 16)); Aa[:n, :n] = A
Aa[n:n + 16, :n] = C; Aa[n + 16:, n:n + 16] = np.eye(16)
Ba = np.r_[B, np.zeros((32, 4))]
Ca = np.zeros((48, n + 32)); Ca[:16, :n] = C; Ca[16:32, n:n + 16] = np.eye(16); Ca[32:, n + 16:] = np.eye(16)
gw = hinf_certified(Aa, Ba, Ca)
rep.check("a window of L+1 samples multiplies the gain by exactly sqrt(L+1) (L = 2: sqrt 3)", abs(gw / gc - np.sqrt(3)) < 1e-4, f"{gw/gc:.6f}")

# ---------------------------------------------------------------- 2. gain over the admissible set
singles = ([dict(mass_ratio=s) for s in (0.8, 0.9, 1.1, 1.2, 1.3)] + [dict(drag_scale=s) for s in (1, 2, 4, 8, 16)]
           + [dict(tau_ratio=s) for s in (0.3, 0.5, 0.7, 1.3, 1.5, 2.0)] + [dict(D=k * DT) for k in (0.5, 1, 1.5, 2)])
gam = {}
for mm in singles:
    gam[tuple(mm.items())[0]] = hinf_certified(*lti(**mm))
combos = [c for c in itertools.product([1.0, 1.2], [1, 4], [0.5, 1.0, 1.5], [0, 1, 2]) if not (c[2] == 0.5 and c[3] == 2)]
cg = {c: hinf_certified(*lti(mass_ratio=c[0], drag_scale=c[1], tau_ratio=c[2], D=c[3] * DT)) for c in combos}
worst = max(cg, key=cg.get)
print("   certified gains:", {f"{k[0]}={k[1]}": round(float(v), 3) for k, v in gam.items()})
print(f"   worst of 32 combinations: (mass, drag, lag, delay/dt) = {worst}: {cg[worst]:.3f}")
rep.check("mass and drag change the gain little (1.00-1.10); lag error and delay raise it toward their cliffs "
          "(lag x0.3: 4.90, x2: 2.62; delay 20 ms: 6.28)",
          all(1.0 <= gam[("mass_ratio", s)] <= 1.10 for s in (0.8, 0.9, 1.1, 1.2, 1.3))
          and all(1.0 <= gam[("drag_scale", s)] <= 1.02 for s in (1, 2, 4, 8, 16))
          and abs(gam[("tau_ratio", 0.3)] - 4.905) < 0.01 and abs(gam[("tau_ratio", 2.0)] - 2.622) < 0.01
          and abs(gam[("D", 0.02)] - 6.284) < 0.01)
rep.check("sup over the admissible set: 13.41, at drag x4 + lag x1.5 + 2 dt of delay", worst == (1.0, 4, 1.5, 2) and abs(cg[worst] - 13.41) < 0.01)
GBAR = cg[worst]

# ---------------------------------------------------------------- 3. validity and conservatism of the small-gain bound
cons = {}
for name, mm in (("nominal", dict()), ("delay 20 ms", dict(D=0.02)), ("worst", dict(drag_scale=4, tau_ratio=1.5, D=0.02))):
    A, B, C = lti(**mm)
    g = hinf_certified(A, B, C)
    _, wp = hinf_sweep(A, B, C)
    U_, s_, Vh = np.linalg.svd(freq_resp(A, B, C, wp))
    a_rank1 = np.inf
    for ph in np.linspace(0, np.pi, 24):
        u = np.real(np.exp(1j * ph) * U_[:, 0]); v = np.real(np.exp(1j * ph) * Vh[0].conj())
        if min(np.linalg.norm(u), np.linalg.norm(v)) < 1e-9:
            continue
        for sg in (1, -1):
            K = sg * np.outer(v, u) / (np.linalg.norm(u) * np.linalg.norm(v))
            a_rank1 = min(a_rank1, first_crossing(lambda al: A + al * B @ K @ C, hi=200))
    Kt = np.zeros((4, 16)); Kt[0, 7] = 1; Kt[1, 6] = -1; Kt /= np.linalg.norm(Kt, 2)
    a_tilt = min(first_crossing(lambda al: A + al * B @ (s * Kt) @ C, hi=200) for s in (1, -1))
    a_rand = []
    for _ in range(30):
        K = rng.normal(size=(4, 16)); K /= np.linalg.norm(K, 2)
        a_rand.append(first_crossing(lambda al: A + al * B @ K @ C, hi=200))
    cons[name] = (g, a_rank1, a_tilt, min(a_rand), np.median(a_rand))
    print(f"   {name:12s} 1/gamma {1/g:.3f}; exact boundary: worst rank-one {a_rank1:.3f}, tilt {a_tilt:.3f}, random min {min(a_rand):.3f} median {np.median(a_rand):.3f}")
rep.check("small-gain is valid: every exact stability boundary for unit-norm linear corrections lies at or above 1/gamma",
          all(min(v[1:4]) >= (1 / v[0]) * (1 - 1e-4) for v in cons.values()))
rep.check("and not loose for the worst direction: worst rank-one boundary / (1/gamma) = 1.13 (nominal), 2.9 (delay 20 ms), 1.9 (worst set point)",
          abs(cons["nominal"][1] * cons["nominal"][0] - 1.13) < 0.02 and abs(cons["delay 20 ms"][1] * cons["delay 20 ms"][0] - 2.92) < 0.05
          and abs(cons["worst"][1] * cons["worst"][0] - 1.92) < 0.05)

# ---------------------------------------------------------------- 4. certified Lipschitz constant of a network
def mlp(sizes):
    return [rng.normal(size=(o, i)) / np.sqrt(i) for i, o in zip(sizes[:-1], sizes[1:])]


def jac(Ws, x):
    J = np.eye(len(x)); h = x
    for i, W in enumerate(Ws):
        pre = W @ h
        if i < len(Ws) - 1:
            J = ((1 - np.tanh(pre) ** 2)[:, None] * W) @ J
            h = np.tanh(pre)
        else:
            J = W @ J
    return J


ratios, norm_ok = [], True
for _ in range(3):
    Ws = mlp([176, 64, 64, 4])
    cert = np.prod([np.linalg.norm(W, 2) for W in Ws])
    emp = max(np.linalg.norm(jac(Ws, rng.normal(size=176) * s), 2) for s in (0.1, 1, 3) for _ in range(200))
    ratios.append(cert / emp)
    Wn = [W / np.linalg.norm(W, 2) * 0.5 ** (1 / 3) for W in Ws]
    norm_ok &= abs(np.prod([np.linalg.norm(W, 2) for W in Wn]) - 0.5) < 1e-12
print("   product-of-spectral-norms bound / sampled lower bound, three random 176-64-64-4 tanh networks:", [round(float(r), 2) for r in ratios])
rep.check("certified bound (product of layer spectral norms, 1-Lipschitz tanh) is >= every sampled Jacobian norm, 2.5-3.1x above the largest",
          all(2.4 <= r <= 3.2 for r in ratios))
rep.check("spectral normalisation sets the certified constant exactly (layers scaled to 0.5^(1/3): bound = 0.5)", norm_ok)
# ensemble mean and saturation
Ws1, Ws2 = mlp([8, 16, 4]), mlp([8, 16, 4])
L1 = np.prod([np.linalg.norm(W, 2) for W in Ws1]); L2 = np.prod([np.linalg.norm(W, 2) for W in Ws2])
f = lambda W, x: W[1] @ np.tanh(W[0] @ x)
sat = lambda v, R=1.0: v if np.linalg.norm(v) <= R else v * R / np.linalg.norm(v)
ok = True
for _ in range(2000):
    a, b = rng.normal(size=8) * 3, rng.normal(size=8) * 3
    m = lambda x: sat(0.5 * (f(Ws1, x) + f(Ws2, x)))
    ok &= np.linalg.norm(m(a) - m(b)) <= 0.5 * (L1 + L2) * np.linalg.norm(a - b) + 1e-12
rep.check("ensemble mean followed by saturation onto a ball: Lipschitz <= mean of the members' constants (2000 random pairs)", ok)

# ---------------------------------------------------------------- 5. is the bound vacuous?  A realistic estimator
hold = np.zeros((6, 700, 4))
_, y0, *_ = oracle(hold, mass_ratio=1.2)
kappa = -y0[250:400, 2].mean() / abs(P.g * (1 / 1.2 - 1))
rep.check("kappa (steady altitude correction per unit vertical acceleration bias) = 0.0643 m per m/s^2", abs(kappa - 0.0643) < 2e-4)


def estimator_system(Lw, **mm):
    A, B, C = lti(**mm)
    n = A.shape[0]
    na = n + 5 * Lw
    Aa = np.zeros((na, na)); Aa[:n, :n] = A
    Ba = np.zeros((na, 1)); Ba[:n, 0] = B[:, 2]
    Aa[n, 5] = 1.0
    for i in range(1, Lw):
        Aa[n + i, n + i - 1] = 1.0
    base = n + Lw
    Aa[base:base + 4, NX:NX + NU] = np.eye(4)
    for i in range(1, Lw):
        Aa[base + 4 * i:base + 4 * (i + 1), base + 4 * (i - 1):base + 4 * i] = np.eye(4)
    K = np.zeros(na)
    K[5] += 1 / (Lw * DT); K[n + Lw - 1] -= 1 / (Lw * DT)
    for i in range(Lw):
        K[base + 4 * i:base + 4 * (i + 1)] -= 1 / (Lw * P.m)
    return A, B, Aa, Ba, kappa * K / S_DELTA[2]


table = {}
for Lw in (10, 30, 100):
    coef = np.r_[np.array([1, -1]) * DEV["v"] / (Lw * DT), -np.ones(4 * Lw) * HEADROOM / (Lw * P.m)] * kappa / S_DELTA[2]
    Lip = np.linalg.norm(coef)
    for name, mm in (("nominal", dict()), ("worst", dict(drag_scale=4, tau_ratio=1.5, D=0.02))):
        A, B, Aa, Ba, K = estimator_system(Lw, **mm)
        n = A.shape[0]
        Cvz = np.zeros((1, n)); Cvz[0, 5] = 1 / DEV["v"]
        Cf = np.zeros((4, n)); Cf[:, NX:NX + NU] = np.eye(4) / HEADROOM
        g_read = hinf_certified(A, B[:, 2:3], np.vstack([np.sqrt(2) * Cvz, np.sqrt(Lw) * Cf]))
        generic = (1 - EPS) / (Lip * g_read)
        structured = (1 - EPS) / hinf_certified(Aa, Ba, K.reshape(1, -1))
        exact = first_crossing(lambda al: Aa + al * np.outer(Ba[:, 0], K), hi=50)
        table[(Lw, name)] = (Lip, generic, structured, exact)
        print(f"   averaging {Lw*DT:.1f} s, {name:7s}: Lipschitz {Lip:6.3f}  generic alpha_sg {generic:6.3f}  structured {structured:7.3f}  exact {exact:7.3f}")
T = table
rep.check("the estimator's certified Lipschitz constant falls as 1/L: 4.60 (0.1 s), 1.57 (0.3 s), 0.50 (1 s)",
          abs(T[(10, "nominal")][0] - 4.595) < 0.01 and abs(T[(30, "nominal")][0] - 1.565) < 0.01 and abs(T[(100, "nominal")][0] - 0.503) < 0.01)
rep.check("GENERIC bound (Lipschitz x window gain) is nearly vacuous: alpha_sg 0.065-0.25 < 0.3 everywhere",
          all(v[1] < 0.3 for v in T.values()) and abs(T[(10, "worst")][1] - 0.065) < 0.003 and abs(T[(100, "nominal")][1] - 0.249) < 0.003)
rep.check("STRUCTURED bound (certified H-inf of the composed loop E G) is valid (<= exact boundary) and allows full trust (>= 1.6) everywhere",
          all(v[2] <= v[3] * (1 + 1e-4) for v in T.values()) and all(v[2] >= 1.6 for v in T.values()))
rep.check("structured bound values: 8.0 / 1.66 (0.1 s), 17.0 / 2.62 (0.3 s), 57.6 / 8.73 (1 s), nominal / worst",
          abs(T[(10, "nominal")][2] - 8.01) < 0.05 and abs(T[(10, "worst")][2] - 1.655) < 0.01
          and abs(T[(30, "nominal")][2] - 17.02) < 0.1 and abs(T[(30, "worst")][2] - 2.62) < 0.02
          and abs(T[(100, "nominal")][2] - 57.6) < 0.3 and abs(T[(100, "worst")][2] - 8.73) < 0.05)
rep.check("exact boundaries: 9.0 / 4.8 (0.1 s), 19.0 / 13.6 (0.3 s), none below 50 / 43.8 (1 s); structured is 11 % (0.1 s) and 10 % (0.3 s) below exact nominally",
          abs(T[(10, "nominal")][3] - 9.00) < 0.05 and abs(T[(10, "worst")][3] - 4.83) < 0.05 and abs(T[(30, "nominal")][3] - 19.0) < 0.1
          and abs(T[(30, "worst")][3] - 13.57) < 0.1 and T[(100, "nominal")][3] == np.inf and abs(T[(100, "worst")][3] - 43.77) < 0.3
          and abs(1 - T[(10, "nominal")][2] / T[(10, "nominal")][3] - 0.11) < 0.005 and abs(1 - T[(30, "nominal")][2] / T[(30, "nominal")][3] - 0.10) < 0.005)

raise SystemExit(rep.finish())
