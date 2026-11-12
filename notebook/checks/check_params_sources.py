"""Cross-check of Crazyflie 2.x parameter sources and the normalising groups of entry 04.

Values are transcribed from the sources verified in notebook/99_audit.md, section E:
  Foerster 2015 (ETH BSc thesis, DOI 10.3929/ethz-b-000214143), Appendix A
  RotorPy crazyflie_params.py (github.com/spencerfolk/rotorpy)
  Bitcraze firmware platform_defaults_cf2.h
  MuJoCo Menagerie bitcraze_crazyflie_2/cf2.xml
Nothing here is a parameter commitment; the commitment happens in the revised entry 01.
"""
import numpy as np
from quad_sym import Report

rep = Report("check_params_sources")
g = 9.81
src = {
    #            mass   arm    Ixx       Izz       c_tf        f_max
    "Foerster": (0.028, 0.046, 16.5717e-6, 29.2617e-6, 0.005964552, 0.15),
    "RotorPy":  (0.030, 0.043, 1.43e-5,    2.89e-5,    7.8e-10 / 2.3e-8, 2.3e-8 * 2500**2),
    "Firmware": (0.029, 0.046, np.nan,     np.nan,     0.0069928949, 0.12),
    "Menagerie": (0.027, np.nan, 2.3951e-5, 3.2347e-5, np.nan, np.nan),
}
print(f"{'source':10s} {'TWR':>6s} {'c_tf/a':>7s} {'a m g/Ixx [1/s^2]':>18s} {'t_att [s]':>9s} {'Izz/Ixx':>8s}")
for k, (m, l, Ixx, Izz, c, fmax) in src.items():
    a = l / np.sqrt(2)
    twr = 4 * fmax / (m * g)
    acc = a * m * g / Ixx
    t_att = 1 / np.sqrt(acc) if np.isfinite(acc) else np.nan
    print(f"{k:10s} {twr:6.2f} {c / a:7.3f} {acc:18.0f} {t_att:9.4f} {Izz / Ixx:8.2f}")

# Foerster motor transfer function (command -> thrust), sampled at 500 Hz: pole -> time constant
Ts = 1 / 500
for name, pole in [("H (eq. 3.16)", 0.9695404), ("series 1 (eq. 3.15)", 0.9745210)]:
    print(f"   Foerster {name}: pole {pole} -> tau_m = {-Ts / np.log(pole):.4f} s")
tau_m = -Ts / np.log(0.9695404)
m, l, Ixx = 0.028, 0.046, 16.5717e-6
t_att = 1 / np.sqrt((l / np.sqrt(2)) * m * g / Ixx)
print(f"   tau_m / t_att (Foerster) = {tau_m / t_att:.2f}")
rep.check("Foerster motor time constant is small compared with the attitude time scale",
          tau_m / t_att < 0.2, "it is NOT: tau_m ~ 1.5 t_att, so motor lag is part of the real nominal plant")

# Foerster inertia is not diagonal
I = np.array([[16.571710, 0.830806, 0.718277], [0.830806, 16.655602, 1.800197], [0.718277, 1.800197, 29.261652]]) * 1e-6
off = np.max(np.abs(I - np.diag(np.diag(I))) / np.diag(I)[:, None])
rep.check("Foerster inertia is diagonal to within 1 %", off < 0.01, f"largest off-diagonal / diagonal = {off:.3f}")

# Entry 01 c_tf range 0.006-0.025 m vs sources
cs = {k: v[4] for k, v in src.items() if np.isfinite(v[4])}
rep.check("all source c_tf values inside entry-01 range 0.006-0.025 m",
          all(0.006 <= c <= 0.025 for c in cs.values()),
          ", ".join(f"{k}={c:.4f}" for k, c in cs.items()))

# Menagerie actuator range vs hover thrust
print(f"   Menagerie thrust ctrlrange 0..0.35 N -> TWR = {0.35 / (0.027 * g):.2f} (README: limits 'arbitrary')")
raise SystemExit(rep.finish())
