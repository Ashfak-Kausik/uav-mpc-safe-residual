"""Entry 01 parameter table: committed Crazyflie 2.1+ set (decision Q5) vs the verified sources.

Replaces check_params_sources.py from the audit. The audit's three FAIL findings there
(motor lag comparable to the attitude time scale, non-diagonal inertia, c_tf range) are now
stated facts in entries 00, 01, 03 and 04, and are asserted here as regression checks.
"""
import numpy as np
import params as P
from quad_sym import Report

rep = Report("check_01_params")
D = P.derived()
for k, v in D.items():
    print(f"   {k:42s} {v:.4g}")

# Sources (transcribed in 99_audit.md section E)
c_FO, c_FW, c_FWlegacy, c_RP = 0.005964552, 0.0069928948992470565, 0.007350862856566459, 7.8e-10 / 2.3e-8

rep.check("committed values are the FW 2.1+ constants (m 0.029, arm 0.046, f_max 0.12, f_min 0.012818, c_tf 0.006993)",
          (P.m, P.arm, P.f_max) == (0.029, 0.046, 0.12) and abs(P.c_tf - c_FW) < 1e-15
          and P.f_min == 0.012817578393224994)
rep.check("f_min is 18 % of hover thrust (not negligible)", abs(D["f_min/f_hover"] - 0.18) < 0.005)
rep.check("TWR = 1.69 (< 2: ceiling-limited, entry 04)", abs(D["TWR"] - 1.69) < 0.005 and D["TWR"] < 2)
rep.check("motor time constant from FO eq. 3.16 = 0.065 s (eq. 3.15: 0.077 s)",
          abs(P.tau_m - 0.0647) < 5e-4 and abs(P.tau_m_alt - 0.0775) < 5e-4)
rep.check("motor lag is slower than the attitude time scale: tau_m/t_att = 1.53 (reason for decision Q1)",
          abs(D["tau_m/t_att"] - 1.53) < 0.005 and D["tau_m/t_att"] > 1)
rep.check("FO inertia off-diagonal reaches 11 % of the diagonal (plant full, MPC diagonal; entry 03)",
          abs(D["max off-diag/diag inertia"] - 0.108) < 0.001)
rep.check("Izz = 0.88 (Ixx + Iyy) and 1.77 Ixx (entry 03)",
          abs(D["Izz/(Ixx+Iyy)"] - 0.88) < 0.005 and abs(D["Izz/Ixx"] - 1.77) < 0.005)
rep.check("RotorPy c_tf is 5.7x FO and 4.8x FW (rejected for the main run, decision Q4)",
          abs(c_RP / c_FO - 5.7) < 0.05 and abs(c_RP / c_FW - 4.85) < 0.05)
rep.check("FO, FW and FW-legacy c_tf agree to within ~25 %", max(c_FO, c_FW, c_FWlegacy) / min(c_FO, c_FW, c_FWlegacy) < 1.25)
rep.check("drag at hover ~ 7e-3 N s/m (order of magnitude, FO K_aero)", abs(D["d_xy ~ K_aero*4*w_hover [N s/m]"] - 0.0071) < 0.0005)
# TWR spread across sources straddles the knee at 2 (entry 01)
twr_fo = 4 * 0.15 / (0.028 * P.g)
twr_kit = 4 * 0.18 / (0.029 * P.g)
rep.check("cross-source TWR spread straddles 2 (1.69 ... ~2.5)", D["TWR"] < 2 < twr_kit and twr_fo > 2,
          f"FO 2.0: {twr_fo:.2f}, FW thrust kit: {twr_kit:.2f}")

raise SystemExit(rep.finish())
