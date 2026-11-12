"""Committed Crazyflie 2.1+ parameter set (entry 01, decision Q5 in decisions.md).

Every value carries its source. Sources (verified, see 99_audit.md section E):
  FW  = Bitcraze crazyflie-firmware, src/platform/interface/platform_defaults_cf2.h (2.1+ propellers)
  FO  = J. Foerster, "System identification of the Crazyflie 2.0 nano quadrocopter",
        BSc thesis, ETH Zurich, 2015, DOI 10.3929/ethz-b-000214143, Appendix A
  RP  = RotorPy rotorpy/vehicles/crazyflie_params.py (robustness check only)
"""
import numpy as np

g = 9.81                       # m/s^2
m = 0.029                      # kg, FW CF_MASS                       (to be weighed on arrival)
arm = 0.046                    # m,  FW ARM_LENGTH (centre to rotor)
a = arm / np.sqrt(2)           # m,  effective moment arm, entry 01
f_max = 0.12                   # N,  FW THRUST_MAX, 2.1+ props         (bench re-identification)
f_min = 0.0                    # N,  assumed                           (bench re-identification)
c_tf = 0.0069928948992470565   # m,  FW THRUST2TORQUE, 2.1+ props

# FO Appendix A, full inertia matrix (Crazyflie 2.0). Plant uses the full matrix, MPC the diagonal.
I_full = np.array([[16.571710, 0.830806, 0.718277],
                   [0.830806, 16.655602, 1.800197],
                   [0.718277, 1.800197, 29.261652]]) * 1e-6   # kg m^2
I_diag = np.diag(np.diag(I_full))

# FO eq. 3.16: command -> thrust, H(z) = b / (1 - p z^-1), sampled at 500 Hz -> first-order lag on thrust.
Ts_id = 1 / 500
tau_m = -Ts_id / np.log(0.9695404)      # s, ~0.0647 (FO eq. 3.15 series gives ~0.0775)
tau_m_alt = -Ts_id / np.log(0.9745210)

# FO drag (Crazyflie 2.0): F = K_aero * (sum of rotor speeds) * v, K_aero = diag(-9.1785, -9.1785, -10.311) e-7 kg/rad
K_aero = np.array([9.1785e-7, 9.1785e-7, 10.311e-7])


def foerster_hover_rotor_speed(f_rotor):
    """Rotor speed for a given per-rotor thrust via FO's cmd->thrust and cmd->speed maps (Crazyflie 2.0)."""
    c2, c1, c0 = 2.130295e-11, 1.032633e-6, 5.484560e-4 - f_rotor
    cmd = (-c1 + np.sqrt(c1**2 - 4 * c2 * c0)) / (2 * c2)
    return 0.04076521 * cmd + 380.8359


def derived():
    d = {}
    d["TWR"] = 4 * f_max / (m * g)
    d["c_tf/a"] = c_tf / a
    d["Izz/Ixx"] = I_diag[2, 2] / I_diag[0, 0]
    d["Izz/(Ixx+Iyy)"] = I_diag[2, 2] / (I_diag[0, 0] + I_diag[1, 1])
    d["alpha_max [1/s^2] = a m g / Ixx"] = a * m * g / I_diag[0, 0]
    d["t_att [s]"] = 1 / np.sqrt(d["alpha_max [1/s^2] = a m g / Ixx"])
    d["tau_m [s]"] = tau_m
    d["tau_m/t_att"] = tau_m / d["t_att [s]"]
    d["tau_x,max hover [N m]"] = a * min(4 * f_max - m * g, m * g - 4 * f_min)
    d["tau_x,max if floor-limited [N m]"] = a * m * g
    d["tau_z,max hover [N m]"] = c_tf * min(4 * f_max - m * g, m * g - 4 * f_min)
    d["yaw/roll angular-accel ratio"] = (d["tau_z,max hover [N m]"] / I_diag[2, 2]) / (d["tau_x,max hover [N m]"] / I_diag[0, 0])
    d["a_up,max [m/s^2]"] = g * (d["TWR"] - 1)
    d["max coordinated tilt [deg]"] = np.degrees(np.arccos(1 / d["TWR"]))
    d["max off-diag/diag inertia"] = np.max(np.abs(I_full - I_diag) / np.diag(I_full)[:, None])
    w_h = foerster_hover_rotor_speed(m * g / 4)
    d["hover rotor speed (FO 2.0 map) [rad/s]"] = w_h
    d["d_xy ~ K_aero*4*w_hover [N s/m]"] = K_aero[0] * 4 * w_h
    d["drag trim tilt per m/s [rad]"] = d["d_xy ~ K_aero*4*w_hover [N s/m]"] / (m * g)
    return d


if __name__ == "__main__":
    for k, v in derived().items():
        print(f"{k:42s} {v:.4g}")
