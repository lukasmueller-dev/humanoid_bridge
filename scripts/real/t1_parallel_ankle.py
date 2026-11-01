"""Numpy kinematics of the Booster T1 parallel ankle, the Python twin of robot_bridge/include/t1_parallel_ankle.hpp.

Copied from loco-mujoco-t1 (branch parallel_act): loco_mujoco/core/utils/parallel_ankle.py (closed-form f and J_A,
numpy only) and parallel_act/ankle_jacobian.py (Newton inverse). q_s = (pitch, roll), q_m = (crank_a, crank_b) in
the convention of the ankle_drive_a/b joints of booster_assets robots/T1/T1_23dof_parallel.xml.
"""

import numpy as np

GEOMETRY = {
    "left": dict(
        p_pitch=np.array([0.0, 0.0, -0.28]),
        p_roll=np.array([0.0, 0.00025, -0.012]),
        anchors=np.array([[-0.0415, 0.0275, 0.0327], [-0.0415, -0.0275, 0.0327]]),
        pivots=np.array([[-0.014, 0.02775, -0.096], [-0.014, -0.02775, -0.156]]),
        tips=np.array([[0.04293593596233938, 0.0, 0.00234635952869361],
                       [0.04259337754140823, 0.0, -0.00589950757394721]]),
        rod_lengths=np.array([0.18, 0.11999999999999984]),
        branch=np.array([-1.0, -1.0]),
    ),
    "right": dict(
        p_pitch=np.array([0.0, 0.0, -0.28]),
        p_roll=np.array([0.0, -0.00025, -0.012]),
        anchors=np.array([[-0.0415, -0.0275, 0.0327], [-0.0415, 0.0275, 0.0327]]),
        pivots=np.array([[-0.014, -0.02775, -0.096], [-0.014, 0.02775, -0.156]]),
        tips=np.array([[0.04293593596233938, 0.0, 0.00234635952869361],
                       [0.04259337754140823, 0.0, -0.00589950757394721]]),
        rod_lengths=np.array([0.18, 0.11999999999999984]),
        branch=np.array([-1.0, -1.0]),
    ),
}


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _crank_terms(side, i, q_s):
    g = GEOMETRY[side]
    qp, qr = q_s
    cp, sp, cr, sr = np.cos(qp), np.sin(qp), np.cos(qr), np.sin(qr)
    a = g["anchors"][i]
    inner = g["p_roll"] + np.array([a[0], cr * a[1] - sr * a[2], sr * a[1] + cr * a[2]])
    b = g["p_pitch"] + np.array([cp * inner[0] + sp * inner[2], inner[1], -sp * inner[0] + cp * inner[2]])
    db_p = np.array([-sp * inner[0] + cp * inner[2], 0.0, -cp * inner[0] - sp * inner[2]])
    v1, v2 = -sr * a[1] - cr * a[2], cr * a[1] - sr * a[2]
    db_r = np.array([sp * v2, v1, cp * v2])

    d = b - g["pivots"][i]
    tx, tz = g["tips"][i][0], g["tips"][i][2]
    u = d[0] * tx + d[2] * tz
    w = d[0] * tz - d[2] * tx
    k = 0.5 * (d @ d + g["tips"][i] @ g["tips"][i] - g["rod_lengths"][i] ** 2)
    r = np.hypot(u, w)
    ratio = k / r
    theta = np.arctan2(w, u) + g["branch"][i] * np.arccos(np.clip(ratio, -1.0, 1.0))
    c, s = np.cos(theta), np.sin(theta)
    F_theta = -u * s + w * c
    F_theta = (1.0 if F_theta >= 0.0 else -1.0) * max(abs(F_theta), 0.02 * r)
    dk = np.array([d @ db_p, d @ db_r])
    du = np.array([tx * db_p[0] + tz * db_p[2], tx * db_r[0] + tz * db_r[2]])
    dw = np.array([tz * db_p[0] - tx * db_p[2], tz * db_r[0] - tx * db_r[2]])
    return theta, -(du * c + dw * s - dk) / F_theta, ratio


def kinematics(q_s, side):
    """Crank angles q_m, J_A = dq_m/dq_s and the loop-closure margin (negative: unreachable) for one ankle."""
    th0, row0, ratio0 = _crank_terms(side, 0, q_s)
    th1, row1, ratio1 = _crank_terms(side, 1, q_s)
    return wrap(np.array([th0, th1])), np.stack([row0, row1]), 1.0 - max(abs(ratio0), abs(ratio1))


def serial_angles(q_m, side, q_s0=(-0.25, 0.0), iters=20, tol=1e-12):
    """f^-1 by Newton with backtracking. Returns (q_s, residual norm [rad])."""
    q_m = np.asarray(q_m, dtype=float)

    def residual(x):
        f, J, margin = kinematics(x, side)
        r = wrap(f - q_m)
        return r, J, margin, np.linalg.norm(r)

    q_s = np.asarray(q_s0, dtype=float).copy()
    r, J, _, norm = residual(q_s)
    for _ in range(iters):
        if norm < tol or abs(np.linalg.det(J)) < 1e-9:
            break
        step = -np.linalg.solve(J, r)
        for _ in range(20):
            r_new, J_new, margin_new, norm_new = residual(q_s + step)
            if margin_new >= 0.0 and norm_new < norm:
                break
            step = 0.5 * step
        else:
            break
        q_s, r, J, norm = q_s + step, r_new, J_new, norm_new
    return q_s, norm
