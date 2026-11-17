"""Right-arm IK, a port of psi0 teleop (`real/teleop/robot_control/robot_arm_ik.py` `G1_29_ArmIK`).

Same problem as upstream: end effector 0.05 m along x past `right_wrist_yaw_joint`; cost
50 |dp|^2 + |log3(R R*^T)|^2 + 0.02 |q|^2 + 0.1 |q - q_last|^2 under the URDF limits; ipopt,
50 iterations, tol 1e-6; the solution through the 0.4/0.3/0.2/0.1 weighted moving filter.
Two changes: the forward kinematics is built in CasADi from a G1 URDF, because PyPI's `pin`
has no `pinocchio.casadi` (upstream takes conda-forge's); a failed solve returns None, and the
caller holds the last solution. Frame: `torso_link`, waist at zero (upstream: pelvis, waist
locked at zero; a constant offset). Python 3.8.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import casadi
import numpy as np

ARM_JOINTS = ("right_shoulder_pitch_joint", "right_shoulder_roll_joint",
              "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
              "right_wrist_pitch_joint", "right_wrist_yaw_joint")
ROOT = "torso_link"
EE_OFFSET = 0.05
FILTER_WEIGHTS = (0.4, 0.3, 0.2, 0.1)   # newest first, as upstream's WeightedMovingFilter


def _floats(text):
    return [float(v) for v in (text or "0 0 0").split()]


def _rpy(r, p, y):
    """URDF rpy: Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, q, lib):
    """Rodrigues for a unit axis; `lib` is numpy or casadi."""
    x, y, z = axis
    c, s = lib.cos(q), lib.sin(q)
    t = 1 - c
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def load_chain(urdf):
    """The joints from ROOT to the right wrist: [(origin 4x4, axis, (lower, upper))], in order."""
    tree = ET.parse(urdf).getroot()
    by_child = {j.find("child").get("link"): j for j in tree.findall("joint")}
    chain, link = [], "right_wrist_yaw_link"
    while link != ROOT:
        joint = by_child[link]
        chain.append(joint)
        link = joint.find("parent").get("link")
    chain.reverse()
    names = tuple(j.get("name") for j in chain)
    if names != ARM_JOINTS:
        raise ValueError(f"{urdf}: chain {ROOT} -> wrist is {names}, expected {ARM_JOINTS}")
    out = []
    for joint in chain:
        origin = joint.find("origin")
        placement = np.eye(4)
        placement[:3, :3] = _rpy(*_floats(origin.get("rpy")))
        placement[:3, 3] = _floats(origin.get("xyz"))
        limit = joint.find("limit")
        axis = np.array(_floats(joint.find("axis").get("xyz")))
        out.append((placement, axis / np.linalg.norm(axis),
                    (float(limit.get("lower")), float(limit.get("upper")))))
    return out


def _fk(chain, q, lib):
    """End effector (rotation 3x3, position 3) in ROOT; numpy, or casadi expressions of `q`."""
    if lib is np:
        rot, pos, matrix = np.eye(3), np.zeros(3), np.array
    else:
        rot, pos = casadi.DM.eye(3), casadi.DM.zeros(3, 1)
        matrix = lambda rows: casadi.vertcat(*[casadi.horzcat(*row) for row in rows])  # noqa: E731
    for i, (placement, axis, _) in enumerate(chain):
        pos = pos + rot @ placement[:3, 3]
        rot = rot @ placement[:3, :3] @ matrix(_axis_angle(axis, q[i], lib))
    return rot, pos + rot @ np.array([EE_OFFSET, 0.0, 0.0])


def log3(rot, lib=np):
    """Rotation vector of a rotation matrix; smooth at identity (a 1e-12 floor under the sine)."""
    v = [rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]
    s = lib.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2 + 1e-12) / 2
    c = (rot[0, 0] + rot[1, 1] + rot[2, 2] - 1) / 2
    k = (np.arctan2 if lib is np else casadi.atan2)(s, c) / (2 * s)
    return k * np.array(v) if lib is np else [k * x for x in v]


class RightArmIK:
    """`solve(rot, pos)`, a target in ROOT -> 7 joint angles (motors 22..28), or None: no solve."""

    def __init__(self, urdf):
        self.chain = load_chain(urdf)
        self.lower = np.array([lim[0] for _, _, lim in self.chain])
        self.upper = np.array([lim[1] for _, _, lim in self.chain])
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(7)
        self.q_last = self.opti.parameter(7)
        self.target_rot = self.opti.parameter(3, 3)
        self.target_pos = self.opti.parameter(3)
        rot, pos = _fk(self.chain, self.var_q, casadi)
        dp = pos - self.target_pos
        dr = casadi.vertcat(*log3(rot @ self.target_rot.T, casadi))
        self.opti.subject_to(self.opti.bounded(self.lower, self.var_q, self.upper))
        self.opti.minimize(50 * casadi.sumsqr(dp) + casadi.sumsqr(dr)
                           + 0.02 * casadi.sumsqr(self.var_q)
                           + 0.1 * casadi.sumsqr(self.var_q - self.q_last))
        self.opti.solver("ipopt", {"ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-6,
                                             "sb": "yes"},
                                   "print_time": False, "calc_lam_p": False})
        self.reset(np.zeros(7))

    def reset(self, q):
        """Seed and filter history from a measured arm."""
        self.q = np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper)
        self._history = [self.q.copy()]

    def fk(self, q):
        """(rotation, position) of the end effector."""
        return _fk(self.chain, np.asarray(q, dtype=np.float64), np)

    def solve(self, rot, pos):
        self.opti.set_initial(self.var_q, self.q)
        self.opti.set_value(self.q_last, self.q)
        self.opti.set_value(self.target_rot, np.asarray(rot, dtype=np.float64))
        self.opti.set_value(self.target_pos, np.asarray(pos, dtype=np.float64))
        try:
            raw = np.asarray(self.opti.solve().value(self.var_q), dtype=np.float64).reshape(7)
        except RuntimeError:
            return None
        if not np.array_equal(raw, self._history[0]):
            self._history = [raw] + self._history[:len(FILTER_WEIGHTS) - 1]
        if len(self._history) < len(FILTER_WEIGHTS):
            self.q = self._history[0]
        else:
            self.q = sum(w * h for w, h in zip(FILTER_WEIGHTS, self._history))
        return self.q.copy()
