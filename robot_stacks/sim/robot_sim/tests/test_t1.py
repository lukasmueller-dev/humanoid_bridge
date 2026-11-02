"""RobotSim on the T1, no ROS. Needs mujoco and a T1 model (fetch_assets.sh t1_ncbf, t1_booster)."""

import pathlib
import re
import types

import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip("mujoco")

from robot_sim import t1_parallel_ankle as pa  # noqa: E402
from robot_sim.physics import Command, RobotSim  # noqa: E402
from robot_sim.robots import t1  # noqa: E402

DATA = pathlib.Path("~/.local/share/robot_web_commander").expanduser()
MODELS = [DATA / "t1_ncbf" / "t1.xml", DATA / "t1_booster" / "T1_23dof.xml"]


@pytest.fixture(params=MODELS, ids=["ncbf", "booster"])
def sim(request):
    if not request.param.is_file():
        pytest.skip(f"no {request.param}")
    return RobotSim(t1.SPEC, request.param)


def hold(sim, target, kp=80.0, kd=2.0):
    cmd = Command(sim.num_joint)
    cmd.mode[:] = 1
    cmd.q[:], cmd.kp[:], cmd.kd[:] = target, kp, kd
    return cmd


def test_motor_order_is_the_bridge_order(sim):
    names = [sim.model.joint(sim.model.jnt_qposadr.tolist().index(a)).name for a in sim.qadr]
    assert names == list(t1.JOINT_NAMES) and sim.num_joint == 23


def test_one_floor_plane(sim):
    """Booster's scene has its own `ground`: a second plane at z 0 z-fights in the camera."""
    planes = [i for i in range(sim.model.ngeom) if sim.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE]
    assert len(planes) == 1


def test_the_head_camera_looks_forward_and_down(sim):
    mujoco.mj_forward(sim.model, sim.data)
    view = -sim.data.cam_xmat[sim.model.camera(t1.CAMERA).id].reshape(3, 3)[:, 2]  # along -z
    up = sim.data.cam_xmat[sim.model.camera(t1.CAMERA).id].reshape(3, 3)[:, 1]
    assert view[0] > 0.95 and view[2] < 0.0, view
    assert up[2] > 0.9, up


def test_the_gantry_holds_a_limp_t1_level(sim):
    for _ in range(int(2.0 / sim.dt)):
        sim.step(Command(sim.num_joint))
    s = sim.state()
    assert abs(s["rpy"][0]) < 0.05 and abs(s["rpy"][1]) < 0.05
    assert sim.data.qpos[2] == pytest.approx(sim.anchor[2], abs=0.05)


def bridge_head():
    config = pathlib.Path(__file__).parents[4] / "bridge/robot_bridge/params/T1_config.yaml"
    return yaml.safe_load(config.read_text())["robot_bridge"]["ros__parameters"]["HeadPitch"]


def test_a_damped_head_stays_inside_the_bridge_limit(sim):
    """sim-t1: the bridge clamps a hold target outside q_min, q_max; the head must stay inside."""
    head = bridge_head()
    i = t1.JOINT_NAMES.index("aahead_pitch_joint")
    cmd = Command(sim.num_joint)
    cmd.mode[:], cmd.kd[:] = 1, 0.5  # the head's damp_kd
    peak = sim.state()["q"][i]
    for _ in range(int(3.0 / sim.dt)):
        sim.step(cmd)
        peak = max(peak, sim.state()["q"][i])
    assert head["q_max"] - 0.1 < peak <= head["q_max"]  # it reached its stop, inside the limit
    assert head["q_min"] <= sim.state()["q"][i]


def test_a_limp_head_rests_where_arm_accepts_it(sim):
    """A limp head is a re-arm's default_position: inside T1_config.yaml's range + 0.1 rad."""
    head = bridge_head()
    cmd = Command(sim.num_joint)
    cmd.mode[:], cmd.kd[:] = 1, 0.5  # the head's damp_kd
    for _ in range(int(6.0 / sim.dt)):
        sim.step(cmd)
    q = sim.state()["q"][t1.JOINT_NAMES.index("aahead_pitch_joint")]
    assert head["q_max"] - 0.1 < q < head["q_max"] + 0.1  # it falls to its stop


def test_pd_holds_a_pose_within_the_effort(sim):
    target = np.zeros(sim.num_joint)
    target[t1.JOINT_NAMES.index("left_elbow_pitch_joint")] = 0.5
    cmd = hold(sim, target)
    for _ in range(int(2.0 / sim.dt)):
        sim.step(cmd)
    s = sim.state()
    assert np.abs(s["q"] - target).max() < 0.1
    assert np.all(np.abs(s["tau_est"]) <= sim.effort + 1e-9)


def test_mode_requests_and_buttons():
    assert t1.mode_request('{"mode":3}') == t1.CUSTOM
    assert t1.mode_request('{"mode":9}') is None
    assert t1.mode_request("not json") is None
    assert set(t1.PAD) >= {"RB", "RT", "LB"}


def test_crank_state_maps_back_through_the_bridge_inverse():
    """motor_state_parallel, float32 on the wire, gives the serial ankles back (f^-1, J^-1)."""
    rng = np.random.default_rng(0)
    q, dq = rng.uniform(-0.3, 0.3, t1.NUM_JOINT), rng.uniform(-2.0, 2.0, t1.NUM_JOINT)
    q_m, dq_m = t1.crank_state(q, dq)
    others = [i for i in range(t1.NUM_JOINT) if not any(i in s for s in t1.ANKLES.values())]
    assert np.array_equal(q_m[others], q[others]) and np.array_equal(dq_m[others], dq[others])
    for side, slots in t1.ANKLES.items():
        wire = q_m[slots].astype(np.float32).astype(float)
        q_s, residual = pa.serial_angles(wire, side, q[slots])
        assert residual < 1e-6
        np.testing.assert_allclose(q_s, q[slots], atol=1e-5)
        J = pa.kinematics(q[slots], side)[1]
        np.testing.assert_allclose(np.linalg.solve(J, dq_m[slots]), dq[slots], atol=1e-9)


def test_crank_pd_maps_to_serial_torque():
    """tau_s = J^T tau_m: equal power at the cranks and the joints; each crank clipped at 43 Nm."""
    rng = np.random.default_rng(1)
    q, dq = rng.uniform(-0.3, 0.3, t1.NUM_JOINT), rng.uniform(-2.0, 2.0, t1.NUM_JOINT)
    q_m, dq_m = t1.crank_state(q, dq)
    cmd = t1.CrankCommand(t1.NUM_JOINT)
    cmd.mode[:] = 1
    cmd.q[:], cmd.dq[:] = q_m + 0.05, rng.uniform(-1.0, 1.0, t1.NUM_JOINT)
    cmd.kp[:], cmd.kd[:], cmd.tau[:] = 40.0, 2.0, rng.uniform(-3.0, 3.0, t1.NUM_JOINT)
    out = t1.serial_command(cmd, q, dq)
    ankles = sum(t1.ANKLES.values(), [])
    others = [i for i in range(t1.NUM_JOINT) if i not in ankles]
    for field in ("mode", "q", "dq", "kp", "kd", "tau"):
        assert np.array_equal(getattr(out, field)[others], getattr(cmd, field)[others])
    assert not out.kp[ankles].any() and not out.kd[ankles].any()
    tau_m = cmd.tau + cmd.kp * (cmd.q - q_m) + cmd.kd * (cmd.dq - dq_m)
    for slots in t1.ANKLES.values():
        assert out.tau[slots] @ dq[slots] == pytest.approx(tau_m[slots] @ dq_m[slots])

    cmd.tau[ankles] = 500.0
    out = t1.serial_command(cmd, q, dq)
    for side, slots in t1.ANKLES.items():
        J = pa.kinematics(q[slots], side)[1]
        np.testing.assert_allclose(np.linalg.solve(J.T, out.tau[slots]), t1.CRANK_TAU_MAX)


def test_a_crank_pd_holds_the_ankles(sim):
    target = np.zeros(sim.num_joint)
    for slots in t1.ANKLES.values():
        target[slots] = (-0.3, 0.15)
    cmd = t1.CrankCommand(sim.num_joint)
    cmd.mode[:], cmd.kp[:], cmd.kd[:] = 1, 80.0, 2.0
    cmd.q[:] = t1.crank_state(target, np.zeros(sim.num_joint))[0]
    for _ in range(int(2.0 / sim.dt)):
        sim.step(t1.serial_command(cmd, sim.data.qpos[sim.qadr], sim.data.qvel[sim.vadr]))
    ankles = sum(t1.ANKLES.values(), [])
    assert np.abs(sim.state()["q"][ankles] - target[ankles]).max() < 0.01


def test_crank_slots_are_the_bridge_ankle_slots():
    """ANKLES is T1_bridge's kAnkleSerialIdx and the ankle joints of the bridge order."""
    header = pathlib.Path(__file__).parents[4] / "bridge/robot_bridge/include/T1_bridge.hpp"
    pattern = r"kAnkleSerialIdx\[2\]\[2\] = \{\{(\d+), (\d+)\}, \{(\d+), (\d+)\}\}"
    a, b, c, d = (int(i) for i in re.search(pattern, header.read_text()).groups())
    assert t1.ANKLES == {"left": [a, b], "right": [c, d]}
    names = [t1.JOINT_NAMES[i] for i in (a, b, c, d)]
    assert names == [f"{s}_ankle_{j}_joint" for s in ("left", "right") for j in ("pitch", "roll")]


def test_only_a_parallel_frame_gets_the_crank_pd(sim):
    """CMD_TYPE_SERIAL: a plain Command, through the filter unchanged. CMD_TYPE_PARALLEL: cranks."""
    taken = []
    node = types.SimpleNamespace(take_cmd=taken.append)
    motors = [types.SimpleNamespace(q=0.1, dq=0.0, tau=1.0, kp=40.0, kd=2.0)] * sim.num_joint
    wire = types.SimpleNamespace(node=node, sim=sim, mode=t1.CUSTOM, damp=sim.command())
    ankles = sum(t1.ANKLES.values(), [])
    for cmd_type in (1, 0):  # CMD_TYPE_SERIAL, CMD_TYPE_PARALLEL
        msg = types.SimpleNamespace(cmd_type=cmd_type, CMD_TYPE_PARALLEL=0, motor_cmd=motors)
        t1.Wire._on_cmd(wire, msg)
    serial, parallel = taken
    assert type(serial) is Command and t1.Wire.filter(wire, serial) is serial
    assert type(parallel) is t1.CrankCommand
    out = t1.Wire.filter(wire, parallel)
    assert not out.kp[ankles].any() and out.tau[ankles].any()
