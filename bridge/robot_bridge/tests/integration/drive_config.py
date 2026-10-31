"""Starts G1_bridge on broken copies of a config and on bad override files: each must fail its load.

Exit 1 on failure. run_config_test.sh pins DDS to loopback. No fake robot: a config
that loads waits for /lowstate, and is killed once its load is logged.
"""

import argparse
import copy
import os
import signal
import subprocess
import sys
import tempfile
import time

import yaml

LOADED = "E-stop: damp_kd"  # the last line of a load that passed
REFUSED = "Failed to load parameters."
TIMEOUT = 10.0  # s per start
R_ELBOW = 25
BRIDGE = ["ros2", "run", "robot_bridge", "G1_bridge", "--ros-args", "-r", "__node:=robot_bridge"]


def variants(params):
    """(name, text the refusal must log, change to the config) per broken copy."""
    names = params["joint_names"]

    def short_ready(p):
        p["ready_q"] = p["ready_q"][:-1]

    def ready_outside(p):
        p["ready_q"][R_ELBOW] = p[names[R_ELBOW]]["q_max"] + 0.5

    def idx_order(p):
        p["joint_names"] = [names[1], names[0]] + names[2:]

    def q_range(p):
        p[names[3]]["q_max"] = p[names[3]]["q_min"] - 0.1

    def tau_limit(p):
        p[names[3]]["tau_limit"] = -1.0

    def dq_trip(p):
        p[names[3]]["dq_trip"] = 0.0

    def gain_range(p):
        p["upper_limbs_kp_max"] = p["upper_limbs_kp_min"] - 1.0

    def torque_control(p):
        p["torque_control"] = True

    def num_joint(p):
        p["num_joint"] = len(names) + 1

    def no_plugin(p):
        p["controllers_allowed"] = p["controllers_allowed"] + ["no_such_controller"]

    def no_kd(p):
        p["controllers_allowed"] = p["controllers_allowed"] + ["example_controller"]

    return [
        ("ready_q one short", "ready_q has 28 values, expected 29", short_ready),
        ("ready_q outside its joint range", f"ready_q[{R_ELBOW}]", ready_outside),
        ("idx not its place in joint_names", "they must match", idx_order),
        ("q_max below q_min", "needs q_min <= q_max", q_range),
        ("negative tau_limit", "needs q_min <= q_max", tau_limit),
        ("dq_trip of zero", "dq_trip must be finite and > 0", dq_trip),
        ("inverted gain range", "gain range is inverted", gain_range),
        ("torque_control true", "Torque control is not supported", torque_control),
        ("num_joint not the joint count", "joint_names has 29 joints, num_joint is 30", num_joint),
        ("a missing controller plugin", "controller no_such_controller does not load", no_plugin),
        ("example_controller without kd", "needs controllers.example_controller.kd", no_kd),
    ]


def refused_overrides(params):
    """(name, text the refusal must log, override values) per override file that is refused."""
    joint = params["joint_names"][3]
    limit = params[joint]["dq_limit"]
    return [
        (
            "override of a key not in override_keys",
            "is not an override key",
            {joint: {"dq_limit": 1.0}},
        ),
        (
            "override of an unknown joint",
            "no joint 'no_such_joint'",
            {"no_such_joint": {"dq_trip": 1.0}},
        ),
        ("override of zero", "needs a finite number > 0", {joint: {"dq_trip": 0.0}}),
        ("negative override", "needs a finite number > 0", {joint: {"dq_trip": -1.0}}),
        ("infinite override", "needs a finite number > 0", {joint: {"dq_trip": float("inf")}}),
        ("NaN override", "needs a finite number > 0", {joint: {"dq_trip": "nan"}}),
        ("override that is text", "needs a finite number > 0", {joint: {"dq_trip": "fast"}}),
        ("override above 2 x dq_limit", "is above 2 x dq_limit", {joint: {"dq_trip": 2.1 * limit}}),
    ]


def valid_overrides(params):
    """(name, WARN the load must log, override values) per override file that loads."""
    joint = params["joint_names"][3]
    limit = params[joint]["dq_limit"]
    base = params[joint].get("dq_trip", limit)  # absent = dq_limit

    def warn(new):
        return f"Override {joint}.dq_trip: {base:.3f} -> {new:.3f}"

    return [
        ("override at 2 x dq_limit", warn(2 * limit), {joint: {"dq_trip": 2 * limit}}),
        ("override below the base", warn(0.5 * limit), {joint: {"dq_trip": 0.5 * limit}}),
        ("override as an integer", warn(int(limit)), {joint: {"dq_trip": int(limit)}}),
    ]


def start(params_files, log_file, until=None):
    """Runs G1_bridge until it exits, logs `until`, or TIMEOUT. Returns (exit code or None, log)."""
    args = [arg for path in params_files for arg in ("--params-file", path)]
    with open(log_file, "w") as log:
        proc = subprocess.Popen(
            BRIDGE + args,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    end = time.monotonic() + TIMEOUT
    while time.monotonic() < end and proc.poll() is None:
        with open(log_file) as handle:
            if until and until in handle.read():
                break
        time.sleep(0.1)
    code = proc.poll()
    if code is None:
        # The group: `ros2 run` is a launcher, and killing it alone orphans G1_bridge.
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    with open(log_file) as handle:
        return code, handle.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="a G1_config.yaml that loads")
    args = parser.parse_args()

    with open(args.params) as handle:
        config = yaml.safe_load(handle)
    out = tempfile.mkdtemp()
    failures = []

    def check(name, ok, detail=""):
        print(
            "{:<48} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            failures.append(name)

    # The copy, unchanged, must load: the round trip through yaml is not what refuses.
    unchanged = os.path.join(out, "unchanged.yaml")
    with open(unchanged, "w") as handle:
        yaml.safe_dump(config, handle)
    _, log = start([unchanged], os.path.join(out, "unchanged.log"), until=LOADED)
    check("unchanged config loads", LOADED in log and REFUSED not in log)

    for i, (name, text, change) in enumerate(variants(config["robot_bridge"]["ros__parameters"])):
        broken = copy.deepcopy(config)
        change(broken["robot_bridge"]["ros__parameters"])
        path = os.path.join(out, f"broken{i}.yaml")
        with open(path, "w") as handle:
            yaml.safe_dump(broken, handle)
        code, log = start([path], os.path.join(out, f"broken{i}.log"))
        # Exit 1, not a signal: `ros2 run` reports an abort as 250.
        refused = code == 1 and REFUSED in log and text in log
        if code is None:
            why = f"still running after {TIMEOUT:.0f} s"
        else:
            why = f"exit {code}" + ("" if text in log else f", no '{text}'")
        check(f"{name}: refused", refused, "" if refused else f"{why}, log {out}/broken{i}.log")

    # A second --params-file after the unchanged copy, as robot-bringup.sh --override passes it.
    params = config["robot_bridge"]["ros__parameters"]

    def override_file(name, values):
        override = os.path.join(out, name + ".yaml")
        with open(override, "w") as handle:
            yaml.safe_dump({"robot_bridge": {"ros__parameters": {"override": values}}}, handle)
        return override

    for i, (name, text, values) in enumerate(refused_overrides(params)):
        override = override_file(f"refused{i}", values)
        code, log = start([unchanged, override], os.path.join(out, f"refused{i}.log"))
        # One ERROR line names the override: the refusal, not a second complaint after it.
        errors = [line for line in log.splitlines() if "ERROR" in line and "override." in line]
        refused = code == 1 and REFUSED in log and text in log and len(errors) == 1
        if code is None:
            why = f"still running after {TIMEOUT:.0f} s"
        else:
            why = f"exit {code}, {len(errors)} override ERROR line(s)" + (
                "" if text in log else f", no '{text}'"
            )
        check(f"{name}: refused", refused, "" if refused else f"{why}, log {out}/refused{i}.log")

    for i, (name, text, values) in enumerate(valid_overrides(params)):
        override = override_file(f"valid{i}", values)
        _, log = start([unchanged, override], os.path.join(out, f"valid{i}.log"), until=LOADED)
        loaded = LOADED in log and REFUSED not in log and text in log
        check(
            f"{name}: loads, logs the change",
            loaded,
            "" if loaded else f"no '{text}', log {out}/valid{i}.log",
        )

    if failures:
        print("\n{} check(s) failed: {}".format(len(failures), ", ".join(failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
