"""g1_link_watch damps as the G1 bridge does: DAMP_KD is G1_config.yaml's estop.damp_kd.

Here, not in the package's tests: g1_link_watch ships to the Jetson without the bridge's params.
"""

from pathlib import Path

import yaml

from g1_link_watch.watch import DAMP_KD

PARAMS = Path(__file__).resolve().parents[1] / "bridge/robot_bridge/params/G1_config.yaml"


def test_damp_kd_is_the_bridges_and_no_joint_overrides_it():
    params = yaml.safe_load(PARAMS.read_text())["robot_bridge"]["ros__parameters"]
    assert params["estop"]["damp_kd"] == DAMP_KD
    # The watcher has one damp kd: a per-joint `<name>.damp_kd` is not supported.
    overrides = [k for k, v in params.items() if isinstance(v, dict) and "damp_kd" in v]
    assert overrides == ["estop"]
