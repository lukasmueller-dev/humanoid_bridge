"""RobotClient default-gain length checks. A fake node; no bridge, no network."""

from unittest.mock import MagicMock

import pytest

# Not importorskip: see test_cmd_client.py.
try:
    from bridge_interface.msg import RobotCmd  # noqa: F401
    from unitree_hg.msg import LowState  # noqa: F401
except ImportError:
    pytest.skip("source the colcon workspace first", allow_module_level=True)

from robot_bridge.robot_client import RobotClient  # noqa: E402


def test_g1_defaults_are_29_long():
    client = RobotClient(MagicMock(), "G1", 29, 50.0)

    for name in ("_default_pos", "_default_kp", "_default_kd"):
        assert len(getattr(client, name)) == 29


@pytest.mark.parametrize("num_dof", [27, 28, 30])
def test_a_default_of_another_length_raises(num_dof):
    with pytest.raises(ValueError, match="G1 default_pos has length 29"):
        RobotClient(MagicMock(), "G1", num_dof, 50.0)
