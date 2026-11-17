"""allow_lo: multicast forced on for lo only, once per process; an unknown template is refused."""

from __future__ import annotations

import sys
import types

import pytest

from g1_stack.joints import dds

TEMPLATE = '<NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>'


@pytest.fixture
def channel(monkeypatch):
    channel = types.SimpleNamespace(ChannelConfigHasInterface=TEMPLATE)
    core = types.ModuleType("unitree_sdk2py.core")
    core.channel = channel
    sdk = types.ModuleType("unitree_sdk2py")
    sdk.core = core
    monkeypatch.setitem(sys.modules, "unitree_sdk2py", sdk)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core", core)
    return channel


def test_lo_forces_multicast_on_and_a_second_call_is_a_no_op(channel):
    dds.allow_lo("lo")
    dds.allow_lo("lo")
    assert channel.ChannelConfigHasInterface == (
        '<NetworkInterface name="$__IF_NAME__$" priority="default" multicast="true"/>')


def test_other_interfaces_keep_the_template(channel):
    dds.allow_lo("eth0")
    dds.allow_lo(None)
    assert channel.ChannelConfigHasInterface == TEMPLATE


def test_an_unknown_template_is_refused(channel):
    channel.ChannelConfigHasInterface = '<NetworkInterface name="$__IF_NAME__$"/>'
    with pytest.raises(RuntimeError, match="another patch"):
        dds.allow_lo("lo")
