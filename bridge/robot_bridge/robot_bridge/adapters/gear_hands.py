"""Drop-in for GEAR's HandCommandSender that publishes /hand_cmd/<side>.

Pass-through, no remap. `HandCommandSender.send_command` takes DDS order --
`g1_hand.queue_action` hands it the same order `HandStateProcessor` reads back --
and `/hand_cmd/<side>` is DDS order too. Remapping here would be the bug.

The goal-order-to-DDS-order remap that GEAR's *goal* array needs is a different
interface, upstream in retargeting; in this repo it is
`g1_stack.gear.goal.hand_from_pose`.
"""

import numpy as np

# GEAR's hand loop runs at 100 Hz; its constructor carries no rate.
DEFAULT_DURATION = 0.01

# g1_hand imports HandCommandSender into its own namespace, so that is what
# install() rebinds.
GEAR_HAND_MODULE = "decoupled_wbc.control.envs.g1.g1_hand"

NUM_HAND_JOINTS = 7

# Exactly what HandCommandSender bakes in (command_sender.py). The adapter is a
# drop-in and its caller passes no gains, so it has to reproduce them. Motor 0 is
# the thumb's loaded axis, hence the higher pair.
GEAR_HAND_KP = (2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
GEAR_HAND_KD = (0.5, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2)


class GearHandsAdapter:
    """Same surface as HandCommandSender: HandCommandSender(is_left=...), then send_command().

    Opens no rt/dex3/<side>/cmd publisher; the bridge owns that topic.
    """

    def __init__(self, client, is_left=True, duration=DEFAULT_DURATION):
        self.client = client
        self.is_left = bool(is_left)
        self.side = "left" if self.is_left else "right"
        self.duration = duration

        self.hand_dof = NUM_HAND_JOINTS
        self.kp = np.asarray(GEAR_HAND_KP, dtype=np.float32)
        self.kd = np.asarray(GEAR_HAND_KD, dtype=np.float32)

    def send_command(self, cmd):
        """GEAR's entry point. One /hand_cmd/<side> per call."""
        q = np.asarray(cmd, dtype=np.float32)
        if q.shape != (self.hand_dof,):
            raise ValueError(f"hand command must have shape ({self.hand_dof},), got {q.shape}")

        return self.client.send_hand_cmd(
            self.side,
            q=q,
            kp=self.kp,
            kd=self.kd,
            duration=self.duration,
            hold_position=False,
        )


def make_factory(client, duration=DEFAULT_DURATION):
    """Wrap client into a HandCommandSender(is_left=...) replacement."""

    def factory(is_left=True):
        return GearHandsAdapter(client, is_left=is_left, duration=duration)

    return factory


def install(client, duration=DEFAULT_DURATION, module=None):
    """Rebind g1_hand.HandCommandSender to this adapter. Call before building the env."""
    if module is None:
        import importlib

        module = importlib.import_module(GEAR_HAND_MODULE)
    factory = make_factory(client, duration)
    module.HandCommandSender = factory
    return factory
