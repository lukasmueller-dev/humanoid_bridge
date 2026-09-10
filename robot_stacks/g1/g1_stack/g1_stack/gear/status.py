"""Reads the GEAR WBC's lower-body policy status.

Transcribed from the publisher that writes it, `run_g1_control_loop.py` via
`decoupled_wbc/control/utils/messaging_zmq.py`. The control loop *binds* this
port and we connect, the opposite way round from the goal topic.

This exists because `toggle_policy_action` is a toggle with no acknowledgement.
Sent blind it is unreliable in both directions: ZMQ PUB drops whatever it sends
before a subscriber has finished connecting, so an early one is simply lost,
and a resend flips a flag that may already be set. `use_policy_action` here is
the only way to tell which state the controller is actually in.
"""

from __future__ import annotations

import msgpack
import msgpack_numpy
import zmq

# messaging_zmq.py DEFAULT_TOPIC_PORTS["ControlPolicy/lower_body_policy_status"]
STATUS_TOPIC = "ControlPolicy/lower_body_policy_status"
STATUS_PORT = 6003


class PolicyStatus:
    """One ZMQ SUB socket, connected. `latest` never blocks."""

    def __init__(self, host="127.0.0.1", port=STATUS_PORT, context=None):
        self._owns_context = context is None
        self._context = context if context is not None else zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        # CONFLATE to match the control loop's own subscribers: the newest
        # status is the only one worth having.
        self._socket.setsockopt(zmq.CONFLATE, True)
        self._socket.setsockopt(zmq.RCVHWM, 3)
        self._socket.connect(f"tcp://{host}:{port}")
        self._last = None

    def latest(self):
        """The newest status seen, or None if the loop has published none yet."""
        while True:
            try:
                packed = self._socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                return self._last
            self._last = msgpack.unpackb(packed, object_hook=msgpack_numpy.decode)

    def engaged(self):
        """True/False once the loop has said, None while it has not."""
        status = self.latest()
        if status is None:
            return None
        return bool(status.get("use_policy_action", False))

    def close(self):
        self._socket.close()
        if self._owns_context:
            self._context.term()


class Engager:
    """Asks the controller for the walk policy until it says yes.

    `toggle_policy_action` flips rather than sets and is acknowledged only in
    the status topic, so neither sending it once nor sending it always works.
    Once is lost when ZMQ PUB drops what it publishes before the subscriber has
    finished connecting -- which is exactly the first frames. Always flips the
    legs on and off at loop rate. So: ask, wait for the status to come back
    engaged, and ask again only if it has not.

    `retry` has to outlast one goal-to-status round trip or the second ask
    undoes the first.
    """

    def __init__(self, status, retry=1.0, out=print):
        self._status = status
        self._retry = retry
        self._out = out
        self._asked_at = None
        self._announced = False

    def wants(self, now):
        """True when this goal should carry the toggle."""
        if self._status is None:
            return False
        engaged = self._status.engaged()
        if engaged is None:
            # The loop has published no status, so it is not reading goals yet
            # either: an ask now is dropped, and we could not tell whether it
            # landed. Wait to be told which state it is in.
            return False
        if engaged:
            if not self._announced:
                self._out("walk policy engaged")
                self._announced = True
            return False
        if self._asked_at is not None and now - self._asked_at < self._retry:
            return False
        self._asked_at = now
        return True
