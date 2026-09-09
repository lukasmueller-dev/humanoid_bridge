"""Publishes goal messages onto the GEAR WBC's ZMQ topic.

Transcribed from the subscriber that reads them,
`decoupled_wbc/control/utils/messaging_zmq.py`. Two things that surprise:

- **The publisher binds and the subscriber connects.** We bind port 6001; the
  control loop connects to it with `--zmq-host`. That is backwards from the
  usual pub/sub picture, and it means the control loop can start after us.
- The subscriber sets `CONFLATE`, so it only ever sees the newest message.
  Publishing faster than the control loop reads costs nothing; publishing
  slower than 1 Hz trips the controller's teleop timeout
  (`g1_decoupled_whole_body_policy.py:118`), which injects a stop.
"""

from __future__ import annotations

import msgpack
import msgpack_numpy
import zmq

# messaging_zmq.py DEFAULT_TOPIC_PORTS["ControlPolicy/upper_body_pose"]
GOAL_TOPIC = "ControlPolicy/upper_body_pose"
GOAL_PORT = 6001

# The control loop's own send high-water mark. Matched so a stalled reader
# drops messages here the same way it would there.
SEND_HWM = 20


class GoalPublisher:
    """One ZMQ PUB socket, bound. `send` never blocks."""

    def __init__(self, port=GOAL_PORT, bind_host="*", context=None):
        self._port = port
        self._owns_context = context is None
        self._context = context if context is not None else zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, SEND_HWM)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(f"tcp://{bind_host}:{port}")
        self.sent = 0
        self.dropped = 0

    @property
    def endpoint(self):
        return f"tcp://*:{self._port}"

    def send(self, goal):
        """Pack and publish one goal. Returns False if it was dropped.

        `default=msgpack_numpy.encode` is required: the subscriber unpacks with
        `object_hook=msgpack_numpy.decode`, and a plain packb cannot encode an
        ndarray at all.
        """
        packed = msgpack.packb(goal, default=msgpack_numpy.encode)
        try:
            self._socket.send(packed, flags=zmq.NOBLOCK)
        except zmq.Again:
            self.dropped += 1
            return False
        self.sent += 1
        return True

    def close(self):
        self._socket.close()
        if self._owns_context:
            self._context.term()
