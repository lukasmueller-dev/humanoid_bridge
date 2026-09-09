"""Client for the camera server on the robot's onboard Jetson.

REQ/REP is lockstep, so a dropped reply wedges the socket instead of erroring:
every request polls with a timeout and rebuilds the socket when it expires.

Runs on Python 3.8 (JetPack 5.1.1, ROS 2 Foxy), where `X | None` in a signature
raises TypeError at import — hence the `annotations` future import.
"""

from __future__ import annotations

import numpy as np

from . import wire


class CameraTimeout(RuntimeError):
    """The server did not answer within the deadline."""


class CameraClient:
    """Pulls one RGB frame per call from the robot's camera server."""

    def __init__(
        self,
        host: str | None = None,
        port: int | None = None,
        timeout_ms: int = 200,
        zmq_module=None,
        decoder=None,
    ):
        self._host = wire.DEFAULT_HOST if host is None else host
        self._port = wire.DEFAULT_PORT if port is None else port
        self._timeout_ms = timeout_ms
        # Injected so tests run without cv2, and a caller can swap the decoder.
        self._zmq = zmq_module or __import__("zmq")
        self._decode = decoder or _decode_jpeg
        self._ctx = self._zmq.Context.instance()
        self._socket = None
        self._connect()

    @property
    def address(self) -> str:
        return f"tcp://{self._host}:{self._port}"

    def _connect(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
        self._socket = self._ctx.socket(self._zmq.REQ)
        self._socket.setsockopt(self._zmq.LINGER, 0)
        self._socket.connect(self.address)

    def read_rgb(self) -> np.ndarray:
        """One RGB frame, (height, width, 3) uint8.

        Raises CameraTimeout rather than reusing a stale frame, which would
        hide a dead camera.
        """
        self._socket.send(wire.REQUEST)
        if not self._socket.poll(self._timeout_ms, self._zmq.POLLIN):
            # The REQ socket is now stuck mid-cycle; only a fresh one recovers.
            self._connect()
            raise CameraTimeout(f"no frame from {self.address} within {self._timeout_ms} ms")
        parts = self._socket.recv_multipart()
        if not parts:
            raise CameraTimeout(f"empty reply from {self.address}")
        frame = self._decode(parts[0])
        if frame is None:
            raise CameraTimeout(f"undecodable frame from {self.address}")
        return frame

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None


def _decode_jpeg(payload: bytes) -> np.ndarray | None:
    """JPEG bytes to an RGB array. The only BGR-to-RGB flip in the package."""
    import cv2

    # A server that failed to grab answers with empty parts. cv2.imdecode
    # raises on an empty buffer, which would escape as cv2.error.
    if not payload:
        return None
    bgr = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return bgr[:, :, ::-1].copy()
