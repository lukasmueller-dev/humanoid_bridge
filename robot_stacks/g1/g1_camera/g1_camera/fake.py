"""A stand-in for the camera server on the robot. Same wire contract, no camera."""

from __future__ import annotations

import argparse
import threading
import time

import numpy as np

from . import wire


def synthetic_rgb(seed: int = 0, size=wire.DEFAULT_SIZE) -> np.ndarray:
    """A deterministic test image with a different gradient per channel.

    A flat colour would pass a shape check with the channels or rows swapped.
    """
    width, height = size
    rows = np.linspace(0, 255, height, dtype=np.uint8)[:, None]
    cols = np.linspace(0, 255, width, dtype=np.uint8)[None, :]
    red = np.broadcast_to(rows, (height, width))
    green = np.broadcast_to(cols, (height, width))
    blue = np.full((height, width), seed % 256, dtype=np.uint8)
    return np.stack([red, green, blue], axis=-1).astype(np.uint8)


class FakeCameraServer:
    """Serves synthetic frames on a ZMQ REP socket, in a background thread."""

    def __init__(self, port: int = 0, stall_seconds: float = 0.0, size=wire.DEFAULT_SIZE):
        import zmq

        self._zmq = zmq
        self._size = size
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.REP)
        # port 0 asks the OS for a free one, so tests never collide.
        self.port = (
            self._socket.bind_to_random_port("tcp://127.0.0.1")
            if port == 0
            else (self._socket.bind(f"tcp://127.0.0.1:{port}"), port)[1]
        )
        # Models a slow server, not a dead one: a REP socket that never answers
        # is wedged on its own side and no client-side reconnect recovers it.
        self.stall_seconds = stall_seconds
        self._stop = threading.Event()
        self._served = 0
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self) -> FakeCameraServer:
        self._thread.start()
        return self

    def _serve(self) -> None:
        import cv2

        width, height = self._size
        while not self._stop.is_set():
            if not self._socket.poll(50, self._zmq.POLLIN):
                continue
            self._socket.recv()
            self._served += 1
            if self.stall_seconds:
                time.sleep(self.stall_seconds)
            frame = synthetic_rgb(self._served, self._size)
            ok, jpeg = cv2.imencode(".jpg", frame[:, :, ::-1])
            assert ok
            depth = np.zeros((height, width), dtype=np.uint16)
            self._socket.send_multipart([jpeg.tobytes(), jpeg.tobytes(), depth.tobytes()])

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._socket.close(linger=0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=wire.DEFAULT_PORT)
    args = parser.parse_args()
    server = FakeCameraServer(port=args.port).start()
    print(f"fake camera server on tcp://127.0.0.1:{server.port}; ctrl-c to stop")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
