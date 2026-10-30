"""Frame sources. Each is a callable returning one BGR array.

- Frames are greyscale (R == G == B): that is /dev/video2, the infrared node.
  videohub_pc4 holds the colour node, so prefer VideoHubSource.
- Geometry looks stretched: videohub serves 16:9. ``crop`` takes the centre 4:3,
  ``squash`` keeps the full view and distorts it.
- No depth, no IR, no exposure control on either path. pyrealsense2 has no
  aarch64 wheel.
"""

from __future__ import annotations

from . import wire

# The only node V4L2 can open here, and it is infrared. Use list_devices() on a
# new robot.
DEFAULT_DEVICE = 2


def list_devices():
    """What each /dev/videoN actually is, from sysfs.

    Run on a new robot: the infrared node yields correctly shaped greyscale
    frames, which a policy expresses as bad behaviour, not as an error.
    """
    import glob
    import os

    found = []
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            with open(os.path.join(path, "name")) as handle:
                found.append((os.path.basename(path), handle.read().strip()))
        except OSError:
            continue
    return found


def fit_frame(bgr, size=wire.DEFAULT_SIZE, mode="crop"):
    """Any aspect ratio onto `size`. `crop` centre-crops, `squash` scales."""
    import cv2

    width, height = size
    if mode == "crop":
        rows, cols = bgr.shape[:2]
        target = width / height
        if cols / rows > target:
            keep = int(round(rows * target))
            start = (cols - keep) // 2
            bgr = bgr[:, start : start + keep]
        elif cols / rows < target:
            keep = int(round(cols / target))
            start = (rows - keep) // 2
            bgr = bgr[start : start + keep, :]
    elif mode != "squash":
        raise ValueError(f"unknown fit mode {mode!r}")

    if bgr.shape[1] != width or bgr.shape[0] != height:
        bgr = cv2.resize(bgr, (width, height), interpolation=cv2.INTER_AREA)
    return bgr


class V4L2Source:
    """One BGR frame per call, from a UVC device."""

    def __init__(self, device=DEFAULT_DEVICE, size=wire.DEFAULT_SIZE):
        import cv2

        width, height = size
        self._capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._capture.isOpened():
            raise RuntimeError(
                f"/dev/video{device} would not open. Use list_devices() to see which "
                "node is the colour sensor."
            )
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def __call__(self):
        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera returned no frame")
        return frame

    def close(self):
        self._capture.release()


class VideoHubSource:
    """One BGR frame per call from Unitree's videohub over DDS.

    `client` is anything with `GetImageSample() -> (code, jpeg)`. Code 0 is
    success; anything else raises rather than returning a stale frame.

    DDS must already be up: call initialize_dds() first.
    """

    def __init__(self, client=None, fit="crop", size=wire.DEFAULT_SIZE, timeout_s=3.0):
        self._client = client if client is not None else _videohub_client(timeout_s)
        self._fit = fit
        self._size = size

    def __call__(self):
        import cv2
        import numpy as np

        code, data = self._client.GetImageSample()
        if code != 0 or not data:
            raise RuntimeError(f"videohub GetImageSample returned code {code}")
        bgr = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError("videohub returned an undecodable JPEG")
        return fit_frame(bgr, self._size, self._fit)


def _videohub_client(timeout_s):
    """The SDK's client. initialize_dds() must have run first."""
    from unitree_sdk2py.go2.video.video_client import VideoClient

    client = VideoClient()
    client.SetTimeout(timeout_s)
    client.Init()
    return client


def initialize_dds(network_interface=None, domain_id=0):
    """Bring up the DDS channel factory. Once per process, before any client.

    `None` auto-detects the interface, which is what running on the robot's own
    Jetson wants; pass a name only when running off-board.
    """
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(domain_id, network_interface)
