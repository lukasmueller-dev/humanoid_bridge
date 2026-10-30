"""Camera server and client for the G1's onboard Jetson.

Self-contained on purpose: nothing here imports outside this package, so the
directory can be rsynced to the Jetson and pip-installed on its own, under
Python 3.8 with no ROS. `tests/test_isolated.py` holds that line.
"""

from .client import CameraClient, CameraTimeout
from .fake import FakeCameraServer, synthetic_rgb
from .server import CameraServer
from .sources import V4L2Source, VideoHubSource, fit_frame, initialize_dds, list_devices

__all__ = [
    "CameraClient",
    "CameraServer",
    "CameraTimeout",
    "FakeCameraServer",
    "V4L2Source",
    "VideoHubSource",
    "fit_frame",
    "initialize_dds",
    "list_devices",
    "synthetic_rgb",
]
