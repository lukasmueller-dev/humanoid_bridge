"""Puts this package's source on the path, so its tests run on their own.

`bridge/` is meant to split out with `git subtree split --prefix=bridge`, and
the repo-root conftest does not travel with it. Without a path entry the tests
fail at collection instead of skipping cleanly.

Not at `bridge/conftest.py`: that would put `bridge/` on `sys.path`, where the
source `bridge_interface/` directory shadows the built ROS package as a
namespace package. `importorskip` then succeeds and the import fails with
"unknown location".
"""

import pathlib
import sys

_SRC = str(pathlib.Path(__file__).parent)
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
