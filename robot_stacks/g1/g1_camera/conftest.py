"""Puts the package source on the path, so `pytest` works here without an install.

This package carries its own pyproject.toml to stay self-contained for the
Jetson, which makes it its own pytest rootdir — the repo-root conftest does not
apply when its tests are run directly.
"""

import pathlib
import sys

_SRC = str(pathlib.Path(__file__).parent)
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
