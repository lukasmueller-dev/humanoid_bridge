"""include/: no two headers share an include guard, or the second one included is empty."""

import re
from collections import Counter
from pathlib import Path

INCLUDE = Path(__file__).resolve().parent.parent / "include"


def test_include_guards_are_unique():
    guards = {}
    for header in sorted(INCLUDE.rglob("*.hpp")):
        match = re.search(r"^#ifndef (\w+)\n#define \1$", header.read_text(), re.MULTILINE)
        if match:
            guards[header.name] = match.group(1)
    shared = {g for g, n in Counter(guards.values()).items() if n > 1}
    assert not shared, {h: g for h, g in guards.items() if g in shared}
