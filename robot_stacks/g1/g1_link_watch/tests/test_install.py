"""install.sh --dry-run: the steps, into the unit's PYTHONPATH; an unknown option exits 2."""

import pathlib
import subprocess

PKG = pathlib.Path(__file__).resolve().parents[1]


def install(*args):
    return subprocess.run(
        ["bash", str(PKG / "install.sh"), *args], capture_output=True, text=True, check=False
    )


def test_dry_run_prints_the_steps_into_the_units_pythonpath():
    result = install("--dry-run")
    assert result.returncode == 0, result.stderr
    steps = result.stdout.splitlines()
    assert [" ".join(s.split()[:2]) for s in steps] == [
        "install -d",
        "rsync -a",
        "install -m",
        "systemctl daemon-reload",
        "systemctl enable",
        "systemctl restart",
    ]
    unit = (PKG / "g1-link-watch.service").read_text()
    dest = steps[0].split()[-1]
    assert f"Environment=PYTHONPATH={dest}\n" in unit
    assert f"{dest}/g1_link_watch/" in steps[1]


def test_an_unknown_option_exits_2():
    assert install("--now").returncode == 2
