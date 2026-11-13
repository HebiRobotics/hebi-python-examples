"""Shared fixtures: simulated HEBI modules via the ``hebi imitation`` CLI.

Run from the repo root:  python -m pytest tests

Safety: every simulated device gets a unique ``pytest-<id>`` family, so tests
never look up (or command) real robots that may be on the same network.

Only one ``hebi imitation`` process can run at a time (it binds UDP port
16665), so each test starts a single process holding all the devices it needs.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

HEBI_CLI = shutil.which("hebi")


def unique_family(tag: str = "") -> str:
    return f"pytest-{tag}{'-' if tag else ''}{uuid.uuid4().hex[:8]}"


@pytest.fixture(scope="session")
def lookup():
    import hebi

    lookup = hebi.Lookup()
    time.sleep(1.0)
    return lookup


@pytest.fixture
def imitation(lookup):
    """Start simulated modules: ``imitation([(family, name), ...])`` returns ``{family: group}``.

    Waits until every family's group is visible to ``lookup``. The process is
    stopped at the end of the test.
    """
    if HEBI_CLI is None:
        pytest.skip("`hebi` CLI (HEBI Scope tools) not on PATH")

    procs: list[subprocess.Popen] = []

    def start(devices: list[tuple[str, str]], timeout_s: float = 15.0):
        families = ",".join(f for f, _ in devices)
        names = ",".join(n for _, n in devices)
        proc = subprocess.Popen(
            [HEBI_CLI, "imitation", "-f", families, "-n", names],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        procs.append(proc)

        by_family: dict[str, list[str]] = {}
        for family, name in devices:
            by_family.setdefault(family, []).append(name)

        groups = {}
        deadline = time.time() + timeout_s
        for family, fam_names in by_family.items():
            while (group := lookup.get_group_from_names([family], fam_names, timeout_ms=500)) is None:
                if proc.poll() is not None:
                    pytest.fail(f"`hebi imitation` exited early (code {proc.returncode})")
                if time.time() > deadline:
                    pytest.fail(f"imitated devices {family}/{fam_names} never appeared")
            groups[family] = group
        return groups

    yield start

    for proc in procs:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    # Let the UDP port free up before the next test starts another process.
    time.sleep(0.5)


class FakeMobileIO:
    """Stands in for a phone: settable buttons/axes, records every call made to it."""

    def __init__(self):
        self.buttons = {i: 0 for i in range(1, 9)}
        self.prev_buttons = dict(self.buttons)
        self.axes = {i: 0.0 for i in range(1, 9)}
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        self.calls: list[tuple] = []
        self.call_threads: list[str] = []  # thread name of each call in self.calls

    def press(self, **buttons):
        """Set button states, e.g. ``press(b1=1, b2=1)``; diffs are relative to the previous sample."""
        self.prev_buttons = dict(self.buttons)
        for key, value in buttons.items():
            self.buttons[int(key[1:])] = value

    def update(self, timeout_ms=0):
        return True

    def get_button_state(self, i):
        return self.buttons[i]

    def get_button_diff(self, i):
        return self.buttons[i] - self.prev_buttons[i]

    def get_axis_state(self, i):
        return self.axes[i]

    def __getattr__(self, name):
        # set_axis_label, set_snap, send_layout, add_text, ...: record and succeed
        def call(*args, **kwargs):
            self.calls.append((name, args))
            self.call_threads.append(threading.current_thread().name)
            return True

        return call


@pytest.fixture
def fake_mobile_io():
    return FakeMobileIO()
