"""Integration test: C++ robot-control binary talks to Python simplerobot via LCM."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

pytestmark = pytest.mark.interop


def test_cpp_receives_pose_and_publishes_twist(
    simplerobot: subprocess.Popen[str],
    cpp_binary: Path,
) -> None:
    """Run the C++ binary for a few seconds and verify message exchange."""
    try:
        result = subprocess.run(
            [str(cpp_binary)],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except subprocess.TimeoutExpired as e:
        stdout = e.stdout or ""
        stderr = e.stderr or ""
    else:
        stdout = result.stdout
        stderr = result.stderr

    assert "[pose]" in stdout, (
        f"C++ binary never received a PoseStamped.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
    assert "[twist]" in stdout, (
        f"C++ binary never published a Twist.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
