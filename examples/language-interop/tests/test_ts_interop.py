"""Integration test: TypeScript (Deno) robot-control talks to Python simplerobot via LCM."""

from __future__ import annotations

import shutil
import subprocess

import pytest

from .conftest import TS_DIR

pytestmark = pytest.mark.interop


@pytest.fixture(scope="module")
def deno_available() -> None:
    if shutil.which("deno") is None:
        pytest.skip("deno not found on PATH")


def test_ts_receives_pose_and_publishes_twist(
    simplerobot: subprocess.Popen[str],
    deno_available: None,
) -> None:
    """Run the Deno TS script for a few seconds and verify message exchange."""
    try:
        result = subprocess.run(
            ["deno", "run", "--allow-net", "--unstable-net", str(TS_DIR / "main.ts")],
            capture_output=True,
            text=True,
            timeout=5,
            cwd=str(TS_DIR),
        )
    except subprocess.TimeoutExpired as e:
        stdout = e.stdout or ""
        stderr = e.stderr or ""
    else:
        stdout = result.stdout
        stderr = result.stderr

    assert "[pose]" in stdout, (
        f"TS script never received a PoseStamped.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
    assert "[twist]" in stdout, (
        f"TS script never published a Twist.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
