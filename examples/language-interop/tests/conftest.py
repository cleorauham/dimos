"""Fixtures for language-interop integration tests."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Generator

import pytest

EXAMPLES_DIR = Path(__file__).resolve().parent.parent  # language-interop/
SIMPLEROBOT_DIR = EXAMPLES_DIR.parent / "simplerobot"
RUST_DIR = EXAMPLES_DIR / "rust"
TS_DIR = EXAMPLES_DIR / "ts"
CPP_DIR = EXAMPLES_DIR / "cpp"


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line("markers", "interop: cross-language interop integration test")


@pytest.fixture(scope="module")
def simplerobot() -> Generator[subprocess.Popen[str], None, None]:
    """Start simplerobot.py --headless as a subprocess, tear down after tests."""
    proc = subprocess.Popen(
        [sys.executable, str(SIMPLEROBOT_DIR / "simplerobot.py"), "--headless"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
        cwd=str(SIMPLEROBOT_DIR),
    )
    # Give it time to start publishing
    time.sleep(2)
    yield proc
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


@pytest.fixture(scope="module")
def rust_binary() -> Path:
    """Build the Rust interop binary and return its path."""
    cargo_toml = RUST_DIR / "Cargo.toml"
    if not cargo_toml.exists():
        pytest.skip("Rust example not found")

    env = os.environ.copy()
    cargo_home = Path.home() / ".cargo" / "bin"
    if cargo_home.is_dir():
        env["PATH"] = str(cargo_home) + os.pathsep + env.get("PATH", "")

    result = subprocess.run(
        ["cargo", "build", "--release"],
        cwd=str(RUST_DIR),
        capture_output=True,
        text=True,
        timeout=120,
        env=env,
    )
    if result.returncode != 0:
        pytest.skip(f"cargo build failed: {result.stderr}")

    # Binary name from Cargo.toml [package] name = "robot-control"
    binary = RUST_DIR / "target" / "release" / "robot-control"
    if not binary.exists():
        # Try to find any binary in release
        release_dir = RUST_DIR / "target" / "release"
        found = next(
            (
                f
                for f in release_dir.iterdir()
                if f.is_file()
                and os.access(f, os.X_OK)
                and not f.suffix
                and ".so" not in f.name
            ),
            None,
        )
        if found is None:
            pytest.skip("No binary found after cargo build")
        binary = found

    return binary


@pytest.fixture(scope="module")
def cpp_binary() -> Path:
    """Build the C++ interop binary and return its path."""
    cmakelists = CPP_DIR / "CMakeLists.txt"
    if not cmakelists.exists():
        pytest.skip("C++ example not found")

    build_dir = CPP_DIR / "build"
    build_dir.mkdir(exist_ok=True)

    cmake_result = subprocess.run(
        ["cmake", ".."],
        cwd=str(build_dir),
        capture_output=True,
        text=True,
        timeout=30,
    )
    if cmake_result.returncode != 0:
        pytest.skip(f"cmake failed: {cmake_result.stderr}")

    make_result = subprocess.run(
        ["make", "-j4"],
        cwd=str(build_dir),
        capture_output=True,
        text=True,
        timeout=60,
    )
    if make_result.returncode != 0:
        pytest.skip(f"make failed: {make_result.stderr}")

    # Find the built binary
    binary = next(
        (
            f
            for f in build_dir.iterdir()
            if f.is_file()
            and os.access(f, os.X_OK)
            and not f.suffix
            and ".so" not in f.name
        ),
        None,
    )
    if binary is None:
        pytest.skip("No C++ binary found after build")
    return binary
