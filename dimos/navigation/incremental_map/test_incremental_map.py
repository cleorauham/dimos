# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for IncrementalMap module (Phase 1)."""

from __future__ import annotations

import pickle

import numpy as np

from dimos.navigation.incremental_map.module import (
    IncrementalMapConfig,
    _icp_verify,
    _IncrementalMapCore,
    _voxel_downsample,
)

# ─── Config defaults ──────────────────────────────────────────────────────────


def test_config_defaults():
    cfg = IncrementalMapConfig()
    assert cfg.voxel_size == 0.15
    assert cfg.key_trans == 0.5
    assert cfg.key_deg == 10.0
    assert cfg.loop_search_radius == 3.0
    assert cfg.loop_time_thresh == 10.0
    assert cfg.loop_score_thresh == 0.5
    assert cfg.map_publish_rate == 0.5


def test_config_custom():
    cfg = IncrementalMapConfig(voxel_size=0.05, key_trans=1.0, loop_search_radius=5.0)
    assert cfg.voxel_size == 0.05
    assert cfg.key_trans == 1.0
    assert cfg.loop_search_radius == 5.0


# ─── _voxel_downsample ───────────────────────────────────────────────────────


def test_voxel_downsample_empty():
    pts = np.empty((0, 3))
    result = _voxel_downsample(pts, 0.1)
    assert len(result) == 0


def test_voxel_downsample_reduces_points():
    # Create a dense cloud with many points in the same voxel
    rng = np.random.default_rng(42)
    pts = rng.uniform(0, 1.0, (1000, 3))
    downsampled = _voxel_downsample(pts, 0.1)
    assert len(downsampled) < len(pts)
    assert len(downsampled) > 0


def test_voxel_downsample_preserves_spread():
    # Points far apart should all survive
    pts = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]], dtype=np.float32)
    result = _voxel_downsample(pts, 0.1)
    assert len(result) == 3


# ─── _icp_verify ─────────────────────────────────────────────────────────────


def test_icp_empty_source():
    T, fitness = _icp_verify(np.empty((0, 3)), np.ones((10, 3)))
    assert fitness == float("inf")


def test_icp_empty_target():
    T, fitness = _icp_verify(np.ones((10, 3)), np.empty((0, 3)))
    assert fitness == float("inf")


def test_icp_identity_convergence():
    """ICP on identical clouds should return near-identity transform with low fitness."""
    rng = np.random.default_rng(0)
    pts = rng.uniform(0, 5, (200, 3)).astype(np.float32)
    T, fitness = _icp_verify(pts, pts, max_iter=30, max_dist=10.0)
    assert fitness < 0.01


def test_icp_small_translation():
    """ICP should align a slightly translated cloud."""
    rng = np.random.default_rng(1)
    pts = rng.uniform(0, 5, (200, 3)).astype(np.float32)
    shifted = pts + np.array([0.1, 0.1, 0.0])
    T, fitness = _icp_verify(shifted, pts, max_iter=50, max_dist=2.0)
    assert fitness < 0.1


# ─── _IncrementalMapCore: basic scan insertion ───────────────────────────────


def _make_box_cloud(center: np.ndarray, n: int = 100) -> np.ndarray:
    """Generate a simple box-shaped cloud around a center point."""
    rng = np.random.default_rng(int(center.sum() * 1000) % (2**31))
    pts = rng.uniform(-1.0, 1.0, (n, 3)).astype(np.float32) + center
    return pts


def test_core_single_scan_insertion():
    cfg = IncrementalMapConfig(voxel_size=0.1, key_trans=0.5, key_deg=10.0)
    core = _IncrementalMapCore(cfg)

    r = np.eye(3)
    t = np.zeros(3)
    cloud = _make_box_cloud(np.zeros(3))

    added = core.add_scan(r, t, cloud, timestamp=0.0)
    assert added, "First scan should always be added as keyframe"
    assert core.num_keyframes == 1


def test_core_keyframe_rejection_small_motion():
    cfg = IncrementalMapConfig(voxel_size=0.1, key_trans=0.5, key_deg=10.0)
    core = _IncrementalMapCore(cfg)

    r = np.eye(3)
    cloud = _make_box_cloud(np.zeros(3))

    core.add_scan(r, np.zeros(3), cloud, timestamp=0.0)

    # Move only 0.1m (below key_trans threshold)
    added = core.add_scan(r, np.array([0.1, 0.0, 0.0]), cloud, timestamp=0.5)
    assert not added, "Sub-threshold motion should not create a keyframe"
    assert core.num_keyframes == 1


def test_core_keyframe_accepted_sufficient_motion():
    cfg = IncrementalMapConfig(voxel_size=0.1, key_trans=0.5, key_deg=10.0)
    core = _IncrementalMapCore(cfg)

    r = np.eye(3)
    cloud = _make_box_cloud(np.zeros(3))

    core.add_scan(r, np.zeros(3), cloud, timestamp=0.0)
    added = core.add_scan(r, np.array([1.0, 0.0, 0.0]), cloud, timestamp=1.0)
    assert added
    assert core.num_keyframes == 2


def test_core_multiple_scans_build_map():
    cfg = IncrementalMapConfig(voxel_size=0.1, key_trans=0.5, key_deg=10.0)
    core = _IncrementalMapCore(cfg)

    r = np.eye(3)
    # Drive in a line, adding 5 keyframes
    for i in range(5):
        t = np.array([float(i) * 1.0, 0.0, 0.0])
        cloud = _make_box_cloud(t)
        core.add_scan(r, t, cloud, timestamp=float(i))

    assert core.num_keyframes == 5

    global_map = core.build_global_map()
    assert len(global_map) > 0, "Global map should have points after multiple scans"


# ─── Loop closure detection ──────────────────────────────────────────────────


def _square_trajectory(side: float = 5.0, steps_per_side: int = 10) -> list[np.ndarray]:
    """Generate positions along a square loop: (0,0)→(side,0)→(side,side)→(0,side)→(0,0)."""
    positions = []
    # Side 1: x 0→side
    for i in range(steps_per_side):
        positions.append(np.array([i * side / steps_per_side, 0.0, 0.0]))
    # Side 2: y 0→side
    for i in range(steps_per_side):
        positions.append(np.array([side, i * side / steps_per_side, 0.0]))
    # Side 3: x side→0
    for i in range(steps_per_side):
        positions.append(np.array([side - i * side / steps_per_side, side, 0.0]))
    # Side 4: y side→0 (returns to origin)
    for i in range(steps_per_side):
        positions.append(np.array([0.0, side - i * side / steps_per_side, 0.0]))
    # Return to start
    positions.append(np.array([0.0, 0.0, 0.0]))
    return positions


def test_core_loop_closure_detected():
    """Robot drives a square and loop closure should fire when it returns to start."""
    cfg = IncrementalMapConfig(
        voxel_size=0.2,
        key_trans=0.5,
        key_deg=10.0,
        loop_search_radius=3.0,
        loop_time_thresh=5.0,  # seconds
        loop_score_thresh=2.0,  # lenient for synthetic data
        loop_submap_half_range=2,
        icp_max_iter=20,
        icp_max_dist=5.0,
        min_loop_detect_duration=3.0,
    )
    core = _IncrementalMapCore(cfg)

    positions = _square_trajectory(side=5.0, steps_per_side=15)
    r = np.eye(3)
    t0 = 0.0
    dt = 1.0  # 1 second per step → total > loop_time_thresh

    loop_detected = False
    for i, pos in enumerate(positions):
        ts = t0 + i * dt
        cloud = _make_box_cloud(pos, n=200)
        added = core.add_scan(r, pos, cloud, timestamp=ts)
        if added:
            detected = core.detect_and_correct_loop()
            if detected:
                loop_detected = True
                break

    assert loop_detected, (
        f"Loop closure should be detected when robot returns to start. "
        f"num_keyframes={core.num_keyframes}, loop_count={core.loop_count}"
    )


def test_core_loop_closure_corrects_map():
    """After loop closure, the map should be geometrically consistent."""
    cfg = IncrementalMapConfig(
        voxel_size=0.2,
        key_trans=0.5,
        key_deg=10.0,
        loop_search_radius=3.0,
        loop_time_thresh=5.0,
        loop_score_thresh=2.0,
        loop_submap_half_range=2,
        icp_max_iter=20,
        icp_max_dist=5.0,
        min_loop_detect_duration=3.0,
    )
    core = _IncrementalMapCore(cfg)

    positions = _square_trajectory(side=5.0, steps_per_side=15)
    r = np.eye(3)

    for i, pos in enumerate(positions):
        ts = float(i)
        cloud = _make_box_cloud(pos, n=200)
        core.add_scan(r, pos, cloud, timestamp=ts)
        core.detect_and_correct_loop()

    # Map should have points near the starting position (0, 0, 0)
    global_map = core.build_global_map()
    assert len(global_map) > 0

    # Find points within 3m of origin
    dists = np.linalg.norm(global_map[:, :2], axis=1)
    near_origin = np.sum(dists < 3.0)
    assert near_origin > 0, "Global map should have points near origin after loop closure"


def test_core_no_spurious_loop_without_revisit():
    """No loop closure should be detected during a straight line trajectory."""
    cfg = IncrementalMapConfig(
        voxel_size=0.2,
        key_trans=0.5,
        key_deg=10.0,
        loop_search_radius=2.0,
        loop_time_thresh=5.0,
        loop_score_thresh=0.5,
        min_loop_detect_duration=1.0,
    )
    core = _IncrementalMapCore(cfg)

    r = np.eye(3)
    # Drive 20m in straight line
    for i in range(40):
        t = np.array([float(i) * 0.5, 0.0, 0.0])
        cloud = _make_box_cloud(t, n=100)
        added = core.add_scan(r, t, timestamp=float(i), body_cloud=cloud)
        if added:
            core.detect_and_correct_loop()

    assert core.loop_count == 0, "No loop closures on a straight-line trajectory"


# ─── Corrected odometry drift ─────────────────────────────────────────────────


def test_corrected_odom_get_corrected_pose():
    """get_corrected_pose should apply the correction offset to raw odom."""
    cfg = IncrementalMapConfig(
        voxel_size=0.2,
        key_trans=0.4,
        key_deg=10.0,
        loop_search_radius=3.0,
        loop_time_thresh=5.0,
        loop_score_thresh=2.0,
        loop_submap_half_range=2,
        icp_max_iter=20,
        icp_max_dist=5.0,
        min_loop_detect_duration=3.0,
    )
    core = _IncrementalMapCore(cfg)

    # Without any loop closure, get_corrected_pose is identity
    r = np.eye(3)
    t = np.array([3.0, 4.0, 0.0])
    r_corr, t_corr = core.get_corrected_pose(r, t)
    np.testing.assert_allclose(t_corr, t, atol=1e-9)

    # After simulating a correction offset, the corrected pose should differ
    core._t_offset = np.array([1.0, 2.0, 0.0])
    r_corr2, t_corr2 = core.get_corrected_pose(r, t)
    expected = t + np.array([1.0, 2.0, 0.0])
    np.testing.assert_allclose(t_corr2, expected, atol=1e-9)


# ─── Pickle / unpickle ───────────────────────────────────────────────────────


def test_incremental_map_module_pickle():
    """IncrementalMap module should survive pickle/unpickle (needed for forkserver workers)."""
    from dimos.navigation.incremental_map.module import IncrementalMap

    mod = IncrementalMap()
    try:
        data = pickle.dumps(mod)
        mod2 = pickle.loads(data)
        assert isinstance(mod2, IncrementalMap)
        assert mod2.config.voxel_size == mod.config.voxel_size
    finally:
        mod.stop()
        mod2.stop()  # type: ignore[possibly-undefined]
