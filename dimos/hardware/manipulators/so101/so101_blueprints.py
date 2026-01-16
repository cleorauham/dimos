# Copyright 2025-2026 Dimensional Inc.
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

"""
Blueprints for SO101 manipulator control systems.

This module provides declarative blueprints for configuring SO101 servo control,
following the same pattern used for Piper and other manipulators.

Usage:
    # Run via CLI:
    dimos run so101-servo           # Driver only
    dimos run so101-cartesian       # Driver + Cartesian motion controller
    dimos run so101-trajectory      # Driver + Joint trajectory controller

    # Or programmatically:
    from dimos.hardware.manipulators.so101.so101_blueprints import so101_servo
    coordinator = so101_servo.build()
    coordinator.loop()
"""

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.manipulators.so101.so101_driver import so101_driver as so101_driver_blueprint
from dimos.manipulation.control import cartesian_motion_controller, joint_trajectory_controller
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import (
    JointCommand,
    JointState,
    RobotState,
)
from dimos.msgs.trajectory_msgs import JointTrajectory


# Create a blueprint wrapper for the component-based driver
def so101_driver(**config: Any) -> Any:
    """Create a blueprint for SO101Driver.

    Args:
        **config: Configuration parameters passed to SO101Driver
            - port: USB port id (default: "/dev/ttyACM0")
            - has_gripper: Whether gripper is attached (default: True)
            - calibration_path: Path of SO101 calibration file (default: None)
            - enable_on_start: Whether to enable servos on start (default: True)
            - control_rate: Control loop + joint feedback rate in Hz (default: 100)
            - monitor_rate: Robot state monitoring rate in Hz (default: 10)

    Returns:
        Blueprint configuration for SO101Driver
    """
    # Set defaults
    config.setdefault("port", "/dev/ttyACM0")
    config.setdefault("has_gripper", True)
    config.setdefault("calibration_path", None)
    config.setdefault("enable_on_start", True)
    config.setdefault("control_rate", 100)
    config.setdefault("monitor_rate", 10)

    # Return the so101_driver blueprint with the config
    return so101_driver_blueprint(**config)


# =============================================================================
# SO101 Servo Control Blueprint
# =============================================================================
# SO101Driver configured for servo control mode using component-based architecture.
# Publishes joint states and robot state, listens for joint commands.
# =============================================================================

so101_servo = so101_driver(
    port="/dev/ttyACM0",
    has_gripper=True,
    enable_on_start=True,
    control_rate=100,
    monitor_rate=10,
).transports(
    {
        # Joint state feedback (position, velocity, effort)
        ("joint_state", JointState): LCMTransport("/so101/joint_states", JointState),
        # Robot state feedback (mode, state, errors)
        ("robot_state", RobotState): LCMTransport("/so101/robot_state", RobotState),
        # Position commands input
        ("joint_position_command", JointCommand): LCMTransport(
            "/so101/joint_position_command", JointCommand
        ),
        # Velocity commands input
        ("joint_velocity_command", JointCommand): LCMTransport(
            "/so101/joint_velocity_command", JointCommand
        ),
    }
)

# =============================================================================
# SO101 Cartesian Control Blueprint (Driver + Controller)
# =============================================================================
# Combines SO101Driver with CartesianMotionController for Cartesian space control.
# The controller receives target_pose and converts to joint commands via IK.
# =============================================================================

so101_cartesian = autoconnect(
    so101_driver(
        port="/dev/ttyACM0",
        has_gripper=True,
        enable_on_start=True,
        control_rate=100,
        monitor_rate=10,
    ),
    cartesian_motion_controller(
        control_frequency=20.0,
        position_kp=5.0,
        position_ki=0.0,
        position_kd=0.1,
        max_linear_velocity=0.2,
        max_angular_velocity=1.0,
    ),
).transports(
    {
        # Shared topics between driver and controller
        ("joint_state", JointState): LCMTransport("/so101/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/so101/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/so101/joint_position_command", JointCommand
        ),
        # Controller-specific topics
        ("target_pose", PoseStamped): LCMTransport("/target_pose", PoseStamped),
        ("current_pose", PoseStamped): LCMTransport("/so101/current_pose", PoseStamped),
    }
)

# =============================================================================
# SO101 Trajectory Control Blueprint (Driver + Trajectory Controller)
# =============================================================================
# Combines SO101Driver with JointTrajectoryController for trajectory execution.
# The controller receives JointTrajectory messages and executes them at 100Hz.
# =============================================================================

so101_trajectory = autoconnect(
    so101_driver(
        port="/dev/ttyACM0",
        has_gripper=True,
        enable_on_start=True,
        control_rate=100,
        monitor_rate=10,
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        # Shared topics between driver and controller
        ("joint_state", JointState): LCMTransport("/so101/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/so101/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/so101/joint_position_command", JointCommand
        ),
        # Trajectory input topic
        ("trajectory", JointTrajectory): LCMTransport("/trajectory", JointTrajectory),
    }
)

__all__ = ["so101_cartesian", "so101_servo", "so101_trajectory"]
