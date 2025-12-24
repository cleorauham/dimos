# Copyright 2025 Dimensional Inc.
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
Coordinate system transformation utilities for VR to ROS conversion.

WebXR Coordinate System (Right-handed):
- X-axis: Right (+) / Left (-)
- Y-axis: Up (+) / Down (-)
- Z-axis: Toward user (+) / Away from user/into scene (-)

ROS REP-103 Coordinate System (Right-handed):
- X-axis: Forward
- Y-axis: Left
- Z-axis: Up

Transform Mapping:
- ROS X (forward) = -WebXR Z (away from user)
- ROS Y (left) = -WebXR X (left)
- ROS Z (up) = WebXR Y (up)
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


# Pre-compute transformation matrices for performance
_WEBXR_TO_ROS_MATRIX = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
_FRAME_TRANSFORM = R.from_matrix(_WEBXR_TO_ROS_MATRIX)
_FRAME_TRANSFORM_INV = _FRAME_TRANSFORM.inv()


def webxr_to_ros_position(webxr_pos: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Transform position from WebXR to ROS coordinate system.

    Args:
        webxr_pos: Position as (x, y, z) in WebXR frame
                   x=right, y=up, z=toward user

    Returns:
        Position as (x, y, z) in ROS frame
        x=forward, y=left, z=up
    """
    x_webxr, y_webxr, z_webxr = webxr_pos

    # ROS X (forward) = -WebXR Z (away from user = negative toward)
    x_ros = -z_webxr

    # ROS Y (left) = -WebXR X (left = negative right)
    y_ros = -x_webxr

    # ROS Z (up) = WebXR Y (up)
    z_ros = y_webxr

    return (x_ros, y_ros, z_ros)


def webxr_to_ros_quaternion(
    webxr_quat: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """
    Transform quaternion from WebXR to ROS coordinate system.

    Uses similarity transformation: q_new = F * q_old * F^(-1)
    This is a passive transformation (changing coordinate frames).

    Args:
        webxr_quat: Quaternion as (x, y, z, w) in WebXR frame

    Returns:
        Quaternion as (x, y, z, w) in ROS frame
    """
    quat_webxr = R.from_quat(webxr_quat)
    quat_ros = _FRAME_TRANSFORM * quat_webxr * _FRAME_TRANSFORM.inv()
    return tuple(quat_ros.as_quat())


def webxr_to_ros_pose(
    webxr_pos: tuple[float, float, float], webxr_quat: tuple[float, float, float, float]
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """
    Transform complete pose (position + orientation) from WebXR to ROS.

    Args:
        webxr_pos: Position as (x, y, z) in WebXR frame
        webxr_quat: Quaternion as (x, y, z, w) in WebXR frame

    Returns:
        Tuple of (position, quaternion) in ROS frame
        - position: (x, y, z)
        - quaternion: (x, y, z, w)
    """
    ros_pos = webxr_to_ros_position(webxr_pos)
    ros_quat = webxr_to_ros_quaternion(webxr_quat)
    return (ros_pos, ros_quat)


def ros_to_webxr_position(ros_pos: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Transform position from ROS to WebXR coordinate system (inverse transform).

    Args:
        ros_pos: Position as (x, y, z) in ROS frame
                 x=forward, y=left, z=up

    Returns:
        Position as (x, y, z) in WebXR frame
        x=right, y=up, z=toward user
    """
    x_ros, y_ros, z_ros = ros_pos

    # WebXR X (right) = -ROS Y (negative left = right)
    x_webxr = -y_ros

    # WebXR Y (up) = ROS Z (up)
    y_webxr = z_ros

    # WebXR Z (toward user) = -ROS X (negative forward = toward)
    z_webxr = -x_ros

    return (x_webxr, y_webxr, z_webxr)


def ros_to_webxr_quaternion(
    ros_quat: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """
    Transform quaternion from ROS to WebXR coordinate system (inverse transform).

    Uses similarity transformation: q_new = F^(-1) * q_old * F
    This is a passive transformation (changing coordinate frames).

    Args:
        ros_quat: Quaternion as (x, y, z, w) in ROS frame

    Returns:
        Quaternion as (x, y, z, w) in WebXR frame
    """
    quat_ros = R.from_quat(ros_quat)
    quat_webxr = _FRAME_TRANSFORM_INV * quat_ros * _FRAME_TRANSFORM
    return tuple(quat_webxr.as_quat())


def ros_to_webxr_pose(
    ros_pos: tuple[float, float, float], ros_quat: tuple[float, float, float, float]
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """
    Transform complete pose (position + orientation) from ROS to WebXR.

    Args:
        ros_pos: Position as (x, y, z) in ROS frame
        ros_quat: Quaternion as (x, y, z, w) in ROS frame

    Returns:
        Tuple of (position, quaternion) in WebXR frame
    """
    webxr_pos = ros_to_webxr_position(ros_pos)
    webxr_quat = ros_to_webxr_quaternion(ros_quat)
    return (webxr_pos, webxr_quat)
