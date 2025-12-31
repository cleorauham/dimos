#!/usr/bin/env python3
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

"""Integration test for ObjectTracker3D with ZED camera.

Usage:
- Click and drag to draw a bounding box around an object to track
- Press 'r' to reset tracking
- Press 'q' to quit
"""

import queue
import time

import cv2
import numpy as np
from dimos_lcm.foxglove_msgs import ImageAnnotations, SceneUpdate

from dimos.core import LCMTransport, start
from dimos.hardware.camera.zed.camera import ZEDModule
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.msgs.vision_msgs import Detection3D
from dimos.perception.object_tracker_3d import ObjectTracker3D
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


mouse_state = {
    "start_point": None,
    "current_point": None,
    "drawing": False,
    "new_bbox": None,
}


def mouse_callback(event, x, y, _flags, _param):
    """Handle mouse events for drawing bounding boxes."""
    global mouse_state

    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_state["start_point"] = (x, y)
        mouse_state["drawing"] = True
        mouse_state["current_point"] = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE:
        if mouse_state["drawing"]:
            mouse_state["current_point"] = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        mouse_state["drawing"] = False
        start_x, start_y = mouse_state["start_point"]

        x1, y1 = min(start_x, x), min(start_y, y)
        x2, y2 = max(start_x, x), max(start_y, y)

        if abs(x2 - x1) > 5 and abs(y2 - y1) > 5:
            mouse_state["new_bbox"] = (x1, y1, x2, y2)


def draw_overlay(image_cv, mouse_info, tracking_active=False, has_detection=False):
    """Draw tracking status and mouse interaction overlay."""
    if tracking_active:
        status_text = "Tracking Active"
        color = (0, 255, 0) if has_detection else (0, 165, 255)
    else:
        status_text = "Click and drag to start tracking"
        color = (200, 200, 200)

    cv2.putText(image_cv, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    if mouse_info["drawing"] and mouse_info["start_point"]:
        start = mouse_info["start_point"]
        curr = mouse_info["current_point"]
        cv2.rectangle(image_cv, start, curr, (0, 0, 255), 2)


def visualize_annotations(image_cv, annotations: ImageAnnotations):
    """Visualize ImageAnnotations on the image."""
    if not annotations:
        return

    # Draw point annotations (contours, bboxes, etc.)
    for point_ann in annotations.points:
        if point_ann.points_length == 0:
            continue
        
        # Convert points to numpy array for OpenCV
        points = np.array([[int(p.x), int(p.y)] for p in point_ann.points], dtype=np.int32)
        
        # Extract color
        outline_color = (
            int(point_ann.outline_color.b * 255),
            int(point_ann.outline_color.g * 255),
            int(point_ann.outline_color.r * 255),
        )
        
        fill_color = (
            int(point_ann.fill_color.b * 255),
            int(point_ann.fill_color.g * 255),
            int(point_ann.fill_color.r * 255),
        )
        
        # Draw filled polygon if fill color has alpha
        if point_ann.fill_color.a > 0:
            overlay = image_cv.copy()
            cv2.fillPoly(overlay, [points], fill_color)
            alpha = point_ann.fill_color.a
            cv2.addWeighted(overlay, alpha, image_cv, 1 - alpha, 0, image_cv)
        
        # Draw outline
        thickness = int(point_ann.thickness) if point_ann.thickness > 0 else 2
        if point_ann.type == point_ann.LINE_LOOP:
            cv2.polylines(image_cv, [points], isClosed=True, color=outline_color, thickness=thickness)
        elif point_ann.type == point_ann.LINE_STRIP:
            cv2.polylines(image_cv, [points], isClosed=False, color=outline_color, thickness=thickness)
    
    # Draw text annotations
    for text_ann in annotations.texts:
        position = (int(text_ann.position.x), int(text_ann.position.y))
        color = (
            int(text_ann.text_color.b * 255),
            int(text_ann.text_color.g * 255),
            int(text_ann.text_color.r * 255),
        )
        font_size = text_ann.font_size if text_ann.font_size > 0 else 0.5
        cv2.putText(image_cv, text_ann.text, position, cv2.FONT_HERSHEY_SIMPLEX, font_size, color, 2)




def main():
    """Main integration test."""
    logger.info("Starting ObjectTracker3D integration test")
    from dimos.protocol import pubsub
    pubsub.lcm.autoconf()

    dimos = start(6)

    zed = dimos.deploy(
        ZEDModule,
        camera_id=0,
        resolution="HD720",
        depth_mode="NEURAL",
        fps=15,
        enable_tracking=False,
        publish_rate=15.0,
        frame_id="zed_camera",
    )

    tracker = dimos.deploy(ObjectTracker3D, tracking_timeout=10.0)

    bridge = dimos.deploy(FoxgloveBridge)

    zed.color_image.transport = LCMTransport("/zed/color_image", Image)
    zed.depth_image.transport = LCMTransport("/zed/depth_image", Image)
    zed.camera_info.transport = LCMTransport("/zed/camera_info", CameraInfo)

    tracker.color_image.connect(zed.color_image)
    tracker.depth_image.connect(zed.depth_image)
    tracker.camera_info.connect(zed.camera_info)

    tracker.annotations.transport = LCMTransport("/tracker3d/annotations", ImageAnnotations)
    tracker.detection3d.transport = LCMTransport("/tracker3d/detection3d", Detection3D)

    zed.start()
    tracker.start()
    bridge.start()

    color_queue = queue.Queue(maxsize=2)
    annotations_queue = queue.Queue(maxsize=2)

    def color_handler(msg):
        if not color_queue.full():
            color_queue.put(msg)

    def annotations_handler(annotations):
        if not annotations_queue.full():
            annotations_queue.put(annotations)

    color_transport = LCMTransport("/zed/color_image", Image)
    annotations_transport = LCMTransport("/tracker3d/annotations", ImageAnnotations)

    color_transport.subscribe(color_handler)
    annotations_transport.subscribe(annotations_handler)

    window_name = "ObjectTracker3D - ZED Integration Test"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    logger.info("Starting interactive visualization")
    print("\nControls:")
    print("- Click and drag to draw a bounding box around an object to track")
    print("- Press 'r' to reset tracking")
    print("- Press 'q' to quit")

    tracking_active = False
    last_annotations = None
    last_image = None

    while True:
        while not color_queue.empty():
            last_image = color_queue.get_nowait()

        while not annotations_queue.empty():
            last_annotations = annotations_queue.get_nowait()

        if last_image is None:
            time.sleep(0.01)
            continue

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('r'):
            logger.info("Resetting tracking")
            tracker.stop()
            time.sleep(0.5)
            tracker.start()
            tracking_active = False
            last_annotations = None
            mouse_state["new_bbox"] = None
            print("Tracking reset")

        if mouse_state["new_bbox"] and not tracking_active:
            x1, y1, x2, y2 = mouse_state["new_bbox"]
            mouse_state["new_bbox"] = None
            logger.info(f"Initializing tracking with bbox: ({x1}, {y1}, {x2}, {y2})")
            bbox = (float(x1), float(y1), float(x2), float(y2))
            tracker.track(bbox)
            tracking_active = True
            print("Tracking initialized")

        if tracking_active:
            tracking_active = tracker.is_tracking()
            if not tracking_active:
                logger.warning("Tracking lost")
                print("Tracking lost - draw a new bbox to restart")

        image_cv = last_image.to_opencv()
        draw_overlay(image_cv, mouse_state, tracking_active=tracking_active, has_detection=(last_annotations is not None))

        if tracking_active and last_annotations:
            visualize_annotations(image_cv, last_annotations)

        cv2.imshow(window_name, image_cv)

    logger.info("Cleaning up")
    cv2.destroyAllWindows()
    tracker.stop()
    zed.stop()
    bridge.stop()
    logger.info("Test completed")


if __name__ == "__main__":
    main()
