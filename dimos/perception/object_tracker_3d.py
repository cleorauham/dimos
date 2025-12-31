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

from __future__ import annotations

import threading
import time

import numpy as np
from dimos_lcm.foxglove_msgs import SceneUpdate
from reactivex.disposable import Disposable

from dimos_lcm.foxglove_msgs import ImageAnnotations

from dimos.core import In, Module, Out, rpc
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.msgs.vision_msgs import Detection3D
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.types.timestamped import align_timestamped
from dimos.utils.gpu_utils import is_cuda_available
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class ObjectTracker3D(Module):
    """3D Object Tracker using EdgeTAM for segmentation/tracking."""

    # Inputs
    color_image: In[Image] = None  # type: ignore
    depth_image: In[Image] = None  # type: ignore
    camera_info: In[CameraInfo] = None  # type: ignore

    # Outputs
    annotations: Out[ImageAnnotations] = None  # type: ignore
    detection3d: Out[Detection3D] = None  # type: ignore
    scene_update: Out[SceneUpdate] = None  # type: ignore

    def __init__(self, tracking_timeout: float = 2.0, **kwargs) -> None:
        super().__init__(**kwargs)
        self.processor: EdgeTAMProcessor | None = None
        self._is_tracking = False
        self._running = False
        self._pending_bbox: tuple[float, float, float, float] | None = None
        self._initializing = False
        self.tracking_timeout = tracking_timeout
        self.last_valid_tracking_time = 0.0
        self.current_detection: Detection3DPC | None = None
        self.camera_info_value: CameraInfo | None = None
        self._frame_lock = threading.Lock()
        self._disposables: set[Disposable] = set()

    @rpc
    def start(self) -> None:
        super().start()

        self.processor = EdgeTAMProcessor(
            device="cuda" if is_cuda_available() else "cpu"
        )
        logger.info(f"EdgeTAM initialized on {self.processor.device}")

        def on_camera_info(camera_info_msg: CameraInfo) -> None:
            self.camera_info_value = camera_info_msg

        unsub = self.camera_info.subscribe(on_camera_info)
        self._disposables.add(Disposable(unsub))

        def on_aligned_frames(frames_tuple) -> None:
            color, depth = frames_tuple
            detections_2d, detection_3d = self._process_frame(color, depth)
            self._publish_detections((detections_2d, detection_3d))
        
        aligned_stream = align_timestamped(
            self.color_image.observable(),
            self.depth_image.observable(),
            match_tolerance=0.1,
            buffer_size=10.0,
        )
        
        unsub = aligned_stream.subscribe(
            on_next=on_aligned_frames,
            on_error=lambda e: logger.error(f"Error in aligned stream: {e}")
        )
        self._disposables.add(unsub)

        self._running = True
        threading.Thread(target=self._scene_thread, daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._running = False
        self._is_tracking = False
        self._initializing = False
        self._pending_bbox = None
        
        for disposable in self._disposables:
            disposable.dispose()
        self._disposables.clear()
        
        if self.processor:
            self.processor.stop()
        super().stop()

    @rpc
    def track(self, bbox: tuple[float, float, float, float]) -> None:
        """Initialize tracking with a 2D bounding box."""
        if not self.processor:
            logger.warning("Tracker not initialized")
            return

        logger.info(f"Initializing tracking with bbox: {bbox}")
        self._pending_bbox = bbox
        self._initializing = True
        self._is_tracking = True

    @rpc
    def is_tracking(self) -> bool:
        return self._is_tracking or self._initializing

    def _process_frame(self, color: Image, depth: Image) -> tuple[ImageDetections2D | None, Detection3DPC | None]:
        if self._pending_bbox and self._initializing:
            box = np.array(self._pending_bbox, dtype=np.float32)
            self.processor.init_track(image=color, box=box, obj_id=1)
            self.last_valid_tracking_time = time.time()
            self._pending_bbox = None
            self._initializing = False
            logger.info("Tracking initialized")

        if not self.processor or not self._is_tracking:
            return None, None

        detections_2d = self.processor.process_image(color)

        if not detections_2d or not detections_2d.detections:
            if time.time() - self.last_valid_tracking_time > self.tracking_timeout:
                logger.warning(f"Tracking lost for {self.tracking_timeout}s")
                self._is_tracking = False
                self._initializing = False
                self.processor.stop()
            return None, None

        self.last_valid_tracking_time = time.time()

        if not self.camera_info_value:
            return detections_2d, None

        img_detections_3d = Detection3DPC.from_2d_depth(
            detections_2d=detections_2d,
            color_image=color,
            depth_image=depth,
            camera_info=self.camera_info_value,
            filters=[],
        )

        if not img_detections_3d.detections:
            return detections_2d, None

        det_pc = img_detections_3d.detections[0]
        self.current_detection = det_pc
        return detections_2d, det_pc

    def _publish_detections(self, detections: tuple[ImageDetections2D | None, Detection3DPC | None]) -> None:
        detections_2d, detection_3d = detections
        
        if detections_2d:
            self.annotations.publish(detections_2d.to_foxglove_annotations())
        
        if detection_3d:
            msg = detection_3d.to_ros_detection3d()
            self.detection3d.publish(msg)

    def _scene_thread(self) -> None:
        while self._running:
            if hasattr(self.scene_update, "_transport") and self.scene_update._transport is not None:
                scene_update = self._to_foxglove_scene_update()
                self.scene_update.publish(scene_update)
            time.sleep(1.0)

    def _to_foxglove_scene_update(self) -> SceneUpdate:
        scene_update = SceneUpdate()
        scene_update.deletions_length = 0
        scene_update.deletions = []
        scene_update.entities = []

        if self.current_detection is not None:
            entity = self.current_detection.to_foxglove_scene_entity(
                entity_id="tracked_object_3d"
            )
            scene_update.entities.append(entity)

        scene_update.entities_length = len(scene_update.entities)
        return scene_update


object_tracker_3d = ObjectTracker3D.blueprint

__all__ = ["ObjectTracker3D", "object_tracker_3d"]
