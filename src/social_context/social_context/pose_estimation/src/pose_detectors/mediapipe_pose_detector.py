#!/usr/bin/env python3
"""
MediaPipe-based 2D pose detector.

Uses the Tasks-based PoseLandmarker API, not the legacy mp.solutions.pose
API (mp.solutions.pose.Pose() only ever detects a single person per frame,
regardless of how many are actually in view). PoseLandmarker supports
num_poses > 1 for genuine multi-person detection.

Requires a downloaded pose_landmarker .task model bundle -- unlike the
legacy API, the Tasks API doesn't bundle a model with the pip package.
Get one from MediaPipe's official Pose Landmarker model page (search
"MediaPipe Pose Landmarker" in Google's docs) and point model_asset_path
at it.
"""

import cv2
import time
import mediapipe as mp
import numpy as np
from typing import List
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python import vision

from social_context.pose_estimation.src.pose_detectors.abstract import (
    AbstractPoseDetector,
    PersonDetection
)

# MediaPipe Pose landmark indices
LEFT_HIP = 23
RIGHT_HIP = 24


class MediaPipePoseDetector(AbstractPoseDetector):
    """MediaPipe implementation of 2D human pose detection (multi-person)."""

    def __init__(
            self,
            model_asset_path: str,
            logger=None,
            num_poses: int = 5,
            min_detection_confidence: float = 0.5,
            min_tracking_confidence: float = 0.5,
            min_pose_presence_confidence: float = 0.5,
    ):
        """
        Initialize MediaPipe detector.

        Args:
            model_asset_path: Path to the downloaded pose_landmarker .task
                model bundle.
            logger: Optional ROS logger
            num_poses: Maximum number of people to detect simultaneously
            min_detection_confidence: Minimum confidence for initial detection (0-1)
            min_tracking_confidence: Minimum confidence for tracking (0-1)
            min_pose_presence_confidence: Minimum confidence for pose presence (0-1)
        """
        super().__init__(logger)
        self.model_asset_path = model_asset_path
        self.num_poses = num_poses
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.min_pose_presence_confidence = min_pose_presence_confidence
        self.landmarker = None
        self._start_time = None

    def initialize(self) -> bool:
        """Initialize MediaPipe PoseLandmarker."""
        try:
            self.log_info("Initializing MediaPipe PoseLandmarker...")

            base_options = BaseOptions(model_asset_path=self.model_asset_path)
            options = vision.PoseLandmarkerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.VIDEO,
                num_poses=self.num_poses,
                min_pose_detection_confidence=self.min_detection_confidence,
                min_pose_presence_confidence=self.min_pose_presence_confidence,
                min_tracking_confidence=self.min_tracking_confidence,
            )
            self.landmarker = vision.PoseLandmarker.create_from_options(options)
            # detect_for_video() requires monotonically increasing timestamps;
            # the abstract detect() interface doesn't receive one, so track
            # our own relative to init time.
            self._start_time = time.monotonic()

            self.is_ready = True

            self.log_info(
                f"MediaPipe PoseLandmarker initialized: "
                f"num_poses={self.num_poses}, "
                f"det_conf={self.min_detection_confidence}, "
                f"track_conf={self.min_tracking_confidence}"
            )
            return True

        except Exception as e:
            self.log_error(f"MediaPipe initialization failed: {e}")
            return False

    def detect(self, image: np.ndarray) -> List[PersonDetection]:
        """
        Detect people in image using MediaPipe PoseLandmarker.

        Args:
            image: BGR image as numpy array (H, W, 3)

        Returns:
            List of PersonDetection objects, one per detected person.
        """
        if not self.is_ready:
            self.log_error("Detector not initialized!")
            return []

        try:
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

            timestamp_ms = int((time.monotonic() - self._start_time) * 1000)
            result = self.landmarker.detect_for_video(mp_image, timestamp_ms)

            detections = []
            height, width = image.shape[:2]

            for landmarks in result.pose_landmarks:
                # Get hip center position (average of left and right hip)
                left_hip = landmarks[LEFT_HIP]
                right_hip = landmarks[RIGHT_HIP]

                # Check if both hips are visible enough
                if left_hip.visibility > 0.5 and right_hip.visibility > 0.5:
                    hip_x = ((left_hip.x + right_hip.x) / 2) * width
                    hip_y = ((left_hip.y + right_hip.y) / 2) * height
                    confidence = (left_hip.visibility + right_hip.visibility) / 2

                    detections.append(PersonDetection(
                        pixel_x=hip_x,
                        pixel_y=hip_y,
                        confidence=confidence
                    ))

            return detections

        except Exception as e:
            self.log_error(f"Error during detection: {e}")
            return []

    def cleanup(self):
        """Clean up MediaPipe resources."""
        if self.landmarker:
            self.landmarker.close()
        self.is_ready = False
        self.log_info("MediaPipe cleaned up")

    def get_name(self) -> str:
        return "MediaPipe"
