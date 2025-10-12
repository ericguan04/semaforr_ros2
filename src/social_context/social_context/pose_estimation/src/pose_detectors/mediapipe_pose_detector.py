#!/usr/bin/env python3
"""
MediaPipe-based 2D pose detector.
"""

import cv2
import mediapipe as mp
import numpy as np
from typing import List

from social_context.pose_estimation.src.pose_detectors.abstract import (
    AbstractPoseDetector,
    PersonDetection
)

class MediaPipePoseDetector(AbstractPoseDetector):
    """MediaPipe implementation of 2D human pose detection"""

    def __init__(
            self,
            logger=None,
            min_detection_confidence: float = 0.5,
            min_tracking_confidence: float = 0.5,
            model_complexity: int = 1
    ):
        """
        Initialize MediaPipe detector.

        Args:
            logger: Optional ROS logger
            min_detection_confidence: Minimum confidence for initial detection (0-1)
            min_tracking_confidence: Minimum confidence for tracking (0-1)
            model_complexity: 0=Lite, 1=Full, 2=Heavy (higher = slower but more accurate)
        """
        super().__init__(logger)
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.model_complexity = model_complexity
        self.pose = None

    def initialize(self) -> bool:
        """Initialize MediaPipe."""
        try:
            self.log_info("Initializing MediaPipe...")

            mp_pose = mp.solutions.pose
            self.pose = mp_pose.Pose(
                static_image_mode=False,  # Video mode for tracking
                model_complexity=self.model_complexity,
                smooth_landmarks=True,
                min_detection_confidence=self.min_detection_confidence,
                min_tracking_confidence=self.min_tracking_confidence
            )

            self.is_ready = True

            self.log_info(
                f"MediaPipe initialized: "
                f"complexity={self.model_complexity}, "
                f"det_conf={self.min_detection_confidence}, "
                f"track_conf={self.min_tracking_confidence}"
            )
            return True

        except Exception as e:
            self.log_error(f"MediaPipe initialization failed: {e}")
            return False

    def detect(self, image: np.ndarray) -> List[PersonDetection]:
        """
        Detect person in image using MediaPipe.

        Args:
            image: BGR image as numpy array (H, W, 3)

        Returns:
            List with PersonDetection objects (one per detected person)
        """
        if not self.is_ready:
            self.log_error("Detector not initialized!")
            return []

        try:
            # Convert BGR to RGB (MediaPipe expects RGB)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb_image)

            detections = []

            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                height, width = image.shape[:2]

                # Get hip center position (average of left and right hip)
                # MediaPipe landmark indices: 23=LEFT_HIP, 24=RIGHT_HIP
                left_hip = landmarks[23]
                right_hip = landmarks[24]

                # Check if both hips are visible enough
                if left_hip.visibility > 0.5 and right_hip.visibility > 0.5:
                    # Calculate center hip in pixel coordinates
                    hip_x = ((left_hip.x + right_hip.x) / 2) * width
                    hip_y = ((left_hip.y + right_hip.y) / 2) * height

                    # Average confidence
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
        if self.pose:
            self.pose.close()
        self.is_ready = False
        self.log_info("MediaPipe cleaned up")

    def get_name(self) -> str:
        return "MediaPipe"





