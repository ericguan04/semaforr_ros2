#!/usr/bin/env python3
"""
OpenPose-based 2D pose detector.
"""

import sys
import os
import numpy as np
from typing import List

from social_context.pose_estimation.src.pose_detectors.abstract import (
    AbstractPoseDetector,
    PersonDetection
)

class OpenPosePoseDetector(AbstractPoseDetector):
    """OpenPose implementation of 2D human pose detection"""

    def __init__(
            self,
            logger=None,
            openpose_path: str = None,
            model_pose: str = "BODY_25",
            num_gpu: int = 0,
            confidence_threshold: float = 0.1
    ):
        """
        Initialize OpenPose detector.

        Args:
            logger: Optional ROS logger
            openpose_path: Path to OpenPose installation (defaults to OPENPOSE_PATH env var)
            model_pose: Model to use ("BODY_25", "COCO", "MPI")
            num_gpu: Number of GPUs to use (0 = CPU only)
            confidence_threshold: Minimum confidence for detection
        """
        super().__init__(logger)
        self.openpose_path = openpose_path or os.environ.get('OPENPOSE_PATH', '')
        self.model_folder = f"{self.openpose_path}/models/"
        self.model_pose = model_pose
        self.num_gpu = num_gpu
        self.confidence_threshold = confidence_threshold
        self.op_wrapper = None
        self.op = None

    def initialize(self) -> bool:
        """Initialize OpenPose."""
        try:
            if not self.openpose_path:
                self.log_error("OPENPOSE_PATH not set! Set environment variable.")
                return False

            # Import OpenPose Python module
            sys.path.append(f'{self.openpose_path}/build/python/openpose')
            import pyopenpose as op
            self.op = op

            params = {
                "model_folder": self.model_folder,
                "model_pose": self.model_pose,
            }

            if self.num_gpu > 0:
                params["num_gpu"] = self.num_gpu
                params["num_gpu_start"] = 0

            self.op_wrapper = op.WrapperPython()
            self.op_wrapper.configure(params)
            self.op_wrapper.start()

            self.is_ready = True
            self.log_info(
                f"OpenPose initialized: "
                f"model={self.model_pose}, "
                f"GPU={self.num_gpu}, "
                f"conf_threshold={self.confidence_threshold}"
            )
            return True

        except Exception as e:
            self.log_error(f"OpenPose initialization failed: {e}")
            return False

    def detect(self, image: np.ndarray) -> List[PersonDetection]:
        """
        Detect people in image using OpenPose.

        Args:
            image: BGR image as numpy array (H, W, 3)

        Returns:
            List of PersonDetection objects (one per detected person)
        """
        if not self.is_ready:
            self.log_error("Detector not initialized!")
            return []

        try:
            # Process image with OpenPose
            datum = self.op.Datum()
            datum.cvInputData = image
            self.op_wrapper.emplaceAndPop(self.op.VectorDatum([datum]))

            detections = []

            if (datum.poseKeypoints is not None and
                    not isinstance(datum.poseKeypoints, (tuple, float, int)) and
                    len(datum.poseKeypoints) > 0):

                # Process each detected person
                for person in datum.poseKeypoints:
                    # Get keypoint 8 (MidHip in BODY_25 model)
                    if len(person) > 8 and len(person[8]) >= 3:
                        confidence = float(person[8, 2])

                        if confidence > self.confidence_threshold:
                            detections.append(PersonDetection(
                                pixel_x=float(person[8, 0]),
                                pixel_y=float(person[8, 1]),
                                confidence=confidence
                            ))

            return detections

        except Exception as e:
            self.log_error(f"Error during detection: {e}")
            return []

    def cleanup(self):
        """Clean up OpenPose resources."""
        # OpenPose doesn't have explicit cleanup
        # The wrapper is cleaned up automatically when Python exits
        self.is_ready = False
        self.log_info("OpenPose cleaned up")

    def get_name(self) -> str:
        """Return detector name."""
        return "OpenPose"


