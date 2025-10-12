#!/usr/bin/env python3
"""
Abstract base class for 2D human pose detectors.

Provides a common interface for different pose detection models
(MediaPipe, OpenPose, etc.).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List
import numpy as np


@dataclass
class PersonDetection:
    """Represents a single detected person in 2D image space."""
    pixel_x: float
    pixel_y: float
    confidence: float

class AbstractPoseDetector(ABC):
    """
    Abstract base class for 2D human pose detectors.
    """

    def __init__(self, logger=None):
        self.logger = logger
        self.is_ready = False

    def log_info(self, msg: str):
        """Helper for logging info messages."""
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"INFO: {msg}")

    def log_error(self, msg: str):
        """Helper for logging error messages."""
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"ERROR: {msg}")

    @abstractmethod
    def initialize(self) -> bool:
        pass

    @abstractmethod
    def detect(self, image: np.ndarray) -> List[PersonDetection]:
        """
        Detect people in an image.

        Args:
            image: BGR image as numpy array (H, W, 3)

        Returns:
            List of PersonDetection objects (one per detected person)
        """
        pass

    @abstractmethod
    def get_name(self) -> str:
        """
        Get the name of this detector for logging.

        Returns:
            Detector name (e.g., "MediaPipe", "OpenPose")
        """
        pass

    def cleanup(self):
        """
        Clean up resources (close models, free memory, etc.).

        Optional to override - default does nothing.
        """
        pass




