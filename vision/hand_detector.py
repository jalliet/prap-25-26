import numpy as np
from typing import List, Optional, TypedDict
from vision.base_detector import BaseDetector, BoundingBox


class HandDetection(TypedDict):
    """
    Data artifact representing a single detected hand.
    
    Attributes:
        confidence: Probability score of the detection (0.0 to 1.0).
        bbox: The bounding box containing the hand.
        is_right_hand: Boolean indicating handedness (True=Right, False=Left).
    """
    confidence: float
    bbox: BoundingBox
    is_right_hand: Optional[bool]
    # Future: landmarks: List[Tuple[float, float]]

class HandDetector(BaseDetector[List[HandDetection]]):
    """
    Detector for identifying hands in the video stream.
    
    This module analyses frames to locate player hands, serving as the primary
    input for gesture recognition and bet tracking.
    """

    def __init__(self, model_path: Optional[str] = None):
        super().__init__(model_path)
        self._load_yolo_model()

    def process(self, image: np.ndarray) -> List[HandDetection]:
        """
        Analyses the image to detect all visible hands.

        Args:
            image: Input BGR image frame.

        Returns:
            A list of HandDetection artifacts, one for each detected hand.
        """
        # TODO: Implement actual inference logic (e.g., using MediaPipe or YOLO)
        return []
