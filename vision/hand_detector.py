import numpy as np
from typing import List, Optional, TypedDict
from vision.base_detector import BaseDetector

class BoundingBox(TypedDict):
    """
    Coordinates representing a rectangular region in the image.
    Values are normalised (0.0 to 1.0) relative to image dimensions.
    """
    x: float
    y: float
    width: float
    height: float

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
        """
        Initialise the hand detector.
        
        Args:
            model_path: Path to the local model weights. If None, runs in dummy mode.
        """
        self.model_path = model_path
        self._load_model()

    def _load_model(self):
        """
        Loads the neural network weights from the local filesystem.
        """
        if self.model_path:
            print(f"HandDetector: Loading model from {self.model_path}...")
        else:
            print("HandDetector: Initialised in dummy mode.")

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
