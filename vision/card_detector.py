import numpy as np
from typing import List, Optional, TypedDict
from vision.base_detector import BaseDetector
from poker.card import Rank, Suit

class BoundingBox(TypedDict):
    """
    Coordinates representing a rectangular region in the image.
    Values are normalised (0.0 to 1.0) relative to image dimensions.
    """
    x: float
    y: float
    width: float
    height: float

class CardDetection(TypedDict):
    """
    Data artifact representing a single identified playing card.
    
    Attributes:
        rank: The identified rank of the card (e.g., Rank.ACE).
        suit: The identified suit of the card (e.g., Suit.SPADES).
        confidence: The probability score of the classification (0.0 to 1.0).
        bbox: The bounding box containing the card.
    """
    rank: Optional[Rank]  # None if rank is ambiguous
    suit: Optional[Suit]  # None if suit is ambiguous
    confidence: float
    bbox: BoundingBox

class CardDetector(BaseDetector[List[CardDetection]]):
    """
    Detector for identifying playing cards in static images.
    
    This module uses deep learning (e.g., YOLOv8) to localise and classify
    cards on the poker table.
    """

    def __init__(self, model_path: Optional[str] = None):
        """
        Initialise the card detector.
        
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
            print(f"CardDetector: Loading model from {self.model_path}...")
        else:
            print("CardDetector: Initialised in dummy mode.")

    def process(self, image: np.ndarray) -> List[CardDetection]:
        """
        Analyses the image to detect and classify all visible cards.

        Args:
            image: Input BGR image frame.

        Returns:
            A list of CardDetection artifacts, containing rank, suit, and location data.
        """
        # TODO: Implement actual inference logic
        return []
