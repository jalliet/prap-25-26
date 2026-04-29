import numpy as np
from typing import List, Optional, TypedDict
from vision.base_detector import BaseDetector, BoundingBox
from poker.card import Rank, Suit


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

# YOLO model class name → Rank.from_code compatible rank code.
# Model outputs names like "AH", "10C", "KS", etc.
# The suit is always the last character; the rank prefix may be "10" instead of "T".
_RANK_MAP = {
    "2": "2", "3": "3", "4": "4", "5": "5", "6": "6",
    "7": "7", "8": "8", "9": "9", "10": "T",
    "A": "A", "J": "J", "Q": "Q", "K": "K",
}

def _parse_class_name(name: str) -> tuple[Optional[Rank], Optional[Suit]]:
    """Parse a YOLO class name like 'AH', '10C' into (Rank, Suit)."""
    name = name.strip().upper()
    # Suit is always the last character
    suit_code = name[-1]
    rank_str = name[:-1]

    rank_code = _RANK_MAP.get(rank_str)
    if rank_code is None:
        return None, None

    try:
        rank = Rank.from_code(rank_code)
        suit = Suit.from_code(suit_code)
        return rank, suit
    except ValueError:
        return None, None


class CardDetector(BaseDetector[List[CardDetection]]):
    """
    Detector for identifying playing cards in static images.

    Uses YOLOv8 to localise and classify cards on the poker table.
    Falls back to dummy mode if ultralytics is not installed or model is missing.
    """

    def __init__(self, model_path: Optional[str] = None, confidence_threshold: float = 0.5):
        super().__init__(model_path, confidence_threshold)
        self._load_yolo_model()

    def process(self, image: np.ndarray) -> List[CardDetection]:
        """
        Analyses the image to detect and classify all visible cards.

        Args:
            image: Input BGR image frame.

        Returns:
            A list of CardDetection artifacts, containing rank, suit, and location data.
        """
        if self.model is None:
            return []

        results = self.model(image, verbose=False)
        img_h, img_w = image.shape[:2]

        # Keep only the highest-confidence detection per unique card,
        # since YOLO often detects both corners of the same card.
        best: dict[tuple, CardDetection] = {}

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue

                cls_id = int(box.cls[0])
                class_name = result.names.get(cls_id, "")
                rank, suit = _parse_class_name(class_name)

                key = (rank, suit)
                if key in best and best[key]["confidence"] >= conf:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                best[key] = {
                    "rank": rank,
                    "suit": suit,
                    "confidence": conf,
                    "bbox": {
                        "x": x1 / img_w,
                        "y": y1 / img_h,
                        "width": (x2 - x1) / img_w,
                        "height": (y2 - y1) / img_h,
                    },
                }

        return list(best.values())
