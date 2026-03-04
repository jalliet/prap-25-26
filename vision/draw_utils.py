import cv2
import numpy as np
from typing import List
from vision.card_detector import CardDetection


# Box colour (BGR) and label styling
_BOX_COLOUR = (0, 255, 0)  # Green
_BOX_THICKNESS = 2
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SCALE = 0.6
_FONT_THICKNESS = 2
_LABEL_BG_COLOUR = (0, 180, 0)  # Darker green background for text
_LABEL_TEXT_COLOUR = (255, 255, 255)  # White text


def _format_label(detection: CardDetection) -> str:
    """Build a display label like 'AH 0.95' from a CardDetection.

    Uses ASCII codes (H, S, C, D) instead of Unicode suit symbols
    because OpenCV's putText cannot render non-ASCII characters.
    """
    rank = detection["rank"]
    suit = detection["suit"]

    rank_str = rank.code if rank is not None else "?"
    suit_str = suit.code if suit is not None else "?"
    conf = detection["confidence"]

    return f"{rank_str}{suit_str} {conf:.2f}"


def draw_card_detections(frame: np.ndarray, detections: List[CardDetection]) -> None:
    """
    Draw bounding boxes and labels for card detections on a frame (in-place).

    Args:
        frame: BGR image (H x W x 3), modified in-place.
        detections: List of CardDetection dicts with normalised bbox coordinates.
    """
    img_h, img_w = frame.shape[:2]

    for det in detections:
        bbox = det["bbox"]
        # Convert normalised coords to pixel coords
        x1 = int(bbox["x"] * img_w)
        y1 = int(bbox["y"] * img_h)
        x2 = int((bbox["x"] + bbox["width"]) * img_w)
        y2 = int((bbox["y"] + bbox["height"]) * img_h)

        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), _BOX_COLOUR, _BOX_THICKNESS)

        # Build label
        label = _format_label(det)
        (tw, th), baseline = cv2.getTextSize(label, _FONT, _FONT_SCALE, _FONT_THICKNESS)

        # Label background (above the box)
        label_y = max(y1 - 6, th + 4)
        cv2.rectangle(frame,
                      (x1, label_y - th - 4),
                      (x1 + tw + 4, label_y + baseline),
                      _LABEL_BG_COLOUR, cv2.FILLED)

        # Label text
        cv2.putText(frame, label,
                    (x1 + 2, label_y - 2),
                    _FONT, _FONT_SCALE, _LABEL_TEXT_COLOUR, _FONT_THICKNESS)