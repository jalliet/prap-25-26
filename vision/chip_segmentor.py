import numpy as np
from typing import Optional, TypedDict
from vision.base_detector import BaseDetector
from poker.chips import ChipColour, ChipStack

class ChipSegmentationResult(TypedDict):
    """
    Data artifact representing the result of a chip counting operation.

    Attributes:
        stack: A ChipStack object containing the breakdown of detected chips.
        confidence: An overall confidence score for the estimation (0.0 to 1.0).
    """
    stack: ChipStack
    confidence: float

# YOLO model class name → ChipColour mapping.
# Model outputs: "Black", "Blue", "Red", "White"
# Black chips are ignored (not in use).
_COLOUR_MAP = {
    "WHITE": ChipColour.WHITE,
    "RED": ChipColour.RED,
    "BLUE": ChipColour.BLUE,
}

class ChipSegmentor(BaseDetector[ChipSegmentationResult]):
    """
    Detector for segmenting and counting poker chips.

    Uses YOLOv8 instance segmentation to detect and classify chips by colour,
    then builds a ChipStack from the counts.
    Falls back to dummy mode if ultralytics is not installed or model is missing.
    """

    def __init__(self, model_path: Optional[str] = None, confidence_threshold: float = 0.5):
        super().__init__(model_path, confidence_threshold)
        self._load_yolo_model()

    def process(self, image: np.ndarray) -> ChipSegmentationResult:
        """
        Analyses the image to segment chips and calculate the total stack value.

        Args:
            image: Input BGR image.

        Returns:
            A ChipSegmentationResult containing the estimated stack and confidence.
        """
        if self.model is None:
            return {"stack": ChipStack(), "confidence": 0.0}

        results = self.model(image, verbose=False)
        chip_counts: dict[ChipColour, int] = {c: 0 for c in ChipColour}
        confidences: list[float] = []

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue

                cls_id = int(box.cls[0])
                class_name = result.names.get(cls_id, "").upper()
                colour = _COLOUR_MAP.get(class_name)

                if colour is not None:
                    chip_counts[colour] += 1
                    confidences.append(conf)

        stack = ChipStack(chip_counts)
        avg_confidence = sum(confidences) / len(confidences) if confidences else 0.0

        return {"stack": stack, "confidence": avg_confidence}
