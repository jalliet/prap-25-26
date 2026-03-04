from abc import ABC, abstractmethod
import numpy as np
from typing import Any, Generic, Optional, TypeVar, TypedDict

# Generic type variable for the detection result
T = TypeVar('T')


class BoundingBox(TypedDict):
    """
    Coordinates representing a rectangular region in the image.
    Values are normalised (0.0 to 1.0) relative to image dimensions.
    """
    x: float
    y: float
    width: float
    height: float


class BaseDetector(ABC, Generic[T]):
    """
    Abstract base class for all computer vision detectors.

    This class enforces a standard interface for processing images and returning
    structured data artifacts (T).

    Subclasses that use YOLO models can call ``_load_yolo_model()`` in their
    ``__init__`` to get the standard load-or-dummy behaviour.
    """

    def __init__(self, model_path: Optional[str] = None,
                 confidence_threshold: float = 0.5):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = None

    @property
    def _name(self) -> str:
        return self.__class__.__name__

    def _load_yolo_model(self):
        """Load a YOLO model from *model_path*, falling back to dummy mode."""
        if not self.model_path:
            print(f"{self._name}: Initialised in dummy mode.")
            return

        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            print(f"{self._name}: Loaded model from {self.model_path}")
        except ImportError:
            print(f"{self._name}: ultralytics not installed. Running in dummy mode.")
        except Exception as e:
            print(f"{self._name}: Failed to load model ({e}). Running in dummy mode.")

    @abstractmethod
    def process(self, image: np.ndarray) -> T:
        """
        Processes an image frame and returns the structured detection results.

        Args:
            image: Input image (numpy array, BGR format).

        Returns:
            A structured data artifact (TypedDict or Dataclass) containing the
            detection results.
        """
        pass
