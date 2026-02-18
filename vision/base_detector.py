from abc import ABC, abstractmethod
import numpy as np
from typing import Any, Generic, TypeVar

# Generic type variable for the detection result
T = TypeVar('T')

class BaseDetector(ABC, Generic[T]):
    """
    Abstract base class for all computer vision detectors.
    
    This class enforces a standard interface for processing images and returning
    structured data artifacts (T).
    """

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
