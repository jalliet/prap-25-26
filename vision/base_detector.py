from abc import ABC, abstractmethod
import numpy as np
from typing import Any, Optional

class BaseDetector(ABC):
    """
    Abstract base class for all vision detectors.
    """

    @abstractmethod
    def process(self, image: np.ndarray) -> Any:
        """
        Process an image frame and return the detection results.
        
        Args:
            image: Input image (numpy array, usually BGR or RGB).
            
        Returns:
            Detection results (format depends on the specific detector).
        """
        pass
