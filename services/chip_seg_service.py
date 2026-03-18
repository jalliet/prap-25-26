"""Chip segmentation camera service for the Logitech C925e (or any standard UVC webcam)."""
import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class ChipSegConfig:
    """Hardware parameters for the chip segmentation webcam.

    Attributes:
        device_index: cv2.VideoCapture device index (0 = first available).
                      On Linux, corresponds to /dev/video<N>.
                      Adjust if the C925e is not on index 0:
                        Linux: v4l2-ctl --list-devices
                        macOS: system_profiler SPCameraDataType
        width: Requested capture width in pixels.
        height: Requested capture height in pixels.
        fps: Requested frames per second.
    """
    device_index: int = 0
    width: int = 1920
    height: int = 1080
    fps: int = 30


class ChipSegService:
    """Manages the Logitech C925e webcam via OpenCV VideoCapture.

    The C925e is positioned to view poker chip stacks for segmentation.
    Mirrors the BirdseyeService interface (start / stop / get_frame / set_fps)
    so VisionController can treat both cameras uniformly.
    """

    def __init__(self, config: Optional[ChipSegConfig] = None):
        self.config = config if config is not None else ChipSegConfig()
        self.cap: Optional[cv2.VideoCapture] = None
        self.running = False

    def start(self):
        """Open the capture device and configure resolution/FPS."""
        if self.running:
            return

        cap = cv2.VideoCapture(self.config.device_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        cap.set(cv2.CAP_PROP_FPS, self.config.fps)

        if cap.isOpened():
            self.cap = cap
            self.running = True
            print(f"ChipSegService: started on device {self.config.device_index}")
        else:
            cap.release()
            print(f"ChipSegService: failed to open device {self.config.device_index}")

    def stop(self):
        """Release the capture device."""
        self.running = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        print("ChipSegService: stopped.")

    def get_frame(self) -> Optional[np.ndarray]:
        """Read the latest frame. Returns None if unavailable."""
        if not self.running or self.cap is None:
            return None
        ret, frame = self.cap.read()
        return frame if ret else None

    def set_fps(self, fps: int):
        """Update the FPS setting. Propagates to the capture device if running."""
        self.config.fps = fps
        if self.running and self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FPS, fps)
