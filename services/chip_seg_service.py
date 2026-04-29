"""Chip segmentation camera service for the Logitech C925e (or any standard UVC webcam)."""
import sys
import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


def _default_backend() -> int:
    if sys.platform == "darwin":
        return cv2.CAP_AVFOUNDATION
    if sys.platform.startswith("linux"):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


@dataclass
class ChipSegConfig:
    """Hardware parameters for the chip segmentation webcam.

    Attributes:
        device_index: cv2.VideoCapture device index.
                        Linux: 0 (/dev/video0) — stable for a single USB webcam.
                        macOS: 0 when only the C925e is plugged in, but AVFoundation
                          REORDERS indices whenever an iPhone Continuity Camera
                          connects/disconnects. If the dashboard shows the wrong
                          feed, run `python scripts/diagnose_cameras.py` and set
                          device_index to whichever index is the C925e right now.
                          Disabling Continuity Camera in System Settings >
                          AirDrop & Handoff makes the order stable.
        backend: OpenCV capture backend. AVFoundation on macOS, V4L2 on Linux.
        width: Requested capture width in pixels.
        height: Requested capture height in pixels.
        fps: Requested frames per second.
    """
    device_index: int = 0
    backend: int = field(default_factory=_default_backend)
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

        cap = cv2.VideoCapture(self.config.device_index, self.config.backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        cap.set(cv2.CAP_PROP_FPS, self.config.fps)

        if cap.isOpened():
            self.cap = cap
            self.running = True
            print(f"ChipSegService: started on device {self.config.device_index} "
                  f"(backend={self.config.backend})")
        else:
            cap.release()
            print(f"ChipSegService: failed to open device {self.config.device_index} "
                  f"(backend={self.config.backend})")

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
