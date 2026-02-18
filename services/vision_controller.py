import cv2
from typing import Optional, Callable
from services.camera_service import CameraService
from enum import Enum, auto

class VisionMode(Enum):
    HAND_MONITORING = auto()
    CARD_READING = auto()
    CHIP_SEGMENTATION = auto() # For side camera (future)
    IDLE = auto()

class VisionController:
    """
    Singleton controller for managing camera services & vision processing.
    """
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(VisionController, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
            
        self.camera_service = CameraService()
        self.mode = VisionMode.IDLE
        self.is_running = False
        self._initialized = True
        
        # Callbacks for processing results
        self.on_frame_processed = None # Signal/Callback(processed_frame)

    def start(self):
        """Starts the camera service and processing loop."""
        if not self.is_running:
            self.camera_service.start()
            self.is_running = True
            print("VisionController started.")

    def stop(self):
        """Stops the camera service and processing loop."""
        if self.is_running:
            self.camera_service.stop()
            self.is_running = False
            print("VisionController stopped.")

    def set_mode(self, mode: VisionMode):
        """Switches the current vision processing mode."""
        if self.mode != mode:
            print(f"Switching Vision Mode: {self.mode.name} -> {mode.name}")
            self.mode = mode
            # Logic to reconfigure pipeline or detectors would go here

    def get_frame(self):
        """
        Retrieves the latest frame from the camera service.
        In the future, this might return a processed frame depending on mode.
        """
        return self.camera_service.get_frame()

    def process_frame(self):
        """
        Main processing loop iteration.
        Should be called periodically (e.g., by a QTimer in the GUI or a separate thread).
        """
        if not self.is_running:
            return

        frame = self.get_frame()
        if frame is None:
            return

        # Placeholder for processing logic based on self.mode
        if self.mode == VisionMode.HAND_MONITORING:
            # self.hand_detector.process(frame)
            pass
        elif self.mode == VisionMode.CARD_READING:
            # self.card_detector.process(frame)
            pass
            
        # Emit/Return result
        if self.on_frame_processed:
            self.on_frame_processed(frame)
