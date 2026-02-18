import cv2
from typing import Optional, Callable
from services.camera_service import CameraService
from enum import Enum, auto
from poker.game_state import GameState, GamePhase

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

    def connect_to_game_state(self, game_state: GameState):
        """Connect VisionController to GameState signals."""
        game_state.on_phase_change.connect(self._handle_phase_change)
        game_state.on_card_detection_required.connect(self._handle_card_detection)
        game_state.on_pot_change.connect(self._handle_pot_change)
        
    def _handle_phase_change(self, phase: GamePhase):
        """Handles game phase changes."""
        print(f"VisionController: Phase changed to {phase.name}")
        # Default behavior: Hand monitoring during active play
        if phase in [GamePhase.PRE_FLOP, GamePhase.FLOP, GamePhase.TURN, GamePhase.RIVER]:
             self.set_mode(VisionMode.HAND_MONITORING)
        elif phase == GamePhase.SHOWDOWN:
             # In showdown, we might want to read cards again or verify
             pass

    def _handle_card_detection(self, cards):
        """Triggered when new community cards are dealt."""
        # Check if it's the right list of cards, or just a signal
        print("VisionController: Card detection required.")
        # Switch to card reading mode temporarily
        self.set_mode(VisionMode.CARD_READING)
        # In a real system, trigger single-shot capture here
        # and revert to hand monitoring after success/timeout.
        # For now,  manually revert or let next phase change handle it.
        
    def _handle_pot_change(self, total_pot: int):
        """Triggered when pot size changes."""
        print(f"VisionController: Pot updated to {total_pot}. Triggering chip check.")
        # Trigger side view chip segmentation if we had a side camera
        # self.set_mode(VisionMode.CHIP_SEGMENTATION) 
        # Note: We might not want to switch MAIN camera mode if using two cameras.

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
            
            # Logic to reconfigure pipeline or detectors
            if self.mode == VisionMode.CARD_READING:
                 # In real implementation, might pause video stream here
                 # to capture a high-res still, or adjust exposure settings.
                 print("VisionController: Configuring for Card Reading (High-Res/Still)")
            elif self.mode == VisionMode.HAND_MONITORING:
                 print("VisionController: Configuring for Hand Monitoring (Video Stream)")
            elif self.mode == VisionMode.CHIP_SEGMENTATION:
                 print("VisionController: Configuring for Chip Segmentation (Side View)")

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
            # TODO: Integrate HandDetector
            # result = self.hand_detector.process(frame)
            # self._draw_hand_overlay(frame, result)
            pass
            
        elif self.mode == VisionMode.CARD_READING:
            # TODO: Integrate CardDetector
            # This might only run once per trigger, not every frame
            # result = self.card_detector.process(frame)
            pass
            
        elif self.mode == VisionMode.CHIP_SEGMENTATION:
             # TODO: Integrate ChipSegmentor
             pass
            
        # Emit/Return result
        if self.on_frame_processed:
            self.on_frame_processed(frame)
