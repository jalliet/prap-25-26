import cv2
from typing import Optional, Callable
from services.camera_service import CameraService, CameraConfig
from enum import Enum, auto
from poker.game_state import GameState, GamePhase
from vision.card_detector import CardDetector
from vision.chip_segmentor import ChipSegmentor
from vision.hand_detector import HandDetector

from PySide6.QtCore import QObject, Signal, QThread
import numpy as np

class VisionMode(Enum):
    """
    Operational modes for the Vision Controller.

    Each mode dictates the active detectors and the camera configuration required
    to perform the specific computer vision task.
    """
    HAND_MONITORING = auto()  # Continuous video stream to track player hands and gestures.
    CARD_READING = auto()     # High-resolution capture to identify card ranks and suits.
    CHIP_SEGMENTATION = auto() # Side-view analysis for chip stack estimation (Future).
    IDLE = auto()             # Camera active but no heavy processing.

class VisionController(QObject):
    """
    Singleton controller for managing camera services and orchestrating vision processing.

    This class acts as the bridge between the high-level GameState and the low-level
    Computer Vision modules. It switches modes based on game events to optimise
    resource usage and data accuracy.
    """
    _instance = None

    # Signals for UI integration
    frame_ready = Signal(np.ndarray)

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(VisionController, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        # Initialise QObject
        super().__init__()

        # Initialise Camera Service with default configuration
        self.camera_service = CameraService(CameraConfig())
        self.mode = VisionMode.IDLE
        self.is_running = False
        self._initialized = True

        # Callbacks for processing results
        self.on_frame_processed: Optional[Callable] = None

        # Arm bridge (connected externally via connect_to_arm_bridge)
        self.arm_bridge = None

        # Initialise detectors
        self.card_detector = CardDetector(
            model_path='vision/models/Card_detection_large_best.pt')
        self.chip_segmentor = ChipSegmentor(
            model_path='vision/models/Chip_segmentation_large_best.pt')
        self.hand_detector = HandDetector()  # No model file, stays in dummy mode

        # Internal polling timer
        from PySide6.QtCore import QTimer
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self.process_frame)
        self._poll_timer.setInterval(1000 // self.camera_service.config.fps)

    @property
    def fps(self) -> int:
        """
        Returns the configured FPS of the underlying camera service.
        This allows the UI to synchronise its update rate with the hardware capabilities.
        """
        return self.camera_service.config.fps

    def set_fps(self, fps: int):
        """
        Updates the polling rate and attempts to update camera hardware FPS.
        """
        if fps < 1: return

        print(f"VisionController: Setting FPS to {fps}")
        # Update internal polling timer
        self._poll_timer.setInterval(1000 // fps)

        # Propagate to CameraService if it supports dynamic reconfiguration
        try:
            self.camera_service.set_fps(fps)
        except Exception as e:
            print(f"VisionController: failed to set camera fps: {e}")

    def connect_to_game_state(self, game_state: GameState):
        """
        Connects the VisionController to GameState signals.

        This allows the vision system to react autonomously to game events,
        switching modes (e.g., to read cards when the Flop is dealt) without manual intervention.
        """
        game_state.on_phase_change.connect(self._handle_phase_change)
        game_state.on_card_detection_required.connect(self._handle_card_detection)
        game_state.on_pot_change.connect(self._handle_pot_change)

    def connect_to_arm_bridge(self, bridge):
        """Connect the vision controller to the arm ROS bridge."""
        self.arm_bridge = bridge

    def request_arm_move(self, x: float, y: float, z: float,
                         pitch: float, roll: float, duration: float):
        """Request the arm to move to a Cartesian pose. No-op if bridge unavailable."""
        if self.arm_bridge and self.arm_bridge.is_available:
            self.arm_bridge.move_pose(x, y, z, pitch, roll, duration)

    def _handle_phase_change(self, phase: GamePhase):
        """
        Handles game phase changes to adjust vision monitoring strategy.
        """
        print(f"VisionController: Phase changed to {phase.name}")
        if phase in [GamePhase.PRE_FLOP, GamePhase.FLOP, GamePhase.TURN, GamePhase.RIVER]:
             self.set_mode(VisionMode.HAND_MONITORING)
        elif phase == GamePhase.SHOWDOWN:
             pass

    def _handle_card_detection(self, cards):
        """
        Triggered when new community cards are dealt.

        Intent: Switch to high-precision mode to identify the new cards on the board.
        """
        print("VisionController: Card detection required.")
        self.set_mode(VisionMode.CARD_READING)

    def _handle_pot_change(self, total_pot: int):
        """
        Triggered when pot size changes.

        Intent: Verify chip counts if a side camera is available.
        """
        print(f"VisionController: Pot updated to {total_pot}. Triggering chip check.")

    def start(self):
        """Starts the camera service and the vision processing loop."""
        if not self.is_running:
            self.camera_service.start()
            self.is_running = True
            self._poll_timer.start()
            print("VisionController started.")

    def stop(self):
        """Stops the camera service and processing loop."""
        if self.is_running:
            self._poll_timer.stop()
            self.camera_service.stop()
            self.is_running = False
            print("VisionController stopped.")

    def set_mode(self, mode: VisionMode):
        """
        Switches the current vision processing mode.

        This reconfiguration may involve changing camera exposure, resolution,
        or the active neural network models in the pipeline.
        """
        if self.mode != mode:
            print(f"Switching Vision Mode: {self.mode.name} -> {mode.name}")
            self.mode = mode

            if self.mode == VisionMode.CARD_READING:
                 print("VisionController: Configuring for Card Reading (High-Res/Still)")
            elif self.mode == VisionMode.HAND_MONITORING:
                 print("VisionController: Configuring for Hand Monitoring (Video Stream)")
            elif self.mode == VisionMode.CHIP_SEGMENTATION:
                 print("VisionController: Configuring for Chip Segmentation (Side View)")

    def get_frame(self):
        """
        Retrieves the latest frame from the camera service.
        """
        return self.camera_service.get_frame()

    def process_frame(self):
        """
        Main processing loop iteration.
        Called periodically by the internal QTimer.
        """
        if not self.is_running:
            return

        frame = self.get_frame()
        if frame is None:
            return

        if self.mode == VisionMode.HAND_MONITORING:
            detections = self.hand_detector.process(frame)
            # Future: gesture recognition → arm commands

        elif self.mode == VisionMode.CARD_READING:
            detections = self.card_detector.process(frame)
            # Future: map detected cards to GameState

        elif self.mode == VisionMode.CHIP_SEGMENTATION:
            result = self.chip_segmentor.process(frame)
            # Future: update pot/stack estimates

        # Emit Signal for UI
        self.frame_ready.emit(frame)

        # Legacy Callback (deprecated, prefer signals)
        if self.on_frame_processed:
            self.on_frame_processed(frame)
