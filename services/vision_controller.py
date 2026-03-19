from typing import Optional
from services.birdseye_service import BirdseyeService, BirdseyeConfig
from services.chip_seg_service import ChipSegService, ChipSegConfig
from enum import Enum, auto
from poker.game_state import GameState, GamePhase
from vision.card_detector import CardDetector
from vision.chip_segmentor import ChipSegmentor

from PySide6.QtCore import QObject, Signal, QTimer
from vision.draw_utils import draw_card_detections
import numpy as np


class VisionMode(Enum):
    """Operational mode for the primary (OAK-D Lite) camera.

    Chip segmentation runs independently on the secondary camera (C925e)
    via its own QTimer and is not represented here.
    """
    CARD_READING = auto()  # OAK-D streams + CardDetector inference.
    IDLE = auto()          # OAK-D streams, no inference.


class VisionController(QObject):
    """Singleton orchestrator for the dual-camera vision pipeline.

    Primary camera (OAK-D Lite, birdseye):
        - Driven by _poll_timer
        - IDLE: stream only; CARD_READING: YOLO card detection + overlay

    Secondary camera (Logitech C925e, chip segmentation):
        - Driven by _chip_poll_timer (always-on when running)
        - Runs ChipSegmentor on every frame, emits chips_detected on change

    Camera index note:
        ChipSegConfig.device_index defaults to 0. Adjust via
        ChipSegConfig(device_index=N) if the C925e is not on index 0.
        On Linux: v4l2-ctl --list-devices
    """
    _instance = None

    # Primary camera signals
    frame_ready = Signal(np.ndarray)      # OAK-D frame (with overlays if in CARD_READING)
    cards_detected = Signal(list)         # List[CardDetection], emitted only on change

    # Secondary camera signals
    chip_frame_ready = Signal(np.ndarray) # C925e raw frame for GUI display
    chips_detected = Signal(dict)         # ChipSegmentationResult, emitted only on change

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(VisionController, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        super().__init__()

        # Primary camera (OAK-D Lite)
        self.camera_service = BirdseyeService(BirdseyeConfig())
        # Secondary camera (Logitech C925e)
        self.chip_seg_service = ChipSegService(ChipSegConfig())

        self.mode = VisionMode.IDLE
        self.is_running = False
        self._initialized = True
        self.arm_bridge = None

        # Dedup state
        self._last_card_set: frozenset = frozenset()
        self._last_chip_total: int = -1  # -1 sentinel: first result always emits

        # Detectors
        self.card_detector = CardDetector(
            model_path='vision/models/Card_detection_large_best.pt')
        self.chip_segmentor = ChipSegmentor(
            model_path='vision/models/Chip_segmentation_large_best.pt')

        # Primary camera timer
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self.process_frame)
        self._poll_timer.setInterval(1000 // self.camera_service.config.fps)

        # Chip camera timer (independent, always-on)
        self._chip_poll_timer = QTimer()
        self._chip_poll_timer.timeout.connect(self._process_chip_frame)
        self._chip_poll_timer.setInterval(1000 // self.chip_seg_service.config.fps)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def fps(self) -> int:
        return self.camera_service.config.fps

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Start both cameras and processing loops."""
        if not self.is_running:
            self.camera_service.start()
            self.chip_seg_service.start()
            self.is_running = True
            self._poll_timer.start()
            self._chip_poll_timer.start()
            print("VisionController started (OAK-D Lite + C925e).")

    def stop(self):
        """Stop both cameras and processing loops."""
        if self.is_running:
            self._poll_timer.stop()
            self._chip_poll_timer.stop()
            self.camera_service.stop()
            self.chip_seg_service.stop()
            self.is_running = False
            print("VisionController stopped.")

    # ------------------------------------------------------------------
    # FPS control
    # ------------------------------------------------------------------

    def set_fps(self, fps: int):
        """Update OAK-D polling rate and hardware FPS."""
        if fps < 1:
            return
        print(f"VisionController: Setting FPS to {fps}")
        self._poll_timer.setInterval(1000 // fps)
        try:
            self.camera_service.set_fps(fps)
        except Exception as e:
            print(f"VisionController: failed to set camera fps: {e}")

    def set_chip_fps(self, fps: int):
        """Update chip camera polling rate."""
        if fps < 1:
            return
        self._chip_poll_timer.setInterval(1000 // fps)
        try:
            self.chip_seg_service.set_fps(fps)
        except Exception as e:
            print(f"VisionController: failed to set chip camera fps: {e}")

    # ------------------------------------------------------------------
    # Mode & connections
    # ------------------------------------------------------------------

    def set_mode(self, mode: VisionMode):
        if self.mode != mode:
            print(f"Switching Vision Mode: {self.mode.name} -> {mode.name}")
            self.mode = mode
            if self.mode == VisionMode.CARD_READING:
                print("VisionController: Configuring for Card Reading (High-Res/Still)")

    def connect_to_game_state(self, game_state: GameState):
        """Connect VisionController to GameState signals for automatic mode switching."""
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

    # ------------------------------------------------------------------
    # Game state handlers
    # ------------------------------------------------------------------

    def _handle_phase_change(self, phase: GamePhase):
        print(f"VisionController: Phase changed to {phase.name}")
        if phase != GamePhase.SHOWDOWN:
            self.set_mode(VisionMode.IDLE)

    def _handle_card_detection(self, cards):
        print("VisionController: Card detection required.")
        self.set_mode(VisionMode.CARD_READING)

    def _handle_pot_change(self, total_pot: int):
        print(f"VisionController: Pot updated to {total_pot}.")

    # ------------------------------------------------------------------
    # Frame processing
    # ------------------------------------------------------------------

    def get_frame(self):
        return self.camera_service.get_frame()

    def process_frame(self):
        """Primary (OAK-D) processing loop — called by _poll_timer."""
        if not self.is_running:
            return
        frame = self.get_frame()
        if frame is None:
            return

        if self.mode == VisionMode.CARD_READING:
            detections = self.card_detector.process(frame)
            draw_card_detections(frame, detections)
            current_set = frozenset(
                (d["rank"], d["suit"]) for d in detections
                if d["rank"] is not None and d["suit"] is not None
            )
            if current_set != self._last_card_set:
                self._last_card_set = current_set
                self.cards_detected.emit(detections)

        self.frame_ready.emit(frame)

    def _process_chip_frame(self):
        """Secondary (C925e) processing loop — called by _chip_poll_timer."""
        if not self.is_running:
            return
        frame = self.chip_seg_service.get_frame()
        if frame is None:
            return

        result = self.chip_segmentor.process(frame)
        self.chip_frame_ready.emit(frame)

        # Only emit chips_detected when the total chip value changes
        new_total = result["stack"].total
        if new_total != self._last_chip_total:
            self._last_chip_total = new_total
            self.chips_detected.emit(result)
