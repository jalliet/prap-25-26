import os
import numpy as np
from PySide6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
                               QLabel, QFrame, QListWidget, QTextEdit, QSizePolicy, QSpinBox, QSplitter, QPushButton)
from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtSvgWidgets import QSvgWidget
from gui.utils import convert_cv_qt
from services.vision_controller import VisionController, VisionMode
from services.arm_ros_bridge import ArmRosBridge
from poker.game_state import GameState, GamePhase
from poker.player import Player

from poker.action import Action, ActionType
from dataclasses import dataclass
from typing import Tuple
import subprocess
import signal

@dataclass(frozen=True)
class WindowConfig:
    """Configuration for the main application window."""
    title: str = "Poker Robot Dashboard"
    width: int = 1200
    height: int = 800
    min_width: int = 800
    min_height: int = 600

@dataclass(frozen=True)
class CardSlotConfig:
    """Configuration for the visual appearance of card slots."""
    width: int = 60
    height: int = 84
    # Style for an empty slot (placeholder) - slightly darker than background (#2F2F2F) for depth
    empty_style: str = "background-color: #222222; border: 1px solid #444444; border-radius: 4px;"
    # Style for a filled slot (transparent to show SVG)
    filled_style: str = "background-color: transparent; border: none;"
    # Spacing between cards in layout
    spacing: int = 10

@dataclass(frozen=True)
class UIConfig:
    """Global UI Configuration container."""
    window: WindowConfig = WindowConfig()
    card_slot: CardSlotConfig = CardSlotConfig()
    
class MainWindow(QMainWindow):
    """
    Main application window acting as the primary User Interface.
    
    This class integrates the GameState logic, Vision Controller, and UI components
    to provide a comprehensive dashboard for monitoring the poker game.
    """
    def __init__(self, config: UIConfig = UIConfig()):
        super().__init__()
        self.config = config
        
        self.setWindowTitle(self.config.window.title)
        self.resize(self.config.window.width, self.config.window.height)
        self.setMinimumSize(self.config.window.min_width, self.config.window.min_height)
        
        # Initialise Backend Components
        self.game_state = GameState()
        self.vision_controller = VisionController()
        
        # Connect Vision Controller to Game State
        self.vision_controller.connect_to_game_state(self.game_state)

        # Initialise Arm Bridge (gracefully degrades if ROS 2 unavailable)
        self.arm_bridge = ArmRosBridge()
        self.vision_controller.connect_to_arm_bridge(self.arm_bridge)
        self.arm_bridge.connection_changed.connect(self._on_arm_connection_changed)
        self.arm_bridge.move_completed.connect(self._on_arm_move_completed)
        
        # Simulation process handle (set when user starts the sim)
        self.sim_process = None
        # Add some dummy players for testing purposes
        self.game_state.add_player(Player(0, "Hero", 0))
        self.game_state.add_player(Player(1, "Villain 1", 1))
        self.game_state.add_player(Player(2, "Villain 2", 2))
        
        # Load Stylesheet
        self.load_stylesheet()

        # Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Main Splitter
        self.splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(self.splitter)

        # Left Panel (Game State)
        left_panel = QWidget()
        left_panel.setObjectName("leftPanel")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(20, 20, 20, 20)
        left_layout.setSpacing(15)

        # Header: GAME STATE
        gs_header = QLabel("Game State")
        gs_header.setObjectName("headerLabel")
        gs_header.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(gs_header)
        
        # Phase Label
        self.phase_label = QLabel(f"Phase: {self.game_state.phase.name}")
        self.phase_label.setObjectName("infoLabel")
        self.phase_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.phase_label)

        # Separator
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setFrameShadow(QFrame.Sunken)
        left_layout.addWidget(line1)

        # Community Cards
        self.community_cards_layout = QHBoxLayout()
        self.community_cards_layout.setSpacing(self.config.card_slot.spacing)
        self.community_cards_layout.setAlignment(Qt.AlignCenter)
        
        # Create 5 slots for community cards
        self.card_slots = []
        for _ in range(5):
            slot = QSvgWidget()
            slot.setFixedSize(self.config.card_slot.width, self.config.card_slot.height)
            # Initial placeholder style
            slot.setStyleSheet(self.config.card_slot.empty_style)
            self.community_cards_layout.addWidget(slot)
            self.card_slots.append(slot)
            
        left_layout.addLayout(self.community_cards_layout)

        # Pot Display
        self.pot_label = QLabel("Pot: 0")
        self.pot_label.setObjectName("potLabel")
        self.pot_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.pot_label)

        # Separator
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setFrameShadow(QFrame.Sunken)
        left_layout.addWidget(line2)

        # Player List
        left_layout.addWidget(QLabel("Players:"))
        self.player_list = QListWidget()
        self.player_list.setObjectName("playerList")
        self.player_list.setWordWrap(True)
        self.player_list.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        left_layout.addWidget(self.player_list)
        
        # Update initial player list
        self.update_player_list()

        # Log Area
        left_layout.addWidget(QLabel("Game Log:"))
        self.log_area = QTextEdit()
        self.log_area.setObjectName("logArea")
        self.log_area.setReadOnly(True)
        self.log_area.setLineWrapMode(QTextEdit.WidgetWidth)
        self.log_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.log_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_layout.addWidget(self.log_area)
        
        # Test Controls (Bottom of Left Panel)
        controls_group = QWidget()
        controls_layout = QHBoxLayout(controls_group)
        controls_layout.setContentsMargins(0,0,0,0)
        
        self.btn_start_hand = QPushButton("Start Hand")
        self.btn_start_hand.clicked.connect(self.start_hand_test)
        controls_layout.addWidget(self.btn_start_hand)
        
        self.btn_bet_test = QPushButton("Test Bet")
        self.btn_bet_test.clicked.connect(self.bet_test)
        controls_layout.addWidget(self.btn_bet_test)

        self.btn_toggle_detection = QPushButton("Toggle Card Detection")
        self.btn_toggle_detection.clicked.connect(self.toggle_card_detection)
        controls_layout.addWidget(self.btn_toggle_detection)

        # Simulation controls
        self.btn_start_sim = QPushButton("Start Simulation")
        self.btn_start_sim.clicked.connect(self.start_simulation)
        controls_layout.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton("Stop Simulation")
        self.btn_stop_sim.clicked.connect(self.stop_simulation)
        controls_layout.addWidget(self.btn_stop_sim)

        left_layout.addWidget(controls_group)

        # Right Panel (Camera Feed)
        right_panel = QWidget()
        right_panel.setObjectName("rightPanel")
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(20, 20, 20, 20)
        right_layout.setSpacing(10)

        cam_header = QLabel("Poker Camera Feed")
        cam_header.setObjectName("cameraHeaderLabel")
        cam_header.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(cam_header)

        # FPS Control
        fps_container = QWidget()
        fps_layout = QHBoxLayout(fps_container)
        fps_layout.setAlignment(Qt.AlignCenter)
        fps_layout.setContentsMargins(0, 0, 0, 0)
        
        fps_label = QLabel("Feed FPS:")
        self.fps_spinbox = QSpinBox()
        self.fps_spinbox.setRange(1, 60)
        self.fps_spinbox.setValue(self.vision_controller.fps)
        self.fps_spinbox.setFixedWidth(60)
        
        # Vision Mode Indicator
        self.mode_label = QLabel("Mode: IDLE")
        self.mode_label.setStyleSheet("font-weight: bold; color: yellow;")
        
        fps_layout.addWidget(fps_label)
        fps_layout.addWidget(self.fps_spinbox)
        fps_layout.addSpacing(20)
        fps_layout.addWidget(self.mode_label)
        right_layout.addWidget(fps_container)

        # Feed Container
        self.camera_feed = QLabel("Camera Feed Placeholder")
        self.camera_feed.setObjectName("cameraFeed")
        self.camera_feed.setAlignment(Qt.AlignCenter)
        self.camera_feed.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_feed.setScaledContents(True)
        self.camera_feed.setStyleSheet("background-color: #111; border: 1px solid #333;")
        right_layout.addWidget(self.camera_feed, 1)

        # Add panels to splitter with initial ratio (30% left, 70% right)
        self.splitter.addWidget(left_panel)
        self.splitter.addWidget(right_panel)
        self.splitter.setStretchFactor(0, 3)
        self.splitter.setStretchFactor(1, 7)

        # Vision Controller & Timer (Timer removed, using signals)
        # self.timer = QTimer(self)
        
        # Connect FPS SpinBox
        # self.fps_spinbox.valueChanged.connect(self.update_timer_interval)
        
        # Connect Game Signals to UI Updates
        self.game_state.on_phase_change.connect(self.on_phase_change)
        self.game_state.on_pot_change.connect(self.on_pot_change)
        self.game_state.on_card_detection_required.connect(self.on_cards_updated)
        self.game_state.on_turn_change.connect(self.on_turn_change)
        
        # Connect Vision Controller Signals
        self.vision_controller.frame_ready.connect(self.update_frame)
        self.vision_controller.cards_detected.connect(self._on_cards_detected)

        # Start Service
        self.vision_controller.start()
        
        # Initial FPS setting
        self.fps_spinbox.valueChanged.connect(self.vision_controller.set_fps)

    def start_hand_test(self):
        """Manually trigger a new hand."""
        self.log_message("--- Manual Trigger: Start Hand ---")
        self.game_state.start_new_hand()
        # Initial betting round needs someone to act
        # This is just a test hook

    def bet_test(self):
        """Manually trigger a bet action for current player."""
        player = self.game_state.current_player
        if player:
            self.log_message(f"--- Manual Trigger: {player.name} Bets 50 ---")
            # In a real game, logic would handle amount.
            # Here we force a bet/raise
            try:
                if self.game_state.current_bet_amount == 0:
                     action = Action(player.player_id, ActionType.BET, 50)
                     self.game_state.process_action(action)
                else:
                     action = Action(player.player_id, ActionType.CALL)
                     self.game_state.process_action(action)
            except Exception as e:
                self.log_message(f"Error: {e}")
            self.update_player_list()

    def toggle_card_detection(self):
        """Toggle between CARD_READING and IDLE vision modes."""
        if self.vision_controller.mode == VisionMode.CARD_READING:
            self.vision_controller.set_mode(VisionMode.IDLE)
            self.log_message("Card detection OFF")
        else:
            self.vision_controller.set_mode(VisionMode.CARD_READING)
            self.log_message("Card detection ON")
        self.update_mode_label()

    def _on_cards_detected(self, detections):
        """Handle card detection results. Only called when detected set changes."""
        valid = [d for d in detections if d["rank"] is not None and d["suit"] is not None]
        if valid:
            valid.sort(key=lambda d: d["bbox"]["x"])
            card_strs = [f"{d['rank']}{d['suit']}" for d in valid]
            self.log_message(f"Detected: {', '.join(card_strs)}")
        else:
            self.log_message("No cards detected")

    def update_player_list(self):
        self.player_list.clear()
        for p in self.game_state.players:
            self.player_list.addItem(str(p))

    def on_phase_change(self, phase: GamePhase):
        self.phase_label.setText(f"Phase: {phase.name}")
        self.log_message(f"Game Phase: {phase.name}")
        self.update_player_list()

    def start_simulation(self):
        """Start the ROS2/Gazebo simulation by launching the bringup launch file.

        This runs a bash subshell that sources the local `install/setup.bash`
        (if present) so the workspace and ROS 2 environment are available.
        The launched process is stored so we can stop it later.
        """
        if self.sim_process is not None:
            self.log_message("Simulation already running")
            return

        # Construct command: source workspace then launch poker_bringup
        workspace_setup = os.path.join(os.getcwd(), "install", "setup.bash")
        if os.path.exists(workspace_setup):
            cmd = f"bash -lc 'source {workspace_setup} && ros2 launch poker_bringup poker_arm.launch.py mode:=sim'"
        else:
            # Fallback to attempting a direct ros2 launch (user must have ROS sourced externally)
            cmd = "bash -lc 'ros2 launch poker_bringup poker_arm.launch.py mode:=sim'"

        try:
            # Start in its own process group so we can terminate whole group later
            self.sim_process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid,
                                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.log_message("Started simulation (ros2 launch)")
        except Exception as e:
            self.log_message(f"Failed to start simulation: {e}")

    def stop_simulation(self):
        """Terminate the simulation process started with `start_simulation`.

        Sends SIGINT to the process group then SIGTERM if needed.
        """
        if self.sim_process is None:
            self.log_message("No simulation process to stop")
            return

        try:
            os.killpg(self.sim_process.pid, signal.SIGINT)
        except Exception:
            pass

        try:
            self.sim_process.wait(timeout=5)
        except Exception:
            try:
                os.killpg(self.sim_process.pid, signal.SIGTERM)
            except Exception:
                pass

        self.sim_process = None
        self.log_message("Simulation stopped")

    def on_pot_change(self, total: int):
        self.pot_label.setText(f"Pot: {total}")
        self.log_message(f"Pot updated: {total}")

    def on_cards_updated(self, cards):
        """
        Updates the UI to display the community cards using SVG assets.
        Expects cards to be a list of Card objects.
        """
        # Format for log
        card_str = " ".join([f"[{str(c)}]" for c in cards])
        self.log_message(f"Cards updated: {card_str}")
        
        # Clear/Reset slots if needed (e.g. new hand)
        # For now, we just overwrite from left to right
        
        for i, slot in enumerate(self.card_slots):
            if i < len(cards):
                card = cards[i]
                # Construct filename: RankCode + SuitCode .svg (e.g. '10H.svg', 'AC.svg')
                # Rank.code returns '2'-'9', 'T', 'J', 'Q', 'K', 'A'
                # Suit.code returns 'H', 'D', 'C', 'S'
                
                # Note: Our asset generation used '10' not 'T', so we need to handle that mapping if Rank.code returns 'T'
                r_code = card.rank.code
                if r_code == 'T':
                    r_code = '10'
                    
                filename = f"{r_code}{card.suit.code}.svg"
                filepath = os.path.join(os.path.dirname(__file__), "..", "assets", filename)
                
                if os.path.exists(filepath):
                    slot.load(filepath)
                    slot.setStyleSheet(self.config.card_slot.filled_style)
                else:
                    self.log_message(f"Error: Asset not found {filepath}")
            else:
                # Reset empty slots
                slot.load(b"") # Clear SVG
                slot.setStyleSheet(self.config.card_slot.empty_style)

    def on_turn_change(self, player):
        if player:
            self.log_message(f"Turn: {player.name}")
            # Highlight player in list (optional)

    _MODE_COLOURS = {
        VisionMode.CARD_READING: "#00FFFF",      # Cyan
        VisionMode.CHIP_SEGMENTATION: "#FFA500", # Orange
    }

    def update_mode_label(self):
        """Updates the vision mode indicator."""
        mode = self.vision_controller.mode
        self.mode_label.setText(f"Mode: {mode.name}")
        colour = self._MODE_COLOURS.get(mode, "yellow")
        self.mode_label.setStyleSheet(f"font-weight: bold; color: {colour};")

    def log_message(self, message):
        self.log_area.append(message)

    def update_frame(self, frame: np.ndarray):
        """
        Updates the camera feed with the latest frame from the VisionController.
        Triggered by the frame_ready signal.
        """
        if frame is not None:
            qt_img = convert_cv_qt(frame)
            self.camera_feed.setPixmap(qt_img)

    def _on_arm_connection_changed(self, available: bool):
        status = "Connected" if available else "Disconnected"
        self.log_message(f"Arm Controller: {status}")

    def _on_arm_move_completed(self, success: bool, final_error: float):
        if success:
            self.log_message(f"Arm move complete (error: {final_error:.3f} rad)")
        else:
            self.log_message(f"Arm move failed (error: {final_error:.3f})")

    def closeEvent(self, event):
        # Stop simulation if running
        try:
            self.stop_simulation()
        except Exception:
            pass

        self.vision_controller.stop()
        self.arm_bridge.shutdown()
        event.accept()

    def load_stylesheet(self):
        style_file = os.path.join(os.path.dirname(__file__), "styles.qss")
        if os.path.exists(style_file):
            with open(style_file, "r") as f:
                self.setStyleSheet(f.read())
        else:
            print(f"Warning: Stylesheet not found at {style_file}")
