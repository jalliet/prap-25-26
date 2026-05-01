import os
import math
import cv2
import numpy as np
from datetime import datetime
from PySide6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
                               QLabel, QFrame, QListWidget, QTextEdit, QSizePolicy, QSpinBox, QSplitter, QPushButton, QGroupBox)
from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QColor, QBrush, QPixmap, QPainter, QFont
from PySide6.QtSvgWidgets import QSvgWidget
from gui.utils import convert_cv_qt
from services.vision_controller import VisionController, VisionMode
from services.arm_ros_bridge import ArmRosBridge
from services.arm_choreographer import ArmChoreographer
from services.table_io_bridge import TableIoBridge
from poker.game_state import GameState, GamePhase
from poker.player import Player, PlayerStatus
from vision.draw_utils import draw_card_detections

from poker.action import Action, ActionType
from dataclasses import dataclass
from typing import Callable
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

        # Choreographer owns multi-step sequences (deal, flip, collect_pot,
        # home, pick_up_deck). XY positions are populated at runtime by the
        # CV pipeline through ``self.choreographer.table_map``.
        self.choreographer = ArmChoreographer(self.arm_bridge, parent=self)
        # UI status label connection lives ~line 447; both fire intentionally
        self.choreographer.sequence_finished.connect(
            self._on_choreographer_sequence_finished)

        # Table-side IO bridge (button + pump)
        self.table_io_bridge = TableIoBridge()
        self.table_io_bridge.turn_advance_requested.connect(self._on_button_press)
        # Pump round-trip: choreographer requests state, bridge confirms
        # settle, choreographer advances on the confirmation.
        self.choreographer.pump_requested.connect(self.table_io_bridge.set_pump)
        self.table_io_bridge.pump_state_set.connect(self.choreographer._on_pump_done)

        # Simulation process handle (set when user starts the sim)
        self.sim_process = None
        # Add some dummy players for testing purposes
        self.game_state.add_player(Player(0, "Hero", 0))
        self.game_state.add_player(Player(1, "Villain 1", 1))
        self.game_state.add_player(Player(2, "Villain 2", 2))
        # Cache player count for choreography helpers (community indices use
        # num_players + i so the choreographer can reuse seat lookups).
        self._num_players = len(self.game_state.players)

        self._sequence_queue: list[Callable[[], bool]] = []
        self._next_community_to_flip: int = 0

        # TODO: replace placeholder XY once CV emits seat/deck/pot positions
        self.choreographer.table_map.set_deck_xy(0.20, 0.00)
        self.choreographer.table_map.set_pot_xy(0.00, 0.00)
        N = self._num_players
        for i in range(N):
            angle = 2 * math.pi * i / N
            self.choreographer.table_map.set_seat_xy(
                i, 0.25 * math.cos(angle), 0.25 * math.sin(angle))
        for i in range(5):
            angle = 2 * math.pi * i / 5
            self.choreographer.table_map.set_seat_xy(
                N + i, 0.10 * math.cos(angle), 0.10 * math.sin(angle))
        
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
        
        # === Action controls (real betting) ===
        action_group = QWidget()
        action_layout = QHBoxLayout(action_group)
        action_layout.setContentsMargins(0, 0, 0, 0)

        self.btn_fold = QPushButton("Fold")
        self.btn_fold.clicked.connect(lambda: self._submit_action(ActionType.FOLD))
        action_layout.addWidget(self.btn_fold)

        self.btn_check = QPushButton("Check")
        self.btn_check.clicked.connect(lambda: self._submit_action(ActionType.CHECK))
        action_layout.addWidget(self.btn_check)

        self.btn_call = QPushButton("Call")
        self.btn_call.clicked.connect(lambda: self._submit_action(ActionType.CALL))
        action_layout.addWidget(self.btn_call)

        self.btn_bet = QPushButton("Bet")
        self.btn_bet.clicked.connect(lambda: self._submit_action_with_size(ActionType.BET))
        action_layout.addWidget(self.btn_bet)

        self.btn_raise = QPushButton("Raise")
        self.btn_raise.clicked.connect(lambda: self._submit_action_with_size(ActionType.RAISE))
        action_layout.addWidget(self.btn_raise)

        self.btn_all_in = QPushButton("All-In")
        self.btn_all_in.clicked.connect(lambda: self._submit_action(ActionType.ALL_IN))
        action_layout.addWidget(self.btn_all_in)

        left_layout.addWidget(action_group)

        # === Bet sizing row ===
        sizing_group = QWidget()
        sizing_layout = QHBoxLayout(sizing_group)
        sizing_layout.setContentsMargins(0, 0, 0, 0)

        sizing_layout.addWidget(QLabel("Size:"))
        self.bet_size_spinbox = QSpinBox()
        self.bet_size_spinbox.setRange(1, 9999)
        self.bet_size_spinbox.setValue(2)  # Default to BB
        self.bet_size_spinbox.setFixedWidth(80)
        sizing_layout.addWidget(self.bet_size_spinbox)

        for label, fn in [
            ("½ pot", lambda: self._set_bet_size_fraction(0.5)),
            ("Pot", lambda: self._set_bet_size_fraction(1.0)),
            ("2× pot", lambda: self._set_bet_size_fraction(2.0)),
            ("All-in", self._set_bet_size_all_in),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(fn)
            sizing_layout.addWidget(btn)

        left_layout.addWidget(sizing_group)

        # === Debug controls (collapsed under a separator) ===
        debug_group = QGroupBox("Debug")
        debug_group.setCheckable(True)
        debug_group.setChecked(False)
        debug_outer_layout = QVBoxLayout(debug_group)
        debug_outer_layout.setContentsMargins(4, 4, 4, 4)
        debug_outer_layout.setSpacing(4)

        debug_row1 = QHBoxLayout()
        debug_row1.setContentsMargins(0, 0, 0, 0)

        self.btn_start_hand = QPushButton("Start Hand")
        self.btn_start_hand.clicked.connect(self.start_hand_test)
        debug_row1.addWidget(self.btn_start_hand)

        self.btn_bet_test = QPushButton("Test Bet")
        self.btn_bet_test.clicked.connect(self.bet_test)
        debug_row1.addWidget(self.btn_bet_test)

        self.btn_toggle_detection = QPushButton("Toggle Card Detection")
        self.btn_toggle_detection.clicked.connect(self.toggle_card_detection)
        debug_row1.addWidget(self.btn_toggle_detection)

        self.btn_start_sim = QPushButton("Start Sim")
        self.btn_start_sim.clicked.connect(self.start_simulation)
        debug_row1.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton("Stop Sim")
        self.btn_stop_sim.clicked.connect(self.stop_simulation)
        debug_row1.addWidget(self.btn_stop_sim)

        debug_outer_layout.addLayout(debug_row1)

        # Arm choreography manual triggers (bypass the queue).
        debug_row2 = QHBoxLayout()
        debug_row2.setContentsMargins(0, 0, 0, 0)

        self.btn_home = QPushButton("Home")
        self.btn_home.clicked.connect(lambda: self.choreographer.home())
        debug_row2.addWidget(self.btn_home)

        self.btn_pick_up_deck = QPushButton("Pick Up Deck")
        self.btn_pick_up_deck.clicked.connect(lambda: self.choreographer.pick_up_deck())
        debug_row2.addWidget(self.btn_pick_up_deck)

        self.deal_seat_spin = QSpinBox()
        self.deal_seat_spin.setRange(0, max(0, self._num_players - 1))
        self.deal_seat_spin.setFixedWidth(50)
        self.btn_deal_to_seat = QPushButton("Deal to Seat")
        self.btn_deal_to_seat.clicked.connect(
            lambda: self.choreographer.deal_card_to_seat(self.deal_seat_spin.value()))
        debug_row2.addWidget(self.btn_deal_to_seat)
        debug_row2.addWidget(self.deal_seat_spin)

        self.flip_card_spin = QSpinBox()
        self.flip_card_spin.setRange(0, 4)
        self.flip_card_spin.setFixedWidth(50)
        self.btn_flip_card = QPushButton("Flip Card")
        self.btn_flip_card.clicked.connect(
            lambda: self.choreographer.flip_card(self._num_players + self.flip_card_spin.value()))
        debug_row2.addWidget(self.btn_flip_card)
        debug_row2.addWidget(self.flip_card_spin)

        self.btn_collect_pot = QPushButton("Collect Pot")
        self.btn_collect_pot.clicked.connect(lambda: self.choreographer.collect_pot())
        debug_row2.addWidget(self.btn_collect_pot)

        debug_outer_layout.addLayout(debug_row2)

        left_layout.addWidget(debug_group)

        # Right Panel (Camera Feed)
        right_panel = QWidget()
        right_panel.setObjectName("rightPanel")
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(20, 20, 20, 20)
        right_layout.setSpacing(10)

        cam_header = QLabel("Birdseye Feed")
        cam_header.setObjectName("cameraHeaderLabel")
        cam_header.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(cam_header)

        # FPS Control
        fps_container = QWidget()
        fps_layout = QHBoxLayout(fps_container)
        fps_layout.setAlignment(Qt.AlignCenter)
        fps_layout.setContentsMargins(0, 0, 0, 0)
        
        fps_label = QLabel("Camera FPS:")
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

        # Choreographer status
        self.sequence_status_label = QLabel("")
        self.sequence_status_label.setAlignment(Qt.AlignCenter)
        self.sequence_status_label.setStyleSheet("font-weight: bold; color: #88CCFF;")
        right_layout.addWidget(self.sequence_status_label)

        self.sequence_rejection_label = QLabel("")
        self.sequence_rejection_label.setAlignment(Qt.AlignCenter)
        self.sequence_rejection_label.setStyleSheet("font-weight: bold; color: #FF5555;")
        right_layout.addWidget(self.sequence_rejection_label)

        # Feed Container — OAK-D Lite primary feed (expands to fill available space)
        self.camera_feed = QLabel("Camera Feed Placeholder")
        self.camera_feed.setObjectName("cameraFeed")
        self.camera_feed.setAlignment(Qt.AlignCenter)
        self.camera_feed.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_feed.setStyleSheet("background-color: #111; border: 1px solid #333;")
        right_layout.addWidget(self.camera_feed, 1)

        # Chip camera section — C925e secondary feed (fixed 180px height)
        chip_header = QLabel("Chip SideView Feed")
        chip_header.setObjectName("cameraHeaderLabel")
        chip_header.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(chip_header)

        self.chip_feed = QLabel("Chip Feed")
        self.chip_feed.setObjectName("cameraFeed")
        self.chip_feed.setAlignment(Qt.AlignCenter)
        self.chip_feed.setFixedHeight(180)
        self.chip_feed.setStyleSheet("background-color: #111; border: 1px solid #333;")
        right_layout.addWidget(self.chip_feed)

        self.chip_result_label = QLabel("Chip stack: —")
        self.chip_result_label.setObjectName("infoLabel")
        self.chip_result_label.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.chip_result_label)

        self._last_birdseye_frame_time: float = 0.0
        self._last_chip_frame_time: float = 0.0
        self._no_signal_timer = QTimer(self)
        self._no_signal_timer.timeout.connect(self._check_no_signal)
        self._no_signal_timer.start(1000)  # 1Hz

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
        self.game_state.on_hand_started.connect(self._on_hand_started)
        self.game_state.on_pot_change.connect(self.on_pot_change)
        self.game_state.on_card_detection_required.connect(self.on_cards_updated)
        self.game_state.on_turn_change.connect(self.on_turn_change)
        self.game_state.on_player_action.connect(lambda _action: self.update_player_list())
        self.game_state.on_turn_change.connect(lambda _player: self.update_player_list())
        self.game_state.on_pot_change.connect(lambda _total: self.update_player_list())
        self.game_state.on_hand_started.connect(self.update_player_list)
        self.game_state.on_action_rejected.connect(self._on_action_rejected)

        # GameState orchestration hooks for the choreography queue.
        self.game_state.on_dealing_required.connect(self._on_dealing_required)
        self.game_state.on_community_flip_required.connect(self._on_community_flip_required)
        self.game_state.on_pot_collection_required.connect(self._on_pot_collection_required)
        self.game_state.on_hand_started.connect(self._reset_community_flip_counter)

        # Choreographer status surfacing.
        self.choreographer.sequence_started.connect(self._on_sequence_started)
        self.choreographer.sequence_step.connect(self._on_sequence_step)
        # Queue dispatch handler wired at ~line 85; both fire intentionally
        self.choreographer.sequence_finished.connect(self._on_sequence_finished_status)

        # Connect Vision Controller Signals
        self.vision_controller.frame_ready.connect(self.update_frame)
        self.vision_controller.cards_detected.connect(self._on_cards_detected)
        self.vision_controller.chip_frame_ready.connect(self._update_chip_feed)
        self.vision_controller.chips_detected.connect(self._on_chips_detected)

        # Start Service
        self.vision_controller.start()
        
        # Initial FPS setting
        self.fps_spinbox.valueChanged.connect(self.vision_controller.set_fps)

        self.game_state.on_player_action.connect(lambda _a: self._update_button_states())
        self.game_state.on_turn_change.connect(lambda _p: self._update_button_states())
        self.game_state.on_pot_change.connect(lambda _t: self._update_button_states())
        self.game_state.on_phase_change.connect(lambda _ph: self._update_button_states())
        self.game_state.on_hand_started.connect(self._update_button_states)

        # Initial button-state evaluation
        self._update_button_states()

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

    def _submit_action(self, action_type: ActionType):
        """Submit an action for the current player to GameState."""
        player = self.game_state.current_player
        if player is None:
            self._on_action_rejected("No current player")
            return
        action = Action(player.player_id, action_type, 0)
        self.game_state.process_action(action)

    def _submit_action_with_size(self, action_type: ActionType):
        player = self.game_state.current_player
        if player is None:
            self._on_action_rejected("No current player")
            return
        amount = self.bet_size_spinbox.value()
        action = Action(player.player_id, action_type, amount)
        self.game_state.process_action(action)

    def _update_button_states(self):
        """Enable/disable action buttons based on current GameState."""
        player = self.game_state.current_player
        in_hand = player is not None and player.status == PlayerStatus.ACTIVE
        current_bet = self.game_state.current_bet_amount
        my_bet = player.current_bet if player else 0

        # Fold always available when it's your turn
        self.btn_fold.setEnabled(in_hand)
        # Check only when no bet to call
        self.btn_check.setEnabled(in_hand and my_bet >= current_bet)
        # Call only when there is a bet to call
        self.btn_call.setEnabled(in_hand and my_bet < current_bet)
        # Bet only when no current bet (otherwise it's a raise)
        self.btn_bet.setEnabled(in_hand and current_bet == 0)
        # Raise only when there is a bet to raise
        self.btn_raise.setEnabled(in_hand and current_bet > 0)
        # All-in always available when it's your turn (and you have chips)
        self.btn_all_in.setEnabled(in_hand and player.stack.total > 0)

        # Sim controls: enable Start when no sim, Stop when sim running
        self.btn_start_sim.setEnabled(self.sim_process is None)
        self.btn_stop_sim.setEnabled(self.sim_process is not None)

    def _set_bet_size_fraction(self, frac: float):
        pot = self.game_state.pot.total
        size = max(1, int(pot * frac))
        self.bet_size_spinbox.setValue(size)

    def _set_bet_size_all_in(self):
        player = self.game_state.current_player
        if player is None:
            return
        self.bet_size_spinbox.setValue(player.stack.total)

    def _on_action_rejected(self, reason: str):
        """Engine-level validation feedback."""
        self.log_message(f"[Action rejected] {reason}")

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
            item = self.player_list.item(self.player_list.count() - 1)
            if p == self.game_state.current_player:
                item.setBackground(QBrush(QColor("#264F4F")))
            if p.status == PlayerStatus.FOLDED:
                item.setForeground(QBrush(QColor("#888888")))
            elif p.status == PlayerStatus.ALL_IN:
                item.setForeground(QBrush(QColor("#FFAA00")))

    def on_phase_change(self, phase: GamePhase):
        self.phase_label.setText(f"Phase: {phase.name}")
        self.log_message(f"Game Phase: {phase.name}")
        self.update_player_list()

    def _on_hand_started(self):
        """Clear card slots and chip-result label at the start of every hand."""
        for slot in self.card_slots:
            slot.load(b"")
            slot.setStyleSheet(self.config.card_slot.empty_style)
        self.chip_result_label.setText("Chip stack: —")
        self.log_message("--- New hand ---")

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
        VisionMode.CARD_READING: "#00FFFF",  # Cyan
        # IDLE falls back to the default yellow in update_mode_label
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
        """Updates the OAK-D feed, preserving aspect ratio."""
        if frame is None:
            return
        self._last_birdseye_frame_time = datetime.now().timestamp()
        qt_img = convert_cv_qt(frame)
        scaled = qt_img.scaled(
            self.camera_feed.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_feed.setPixmap(scaled)

    def _update_chip_feed(self, frame: np.ndarray):
        """Updates the C925e feed, preserving aspect ratio."""
        if frame is None:
            return
        self._last_chip_frame_time = datetime.now().timestamp()
        qt_img = convert_cv_qt(frame)
        scaled = qt_img.scaled(
            self.chip_feed.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.chip_feed.setPixmap(scaled)

    def _check_no_signal(self):
        """Paint a 'no signal' fallback when a feed has been silent for >1.5s."""
        now = datetime.now().timestamp()
        STALE_S = 1.5
        if now - self._last_birdseye_frame_time > STALE_S:
            self._paint_no_signal(self.camera_feed, "OAK-D")
        if now - self._last_chip_frame_time > STALE_S:
            self._paint_no_signal(self.chip_feed, "C925e")

    def _paint_no_signal(self, label, name: str):
        """Render a black QPixmap with centred 'No signal: NAME' text."""
        size = label.size()
        if size.width() <= 0 or size.height() <= 0:
            return
        pix = QPixmap(size)
        pix.fill(QColor("#111111"))
        painter = QPainter(pix)
        painter.setPen(QColor("#FF4444"))
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(pix.rect(), Qt.AlignCenter, f"No signal: {name}")
        painter.end()
        label.setPixmap(pix)

    def _on_chips_detected(self, result: dict):
        """Updates the chip stack label. Triggered by chips_detected on total change."""
        stack = result.get("stack")
        if stack is not None:
            self.chip_result_label.setText(f"Chip stack: {stack.total}")

    def _on_button_press(self, seat: int):
        """Handle physical button press: advance turn in GameState."""
        self.game_state.next_turn()
        self.log_message(f"[Button] Turn advance (seat hint={seat})")

    def _on_arm_connection_changed(self, available: bool):
        status = "Connected" if available else "Disconnected"
        self.log_message(f"Arm Controller: {status}")

    def _on_arm_move_completed(self, success: bool, final_error: float):
        if success:
            self.log_message(f"Arm move complete (error: {final_error:.3f} rad)")
        else:
            self.log_message(f"Arm move failed (error: {final_error:.3f})")

    def _on_choreographer_sequence_finished(self, name: str, success: bool):
        """Log the terminal status of a multi-step arm sequence."""
        status = "OK" if success else "FAILED"
        self.log_message(f"Sequence '{name}': {status}")
        if success:
            self._dispatch_next_sequence()
        else:
            # On failure: drop pending sequences and force the pump off so a
            # held card is released. The status label is updated separately
            # by ``_on_sequence_finished_status``.
            self._sequence_queue.clear()
            try:
                self.table_io_bridge.set_pump(False)
            except Exception:
                pass

    def _dispatch_next_sequence(self):
        """Pop and invoke the next queued sequence factory, if any."""
        if not self._sequence_queue:
            return
        next_call = self._sequence_queue.pop(0)
        next_call()

    def _on_dealing_required(self, active_seats: list, cards_per_seat: int):
        """Build the dealing queue: pick_up_deck, deal_to_seat per card per seat."""
        for _round in range(cards_per_seat):
            for seat in active_seats:
                self._sequence_queue.append(lambda: self.choreographer.pick_up_deck())
                self._sequence_queue.append(
                    lambda s=seat: self.choreographer.deal_card_to_seat(s))
        self._dispatch_next_sequence()

    def _on_community_flip_required(self, count: int):
        """Append flips for the next ``count`` un-flipped community cards."""
        base = self._next_community_to_flip
        for i in range(count):
            seat_index = self._num_players + base + i
            self._sequence_queue.append(
                lambda s=seat_index: self.choreographer.flip_card(s))
        self._next_community_to_flip += count
        self._dispatch_next_sequence()

    def _on_pot_collection_required(self, winner_seat: int):
        """Queue a pot collection sequence for the winning seat."""
        # winner_seat is informational; collect_pot reads pot_xy from TableMap directly
        self._sequence_queue.append(lambda: self.choreographer.collect_pot())
        self._dispatch_next_sequence()

    def _reset_community_flip_counter(self):
        """Reset the community-card flip cursor at the start of every hand."""
        self._num_players = len(self.game_state.players)
        self._next_community_to_flip = 0

    def _on_sequence_started(self, name: str):
        self.sequence_status_label.setText(f"Sequence: {name} (step 0)")
        self.sequence_rejection_label.setText("")

    def _on_sequence_step(self, name: str, idx: int):
        self.sequence_status_label.setText(f"Sequence: {name} (step {idx})")

    def _on_sequence_finished_status(self, name: str, success: bool):
        self.sequence_status_label.setText("")
        if not success:
            self.sequence_rejection_label.setText(f"Last rejection: {name}")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_B:
            self._debug_birdseye()
        elif event.key() == Qt.Key_C:
            self._debug_chip_seg()
        else:
            super().keyPressEvent(event)

    def _debug_birdseye(self):
        """One-shot card detection: annotate + save to debug_inference/birdseye/."""
        frame = self.vision_controller.get_frame()
        if frame is None:
            self.log_message("Debug birdseye: no frame available")
            return

        detections = self.vision_controller.card_detector.process(frame)
        draw_card_detections(frame, detections)

        out_dir = os.path.join(os.getcwd(), "debug_inference", "birdseye")
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(out_dir, f"{ts}.png")
        cv2.imwrite(path, frame)
        self.log_message(f"Debug birdseye saved: {path}")

    def _debug_chip_seg(self):
        """One-shot chip segmentation: annotate + save to debug_inference/chip_seg/."""
        frame = self.vision_controller.chip_seg_service.get_frame()
        if frame is None:
            self.log_message("Debug chip seg: no frame available")
            return

        model = self.vision_controller.chip_segmentor.model
        if model is not None:
            results = model(frame, verbose=False)
            annotated = results[0].plot()
        else:
            annotated = frame.copy()

        out_dir = os.path.join(os.getcwd(), "debug_inference", "chip_seg")
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(out_dir, f"{ts}.png")
        cv2.imwrite(path, annotated)
        self.log_message(f"Debug chip seg saved: {path}")

    def closeEvent(self, event):
        # Stop simulation if running
        try:
            self.stop_simulation()
        except Exception:
            pass

        self.vision_controller.stop()
        self.arm_bridge.shutdown()
        try:
            self.table_io_bridge.shutdown()
        except Exception:
            pass
        event.accept()

    def load_stylesheet(self):
        style_file = os.path.join(os.path.dirname(__file__), "styles.qss")
        if os.path.exists(style_file):
            with open(style_file, "r") as f:
                self.setStyleSheet(f.read())
        else:
            print(f"Warning: Stylesheet not found at {style_file}")
