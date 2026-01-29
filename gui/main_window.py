import os
from PySide6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
                               QLabel, QFrame, QListWidget, QTextEdit, QSizePolicy, QSpinBox)
from PySide6.QtCore import Qt, QFile, QTextStream, QTimer
from gui.utils import convert_cv_qt
from services.camera_service import CameraService

class MainWindow(QMainWindow):
    DEFAULT_FPS = 30

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Poker Robot Dashboard")
        self.resize(1200, 800)
        self.setMinimumSize(800, 600)
        
        # Load Stylesheet
        self.load_stylesheet()

        # Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

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

        # Separator
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setFrameShadow(QFrame.Sunken)
        left_layout.addWidget(line1)

        # Community Cards
        self.community_cards_label = QLabel("Community Cards: [ ] [ ] [ ] [ ] [ ]")
        self.community_cards_label.setObjectName("infoLabel")
        self.community_cards_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.community_cards_label)

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
        left_layout.addWidget(self.player_list)

        # Log Area
        left_layout.addWidget(QLabel("Game Log:"))
        self.log_area = QTextEdit()
        self.log_area.setObjectName("logArea")
        self.log_area.setReadOnly(True)
        self.log_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_layout.addWidget(self.log_area)

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
        self.fps_spinbox.setValue(self.DEFAULT_FPS)
        self.fps_spinbox.setFixedWidth(60)
        
        fps_layout.addWidget(fps_label)
        fps_layout.addWidget(self.fps_spinbox)
        right_layout.addWidget(fps_container)

        # Feed Container
        self.camera_feed = QLabel("Camera Feed Placeholder")
        self.camera_feed.setObjectName("cameraFeed")
        self.camera_feed.setAlignment(Qt.AlignCenter)
        self.camera_feed.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_feed.setScaledContents(True)
        self.camera_feed.setStyleSheet("background-color: #111; border: 1px solid #333;")
        right_layout.addWidget(self.camera_feed, 1)

        # Add panels to main layout with ratio (30% left, 70% right)
        main_layout.addWidget(left_panel, 3)
        main_layout.addWidget(right_panel, 7)

        # Camera Service & Timer
        self.camera_service = CameraService()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        
        # Connect FPS SpinBox
        self.fps_spinbox.valueChanged.connect(self.update_timer_interval)
        
        # Start Service and Timer
        self.camera_service.start()
        self.update_timer_interval(self.fps_spinbox.value())
        self.timer.start()

    def update_timer_interval(self, fps):
        if fps > 0:
            interval = int(1000 / fps)
            self.timer.setInterval(interval)

    def update_frame(self):
        frame = self.camera_service.get_frame()
        if frame is not None:
            qt_img = convert_cv_qt(frame)
            self.camera_feed.setPixmap(qt_img)

    def closeEvent(self, event):
        self.camera_service.stop()
        event.accept()

    def load_stylesheet(self):
        style_file = os.path.join(os.path.dirname(__file__), "styles.qss")
        if os.path.exists(style_file):
            with open(style_file, "r") as f:
                self.setStyleSheet(f.read())
        else:
            print(f"Warning: Stylesheet not found at {style_file}")
