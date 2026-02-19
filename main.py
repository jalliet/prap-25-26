import sys
from PySide6.QtWidgets import QApplication
from gui.main_window import MainWindow

def main():
    """
    Application Entry Point.
    
    This function initialises the Qt Application framework and launches the main
    dashboard window. It serves as the root of the dependency graph, instantiating
    the GUI which in turn bootstraps the GameState and VisionController services.
    """
    app = QApplication(sys.argv)
    
    # Create and display the main window
    window = MainWindow()
    window.show()
    
    # Start the Qt event loop
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
