"""Tests for VisionController dual-camera refactor."""
import sys
import unittest
from unittest.mock import patch, MagicMock


def _block_hardware():
    """Return a patch.dict that prevents real hardware imports.

    Blocks depthai and ultralytics. cv2 is NOT blocked — it is a standard
    dependency present in the test environment. Individual tests patch
    specific cv2 symbols (e.g. VideoCapture) as needed.
    """
    return patch.dict('sys.modules', {
        'depthai': MagicMock(),
        'ultralytics': MagicMock(),
    })


class TestVisionControllerDualCamera(unittest.TestCase):
    """Verify VisionController has chip camera signals and no CHIP_SEGMENTATION mode."""

    def test_chip_segmentation_not_in_vision_mode(self):
        with _block_hardware():
            sys.modules.pop('services.vision_controller', None)
            from services.vision_controller import VisionMode
            mode_names = [m.name for m in VisionMode]
            self.assertNotIn('CHIP_SEGMENTATION', mode_names)

    def test_vision_mode_only_has_idle_and_card_reading(self):
        with _block_hardware():
            sys.modules.pop('services.vision_controller', None)
            from services.vision_controller import VisionMode
            mode_names = sorted(m.name for m in VisionMode)
            self.assertEqual(mode_names, ['CARD_READING', 'IDLE'])

    def test_chip_frame_ready_signal_exists(self):
        with _block_hardware():
            sys.modules.pop('services.vision_controller', None)
            from services.vision_controller import VisionController
            self.assertTrue(hasattr(VisionController, 'chip_frame_ready'))

    def test_chips_detected_signal_exists(self):
        with _block_hardware():
            sys.modules.pop('services.vision_controller', None)
            from services.vision_controller import VisionController
            self.assertTrue(hasattr(VisionController, 'chips_detected'))

    def test_webcam_service_attribute_after_init(self):
        # Clear all relevant module caches so patching takes effect before
        # class bodies are evaluated (BirdseyeConfig default args reference dai
        # at class-definition time, so birdseye_service must be re-imported too).
        for mod in ['services.vision_controller', 'services.birdseye_service',
                    'services.chip_seg_service']:
            sys.modules.pop(mod, None)

        # QTimer construction requires a running QApplication — create a minimal
        # one if none exists.
        try:
            from PySide6.QtWidgets import QApplication
            app = QApplication.instance() or QApplication([])
        except Exception:
            app = None  # PySide6 unavailable in this env

        with _block_hardware():
            with patch('services.chip_seg_service.cv2', MagicMock()):
                from services.vision_controller import VisionController
                VisionController._instance = None
                vc = VisionController()
                self.assertTrue(hasattr(vc, 'chip_seg_service'))
                VisionController._instance = None


class TestMainWindowModeColours(unittest.TestCase):
    """Ensure _MODE_COLOURS only references valid VisionMode values."""

    def test_mode_colours_keys_are_valid_vision_modes(self):
        with _block_hardware():
            sys.modules.pop('services.vision_controller', None)
            sys.modules.pop('gui.main_window', None)
            with patch.dict('sys.modules', {
                'PySide6': MagicMock(),
                'PySide6.QtWidgets': MagicMock(),
                'PySide6.QtCore': MagicMock(),
                'PySide6.QtSvgWidgets': MagicMock(),
                'services.arm_ros_bridge': MagicMock(),
                'poker.game_state': MagicMock(),
                'poker.player': MagicMock(),
                'poker.action': MagicMock(),
                'gui.utils': MagicMock(),
            }):
                from services.vision_controller import VisionMode
                from gui.main_window import MainWindow
                valid_modes = set(VisionMode)
                for key in MainWindow._MODE_COLOURS:
                    self.assertIn(key, valid_modes,
                        f"_MODE_COLOURS has stale key {key} not in VisionMode")


if __name__ == '__main__':
    unittest.main()
