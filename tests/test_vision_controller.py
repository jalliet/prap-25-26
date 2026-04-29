"""Tests for VisionController dual-camera refactor.

Signal-existence tests use AST inspection rather than live imports so that
this file can run in the same pytest process as test_arm_ros_bridge.py.
Importing VisionController (a QObject subclass) after a different QObject
instance has been created and GC'd in the same process can corrupt PySide6's
shiboken C++ type registry and cause a segfault on macOS.
"""
import ast
import sys
import pathlib
import unittest
from unittest.mock import patch, MagicMock


_VC_SRC = pathlib.Path(__file__).parent.parent / 'services' / 'vision_controller.py'


def _block_hardware():
    """Prevent hardware and PySide6 C-extension imports (mocking PySide6 avoids
    shiboken segfaults when Qt is re-imported after a QObject has been GC'd)."""
    return patch.dict('sys.modules', {
        'depthai': MagicMock(),
        'ultralytics': MagicMock(),
        'PySide6': MagicMock(),
        'PySide6.QtCore': MagicMock(),
    })


def _vc_class_node():
    """Return the AST ClassDef node for VisionController."""
    tree = ast.parse(_VC_SRC.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == 'VisionController':
            return node
    raise AssertionError("VisionController class not found in vision_controller.py")


def _class_assign_names(class_node):
    """Return top-level assignment target names in a class body."""
    names = []
    for stmt in class_node.body:
        if isinstance(stmt, ast.Assign):
            for target in stmt.targets:
                if isinstance(target, ast.Name):
                    names.append(target.id)
    return names


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
        """VisionController source declares chip_frame_ready as a class attribute."""
        self.assertIn('chip_frame_ready', _class_assign_names(_vc_class_node()))

    def test_chips_detected_signal_exists(self):
        """VisionController source declares chips_detected as a class attribute."""
        self.assertIn('chips_detected', _class_assign_names(_vc_class_node()))


_MW_SRC = pathlib.Path(__file__).parent.parent / 'gui' / 'main_window.py'


class TestMainWindowModeColours(unittest.TestCase):
    """Ensure _MODE_COLOURS only references valid VisionMode values."""

    def test_mode_colours_keys_are_valid_vision_modes(self):
        """Parse _MODE_COLOURS from source; validate keys against VisionMode names."""
        with _block_hardware():
            sys.modules.pop('services.vision_controller', None)
            from services.vision_controller import VisionMode
        valid_names = {m.name for m in VisionMode}

        tree = ast.parse(_MW_SRC.read_text())
        for cls in ast.walk(tree):
            if not (isinstance(cls, ast.ClassDef) and cls.name == 'MainWindow'):
                continue
            for stmt in cls.body:
                if not (isinstance(stmt, ast.Assign) and
                        any(isinstance(t, ast.Name) and t.id == '_MODE_COLOURS'
                            for t in stmt.targets)):
                    continue
                if not isinstance(stmt.value, ast.Dict):
                    self.fail("_MODE_COLOURS is not a dict literal")
                for key in stmt.value.keys:
                    if isinstance(key, ast.Attribute):
                        self.assertIn(key.attr, valid_names,
                            f"_MODE_COLOURS has stale VisionMode key: {key.attr}")
                return
        self.fail("_MODE_COLOURS not found in MainWindow")


if __name__ == '__main__':
    unittest.main()
