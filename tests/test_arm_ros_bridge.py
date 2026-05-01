"""
Tests for the ArmRosBridge and vision detectors.

Unit tests run without ROS 2 by mocking rclpy imports.
"""
import sys
import unittest
from unittest.mock import patch, MagicMock
import importlib
import numpy as np


class TestArmRosBridgeWithoutRos(unittest.TestCase):
    """Verify graceful degradation when rclpy is not installed."""

    def _load_bridge_module(self):
        """Re-import services.arm_ros_bridge with rclpy/poker_interfaces masked.

        Returns the freshly-loaded ``ArmRosBridge`` class.
        """
        blocked_modules = {
            'rclpy': None,
            'rclpy.node': None,
            'rclpy.action': None,
            'poker_interfaces': None,
            'poker_interfaces.msg': None,
            'poker_interfaces.action': None,
        }
        # patch.dict alone is not enough — we need to nuke the cached module so
        # the conditional import block re-evaluates with the masked modules.
        if 'services.arm_ros_bridge' in sys.modules:
            del sys.modules['services.arm_ros_bridge']
        return blocked_modules

    def test_bridge_no_ros_available(self):
        """Bridge creates successfully and reports unavailable when rclpy missing."""
        with patch.dict('sys.modules', self._load_bridge_module()):
            from services.arm_ros_bridge import ArmRosBridge
            ArmRosBridge._instance = None

            bridge = ArmRosBridge()
            self.assertFalse(bridge.is_available)

            # The supported action API must be safe to call even with no ROS.
            # ``move_completed`` will emit (False, -1.0) but should not raise.
            bridge.move_pose(0.3, 0.0, 0.2, 0.0, 0.0, 5.0)
            bridge.move_joints([0.0] * 6, 5.0)
            bridge.shutdown()

            ArmRosBridge._instance = None

    def test_validate_pose_rejects_bad_input(self):
        """Validation helpers reject obvious garbage even with no ROS."""
        with patch.dict('sys.modules', self._load_bridge_module()):
            from services.arm_ros_bridge import ArmRosBridge

            # Non-positive duration
            self.assertIsNotNone(
                ArmRosBridge._validate_pose(0.1, 0.0, 0.1, 0.0, 0.0, 0.0))
            # Out-of-workspace reach
            self.assertIsNotNone(
                ArmRosBridge._validate_pose(2.0, 2.0, 2.0, 0.0, 0.0, 1.0))
            # Tilt > pi
            self.assertIsNotNone(
                ArmRosBridge._validate_pose(0.1, 0.0, 0.1, 5.0, 0.0, 1.0))
            # Plausible pose
            self.assertIsNone(
                ArmRosBridge._validate_pose(0.2, 0.0, 0.1, 0.0, 0.0, 1.5))

    def test_validate_joints_rejects_bad_input(self):
        """Joint vector validation rejects wrong length / non-positive duration."""
        with patch.dict('sys.modules', self._load_bridge_module()):
            from services.arm_ros_bridge import ArmRosBridge

            # Wrong length
            self.assertIsNotNone(
                ArmRosBridge._validate_joints([0.0] * 5, 1.0))
            # Non-positive duration
            self.assertIsNotNone(
                ArmRosBridge._validate_joints([0.0] * 6, 0.0))
            # Plausible
            self.assertIsNone(
                ArmRosBridge._validate_joints([0.0] * 6, 1.5))


class TestCardDetector(unittest.TestCase):
    """Test CardDetector without requiring ultralytics."""

    def test_dummy_mode_returns_empty(self):
        """CardDetector with no model should return empty list."""
        from vision.card_detector import CardDetector
        detector = CardDetector(model_path=None)
        result = detector.process(np.zeros((480, 640, 3), dtype=np.uint8))
        self.assertEqual(result, [])

    def test_parse_class_name(self):
        """Test YOLO class name to Rank/Suit parsing."""
        from vision.card_detector import _parse_class_name
        from poker.card import Rank, Suit

        # Standard cards
        rank, suit = _parse_class_name("AH")
        self.assertEqual(rank, Rank.ACE)
        self.assertEqual(suit, Suit.HEARTS)

        rank, suit = _parse_class_name("KS")
        self.assertEqual(rank, Rank.KING)
        self.assertEqual(suit, Suit.SPADES)

        # 10 → T mapping
        rank, suit = _parse_class_name("10C")
        self.assertEqual(rank, Rank.TEN)
        self.assertEqual(suit, Suit.CLUBS)

        rank, suit = _parse_class_name("2D")
        self.assertEqual(rank, Rank.TWO)
        self.assertEqual(suit, Suit.DIAMONDS)

        # Invalid
        rank, suit = _parse_class_name("XX")
        self.assertIsNone(rank)
        self.assertIsNone(suit)


class TestChipSegmentor(unittest.TestCase):
    """Test ChipSegmentor without requiring ultralytics."""

    def test_dummy_mode_returns_zero_stack(self):
        """ChipSegmentor with no model should return zero-value stack."""
        from vision.chip_segmentor import ChipSegmentor
        segmentor = ChipSegmentor(model_path=None)
        result = segmentor.process(np.zeros((480, 640, 3), dtype=np.uint8))
        self.assertEqual(result["stack"].total, 0)
        self.assertEqual(result["confidence"], 0.0)


class TestChipColourValues(unittest.TestCase):
    """Verify chip colour values match updated denominations."""

    def test_chip_values(self):
        from poker.chips import ChipColour
        self.assertEqual(ChipColour.RED.value, 1)
        self.assertEqual(ChipColour.BLUE.value, 5)
        self.assertEqual(ChipColour.WHITE.value, 20)

    def test_no_black_chip(self):
        from poker.chips import ChipColour
        names = [c.name for c in ChipColour]
        self.assertNotIn("BLACK", names)


if __name__ == '__main__':
    unittest.main()
