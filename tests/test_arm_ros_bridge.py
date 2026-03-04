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

    def test_bridge_no_ros_available(self):
        """Bridge should create successfully and report unavailable when rclpy missing."""
        # Patch rclpy as missing at module level
        blocked_modules = {
            'rclpy': None,
            'rclpy.node': None,
            'rclpy.action': None,
            'poker_interfaces': None,
            'poker_interfaces.msg': None,
            'poker_interfaces.action': None,
        }
        with patch.dict('sys.modules', blocked_modules):
            # Force re-import to pick up the mocked imports
            if 'services.arm_ros_bridge' in sys.modules:
                del sys.modules['services.arm_ros_bridge']

            from services.arm_ros_bridge import ArmRosBridge
            # Reset singleton for test isolation
            ArmRosBridge._instance = None

            bridge = ArmRosBridge()
            self.assertFalse(bridge.is_available)

            # These should not raise
            bridge.publish_pose_proposal(0.3, 0.0, 0.2, 0.0, 0.0, 5.0)
            bridge.publish_joint_proposal([0.0] * 6, 5.0)
            bridge.shutdown()

            # Clean up singleton
            ArmRosBridge._instance = None


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
