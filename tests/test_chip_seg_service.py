"""Tests for ChipSegService — no physical webcam required (cv2 mocked)."""
import unittest
from unittest.mock import patch, MagicMock
import numpy as np


class TestChipSegServiceInit(unittest.TestCase):
    def test_default_config(self):
        from services.chip_seg_service import ChipSegService, ChipSegConfig
        svc = ChipSegService()
        self.assertIsInstance(svc.config, ChipSegConfig)
        self.assertEqual(svc.config.device_index, 0)
        self.assertEqual(svc.config.fps, 30)
        self.assertFalse(svc.running)

    def test_custom_config(self):
        from services.chip_seg_service import ChipSegService, ChipSegConfig
        cfg = ChipSegConfig(device_index=2, width=1280, height=720, fps=15)
        svc = ChipSegService(cfg)
        self.assertEqual(svc.config.device_index, 2)
        self.assertEqual(svc.config.fps, 15)


class TestChipSegServiceLifecycle(unittest.TestCase):
    def _make_mock_cap(self, is_opened=True):
        cap = MagicMock()
        cap.isOpened.return_value = is_opened
        return cap

    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_start_sets_running_when_device_opens(self, mock_vc):
        mock_vc.return_value = self._make_mock_cap(is_opened=True)
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        self.assertTrue(svc.running)

    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_start_clears_running_when_device_fails(self, mock_vc):
        mock_vc.return_value = self._make_mock_cap(is_opened=False)
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        self.assertFalse(svc.running)

    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_start_is_idempotent(self, mock_vc):
        mock_vc.return_value = self._make_mock_cap()
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        svc.start()  # second call must be no-op
        mock_vc.assert_called_once()

    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_stop_releases_capture(self, mock_vc):
        cap = self._make_mock_cap()
        mock_vc.return_value = cap
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        svc.stop()
        cap.release.assert_called_once()
        self.assertFalse(svc.running)
        self.assertIsNone(svc.cap)

    def test_stop_when_not_running_is_safe(self):
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.stop()  # must not raise


class TestChipSegServiceGetFrame(unittest.TestCase):
    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_get_frame_returns_array_on_success(self, mock_vc):
        cap = MagicMock()
        cap.isOpened.return_value = True
        fake_frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        cap.read.return_value = (True, fake_frame)
        mock_vc.return_value = cap

        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        frame = svc.get_frame()
        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape, (1080, 1920, 3))

    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_get_frame_returns_none_on_read_failure(self, mock_vc):
        cap = MagicMock()
        cap.isOpened.return_value = True
        cap.read.return_value = (False, None)
        mock_vc.return_value = cap

        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        frame = svc.get_frame()
        self.assertIsNone(frame)

    def test_get_frame_returns_none_when_not_running(self):
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        self.assertIsNone(svc.get_frame())


class TestChipSegServiceSetFps(unittest.TestCase):
    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_set_fps_updates_config(self, mock_vc):
        mock_vc.return_value = MagicMock(**{'isOpened.return_value': True})
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        svc.set_fps(15)
        self.assertEqual(svc.config.fps, 15)

    @patch('services.chip_seg_service.cv2.VideoCapture')
    def test_set_fps_propagates_to_capture(self, mock_vc):
        cap = MagicMock(**{'isOpened.return_value': True})
        mock_vc.return_value = cap
        import cv2
        from services.chip_seg_service import ChipSegService
        svc = ChipSegService()
        svc.start()
        svc.set_fps(15)
        cap.set.assert_any_call(cv2.CAP_PROP_FPS, 15)


if __name__ == '__main__':
    unittest.main()
