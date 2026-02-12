import unittest
from unittest.mock import Mock
import numpy as np

# Adjust import based on your project structure
from FROGlib.vision import FROGDetector, PhotonTrackedTarget


class MockPhotonTrackedTarget(PhotonTrackedTarget):
    """Simple stand-in for PhotonTrackedTarget with only the attributes we need."""

    def __init__(
        self, yaw: float, pitch: float, area: float = 0.05, objDetectConf: float = 0.9
    ):
        self.yaw = yaw
        self.pitch = pitch
        self.area = area
        self.objDetectConf = objDetectConf


class TestFROGDetectorHasTargetNearBottomCenter(unittest.TestCase):
    def setUp(self):
        # Create a detector instance (camera name doesn't matter for this test)
        mock_config = Mock(name="mock_camera_config")
        mock_config.name = "MockCamera"
        self.detector = FROGDetector(mock_config)

    def test_no_targets_returns_false(self):
        self.detector.targets = []
        self.detector.process_targets()
        self.assertFalse(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-18.0, min_area=0.01
            )
        )

    def test_targets_present_but_none_in_zone_returns_false(self):
        self.detector.targets = [
            MockPhotonTrackedTarget(yaw=15.0, pitch=-5.0),  # too far right, too high
            MockPhotonTrackedTarget(yaw=-12.0, pitch=-10.0),  # too far left
        ]
        self.detector.process_targets()
        self.assertFalse(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-18.0, min_area=0.01
            )
        )

    def test_single_target_exactly_at_center_bottom_returns_true(self):
        self.detector.targets = [
            MockPhotonTrackedTarget(yaw=0.0, pitch=-18.0, area=0.03)
        ]
        self.detector.process_targets()
        self.assertTrue(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-18.0, min_area=0.01
            )
        )

    def test_multiple_targets_one_in_zone_returns_true(self):
        self.detector.targets = [
            MockPhotonTrackedTarget(yaw=20.0, pitch=-5.0),  # out
            MockPhotonTrackedTarget(yaw=-4.0, pitch=-22.0, area=0.04),  # in
            MockPhotonTrackedTarget(yaw=8.0, pitch=-12.0),  # out
        ]
        self.detector.process_targets()
        self.assertTrue(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-18.0, min_area=0.01
            )
        )

    def test_target_just_outside_yaw_threshold_returns_false(self):
        self.detector.targets = [MockPhotonTrackedTarget(yaw=10.1, pitch=-20.0)]
        self.detector.process_targets()
        self.assertFalse(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-18.0, min_area=0.01
            )
        )

    def test_small_area_target_ignored_returns_false(self):
        self.detector.targets = [
            MockPhotonTrackedTarget(yaw=0.0, pitch=-20.0, area=0.005)
        ]
        self.detector.process_targets()
        self.assertFalse(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-18.0, min_area=0.01
            )
        )

    def test_custom_thresholds_work(self):
        self.detector.targets = [MockPhotonTrackedTarget(yaw=0.0, pitch=-15.0)]
        self.detector.process_targets()
        # Default would be True, but tighter pitch threshold makes it False
        self.assertFalse(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-18.0, min_area=0.0
            )
        )
        # Looser pitch should now pass
        self.assertTrue(
            self.detector.has_targets_close(
                yaw_threshold_deg=10.0, pitch_threshold_deg=-14.0, min_area=0.0
            )
        )


if __name__ == "__main__":
    unittest.main()
