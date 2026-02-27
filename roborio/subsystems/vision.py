from FROGlib.subsystem import FROGSubsystem
from FROGlib.vision import FROGPoseEstimator, FROGDetector
from constants import kCameraConfigs, kDetectorConfigs
from typing import Optional


class FuelDetector(FROGDetector, FROGSubsystem):
    def __init__(self):
        super().__init__(kDetectorConfigs[0])
        self._min_confidence = 0.8
        self._cluster_radius_deg = 12.0
        self._min_neighbors = 2
        self._use_area_weighting = True

    def periodic(self) -> None:
        super().periodic()  # FROGSubsystem updates telemetry
        # update fuel detection data
        self.get_targets()
        self.process_targets()

    @FROGSubsystem.tunable(0.8, "Min Confidence")
    def min_confidence_tunable(self, val):
        self._min_confidence = val

    @FROGSubsystem.tunable(12.0, "Cluster Radius (deg)")
    def cluster_radius_tunable(self, val):
        self._cluster_radius_deg = val

    @FROGSubsystem.tunable(2, "Min Neighbors")
    def min_neighbors_tunable(self, val):
        self._min_neighbors = int(val)

    @FROGSubsystem.tunable(True, "Use Area Weighting")
    def use_area_weighting_tunable(self, val):
        self._use_area_weighting = val
