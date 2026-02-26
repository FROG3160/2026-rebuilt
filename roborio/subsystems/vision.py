from FROGlib.subsystem import FROGSubsystem
from FROGlib.vision import FROGPoseEstimator, FROGDetector, DetectorTunables
from constants import kCameraConfigs, kDetectorConfigs
from typing import Optional


class FuelDetector(FROGDetector, FROGSubsystem):
    def __init__(self):
        super().__init__(kDetectorConfigs[0])
        self.fuel_detector_tunables = DetectorTunables()

    def periodic(self) -> None:
        super().periodic()  # FROGSubsystem updates telemetry
        # update fuel detection data
        self.get_targets()
        self.process_targets()
