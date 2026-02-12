from commands2 import Subsystem
from FROGlib.vision import FROGPoseEstimator, FROGDetector, DetectorTunables
from constants import kCameraConfigs, kDetectorConfigs
from typing import Optional


class FuelDetector(FROGDetector, Subsystem):
    def __init__(self):
        super().__init__(kDetectorConfigs[0])
        self.fuel_detector_tunables = DetectorTunables()

    def periodic(self) -> None:
        # update fuel detection data
        self.get_targets()
        self.process_targets()
        super().periodic()
