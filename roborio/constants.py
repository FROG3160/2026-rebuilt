from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue

from FROGlib.utils import GearStage
from wpimath.units import inchesToMeters

## Swerve Module Constants
kSwerveDriveGearing = [
    GearStage(16, 50),
    GearStage(28, 16),
    GearStage(15, 45),
]  # Mk4c L3
kWheelDiameter = inchesToMeters(4)  # The tread has worn down
