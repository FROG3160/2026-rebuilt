from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue

from FROGlib.utils import GearStage
from wpimath.units import inchesToMeters

## Swerve Module Constants
drive_motor_output_params = {
    # set neutral mode for drive motor
    "neutral_mode": NeutralModeValue.BRAKE,
    # with the non-inverted SwerveDriveSpecialties swerve modules, and
    # the bevel gears facing left, the drive motors need to be inverted
    # in order to move the drivetrain forward with a positive value.
    # the default inverted setting is CCW positive.
    "inverted": InvertedValue.CLOCKWISE_POSITIVE,
}
steer_motor_output_params = {
    "neutral_mode": NeutralModeValue.BRAKE,
    "inverted": InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
}
kSwerveDriveGearing = [
    GearStage(16, 50),
    GearStage(28, 16),
    GearStage(15, 45),
]  # Mk4c L3
kWheelDiameter = inchesToMeters(4)  # The tread has worn down
