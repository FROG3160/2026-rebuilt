import math
import numpy as np
from typing import Final
from wpimath.units import degreesToRadians, feetToMeters, inchesToMeters
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Rotation2d, Pose2d
from FROGlib.vision import FROGCameraConfig

# Camera Configs
CAMERA_CONFIGS: Final = (
    FROGCameraConfig(
        "OV9281-AprilTag1",
        Transform3d(
            Translation3d(
                inchesToMeters(-7.875), inchesToMeters(11.9375), inchesToMeters(17.1875)
            ),
            Rotation3d(0.0, degreesToRadians(-20.0), degreesToRadians(92.0)),
        ),
    ),
    FROGCameraConfig(
        "OV9281-AprilTag2",
        Transform3d(
            Translation3d(
                inchesToMeters(-6.5), inchesToMeters(-13.9375), inchesToMeters(19.5)
            ),
            Rotation3d(0.0, degreesToRadians(-20.0), degreesToRadians(-89.86)),
        ),
    ),
)

DETECTOR_CONFIGS: Final = ()


class CANIDs:
    FRONT_LEFT_DRIVE: Final = 11
    FRONT_LEFT_STEER: Final = 21
    FRONT_LEFT_SENSOR: Final = 31

    FRONT_RIGHT_DRIVE: Final = 12
    FRONT_RIGHT_STEER: Final = 22
    FRONT_RIGHT_SENSOR: Final = 32

    BACK_LEFT_DRIVE: Final = 13
    BACK_LEFT_STEER: Final = 23
    BACK_LEFT_SENSOR: Final = 33

    BACK_RIGHT_DRIVE: Final = 14
    BACK_RIGHT_STEER: Final = 24
    BACK_RIGHT_SENSOR: Final = 34

    GYRO: Final = 39

    ROLLER_MOTOR: Final = 40
    DEPLOY_MOTOR: Final = 41

    FEED_MOTOR: Final = 42
    SHOOTER_LEFT_FLYWHEEL: Final = 43
    SHOOTER_RIGHT_FLYWHEEL: Final = 44
    HOOD_MOTOR: Final = 45

    HOPPER_LEFT_MOTOR: Final = 46
    HOPPER_RIGHT_MOTOR: Final = 47
    HOPPER_LEFT_SENSOR: Final = 48
    HOPPER_RIGHT_SENSOR: Final = 49

    SHOOTER_PDH_CHANNEL: Final = 5


class Drive:
    # CANCoder offsets
    FRONT_LEFT_OFFSET: Final = 0.042480
    FRONT_RIGHT_OFFSET: Final = 0.250244
    BACK_LEFT_OFFSET: Final = -0.236572
    BACK_RIGHT_OFFSET: Final = -0.448486

    # Steer gains
    STEER_P: Final = 39.236
    STEER_I: Final = 0
    STEER_S: Final = 0.215
    STEER_V: Final = 0.10583

    # Drive motor gains
    VOLTAGE_DRIVE_V: Final = 2.2
    VOLTAGE_DRIVE_S: Final = 0.14
    VOLTAGE_DRIVE_P: Final = 2.5
    VOLTAGE_DRIVE_A: Final = 0.23

    # Physical dimensions
    TRACK_WIDTH_METERS: Final = inchesToMeters(24.25)
    WHEEL_BASE_METERS: Final = inchesToMeters(19.75)

    MAX_METERS_PER_SECOND: Final = feetToMeters(16)
    MAX_CHASSIS_RADIANS_PER_SEC: Final = 2 * math.tau

    PROFILED_ROTATION_MAX_VELOCITY: Final = MAX_CHASSIS_RADIANS_PER_SEC * 2
    PROFILED_ROTATION_MAX_ACCEL: Final = MAX_CHASSIS_RADIANS_PER_SEC * 4

    PROFILED_ROTATION_P: Final = 0.6
    PROFILED_ROTATION_I: Final = 0.0
    PROFILED_ROTATION_D: Final = 0.0

    # PathPlanner
    PP_TRANSLATION_P: Final = 2.0
    PP_TRANSLATION_I: Final = 0.0
    PP_TRANSLATION_D: Final = 0.0
    PP_ROTATION_P: Final = 4.0
    PP_ROTATION_I: Final = 0.0
    PP_ROTATION_D: Final = 0.0
    PP_MAX_VELOCITY: Final = 3.0
    PP_MAX_ACCELERATION: Final = 3.0
    PP_MAX_ANGULAR_VELOCITY: Final = 4.0
    PP_MAX_ANGULAR_ACCELERATION: Final = 8.0


class Intake:
    VOLTAGE_ROLLER_S: Final = 0.14
    ROLLER_V: Final = 1.41
    ROLLER_P: Final = 0.0
    ROLLER_MIN_SPEED: Final = 4.0
    ROLLER_SPEED_MULTIPLIER: Final = 1.5

    INTAKE_DEPLOY_DISTANCE_PER_ROTATION: Final = 0.151613 / 9
    INTAKE_DEPLOY_TARGET_METERS: Final = 0.259
    INTAKE_CYCLE_TARGET_METERS: Final = 0.09
    INTAKE_DEPLOY_P: Final = 16.0
    INTAKE_DEPLOY_I: Final = 0.0
    INTAKE_DEPLOY_D: Final = 0.0
    INTAKE_DEPLOY_S: Final = 0.25
    INTAKE_DEPLOY_V: Final = 6.0  #
    INTAKE_DEPLOY_MM_V: Final = 1.0
    INTAKE_DEPLOY_MM_A: Final = 2.0
    INTAKE_DEPLOY_CURRENT_LIMIT: Final = 10.0
    HOMING_VOLTAGE: Final = -1.0
    HOMING_CURRENT: Final = 10.0


class Hopper:
    VOLTAGE_HOPPER_S: Final = 0.25
    HOPPER_V: Final = 1.45
    HOPPER_P: Final = 0.0
    HOPPER_I: Final = 0.0
    HOPPER_D: Final = 0.0
    HOPPER_MM_V: Final = 6.38
    HOPPER_MM_A: Final = 12.77
    SENSOR_TO_MECHANISM_RATIO: Final = 15.665


class Shooter:
    FLYWHEEL_P: Final = 0.50197
    FLYWHEEL_I: Final = 0.0
    FLYWHEEL_D: Final = 0.0
    FLYWHEEL_S: Final = 0.095535
    FLYWHEEL_V: Final = 0.34733
    FLYWHEEL_A: Final = 0.018088

    SHOOTERS_HUB_DISTANCES: Final = np.array([2.06, 2.20, 2.89, 3.57, 5.05])
    SHOOTERS_HUB_SPEEDS: Final = np.array([17.12, 17.61, 19.25, 20.76, 23.05])

    HOOD_S: Final = 0.425
    HOOD_P: Final = 3.0
    HOOD_V: Final = 0.12
    HOOD_G: Final = 0.025
    HOOD_MMV: Final = 8.0
    HOOD_MMA: Final = 16.0
    HOOD_FORWARD_LIMIT: Final = 1.3
    HOOD_REVERSE_LIMIT: Final = 0.0

    FLYWHEEL_TOLERANCE: Final = 1.0
    HOOD_POSITION_TOLERANCE = Final = 0.05
    HOOD_HOMING_VOLTAGE = Final = -0.75
    HOOD_HOMING_CURRENT = Final = 15.0


class Feeder:
    FEED_S: Final = 0.24257
    FEED_V: Final = 1.2853
    FEED_A: Final = 0.1092
    FEED_VELOCITY_P: Final = 0.0
    FEED_VELOCITY_I: Final = 0.0
    FEED_VELOCITY_D: Final = 0.0
    FEED_POSITION_P: Final = 9.9692
    FEED_POSITION_I: Final = 0.0
    FEED_POSITION_D: Final = 0.62293


class Controller:
    DRIVER_CONTROLLER_PORT: Final = 0
    OPERATOR_CONTROLLER_PORT: Final = 1
    DEADBAND: Final = 0.15
    DEBOUNCE_PERIOD: Final = 0.5
    TRANSLATION_SLEW: Final = 2
    ROT_SLEW: Final = 2


class FieldPositions:
    FIELD_LENGTH: Final = 16.541
    FIELD_WIDTH: Final = 8.07
    FIELD_MIDLINE_X: Final = FIELD_LENGTH / 2
    FIELD_MIDLINE_Y: Final = FIELD_WIDTH / 2

    HUB_X_BLUE_FACING_CENTER: Final = 5.229
    HUB_X_RED_FACING_CENTER: Final = 11.312
    HUB_X_BLUE_FACING_ALLIANCE: Final = 4.022
    HUB_X_RED_FACING_ALLIANCE: Final = 12.519

    BLUE_HUB: Final = Pose2d(4.626, FIELD_MIDLINE_Y, Rotation2d(0))
    RED_HUB: Final = Pose2d(11.915, FIELD_MIDLINE_Y, Rotation2d(0))
    BLUE_RIGHT_TRENCH: Final = Pose2d(4.692, 0.635, Rotation2d(0))
    BLUE_LEFT_TRENCH: Final = Pose2d(4.692, 7.489, Rotation2d(0))
    RED_RIGHT_TRENCH: Final = Pose2d(11.849, 0.635, Rotation2d(math.pi))
    RED_LEFT_TRENCH: Final = Pose2d(11.849, 7.489, Rotation2d(math.pi))

    BLUE_RIGHT_CORNER: Final = Pose2d(1.0, 1.0, Rotation2d(0))
    BLUE_LEFT_CORNER: Final = Pose2d(1.0, 7.2, Rotation2d(0))
    RED_RIGHT_CORNER: Final = Pose2d(15.5, 1.0, Rotation2d(math.pi))
    RED_LEFT_CORNER: Final = Pose2d(15.5, 7.2, Rotation2d(math.pi))
