import math
from wpimath.units import feetToMeters, inchesToMeters


## CANCoder offsets
########################################
kFrontLeftOffset = 0.168701  # -0.246338
kFrontRightOffset = -0.260986  # 0.036377
kBackLeftOffset = 0.383057  # 0.478027
kBackRightOffset = -0.244141  # 0.207031
KShoulderOffset = (
    0.124267875  # 0.123047  # -0.05859375  # offset with arm parallel to the floor
)

## Swerve Drive Gains
########################################
# steer motor gains
kSteerP = 39.236  # 2.402346
kSteerI = 0  # 0.200195\
kSteerS = 0.15
kSteerV = 0.10583

# drive motor gains
kDriveFeedForward = 0.53
kDutyCycleDriveV = 0.00916
kDutyCycleDriveS = 0.01125

kVoltageDriveV = 1.928
kVoltageDriveS = 0.14
kVoltageDriveP = 2.0
kVoltageDriveA = 0.027631


## CAN ID assignments
########################################
# TODO: Update these CAN IDs to match the robot
# Swerve Drive Motor/Encoder IDs
kFrontLeftDriveID = 11
kFrontLeftSteerID = 21
kFrontLeftSensorID = 31

kFrontRightDriveID = 12
kFrontRightSteerID = 22
kFrontRightSensorID = 32

kBackLeftDriveID = 13
kBackLeftSteerID = 23
kBackLeftSensorID = 33

kBackRightDriveID = 14
kBackRightSteerID = 24
kBackRightSensorID = 34

kGyroID = 39


# ROGOT CHARACTERISTICS
#########################################

# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(22.750)
kWheelBaseMeters = inchesToMeters(25.750)

kMaxMetersPerSecond = feetToMeters(16)  # 16 feet per second
kMaxChassisRadiansPerSec = 2 * math.tau  # revolutions per sec * tau

kProfiledRotationMaxVelocity = 1.5 * math.tau  # 1.5 rotations per second
kProfiledRotationMaxAccel = 2 * math.tau  # 2 rotations per second per second

kProfiledRotationP = 0.4
kProfiledRotationI = 0.0
kProfiledRotationD = 0.0

## Network Tables names
#########################################
kComponentSubtableName = "Components"

## Xbox Controller Constants
#########################################
# Xbox controller ports
kDriverControllerPort = 0
kOperatorControllerPort = 1

# Xbox controller constants
kDeadband = 0.15
kDebouncePeriod = 0.5
kTranslationSlew = 2
kRotSlew = 2
