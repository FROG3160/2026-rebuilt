import math
from wpimath.units import feetToMeters, inchesToMeters
from FROGlib.vision import FROGCameraConfig
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Rotation2d, Pose2d

# Camera Configs
kCameraConfigs = (
    FROGCameraConfig(
        "Arducam_OV9281_Apriltag_2",
        Transform3d(
            Translation3d(
                inchesToMeters(7.45),  # 7.1998),  # Forward from center
                inchesToMeters(-9.9),  # -8.125),  # Left from center
                0.29,  # inchesToMeters(11.51),  # 11.375),  # Up from the floor
            ),
            Rotation3d(0, 0, -math.pi / 2),
        ),
    ),
    FROGCameraConfig(
        "Arducam_OV9281_Apriltag_1",
        Transform3d(
            Translation3d(
                inchesToMeters(7.0),  # 7.0625),  # Forward from center
                inchesToMeters(9.9),  # 8.125),  # Left from center
                0.69,  # inchesToMeters(27.32),  # 27.25),  # Up from the floor
            ),
            Rotation3d(0, 0, math.pi / 2),
        ),
    ),
)
kDetectorConfigs = (FROGCameraConfig("Arducam_OV9782_Obj_Detection", Transform3d()),)

## CANCoder offsets
########################################
kFrontLeftOffset = 0.042480
kFrontRightOffset = 0.250244
kBackLeftOffset = -0.236572
kBackRightOffset = -0.448486


## Swerve Drive Gains
########################################
# steer motor gains
kSteerP = 39.236  # 2.402346
kSteerI = 0  # 0.200195\
kSteerS = 0.215
kSteerV = 0.10583

# drive motor gains
kDriveFeedForward = 0.53
kDutyCycleDriveV = 0.00916
kDutyCycleDriveS = 0.01125

kVoltageDriveV = 1.795
kVoltageDriveS = 0.211
kVoltageDriveP = 0.5
kVoltageDriveA = 0.027631

# intake gains
kVoltageIntakeS = 0.12

# hopper gains
kVoltageHopperS = 0.12

# climber gains
kDeployP = 1.0
kDeployI = 0.0
kDeployD = 0.0
kDeployS = 0.0
kDeployV = 0.0

# climber lift gains
kLiftP = 1.0
kLiftI = 0.0
kLiftD = 0.0
kLiftS = 0.0
kLiftV = 0.0
kLiftG = 0.0

# Feed/transfer motor gains
kFeedS = 0.2

# flywheel gains
kFlywheelP = 0
kFlywheelI = 0.0
kFlywheelD = 0.0
kFlywheelS = 0.19  # determined with follower helping
kFlywheelV = 0.351
kFlywheelA = 0

# Hood motor gains
kHoodS = 0.15
kHoodForwardLimit = 0.07646483
kHoodReverseLimit = 0.0

# tolerances
kFlywheelTolerance = 1.0  # tolerance in m/s

## CAN ID assignments
########################################
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

kIntakeMotorID = 40
kHopperMotorID = 41
kFeedMotorID = 42
kShooterLeftFlywheelID = 43
kShooterRightFlywheelID = 44
kHoodMotorID = 45

kClimberDeployMotorID = 50
kClimberLeftLiftMotorID = 51
kClimberRightLiftMotorID = 52


# ROGOT CHARACTERISTICS
#########################################

# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(24.25)  # 0.62
kWheelBaseMeters = inchesToMeters(19.75)  # 0.5

kMaxMetersPerSecond = feetToMeters(16)  # 16 feet per second
kMaxChassisRadiansPerSec = 2 * math.tau  # revolutions per sec * tau

kProfiledRotationMaxVelocity = kMaxChassisRadiansPerSec
kProfiledRotationMaxAccel = (
    kMaxChassisRadiansPerSec * 2
)  # 2 rotations per second per second

kProfiledRotationP = 0.4
kProfiledRotationI = 0.0
kProfiledRotationD = 0.0

## Network Tables names
#########################################
kComponentSubtableName = "Subsystems"

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

# Field Positions
kBlueHub = Pose2d(4.626, 4.035, Rotation2d(0))  # Facing away from blue hub
kRedHub = Pose2d(11.915, 4.035, Rotation2d(0))
kBlueRightTrench = Pose2d(4.692, 0.635, Rotation2d(0))
kBlueLeftTrench = Pose2d(4.692, 7.489, Rotation2d(0))
kRedRightTrench = Pose2d(11.849, 0.635, Rotation2d(math.pi))
kRedLeftTrench = Pose2d(11.849, 7.489, Rotation2d(math.pi))
