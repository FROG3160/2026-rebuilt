import math
import numpy as np
from wpimath.units import degreesToRadians, feetToMeters, inchesToMeters
from FROGlib.vision import FROGCameraConfig
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Rotation2d, Pose2d

# Camera Configs
kCameraConfigs = (
    FROGCameraConfig(
        "OV9281-AprilTag2",
        Transform3d(
            Translation3d(
                inchesToMeters(10.5 + (1 / 16)),  # Forward from center
                inchesToMeters(-9.375),  # Left from center
                0.435,  # Up from the floor
            ),
            Rotation3d(0, 0, -math.pi / 2),
        ),
    ),
    FROGCameraConfig(
        "OV9281-AprilTag1",
        Transform3d(
            Translation3d(
                inchesToMeters((25.25 / 2) - 0.75),  # Forward from center
                inchesToMeters(0),  # Left from center
                inchesToMeters(19.25),  # Up from the floor
            ),
            Rotation3d(0, degreesToRadians(35), 0),
        ),
    ),
)
kDetectorConfigs = (FROGCameraConfig("Object1", Transform3d()),)

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
# kDriveFeedForward = 0.53
# kDutyCycleDriveV = 0.00916
# kDutyCycleDriveS = 0.01125

kVoltageDriveV = 1.8  # 2.0164  # 1.795
kVoltageDriveS = 0.060831  # 0.211
kVoltageDriveP = 2.9646  # 0.5
kVoltageDriveA = 0.14705  # 0.027631

# intake gains
kVoltageIntakeS = 0.12
kIntakeV = 1.41  # V/(m/s) feedforward - approx 12V / 8.48 m/s free speed
kIntakeP = (
    0.0  # velocity PID proportional gain - set to 0.0 until characterized with SysId
)
kIntakeMinSpeed = 2.0  # m/s - minimum intake surface speed floor
kIntakeSpeedMultiplier = 1.5  # intake speed = max(min, robot_speed * multiplier)
kIntakeReverseSpeed = 3.0  # m/s - fixed reverse speed for ejecting

# intake deploy gains
kIntakeDeployDistancePerRotation = 0.151613  # meters
kIntakeDeployTargetMeters = 0.26113
kIntakeDeployP = 10.0
kIntakeDeployI = 0.0
kIntakeDeployD = 0.0
kIntakeDeployS = 0.0
kIntakeDeployV = 0.0
kIntakeDeployMM_V = 2.0  # m/s
kIntakeDeployMM_A = 8.0  # m/s^2
kIntakeDeployCurrentLimit = 20.0  # Amps

# hopper gains
kVoltageHopperS = 0.12
kHopperV = 0.11  # estimated V/(rad/s)
kHopperP = 0.5  # Position PID proportional gain
kHopperI = 0.0
kHopperD = 0.0
kHopperMM_V = 20.0  # Max velocity (RPS) for MotionMagic
kHopperMM_A = 40.0  # Max acceleration (RPS/s) for MotionMagic

kLiftG = 0.0

kLiftRatio = 45 / (
    14 * 0.25
)  # (gear reduction (45) / (sprocket teeth (14) * pitch (0.25 inches per tooth)))

# Feed/transfer motor gains
kFeedS = 0.24257
kFeedV = 1.2853
kFeedA = 0.1092
kFeedVelocityP = 0.0  # velocity PID
kFeedVelocityI = 0.0
kFeedVelocityD = 0.0
kFeedPositionP = 9.9692
kFeedPositionI = 0.0
kFeedPositionD = 0.62293

# flywheel gains
kFlywheelP = 0.50197
kFlywheelI = 0.0
kFlywheelD = 0.0
kFlywheelS = 0.095535  # determined with follower helping
kFlywheelV = 0.34733  # 0.342  # 0.351
kFlywheelA = 0.018088

# Distance (meters) to Flywheel Speed (rotations/sec) interpolation data for Hub target
# Important: np.interp requires the x-coordinates to be in increasing order.
kShootersHubDistances = np.array([2.06, 2.20, 2.89, 3.57, 5.05])
kShootersHubSpeeds = np.array([17.65, 18.15, 19.85, 21.40, 23.76])

# Hood motor gains
# It takes about 0.45 volts to move the lead screw to push the hood up,
# and about -0.4 volts to move it down, so we can calculate kS and kG from those values
kHoodS = 0.425  # (|Upward V| + |Downward V|) / 2
kHoodP = 3.0
kHoodV = 0.12
kHoodG = 0.025
kHoodMMV = 8.0
kHoodMMA = 16.0  # (Upward V - Downward V) / 2
kHoodForwardLimit = 1.15  # rotations
kHoodReverseLimit = 0.0

# tolerances
kFlywheelTolerance = 0.2  # tolerance in m/s

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
kIntakeDeployMotorID = 41
kHopperLeftMotorID = 46
kHopperRightMotorID = 47
kHopperLeftSensorID = 48
kHopperRightSensorID = 49
kFeedMotorID = 42
kShooterLeftFlywheelID = 43
kShooterRightFlywheelID = 44
kHoodMotorID = 45


# PDH Channels
kShooterPDHChannel = 5  # an example.


# ROGOT CHARACTERISTICS
#########################################

# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(24.25)  # 0.62
kWheelBaseMeters = inchesToMeters(19.75)  # 0.5

kMaxMetersPerSecond = feetToMeters(16)  # 16 feet per second
kMaxChassisRadiansPerSec = 2 * math.tau  # revolutions per sec * tau

kProfiledRotationMaxVelocity = kMaxChassisRadiansPerSec * 2
kProfiledRotationMaxAccel = (
    kMaxChassisRadiansPerSec * 4
)  # 4 rotations per second per second

kProfiledRotationP = 0.6
kProfiledRotationI = 0.0
kProfiledRotationD = 0.0

## PathPlanner Constants
#########################################
kPPTranslationP = 2.0
kPPTranslationI = 0.0
kPPTranslationD = 0.0

kPPRotationP = 4.0
kPPRotationI = 0.0
kPPRotationD = 0.0

kPPMaxVelocity = 3.0
kPPMaxAcceleration = 3.0
kPPMaxAngularVelocity = 4.0
kPPMaxAngularAcceleration = 8.0

## Network Tables names
#########################################
kComponentSubtableName = "FROGSubsystems"

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
kFieldLength = 16.541
kFieldWidth = 8.07
kFieldMidlineX = kFieldLength / 2
kFieldMidlineY = kFieldWidth / 2

# Hub X boundaries (based on AprilTags)
kHubXBlueFacingCenter = 5.229
kHubXRedFacingCenter = 11.312
kHubXBlueFacingAlliance = 4.022
kHubXRedFacingAlliance = 12.519

kBlueHub = Pose2d(4.626, kFieldMidlineY, Rotation2d(0))  # Facing away from blue hub
kRedHub = Pose2d(11.915, kFieldMidlineY, Rotation2d(0))
kBlueRightTrench = Pose2d(4.692, 0.635, Rotation2d(0))
kBlueLeftTrench = Pose2d(4.692, 7.489, Rotation2d(0))
kRedRightTrench = Pose2d(11.849, 0.635, Rotation2d(math.pi))
kRedLeftTrench = Pose2d(11.849, 7.489, Rotation2d(math.pi))

kBlueRightCorner = Pose2d(1.0, 1.0, Rotation2d(0))
kBlueLeftCorner = Pose2d(1.0, 7.2, Rotation2d(0))
kRedRightCorner = Pose2d(15.5, 1.0, Rotation2d(math.pi))
kRedLeftCorner = Pose2d(15.5, 7.2, Rotation2d(math.pi))
