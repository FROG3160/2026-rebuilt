import math
import numpy as np
from wpimath.units import degreesToRadians, feetToMeters, inchesToMeters
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Rotation2d, Pose2d
from FROGlib.vision import FROGCameraConfig

# Camera Configs remain global as before
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


class CANIDs:
    FrontLeftDrive = 11
    FrontLeftSteer = 21
    FrontLeftSensor = 31

    FrontRightDrive = 12
    FrontRightSteer = 22
    FrontRightSensor = 32

    BackLeftDrive = 13
    BackLeftSteer = 23
    BackLeftSensor = 33

    BackRightDrive = 14
    BackRightSteer = 24
    BackRightSensor = 34

    Gyro = 39

    IntakeMotor = 40
    IntakeDeployMotor = 41

    FeedMotor = 42
    ShooterLeftFlywheel = 43
    ShooterRightFlywheel = 44
    HoodMotor = 45

    HopperLeftMotor = 46
    HopperRightMotor = 47
    HopperLeftSensor = 48
    HopperRightSensor = 49

    ShooterPDHChannel = 5


class Drive:
    # CANCoder offsets
    FrontLeftOffset = 0.042480
    FrontRightOffset = 0.250244
    BackLeftOffset = -0.236572
    BackRightOffset = -0.448486

    # Steer gains
    SteerP = 39.236
    SteerI = 0
    SteerS = 0.215
    SteerV = 0.10583

    # Drive motor gains
    VoltageDriveV = 1.8
    VoltageDriveS = 0.060831
    VoltageDriveP = 2.9646
    VoltageDriveA = 0.14705

    # Physical dimensions
    TrackWidthMeters = inchesToMeters(24.25)
    WheelBaseMeters = inchesToMeters(19.75)

    MaxMetersPerSecond = feetToMeters(16)
    MaxChassisRadiansPerSec = 2 * math.tau

    ProfiledRotationMaxVelocity = MaxChassisRadiansPerSec * 2
    ProfiledRotationMaxAccel = MaxChassisRadiansPerSec * 4

    ProfiledRotationP = 0.6
    ProfiledRotationI = 0.0
    ProfiledRotationD = 0.0

    # PathPlanner
    PPTranslationP = 2.0
    PPTranslationI = 0.0
    PPTranslationD = 0.0
    PPRotationP = 4.0
    PPRotationI = 0.0
    PPRotationD = 0.0
    PPMaxVelocity = 3.0
    PPMaxAcceleration = 3.0
    PPMaxAngularVelocity = 4.0
    PPMaxAngularAcceleration = 8.0


class Intake:
    VoltageIntakeS = 0.12
    IntakeV = 1.41
    IntakeP = 0.0
    IntakeMinSpeed = 2.0
    IntakeSpeedMultiplier = 1.5
    IntakeReverseSpeed = 3.0

    IntakeDeployDistancePerRotation = 0.151613
    IntakeDeployTargetMeters = 0.26113
    IntakeDeployP = 10.0
    IntakeDeployI = 0.0
    IntakeDeployD = 0.0
    IntakeDeployS = 0.0
    IntakeDeployV = 0.0
    IntakeDeployMM_V = 2.0
    IntakeDeployMM_A = 8.0
    IntakeDeployCurrentLimit = 20.0


class Hopper:
    VoltageHopperS = 0.12
    HopperV = 0.11
    HopperP = 0.5
    HopperI = 0.0
    HopperD = 0.0
    HopperMM_V = 20.0
    HopperMM_A = 40.0


class Shooter:
    FlywheelP = 0.50197
    FlywheelI = 0.0
    FlywheelD = 0.0
    FlywheelS = 0.095535
    FlywheelV = 0.34733
    FlywheelA = 0.018088

    ShootersHubDistances = np.array([2.06, 2.20, 2.89, 3.57, 5.05])
    ShootersHubSpeeds = np.array([17.65, 18.15, 19.85, 21.40, 23.76])

    HoodS = 0.425
    HoodP = 3.0
    HoodV = 0.12
    HoodG = 0.025
    HoodMMV = 8.0
    HoodMMA = 16.0
    HoodForwardLimit = 1.15
    HoodReverseLimit = 0.0

    FlywheelTolerance = 0.2


class Feeder:
    FeedS = 0.0
    FeedV = 0.0
    FeedVelocityP = 0.0
    FeedVelocityI = 0.0
    FeedVelocityD = 0.0


class Controller:
    DriverControllerPort = 0
    OperatorControllerPort = 1
    Deadband = 0.15
    DebouncePeriod = 0.5
    TranslationSlew = 2
    RotSlew = 2


class FieldPositions:
    FieldLength = 16.541
    FieldWidth = 8.07
    FieldMidlineX = FieldLength / 2
    FieldMidlineY = FieldWidth / 2

    HubXBlueFacingCenter = 5.229
    HubXRedFacingCenter = 11.312
    HubXBlueFacingAlliance = 4.022
    HubXRedFacingAlliance = 12.519

    BlueHub = Pose2d(4.626, FieldMidlineY, Rotation2d(0))
    RedHub = Pose2d(11.915, FieldMidlineY, Rotation2d(0))
    BlueRightTrench = Pose2d(4.692, 0.635, Rotation2d(0))
    BlueLeftTrench = Pose2d(4.692, 7.489, Rotation2d(0))
    RedRightTrench = Pose2d(11.849, 0.635, Rotation2d(math.pi))
    RedLeftTrench = Pose2d(11.849, 7.489, Rotation2d(math.pi))

    BlueRightCorner = Pose2d(1.0, 1.0, Rotation2d(0))
    BlueLeftCorner = Pose2d(1.0, 7.2, Rotation2d(0))
    RedRightCorner = Pose2d(15.5, 1.0, Rotation2d(math.pi))
    RedLeftCorner = Pose2d(15.5, 7.2, Rotation2d(math.pi))
